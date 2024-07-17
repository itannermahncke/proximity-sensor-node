#!/usr/bin/env python3

"""
Test file for VCNL4020 prox/light sensor.
"""

import time
import logging
import statistics
from enum import Enum

import board
import adafruit_vcnl4020

import rospy
from std_msgs.msg import String, Bool


class SensorState(Enum):
    """
    Enum to control the state of the proximity sensor.
    """

    UNCALIBRATED = "uncalibrated"
    CALIBRATING = "calibrating"
    SENSING = "sensing"


class ButtonState(Enum):
    """
    Enum to describe the estimated state of the button.
    """

    UP = "up"
    DOWN = "down"


class ProximitySensorNode(object):
    """
    Node for reporting on button position.
    """

    def __init__(self):
        """
        Initialize an instance of the ProximitySensorNode class.
        """
        # init node
        rospy.init_node("proximity_sensor")

        # create a logger
        self.log = logging.getLogger("rosout.recorder")
        self.log.setLevel(logging.INFO)

        # subscribers
        rospy.Subscriber("/calibrate_proximity_sensor", String, self.on_calibrate)
        rospy.Subscriber("/reset_proximity_sensor", Bool, self.on_reset)
        rospy.Subscriber("/proximity_sensor_state", String, self.on_sensor_state)

        # publishers
        self.state_publisher = rospy.Publisher(
            "/proximity_sensor_state", String, queue_size=1
        )
        self.button_publisher = rospy.Publisher("/button_state", String, queue_size=1)

        # timers
        self.acquisition_timer = None

        # sensor information
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_vcnl4020.Adafruit_VCNL4020(i2c)
        self.sensor.proximity_enabled = True

        # state attributes, fetched from param server if available
        self.state = None
        self.button_up_range = rospy.get_param("button_up_range", None)
        self.button_down_range = rospy.get_param("button_down_range", None)
        self.set_state_by_calibration()

    def set_state_by_calibration(self):
        """
        Updates the sensor state based on its calibration status.
        """
        if self.button_up_range is not None and self.button_down_range is not None:
            self.update_state(SensorState.SENSING)
        else:  # if both positions are not yet calibrated
            self.update_state(SensorState.UNCALIBRATED)

    def update_state(self, new_state: SensorState):
        """
        Helper function that sets the sensor's state attribute to the provided
        value and also publishing it on the relevant topic.

        Args:
            new_state (SensorState): The new state of the sensor.
        """
        self.state = new_state
        state_msg = String()
        state_msg.data = self.state.value
        self.state_publisher.publish(state_msg)
        self.log.info("Sensor node state set to %s.", self.state.value)

    def find_distance_range(self):
        """
        Hlper function that takes measurements and provides a list range of
        acceptable measurements to represent the current button state.
        """
        # create the dataset of measurements
        dataset = [1, 1, 1, 1, 1]  # temp fix
        for _ in range(100):
            dataset.append(self.sensor.proximity)
            time.sleep(1 / self.sensor.proximity_rate)
            continue

        # find the average and the standard deviation
        mean_measurement = statistics.mean(dataset)
        acceptable_deviation = statistics.stdev(dataset)

        # return a list of acceptable value range
        return [
            mean_measurement - acceptable_deviation,
            mean_measurement + acceptable_deviation,
        ]

    def on_reset(self, reset_msg):
        """
        Callback function to return the sensor to an uncalibrated (and
        inactive) state, and forget all history of button ranges. Allows the
        sensor to be recalibrated without destroying the node and editing the
        parameter server elsewhere.
        Args:
            reset_msg (Bool): Boolean message indicating whether the sensor
                calibration should be reset or not.
        """
        if reset_msg.data is True:
            rospy.delete_param("button_up_range")
            rospy.delete_param("button_down_range")
            self.button_up_range = None
            self.button_down_range = None
            self.set_state_by_calibration()
            self.log.info(
                "Sensor node was decalibrated. Please recalibrate before continuing."
            )

    def on_calibrate(self, string_msg):
        """
        Callback function to calibrate the sensor to detect the provided button
        position.

        Args:
            string_msg (String): A message containing the current button
                position, either "up" or "down"
        """
        # current button position
        button_pos = string_msg.data

        # if allowed to make a measurement
        if self.state == SensorState.UNCALIBRATED:
            # set sensor state and retrieve range
            self.update_state(SensorState.CALIBRATING)
            acceptable_range = self.find_distance_range()

            # update state attributes, also in param server
            if button_pos == "up":
                self.button_up_range = acceptable_range
                rospy.set_param("button_up_range", acceptable_range)
                self.log.info("Button up position registered.")
            elif button_pos == "down":
                self.button_down_range = acceptable_range
                rospy.set_param("button_down_range", acceptable_range)
                self.log.info("Button down position registered.")
            else:
                self.log.warning(
                    "Message did not specify button position. Data thrown out."
                )

            # update sensor state
            self.set_state_by_calibration()

        # do not calibrate if in any other state
        elif self.state == SensorState.CALIBRATING:
            self.log.warning("Cannot interrupt an active calibration!")
        elif self.state == SensorState.SENSING:
            self.log.warning("Cannot recalibrate while the sensor node is active!")

    def on_sensor_state(self, state_msg):
        """
        Callback function to react to state changes for this sensor.

        Args:
            state_msg (String): A message containing the current sensor state.
        """
        # get most recent state
        state = state_msg.data

        # if timer should be running and it isn't
        if state == "sensing" and self.acquisition_timer is None:
            self.log.info("Sensor node active. Running acquisition timer.")
            self.acquisition_timer = rospy.Timer(rospy.Duration(10), self.on_aq_timer)
        # if timer shouldn't be running but it is
        elif state != "sensing" and self.acquisition_timer is not None:
            self.log.info("Sensor node deactivated. Stopping acquisition timer.")
            self.acquisition_timer.shutdown()
            self.acquisition_timer = None

    def on_aq_timer(self, _):
        """
        Callback function for when the data acquisition timer goes off.
        """
        # make a measurement
        measurements = [1, 1, 1]
        for _ in range(10):
            measurements.append(self.sensor.proximity)
            time.sleep(1 / self.sensor.proximity_rate)
            continue
        proximity_estimate = statistics.mean(measurements)

        # create a message
        button_msg = String()

        # if estimating a nondepressed button
        if self.button_up_range[0] < proximity_estimate < self.button_up_range[1]:
            button_msg.data = ButtonState.UP.value
            self.log.info("Detected the button in nondepressed position.")
            # publish the state
            self.button_publisher.publish(button_msg)
        # if estimating a depressed button
        elif self.button_down_range[0] < proximity_estimate < self.button_down_range[1]:
            button_msg.data = ButtonState.DOWN.value
            self.log.info("Detected the button in depressed position.")
            # publish the state
            self.button_publisher.publish(button_msg)


if __name__ == "__main__":
    try:
        prox_sensor_node = ProximitySensorNode()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("Proximity sensor node shutdown")

    except rospy.ROSInterruptException:
        rospy.logwarn("Proximity sensor node shutdown (interrupt)")
