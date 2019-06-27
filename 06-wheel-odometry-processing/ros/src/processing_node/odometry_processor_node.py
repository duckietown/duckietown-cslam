#!/usr/bin/env python

import logging
import cPickle as pickle
import math
import os
import sys
import multiprocessing

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
from pathos.multiprocessing import ProcessingPool


class OdometryProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger):

        # Get the environment variables
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', 'watchtower10')
        self.logger = logger
        self.ACQ_TOPIC_WHEEL_COMMAND = os.getenv(
            "ACQ_TOPIC_WHEEL_COMMAND", "wheels_driver_node/wheels_cmd")
        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('acquisition_processor',
                        anonymous=True, disable_signals=True)
        self.ACQ_ODOMETRY_TOPIC = os.getenv('ACQ_ODOMETRY_TOPIC', "odometry")

        self.subscriber = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_WHEEL_COMMAND, WheelsCmdStamped,
                                           self.wheels_cmd_callback,  queue_size=1)

        # Only if set (probably not for watchtowers)
        # self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
        #                                              self.odometry_callback,  queue_size=1)

        self.last_message_time = -1.0

        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        self.odometry_processed = False

        self.last_header = None
        self.odometry_publisher = rospy.Publisher(
            "/poses_acquisition/"+self.ACQ_ODOMETRY_TOPIC, TransformStamped, queue_size=1)

        self.logger.info('Acquisition processor is set up.')

    def wheels_cmd_callback(self, wheels_msg):
        """
        Callback function that is executed upon reception of a new wheel command.
        """
        wheel_radius = 0.065
        duckiebot_width = 0.10
        speed_factor = 0.65  # full speed is one, that is 0.65m/s

        current_time = float(wheels_msg.header.stamp.secs) + \
            10**(-9) * float(wheels_msg.header.stamp.nsecs)
        if self.last_message_time != -1.0:
            time_diff = current_time - self.last_message_time
            self.compute_and_publish_odometry(time_diff)

        self.last_header = wheels_msg.header

        self.last_linear_velocity = speed_factor * \
            (wheels_msg.vel_left + wheels_msg.vel_right)/2.0
        self.last_angular_velocity = speed_factor * \
            (wheels_msg.vel_right - wheels_msg.vel_left)/(duckiebot_width)

        self.last_message_time = current_time
        self.logger.info("processed wheel command")

    def compute_and_publish_odometry(self, time_diff):
        odometry = TransformStamped()
        odometry.header.seq = 0
        odometry.header = self.last_header
        odometry.header.frame_id = self.ACQ_DEVICE_NAME
        odometry.child_frame_id = self.ACQ_DEVICE_NAME

        x = 0.0
        y = 0.0
        theta = 0.0

        if (self.last_angular_velocity == 0.0):
            x = time_diff * self.last_linear_velocity
            y = 0.0
            theta = 0.0
        else:
            radius = self.last_linear_velocity / self.last_angular_velocity
            theta = time_diff * self.last_angular_velocity
            x = radius * math.sin(theta)
            y = radius * (1 - math.cos(theta))

        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        cp = 1.0
        sp = 0.0
        cr = 1.0
        sr = 0.0

        q_w = cy * cp * cr + sy * sp * sr
        q_x = cy * cp * sr - sy * sp * cr
        q_y = sy * cp * sr + cy * sp * cr
        q_z = sy * cp * cr - cy * sp * sr

        # Save the resuts to the new odometry relative pose message
        odometry.transform.translation.x = x
        odometry.transform.translation.y = y
        odometry.transform.translation.z = 0
        odometry.transform.rotation.x = q_x
        odometry.transform.rotation.y = q_y
        odometry.transform.rotation.z = q_z
        odometry.transform.rotation.w = q_w
        self.logger.info("publishing odometry")

        self.odometry_publisher.publish(odometry)


def main():
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    logger.info('Odometry processor started')
    odometryprocessor = OdometryProcessor(logger)
    rospy.spin()


if __name__ == "__main__":
    main()
