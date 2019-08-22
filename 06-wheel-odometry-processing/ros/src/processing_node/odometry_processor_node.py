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
from multiprocessing import Process


class OdometryProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger, config):

        self.config = config
        self.logger = logger

        # Get the environment variables
        self.ACQ_DEVICE_NAME = self.config['ACQ_DEVICE_NAME']
        self.ACQ_TOPIC_WHEEL_COMMAND = self.config["ACQ_TOPIC_WHEEL_COMMAND"]
        self.ACQ_ODOMETRY_TOPIC = self.config['ACQ_ODOMETRY_TOPIC']
        self.INPUT_BAG_PATH = self.config["INPUT_BAG_PATH"]
        self.OUTPUT_BAG_PATH = self.config["OUTPUT_BAG_PATH"]

        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('acquisition_processor',
                        anonymous=True, disable_signals=True)
        self.last_call_back_time = rospy.get_time()

        self.subscriber = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_WHEEL_COMMAND, WheelsCmdStamped,
                                           self.wheels_cmd_callback,  queue_size=150)

        # Only if set (probably not for watchtowers)
        # self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
        #                                              self.odometry_callback,  queue_size=1)

        self.wheel_radius = 0.065
        self.duckiebot_width = 0.10
        self.speed_factor = 0.65  # full speed is one, that is 0.65m/s
        self.turn_factor = 0.5
        self.linear_velocity_factor = self.speed_factor / 2.0
        self.angular_velocity_factor = self.turn_factor * \
            self.speed_factor / self.duckiebot_width

        self.last_message_time = -1.0

        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        self.odometry_processed = False

        self.last_header = None
        self.odometry_topic = str(
            "/poses_acquisition/"+self.ACQ_ODOMETRY_TOPIC)

        self.odometry_msg = TransformStamped()
        self.odometry_msg.header.frame_id = self.ACQ_DEVICE_NAME
        self.odometry_msg.child_frame_id = self.ACQ_DEVICE_NAME
        self.odometry_msg.transform.translation.z = 0.0
        self.odometry_msg.transform.rotation.x = 0.0
        self.odometry_msg.transform.rotation.y = 0.0
        if self.OUTPUT_BAG_PATH is not None:
            self.odometry_publisher = rospy.Publisher(
                self.odometry_topic, TransformStamped, queue_size=1)

        if self.OUTPUT_BAG_PATH is not None:
            self.outputbag = rosbag.Bag(self.OUTPUT_BAG_PATH, 'w')

        if self.INPUT_BAG_PATH is not None:
            self.bag_reader = Process(
                target=self.read_bag)
            self.bag_reader.start()

        self.heartbeat = rospy.Timer(rospy.Duration(1.0),
                                     self.heartbeat_callback)
        self.logger.info('Acquisition processor is set up.')

    def heartbeat_callback(self, timerevent):
        if not self.bag_reader.is_alive():
            rospy.signal_shutdown("exiting from heartbeat")

    def read_bag(self):
        bag = rosbag.Bag(self.INPUT_BAG_PATH + ".bag", 'r')
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        self.duration = end_time - start_time
        rospy.sleep(4)
        os.system("rosbag play %s.bag -r 3" % self.INPUT_BAG_PATH)
        rospy.sleep(self.duration * 0.5)
        while rospy.get_time() - self.last_call_back_time < 5.0:
            rospy.sleep(1.0)
        self.outputbag.close()
        rospy.signal_shutdown("the bag is finished")

    def wheels_cmd_callback(self, wheels_msg):
        """
        Callback function that is executed upon reception of a new wheel command.
        """
        self.last_call_back_time = rospy.get_time()
        # self.logger.info("got wheel message")

        current_time = float(wheels_msg.header.stamp.secs) + \
            10**(-9) * float(wheels_msg.header.stamp.nsecs)
        if self.last_message_time != -1.0:
            time_diff = current_time - self.last_message_time
            self.compute_and_publish_odometry(time_diff)

        self.last_header = wheels_msg.header

        self.last_linear_velocity = self.linear_velocity_factor * \
            (wheels_msg.vel_left + wheels_msg.vel_right)
        self.last_angular_velocity = self.angular_velocity_factor * \
            (wheels_msg.vel_right - wheels_msg.vel_left)

        self.last_message_time = current_time
        # self.logger.info("processed wheel command")

    def compute_and_publish_odometry(self, time_diff):
        self.odometry_msg.header = self.last_header

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

        # easy form of quartenion in the 2D plane
        q_w = math.cos(theta * 0.5)
        # q_x = 0.0
        # q_y = 0.0
        q_z = math.sin(theta * 0.5)

        # Save the resuts to the new odometry relative pose message
        self.odometry_msg.transform.translation.x = x
        self.odometry_msg.transform.translation.y = y
        # self.odometry_msg.transform.translation.z = 0
        # self.odometry_msg.transform.rotation.x = q_x
        # self.odometry_msg.transform.rotation.y = q_y
        self.odometry_msg.transform.rotation.z = q_z
        self.odometry_msg.transform.rotation.w = q_w
        # self.logger.info("publishing odometry")
        if self.OUTPUT_BAG_PATH is None:
            self.odometry_publisher.publish(self.odometry_msg)
        else:
            self.outputbag.write(self.odometry_topic, self.odometry_msg)


def get_environment_variables():
    config = dict()
    config["ACQ_DEVICE_NAME"] = os.getenv('ACQ_DEVICE_NAME', 'autobot01')
    config["ACQ_TOPIC_WHEEL_COMMAND"] = os.getenv(
        'ACQ_TOPIC_WHEEL_COMMAND', "wheels_driver_node/wheels_cmd")
    config["ACQ_ODOMETRY_TOPIC"] = os.getenv(
        'ACQ_ODOMETRY_TOPIC', 'odometry')
    config['INPUT_BAG_PATH'] = os.getenv('INPUT_BAG_PATH')
    config['OUTPUT_BAG_PATH'] = os.getenv('OUTPUT_BAG_PATH')
    return config


def main():
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    logger.info('Odometry processor started')
    config = get_environment_variables()
    odometryprocessor = OdometryProcessor(logger, config)
    rospy.spin()


if __name__ == "__main__":
    main()
