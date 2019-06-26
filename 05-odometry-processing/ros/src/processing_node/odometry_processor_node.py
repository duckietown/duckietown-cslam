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
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image,
                             RegionOfInterest)
from std_msgs.msg import Header

import cv2
from Common_Modules import *
from duckietown_msgs.msg import Pose2DStamped
from image_rectifier import ImageRectifier
from pathos.multiprocessing import ProcessingPool
from py_MVO import VisualOdometry


ACQ_VISUAL_ODOMETRY_LIB = os.getenv('ACQ_VISUAL_ODOMETRY_LIB')
sys.path.append(ACQ_VISUAL_ODOMETRY_LIB)


class OdometryProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger):

        # Get the environment variables
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', 'watchtower10')
        self.ACQ_TOPIC_RAW = os.getenv(
            'ACQ_TOPIC_RAW', 'camera_node/image/compressed')
        self.ACQ_TOPIC_CAMERAINFO = os.getenv(
            'ACQ_TOPIC_CAMERAINFO', 'camera_node/camera_info')
        self.ACQ_TOPIC_VELOCITY_TO_POSE = os.getenv(
            'ACQ_TOPIC_VELOCITY_TO_POSE', None)
        self.ACQ_STATIONARY_ODOMETRY = bool(
            int(os.getenv('ACQ_STATIONARY_ODOMETRY', 0)))
        self.ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES = os.getenv(
            'ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES', 'SURF')
        self.ACQ_ODOMETRY_TOPIC = os.getenv('ACQ_ODOMETRY_TOPIC', "odometry")
        self.logger = logger

        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('acquisition_processor',
                        anonymous=True, disable_signals=True)

        self.logger.info(str('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW))

        self.subscriberCompressedImage = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW, CompressedImage,
                                                          self.camera_image_callback,  queue_size=1)
        self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                     self.camera_info_callback,  queue_size=1)

        # Only if set (probably not for watchtowers)
        # self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
        #                                              self.odometry_callback,  queue_size=1)

        self.odometry_publisher = rospy.Publisher(
            "/poses_acquisition/"+self.ACQ_ODOMETRY_TOPIC, TransformStamped, queue_size=1)

        # CvBridge is necessary for the image processing
        self.bridge = CvBridge()

        # Initialize attributes
        self.lastCameraInfo = None

        self.ImageRectifier = None

        # Initialize the device side processor

        self.vo = None
        self.clahe = None
        self.logger.info('Acquisition processor is set up.')

    def visual_odometry(self, outputdict):
        if self.vo is None:
            K = outputdict['new_camera_matrix'].reshape((3, 3))

            # Setting up the visual odometry
            self.vo = VisualOdometry(
                K, self.ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES, pitch_adjust=np.deg2rad(10.0))
            self.clahe = cv2.createCLAHE(clipLimit=5.0)

        img = outputdict['rect_image']
        img = self.clahe.apply(img)

        img_id = 3
        self.logger.info("Inside visual odometry")

        if self.vo.update(img, img_id):
            self.logger.info("inside if update")

            # The scaling is rather arbitrary, it seems that VO gives centimeters, but in general the scaling is fishy...
            odometry = TransformStamped()
            odometry.header.seq = 0
            odometry.header = outputdict['header']
            odometry.header.frame_id = self.ACQ_DEVICE_NAME
            odometry.child_frame_id = self.ACQ_DEVICE_NAME

            cy = math.cos(self.vo.relative_pose_theta * 0.5)
            sy = math.sin(self.vo.relative_pose_theta * 0.5)
            cp = 1.0
            sp = 0.0
            cr = 1.0
            sr = 0.0

            q_w = cy * cp * cr + sy * sp * sr
            q_x = cy * cp * sr - sy * sp * cr
            q_y = sy * cp * sr + cy * sp * cr
            q_z = sy * cp * cr - cy * sp * sr

            # Save the resuts to the new odometry relative pose message
            odometry.transform.translation.x = self.vo.relative_pose_x
            odometry.transform.translation.y = self.vo.relative_pose_y
            odometry.transform.translation.z = 0
            odometry.transform.rotation.x = q_x
            odometry.transform.rotation.y = q_y
            odometry.transform.rotation.z = q_z
            odometry.transform.rotation.w = q_w
            self.logger.info("publishing odometry")

            self.odometry_publisher.publish(odometry)

    def camera_image_process(self, currRawImage, currCameraInfo):
        """
        Contains the necessary camera image processing.
        """

        # Convert from ROS image message to numpy array
        cv_image = self.bridge.compressed_imgmsg_to_cv2(
            currRawImage, desired_encoding='mono8')

        # Scale the K matrix if the image resolution is not the same as in the calibration
        currRawImage_height = cv_image.shape[0]
        currRawImage_width = cv_image.shape[1]

        scale_matrix = np.ones(9)
        if currCameraInfo.height != currRawImage_height or currCameraInfo.width != currRawImage_width:
            scale_width = float(currRawImage_width) / currCameraInfo.width
            scale_height = float(currRawImage_height) / currCameraInfo.height

            scale_matrix[0] *= scale_width
            scale_matrix[2] *= scale_width
            scale_matrix[4] *= scale_height
            scale_matrix[5] *= scale_height

        outputDict = dict()

        # Process the image and extract the apriltags

        cameraMatrix = (np.array(
            currCameraInfo.K)*scale_matrix).reshape((3, 3))

        if self.ImageRectifier is None:
            self.ImageRectifier = ImageRectifier(
                cv_image, cameraMatrix, currCameraInfo.D)

        # Rectify the raw image and get the new camera cameraMatrix
        rect_image, newCameraMatrix = self.ImageRectifier.rectify(
            cv_image)

        # Package output
        outputDict['rect_image'] = rect_image
        outputDict['new_camera_matrix'] = newCameraMatrix
        outputDict['header'] = currRawImage.header
        self.logger.info("end of camera process")

        return outputDict

    def camera_info_callback(self, ros_data):
        """
        Callback function that is executed upon reception of new camera info data.
        """
        self.lastCameraInfo = ros_data

    def camera_image_callback(self, ros_data):
        """
        Callback function that is executed upon reception of a new camera image.
        """
        self.logger.info("received image")
        if self.lastCameraInfo is not None:
            self.visual_odometry(self.camera_image_process(
                ros_data, self.lastCameraInfo))
            self.logger.info("processed image")


def main():
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    logger.info('Odometry processor started')
    odometryprocessor = OdometryProcessor(logger)
    rospy.spin()


if __name__ == "__main__":
    main()
