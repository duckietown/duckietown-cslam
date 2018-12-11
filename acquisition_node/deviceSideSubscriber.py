#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import cPickle as pickle
import numpy as np

import math

from deviceSideProcessor import deviceSideProcessor
from interprocess_communication import socketClient

ACQ_POSES_UPDATE_RATE = float(os.getenv('ACQ_POSES_UPDATE_RATE', 10)) #Hz
ACQ_ODOMETRY_UPDATE_RATE = float(os.getenv('ACQ_ODOMETRY_UPDATE_RATE', 30)) #Hz
ACQ_STATIONARY_ODOMETRY = bool(int(os.getenv('ACQ_STATIONARY_ODOMETRY', 0)))
ACQ_SOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
ACQ_SOCKET_PORT = int(os.getenv('ACQ_SOCKET_PORT', 65432))
ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
ACQ_TOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', "camera_node/image/raw")
ACQ_TOPIC_CAMERAINFO = os.getenv('ACQ_TOPIC_CAMERAINFO', "camera_node/camera_info")
ACQ_TOPIC_VELOCITY_TO_POSE = os.getenv('ACQ_TOPIC_VELOCITY_TO_POSE', None)
ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
ACQ_BEAUTIFY = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
ACQ_TAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))

class acquisitionProcessor():
    def __init__(self):
        rospy.init_node('acquisition_processor', anonymous=True)
        self.subscriberRawImage = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_RAW, CompressedImage,
                                                    self.camera_image_callback,  queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                    self.camera_info_callback,  queue_size = 1)

        if ACQ_TOPIC_VELOCITY_TO_POSE and ACQ_ODOMETRY_UPDATE_RATE>0: #Only if set (probably not for watchtowers)
            self.subscriberCameraInfo = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
                                                        self.odometry_callback,  queue_size = 1)

        self.bridge = CvBridge()

        self.lastCameraInfo = None
        self.lastCameraImage = None
        self.lastImageProcessed = False

        self.previousOdometry = Pose2DStamped()
        self.previousOdometry.x = 0.0
        self.previousOdometry.y = 0.0
        self.previousOdometry.theta = 0.0

        self.timeLastPub_odometry = rospy.get_time()
        self.timeLastPub_poses = rospy.get_time()


        self.socketClient = socketClient(host=ACQ_SOCKET_HOST, port=ACQ_SOCKET_PORT)

    def update(self):
        while not rospy.is_shutdown():
            #check if we want odometry and if it has been sent in the last X secs
            if ACQ_STATIONARY_ODOMETRY and ACQ_TOPIC_VELOCITY_TO_POSE and ACQ_ODOMETRY_UPDATE_RATE>0 and rospy.get_time() - self.timeLastPub_odometry >= 1.0/ACQ_ODOMETRY_UPDATE_RATE:
                odometry = TransformStamped()
                odometry.header.seq = 0
                odometry.header.stamp = rospy.get_rostime()
                odometry.header.frame_id = ACQ_DEVICE_NAME
                odometry.child_frame_id = ACQ_DEVICE_NAME
                odometry.transform.translation.x = 0
                odometry.transform.translation.y = 0
                odometry.transform.translation.z = 0
                odometry.transform.rotation.x = 0
                odometry.transform.rotation.y = 0
                odometry.transform.rotation.z = 0
                odometry.transform.rotation.w = 1

                self.timeLastPub_odometry = rospy.get_time()

                self.socketClient.submitData(pickle.dumps({"odometry": odometry}, protocol=-1))

            if rospy.get_time() - self.timeLastPub_poses >= 1.0/ACQ_POSES_UPDATE_RATE and not self.lastImageProcessed:
                self.timeLastPub_poses = rospy.get_time()
                outputDict = self.camera_image_process()
                if outputDict is not None:
                    self.socketClient.submitData(pickle.dumps(outputDict, protocol=-1))
                    self.lastImageProcessed = True


    def odometry_callback(self, ros_data):
        try:
            odometry = TransformStamped()
            odometry.header.seq = 0
            odometry.header = ros_data.header
            odometry.header.frame_id = ACQ_DEVICE_NAME
            odometry.child_frame_id = ACQ_DEVICE_NAME

            transform_current = np.array([[math.cos(ros_data.theta), -1.0 * math.sin(ros_data.theta), ros_data.x],
                                          [math.sin(ros_data.theta), math.cos(ros_data.theta), ros_data.y],
                                          [0.0, 0.0, 1.0]])

            transform_previous = np.array([[math.cos(self.previousOdometry.theta), -1.0 * math.sin(self.previousOdometry.theta), self.previousOdometry.x],
                                               [math.sin(self.previousOdometry.theta), math.cos(self.previousOdometry.theta), self.previousOdometry.y],
                                               [0.0, 0.0, 1.0]])

            transform_previous_inv = np.linalg.inv(transform_previous)

            self.previousOdometry = ros_data

            # transform_relative = transform_previous_inv.dot(transform_current)
            transform_relative = np.matmul(transform_previous_inv, transform_current)

            angle = math.atan2(transform_relative[1][0], transform_relative[0][0])

            x = transform_relative[0][2]

            y = transform_relative[1][2]


            cy = math.cos(angle * 0.5)
            sy = math.sin(angle * 0.5)
            cp = 1.0
            sp = 0.0
            cr = 1.0
            sr = 0.0

            q_w = cy * cp * cr + sy * sp * sr
            q_x = cy * cp * sr - sy * sp * cr
            q_y = sy * cp * sr + cy * sp * cr
            q_z = sy * cp * cr - cy * sp * sr


            odometry.transform.translation.x = x
            odometry.transform.translation.y = y
            odometry.transform.translation.z = 0
            odometry.transform.rotation.x = q_x
            odometry.transform.rotation.y = q_y
            odometry.transform.rotation.z = q_z
            odometry.transform.rotation.w = q_w

            self.socketClient.submitData(pickle.dumps({"odometry": odometry}, protocol=-1))

            self.timeLastPub_odometry = rospy.get_time()

        except Exception as e:
            print Exception("Odometry data not generated, exception encountered: %s" % str(e))
            pass

    def camera_image_process(self):
        outputDict = None
        if self.lastCameraInfo is not None and self.lastCameraImage is not None:
            # Collect latest ros_data
            currRawImage = self.lastCameraImage
            currCameraInfo = self.lastCameraInfo

            cv_image = self.bridge.compressed_imgmsg_to_cv2(currRawImage, desired_encoding="mono8")

            outputDict = dict()

            # Process the image and extract the apriltags
            dsp_options={"beautify": ACQ_BEAUTIFY, "tag_size": ACQ_TAG_SIZE}
            dsp = deviceSideProcessor(dsp_options)
            outputDict = dsp.process(cv_image,  np.array(currCameraInfo.K).reshape((3,3)), currCameraInfo.D)

            # Add the time stamp and source of the input image to the output
            for idx in range(len(outputDict["apriltags"])):
                outputDict["apriltags"][idx]["timestamp_secs"] = currRawImage.header.stamp.secs
                outputDict["apriltags"][idx]["timestamp_nsecs"] = currRawImage.header.stamp.nsecs
                outputDict["apriltags"][idx]["source"] = ACQ_DEVICE_NAME

            # Generate a diagnostic image
            if ACQ_TEST_STREAM==1:
                try:
                    image = np.copy(outputDict['rect_image'])
                    for tag in outputDict["apriltags"]:
                        for idx in range(len(tag["corners"])):
                            cv2.line(image, tuple(tag["corners"][idx-1, :].astype(int)), tuple(tag["corners"][idx, :].astype(int)), (0, 255, 0))
                            # cv2.rectangle(image, (tag["corners"][0, 0].astype(int)-10,tag["corners"][0, 1].astype(int)-10), (tag["corners"][0, 0].astype(int)+15,tag["corners"][0, 1].astype(int)+15), (0, 0, 255), cv2.FILLED)
                            cv2.putText(image,str(tag["tag_id"]),
                                        org=(tag["corners"][0, 0].astype(int)+10,tag["corners"][0, 1].astype(int)+10),
                                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                        fontScale=0.4,
                                        color=(255, 0, 0))

                    cv2.putText(image,'device: '+ ACQ_DEVICE_NAME +', timestamp: '+str(currRawImage.header.stamp.secs)+"+"+str(currRawImage.header.stamp.nsecs),
                                org=(30,30),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.6,
                                color=(0, 255, 0))

                    outputDict["test_stream_image"] = self.bridge.cv2_to_imgmsg(image)#, dst_format='jpg')
                    outputDict["test_stream_image"].header.stamp.secs = currRawImage.header.stamp.secs
                    outputDict["test_stream_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                    outputDict["test_stream_image"].header.frame_id = ACQ_DEVICE_NAME

                    outputDict["raw_image"] = self.lastCameraImage
                    outputDict["raw_camera_info"] = self.lastCameraInfo

                    outputDict["rectified_image"] = self.bridge.cv2_to_imgmsg(outputDict["rect_image"])#, dst_format='jpg')
                    outputDict["rectified_image"].header.stamp.secs = currRawImage.header.stamp.secs
                    outputDict["rectified_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                    outputDict["rectified_image"].header.frame_id = ACQ_DEVICE_NAME

                    #THIS DOESN"T WORK YET
                    # print(outputDict["newCameraMatrix"])
                    # outputDict["rectified_camera_info"] = self.lastCameraInfo
                    # outputDict["rectified_camera_info"].K = list(np.array(outputDict["newCameraMatrix"].flatten(order="C")))

                except Exception as e:
                    print("Test data not generated, exception encountered: %s" % str(e))
                    pass

        return outputDict

    def camera_info_callback(self, ros_data):
        self.lastCameraInfo = ros_data

    def camera_image_callback(self, ros_data):
        self.lastCameraImage = ros_data
        self.lastImageProcessed = False


if __name__ == '__main__':

    print("STARTS")

    try:
        ap = acquisitionProcessor()
        ap.update()
    except KeyboardInterrupt:
        raise( Exception("Exiting") )
