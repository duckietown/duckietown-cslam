#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import cPickle as pickle
import numpy as np


from deviceSideProcessor import deviceSideProcessor
from interprocess_communication import socketClient

def TF(inint):
    if inint==0:
        return False
    else:
        return True

ACQ_UPDATE_RATE = int(os.getenv('ACQ_UPDATE_RATE', 2)) #Hz
ACQ_SOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
ACQ_SOCKET_PORT = int(os.getenv('ACQ_SOCKET_PORT', 65432))
ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
ACQ_TOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', "camera_node/image/raw")
ACQ_TOPIC_CAMERAINFO = os.getenv('ACQ_TOPIC_CAMERAINFO', "camera_node/camera_info")
ACQ_TOPIC_VELOCITY_TO_POSE = os.getenv('ACQ_TOPIC_VELOCITY_TO_POSE', "velocity_to_pose_node/pose")
ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
ACQ_BEAUTIFY = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
ACQ_TAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))

class acquisitionProcessor():
    def __init__(self):
        rospy.init_node('acquisition_processor', anonymous=True)
        self.subscriberRawImage = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_RAW, Image,
                                                    lambda data: self.callback("lastRawImage", data),  queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                    lambda data: self.callback("lastCameraInfo", data),  queue_size = 1)

        if ACQ_TOPIC_VELOCITY_TO_POSE: #Only if set (probably not for watchtowers)
            self.subscriberCameraInfo = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
                                                        self.odometry_callback,  queue_size = 1)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(ACQ_UPDATE_RATE)

        self.lastRawImage = None
        self.lastCameraInfo = None

        self.velocityToPose = None
        self.previousVelocityToPose = None

        self.socketClient = socketClient(host=ACQ_SOCKET_HOST, port=ACQ_SOCKET_PORT)

    def update(self):
        while not rospy.is_shutdown():
            if self.lastRawImage is not None and self.lastCameraInfo is not None:
                # Collect latest ros_data
                currRawImage = self.lastRawImage
                currCameraInfo = self.lastCameraInfo

                cv_image = self.bridge.imgmsg_to_cv2(currRawImage, desired_encoding="rgb8")

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

                        outputDict["test_stream_image"] = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
                        outputDict["test_stream_image"].header.stamp.secs = currRawImage.header.stamp.secs
                        outputDict["test_stream_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                        outputDict["test_stream_image"].header.frame_id = ACQ_DEVICE_NAME

                        outputDict["raw_image"] = self.lastRawImage
                        outputDict["raw_camera_info"] = self.lastCameraInfo

                        outputDict["rectified_image"] = self.bridge.cv2_to_imgmsg(outputDict["rect_image"], encoding="rgb8")
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

                # Submit the data to the server register
                if outputDict:
                    self.socketClient.submitData(pickle.dumps(outputDict, protocol=-1))
                # print("Image processed and sent to server side to submit via ROS.")
            self.rate.sleep()

    def odometry_callback(self, ros_data):

        odometry = TransformStamped()
        odometry.header.seq = 0
        odometry.header = ros_data.header
        odometry.header.frame_id = ACQ_DEVICE_NAME
        odometry.child_frame_id = ACQ_DEVICE_NAME
        odometry.transform.translation.x = 0
        odometry.transform.translation.y = 0
        odometry.transform.translation.z = 0
        odometry.transform.rotation.x = 0
        odometry.transform.rotation.y = 0
        odometry.transform.rotation.z = 0
        odometry.transform.rotation.w = 1

        self.socketClient.submitData(pickle.dumps({"odometry": odometry}, protocol=-1))


    def callback(self, property, ros_data):
        if property=="lastRawImage":
            self.lastRawImage = ros_data
        if property=="lastCameraInfo":
            self.lastCameraInfo = ros_data
        # print("Registered %s", property)

if __name__ == '__main__':

    print("STARTS")

    # if not ACQ_UPDATE_RATE:
    #     raise Exception("ACQ_UPDATE_RATE not set")
    # if not ACQ_SOCKET_HOST:
    #     raise Exception("ACQ_SOCKET_HOST not set")
    # if not ACQ_SOCKET_PORT:
    #     raise Exception("ACQ_SOCKET_PORT not set")
    # if not ACQ_DEVICE_NAME:
    #     raise Exception("ACQ_DEVICE_NAME not set")
    # if not ACQ_TOPIC_RAW:
    #     raise Exception("ACQ_TOPIC_RAW not set")
    # if not ACQ_TOPIC_CAMERAINFO:
    #     raise Exception("ACQ_TOPIC_CAMERAINFO not set")
    # if not ACQ_TEST_STREAM:
    #     raise Exception("ACQ_TEST_STREAM not set")
    # if not ACQ_BEAUTIFY:
    #     raise Exception("ACQ_BEAUTIFY not set")
    # if not ACQ_TAG_SIZE:
    #     raise Exception("ACQ_TAG_SIZE not set")

    try:
        ap = acquisitionProcessor()
        ap.update()
    except KeyboardInterrupt:
        raise( Exception("Exiting") )
