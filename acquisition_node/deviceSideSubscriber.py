#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import cPickle as pickle
import numpy as np


from deviceSideProcessor import deviceSideProcessor
from interprocess_communication import socketClient

ACQ_UPDATE_RATE = int(os.getenv('ACQ_UPDATE_RATE', 2)) #Hz
ACQ_SOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
ACQ_SOCKET_PORT = int(os.getenv('ACQ_SOCKET_PORT', 65432))
ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
ACQ_TOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', "camera_node/image/raw")
ACQ_TOPIC_CAMERAINFO = os.getenv('ACQ_TOPIC_CAMERAINFO', "camera_node/camera_info")
ACQ_TEST_STREAM = bool(os.getenv('ACQ_TEST_STREAM', True))
ACQ_BEAUTIFY = bool(os.getenv('ACQ_BEAUTIFY', True))
ACQ_TAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))

class acquisitionProcessor():
    def __init__(self):
        rospy.init_node('acquisition_processor', anonymous=True)
        self.subscriberRawImage = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_RAW, Image,
                                                    lambda data: self.callback("lastRawImage", data),  queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber("/"+ACQ_DEVICE_NAME+"/"+ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                    lambda data: self.callback("lastCameraInfo", data),  queue_size = 1)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(ACQ_UPDATE_RATE)

        self.lastRawImage = None
        self.lastCameraInfo = None

        self.socketClient = socketClient(host=ACQ_SOCKET_HOST, port=ACQ_SOCKET_PORT)

    def update(self):
        while not rospy.is_shutdown():
            if self.lastRawImage is not None and self.lastCameraInfo is not None:
                # Collect latest ros_data
                currRawImage = self.lastRawImage
                currCameraInfo = self.lastCameraInfo

                cv_image = self.bridge.imgmsg_to_cv2(currRawImage, desired_encoding="rgb8")

                # Process the data and extract the apriltags
                dsp_options={"beautify": ACQ_BEAUTIFY, "tag_size": ACQ_TAG_SIZE}
                dsp = deviceSideProcessor(dsp_options)
                output = dsp.process(cv_image,  np.array(currCameraInfo.K).reshape((3,3)), currCameraInfo.D)

                # Generate a diagnostic image
                if ACQ_TEST_STREAM:
                    try:
                        image = output[0]['rect_image']
                        for tag in output:
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
                        pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="rgb8"))
                    except:
                        print("Image not generated, probably no tags were detected.")
                        pass

                # Add the time stamp and source of the input image to the output
                for idx in range(len(output)):
                    output[idx]["timestamp_secs"] = currRawImage.header.stamp.secs
                    output[idx]["timestamp_nsecs"] = currRawImage.header.stamp.nsecs
                    output[idx]["source"] = ACQ_DEVICE_NAME
                    if image is not None:
                        output[idx]["overlayed_image"] = image
                # Submit the data to the server register
                if len(output)>0:
                    self.socketClient.submitData(pickle.dumps(output, protocol=-1))
                # print("Image processed and sent to server side to submit via ROS.")
            self.rate.sleep()

    def callback(self, property, ros_data):
        if property=="lastRawImage":
            self.lastRawImage = ros_data
        if property=="lastCameraInfo":
            self.lastCameraInfo = ros_data
        # print("Registered %s", property)

if __name__ == '__main__':

    print("STARTS")

    if not ACQ_UPDATE_RATE:
        raise Exception("ACQ_UPDATE_RATE not set")
    if not ACQ_SOCKET_HOST:
        raise Exception("ACQ_SOCKET_HOST not set")
    if not ACQ_SOCKET_PORT:
        raise Exception("ACQ_SOCKET_PORT not set")
    if not ACQ_DEVICE_NAME:
        raise Exception("ACQ_DEVICE_NAME not set")
    if not ACQ_TOPIC_RAW:
        raise Exception("ACQ_TOPIC_RAW not set")
    if not ACQ_TOPIC_CAMERAINFO:
        raise Exception("ACQ_TOPIC_CAMERAINFO not set")
    if not ACQ_TEST_STREAM:
        raise Exception("ACQ_TEST_STREAM not set")
    if not ACQ_BEAUTIFY:
        raise Exception("ACQ_BEAUTIFY not set")
    if not ACQ_TAG_SIZE:
        raise Exception("ACQ_TAG_SIZE not set")

    if ACQ_TEST_STREAM: pub = rospy.Publisher("/"+ACQ_DEVICE_NAME+"/acquisionTestStream", Image, queue_size=1)

    try:
        ap = acquisitionProcessor()
        ap.update()
    except KeyboardInterrupt:
        raise( Exception("Exiting") )
