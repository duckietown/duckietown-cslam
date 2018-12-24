#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import cPickle as pickle
import numpy as np

import math

from image_rectifier import ImageRectifier

from aprilTagProcessor import aprilTagProcessor

class acquisitionProcessor():
    def __init__(self, outputDictQueue):

        # Get the environment variables
        self.ACQ_POSES_UPDATE_RATE = float(os.getenv('ACQ_POSES_UPDATE_RATE', 10)) #Hz
        self.ACQ_ODOMETRY_UPDATE_RATE = float(os.getenv('ACQ_ODOMETRY_UPDATE_RATE', 30)) #Hz
        self.ACQ_STATIONARY_ODOMETRY = bool(int(os.getenv('ACQ_STATIONARY_ODOMETRY', 0)))
        self.ACQ_SOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
        self.ACQ_SOCKET_PORT = int(os.getenv('ACQ_SOCKET_PORT', 65432))
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
        self.ACQ_TOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', "camera_node/image/raw")
        self.ACQ_TOPIC_CAMERAINFO = os.getenv('ACQ_TOPIC_CAMERAINFO', "camera_node/camera_info")
        self.ACQ_TOPIC_VELOCITY_TO_POSE = os.getenv('ACQ_TOPIC_VELOCITY_TO_POSE', None)
        self.ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
        self.ACQ_BEAUTIFY = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
        self.ACQ_TAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))

        rospy.init_node('acquisition_processor', anonymous=True, disable_signals=True)
        self.subscriberRawImage = rospy.Subscriber("/"+self.ACQ_DEVICE_NAME+"/"+self.ACQ_TOPIC_RAW, CompressedImage,
                                                    self.camera_image_callback,  queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber("/"+self.ACQ_DEVICE_NAME+"/"+self.ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                    self.camera_info_callback,  queue_size = 1)

        if self.ACQ_TOPIC_VELOCITY_TO_POSE and self.ACQ_ODOMETRY_UPDATE_RATE>0: #Only if set (probably not for watchtowers)
            self.subscriberCameraInfo = rospy.Subscriber("/"+self.ACQ_DEVICE_NAME+"/"+self.ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
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

        self.outputDictQueue = outputDictQueue

    def update(self, quitEvent):
        while not quitEvent.is_set():
            #check if we want odometry and if it has been sent in the last X secs
            if self.ACQ_STATIONARY_ODOMETRY and self.ACQ_TOPIC_VELOCITY_TO_POSE and self.ACQ_ODOMETRY_UPDATE_RATE>0 and rospy.get_time() - self.timeLastPub_odometry >= 1.0/self.ACQ_ODOMETRY_UPDATE_RATE:
                odometry = TransformStamped()
                odometry.header.seq = 0
                odometry.header.stamp = rospy.get_rostime()
                odometry.header.frame_id = self.ACQ_DEVICE_NAME
                odometry.child_frame_id = self.ACQ_DEVICE_NAME
                odometry.transform.translation.x = 0
                odometry.transform.translation.y = 0
                odometry.transform.translation.z = 0
                odometry.transform.rotation.x = 0
                odometry.transform.rotation.y = 0
                odometry.transform.rotation.z = 0
                odometry.transform.rotation.w = 1

                self.timeLastPub_odometry = rospy.get_time()

                self.outputDictQueue.put(obj=pickle.dumps({"odometry": odometry}, protocol=-1),
                                         block=True,
                                         timeout=None)

            if rospy.get_time() - self.timeLastPub_poses >= 1.0/self.ACQ_POSES_UPDATE_RATE and not self.lastImageProcessed:
                self.timeLastPub_poses = rospy.get_time()
                outputDict = self.camera_image_process()
                if outputDict is not None:
                    self.outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                             block=True,
                                             timeout=None)
                    self.lastImageProcessed = True


    def odometry_callback(self, ros_data):
        try:
            odometry = TransformStamped()
            odometry.header.seq = 0
            odometry.header = ros_data.header
            odometry.header.frame_id = self.ACQ_DEVICE_NAME
            odometry.child_frame_id = self.ACQ_DEVICE_NAME

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

            self.outputDictQueue.put(obj=pickle.dumps({"odometry": odometry}, protocol=-1),
                                     block=True,
                                     timeout=None)

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
            dsp_options={"beautify": self.ACQ_BEAUTIFY, "tag_size": self.ACQ_TAG_SIZE}
            dsp = deviceSideProcessor(dsp_options)
            outputDict = dsp.process(cv_image,  np.array(currCameraInfo.K).reshape((3,3)), currCameraInfo.D)

            # Add the time stamp and source of the input image to the output
            for idx in range(len(outputDict["apriltags"])):
                outputDict["apriltags"][idx]["timestamp_secs"] = currRawImage.header.stamp.secs
                outputDict["apriltags"][idx]["timestamp_nsecs"] = currRawImage.header.stamp.nsecs
                outputDict["apriltags"][idx]["source"] = self.ACQ_DEVICE_NAME

            # Generate a diagnostic image
            if self.ACQ_TEST_STREAM==1:
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

                    cv2.putText(image,'device: '+ self.ACQ_DEVICE_NAME +', timestamp: '+str(currRawImage.header.stamp.secs)+"+"+str(currRawImage.header.stamp.nsecs),
                                org=(30,30),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.6,
                                color=(0, 255, 0))

                    outputDict["test_stream_image"] = self.bridge.cv2_to_compressed_imgmsg(image, dst_format='png')
                    outputDict["test_stream_image"].header.stamp.secs = currRawImage.header.stamp.secs
                    outputDict["test_stream_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                    outputDict["test_stream_image"].header.frame_id = self.ACQ_DEVICE_NAME

                    outputDict["raw_image"] = self.lastCameraImage
                    outputDict["raw_camera_info"] = self.lastCameraInfo

                    outputDict["rectified_image"] = self.bridge.cv2_to_compressed_imgmsg(outputDict["rect_image"], dst_format='png')
                    outputDict["rectified_image"].header.stamp.secs = currRawImage.header.stamp.secs
                    outputDict["rectified_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                    outputDict["rectified_image"].header.frame_id = self.ACQ_DEVICE_NAME

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


class deviceSideProcessor():
    def __init__(self, options):
        self.ImageRectifier = None
        self.opt_beautify = options.get("beautify", False)
        self.aprilTagProcessor = aprilTagProcessor(options.get("tag_size", 0.065))

    def process(self, raw_image, cameraMatrix, distCoeffs):



        # 0. Initialize the image rectifier if it hasn't been already (that's done so that we don't recompute the remapping)
        if self.ImageRectifier is None:
            self.ImageRectifier =  ImageRectifier(raw_image, cameraMatrix, distCoeffs)

        # 1. Rectify the raw image and get the new camera cameraMatrix
        rect_image, newCameraMatrix = self.ImageRectifier.rectify(raw_image)

        # 2. Extract april tags data
        if len(rect_image.shape) == 3:
            tect_image_gray = cv2.cvtColor(rect_image, cv2.COLOR_BGR2GRAY)
        else:
            tect_image_gray = rect_image

        # Beautify if wanted:
        if self.opt_beautify and self.ImageRectifier:
            raw_image = self.ImageRectifier.beautify(raw_image)

        aprilTags = self.aprilTagProcessor.configureDetector(newCameraMatrix).detect(tect_image_gray, return_image=False)

        # 3. Extract poses from april tags data
        aprilTags = self.aprilTagProcessor.resultsToPose(aprilTags)

        # 4. Package output
        outputDict = dict()
        outputDict["rect_image"] = rect_image
        outputDict["new_camera_matrix"] = newCameraMatrix
        outputDict["apriltags"] = list()
        for atag in aprilTags:
            outputDict["apriltags"].append({'tag_id': atag.tag_id,
                                            'goodness': atag.goodness,
                                            'corners': atag.corners,
                                            'qvec': atag.qvec,
                                            'tvec': atag.tvec })
        return outputDict
