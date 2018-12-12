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

from image_rectifier import ImageRectifier
from argparse import ArgumentParser

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB', '/media/sf_Duckietown/SLAM/WIP/apriltag/python')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO', '/media/sf_Duckietown/SLAM/WIP/apriltag/build/lib')

sys.path.append(ACQ_APRILTAG_LIB)
aprilTagSooPath = [ACQ_APRILTAG_SO, ACQ_APRILTAG_SO]
import apriltag

class acquisitionProcessor():
    def __init__(self, outputDictQueue):

        # Get the environment variables
        self.ACQPOSES_UPDATE_RATE = float(os.getenv('ACQ_POSES_UPDATE_RATE', 10)) #Hz
        self.ACQODOMETRY_UPDATE_RATE = float(os.getenv('ACQ_ODOMETRY_UPDATE_RATE', 30)) #Hz
        self.ACQSTATIONARY_ODOMETRY = bool(int(os.getenv('ACQ_STATIONARY_ODOMETRY', 0)))
        self.ACQSOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
        self.ACQSOCKET_PORT = int(os.getenv('ACQ_SOCKET_PORT', 65432))
        self.ACQDEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
        self.ACQTOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', "camera_node/image/raw")
        self.ACQTOPIC_CAMERAINFO = os.getenv('ACQ_TOPIC_CAMERAINFO', "camera_node/camera_info")
        self.ACQTOPIC_VELOCITY_TO_POSE = os.getenv('ACQ_TOPIC_VELOCITY_TO_POSE', None)
        self.ACQTEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
        self.ACQBEAUTIFY = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
        self.ACQTAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))

        rospy.init_node('acquisition_processor', anonymous=True, disable_signals=True)
        self.subscriberRawImage = rospy.Subscriber("/"+self.ACQDEVICE_NAME+"/"+self.ACQTOPIC_RAW, CompressedImage,
                                                    self.camera_image_callback,  queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber("/"+self.ACQDEVICE_NAME+"/"+self.ACQTOPIC_CAMERAINFO, CameraInfo,
                                                    self.camera_info_callback,  queue_size = 1)

        if self.ACQTOPIC_VELOCITY_TO_POSE and self.ACQODOMETRY_UPDATE_RATE>0: #Only if set (probably not for watchtowers)
            self.subscriberCameraInfo = rospy.Subscriber("/"+self.ACQDEVICE_NAME+"/"+self.ACQTOPIC_VELOCITY_TO_POSE, Pose2DStamped,
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
            if self.ACQSTATIONARY_ODOMETRY and self.ACQTOPIC_VELOCITY_TO_POSE and self.ACQODOMETRY_UPDATE_RATE>0 and rospy.get_time() - self.timeLastPub_odometry >= 1.0/self.ACQODOMETRY_UPDATE_RATE:
                odometry = TransformStamped()
                odometry.header.seq = 0
                odometry.header.stamp = rospy.get_rostime()
                odometry.header.frame_id = self.ACQDEVICE_NAME
                odometry.child_frame_id = self.ACQDEVICE_NAME
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

            if rospy.get_time() - self.timeLastPub_poses >= 1.0/self.ACQPOSES_UPDATE_RATE and not self.lastImageProcessed:
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
            odometry.header.frame_id = self.ACQDEVICE_NAME
            odometry.child_frame_id = self.ACQDEVICE_NAME

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
            dsp_options={"beautify": self.ACQBEAUTIFY, "tag_size": self.ACQTAG_SIZE}
            dsp = deviceSideProcessor(dsp_options)
            outputDict = dsp.process(cv_image,  np.array(currCameraInfo.K).reshape((3,3)), currCameraInfo.D)

            # Add the time stamp and source of the input image to the output
            for idx in range(len(outputDict["apriltags"])):
                outputDict["apriltags"][idx]["timestamp_secs"] = currRawImage.header.stamp.secs
                outputDict["apriltags"][idx]["timestamp_nsecs"] = currRawImage.header.stamp.nsecs
                outputDict["apriltags"][idx]["source"] = self.ACQDEVICE_NAME

            # Generate a diagnostic image
            if self.ACQTEST_STREAM==1:
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

                    cv2.putText(image,'device: '+ self.ACQDEVICE_NAME +', timestamp: '+str(currRawImage.header.stamp.secs)+"+"+str(currRawImage.header.stamp.nsecs),
                                org=(30,30),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.6,
                                color=(0, 255, 0))

                    outputDict["test_stream_image"] = self.bridge.cv2_to_compressed_imgmsg(image, dst_format='png')
                    outputDict["test_stream_image"].header.stamp.secs = currRawImage.header.stamp.secs
                    outputDict["test_stream_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                    outputDict["test_stream_image"].header.frame_id = self.ACQDEVICE_NAME

                    outputDict["raw_image"] = self.lastCameraImage
                    outputDict["raw_camera_info"] = self.lastCameraInfo

                    outputDict["rectified_image"] = self.bridge.cv2_to_compressed_imgmsg(outputDict["rect_image"], dst_format='png')
                    outputDict["rectified_image"].header.stamp.secs = currRawImage.header.stamp.secs
                    outputDict["rectified_image"].header.stamp.nsecs = currRawImage.header.stamp.nsecs
                    outputDict["rectified_image"].header.frame_id = self.ACQDEVICE_NAME

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


class aprilTagProcessor():
    def __init__(self, tag_size):

        # The python bindings for the April Tag library require this strange configuration:
        parser = ArgumentParser(description='test apriltag Python bindings')
        parser.families='tag36h11'
        parser.border=1 # how wide (in bit-sizes) is the black border? (usually 1)
        parser.nthreads=4 #(int) number of threads to use
        parser.quad_decimate=1.0 #(float) Decimate input image by this factor
        parser.quad_blur=0.0 #(float) Apply low-pass blur to input; negative sharpens
        parser.refine_edges=True #(bool) Set to True to spend more time to align edges of tags
        parser.refine_decode=True #(bool) Set to True to spend more time to decode tags
        parser.refine_pose=True #(bool) Set to True to spend more time to precisely localize tags
        parser.debug=False #(bool)Enable/Disable debugging output (slow when enabled)
        parser.quad_contours=True
        apriltag.add_arguments(parser)

        self.options = parser.parse_args()
        self.options.tag_size = tag_size

    def configureDetector(self, cameraMatrix):

        ### configures the dector
        ### returns from detector
        ##tag_family (string)
        #Family that the tag belongs to
        ##tag_id (int)
        #The decoded ID of the tag
        ##hamming (int)
        #How many error bits were corrected? Note: accepting large numbers of corrected errors leads to greatly increased false positive rates.
        ##goodness (float)
        #A measure of the quality of tag localization: measures the average contrast of the pixels around the border of the tag. refine_pose must be enabled, or this field will be zero.
        ##decision_margin(float)
        #A measure of the quality of the binary decoding process: the average difference between the intensity of a data bit versus the decision threshold. Higher numbers roughly indicate better decodes. This is a reasonable measure of detection accuracy only for very small tags-- not effective for larger tags (where we could have sampled anywhere within a bit cell and still gotten a good detection.)
        ##homography (3x3 array)
        #The 3x3 homography matrix describing the projection from an "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1, 1)) to pixels in the image.
        ##center (1x2 array)
        #The center of the detection in image pixel coordinates.
        ##corners (4x2 array)
        #The corners of the tag in image pixel coordinates. These always wrap counter-clock wise around the tag. (top left is origin, x axis is horizontal, y axis is vertical)

        camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

        self.options.camera_params = camera_params
        self.detector = apriltag.Detector(self.options, searchpath=aprilTagSooPath)
        return self.detector

    def resultsToPose(self, results):
        for i, detection in enumerate(results):
            pose, e0, e1 = self.detector.detection_pose(detection,
                                                   self.options.camera_params,
                                                   self.options.tag_size)

            qvec = self.mat2quat(pose[:3, :3])
            tvec = pose[:3, 3]
            results[i].qvec = qvec
            results[i].tvec = tvec

        return results

    def mat2quat(self, M):
        Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
        K = np.array([[Qxx - Qyy - Qzz, 0, 0, 0],
                      [Qyx + Qxy, Qyy - Qxx - Qzz, 0, 0],
                      [Qzx + Qxz, Qzy + Qyz, Qzz - Qxx - Qyy, 0],
                      [Qyz - Qzy, Qzx - Qxz, Qxy - Qyx, Qxx + Qyy + Qzz]]) / 3.0
        # Use Hermitian eigenvectors, values for speed
        vals, vecs = np.linalg.eigh(K)
        # Select largest eigenvector, reorder to x,y,z,w quaternion
        q = vecs[[0, 1, 2, 3], np.argmax(vals)]
        # Prefer quaternion with positive w
        # (q * -1 corresponds to same rotation as q)
        if q[3] < 0:
            q *= -1
        return q



class deviceSideProcessor():
    def __init__(self, options):
        self.ImageRectifier = None
        self.opt_beautify = options.get("beautify", False)
        self.aprilTagProcessor = aprilTagProcessor(options.get("tag_size", 0.065))

    def process(self, raw_image, cameraMatrix, distCoeffs):

        # 0. Beautify if wanted:
        if self.opt_beautify:
            raw_image = self.ImageRectifier.beautify(raw_image)

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
