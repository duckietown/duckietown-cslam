#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import sys
import cPickle as pickle
import numpy as np

import math

from image_rectifier import ImageRectifier

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO')
sys.path.append(ACQ_APRILTAG_LIB)
import apriltags3

class acquisitionProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """
    def __init__(self, outputDictQueue, logger):

        try:
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

            # Initialize ROS nodes and subscribe to topics
            rospy.init_node('acquisition_processor', anonymous=True, disable_signals=True)
            self.subscriberRawImage = rospy.Subscriber("/"+self.ACQ_DEVICE_NAME+"/"+self.ACQ_TOPIC_RAW, CompressedImage,
                                                        self.camera_image_callback,  queue_size = 1)
            self.subscriberCameraInfo = rospy.Subscriber("/"+self.ACQ_DEVICE_NAME+"/"+self.ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                        self.camera_info_callback,  queue_size = 1)

            if self.ACQ_TOPIC_VELOCITY_TO_POSE and self.ACQ_ODOMETRY_UPDATE_RATE>0: #Only if set (probably not for watchtowers)
                self.subscriberCameraInfo = rospy.Subscriber("/"+self.ACQ_DEVICE_NAME+"/"+self.ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
                                                            self.odometry_callback,  queue_size = 1)

            # CvBridge is neccessary for the image processing
            self.bridge = CvBridge()

            # Initialize atributes
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
            self.logger = logger

        except Exception as e:
            self.logger.warning("acquisitionProcessor initialization failed: : %s" % str(e))
            raise Exception("acquisitionProcessor initialization failed: : %s" % str(e))

    def update(self, quitEvent):
        """
        Runs constantly and processes new data as it comes.
        """

        try:
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

                # Check if the last data was not yet processed and if it's time to process it (in order to sustain the deisred update rate)
                if rospy.get_time() - self.timeLastPub_poses >= 1.0/self.ACQ_POSES_UPDATE_RATE and not self.lastImageProcessed:
                    self.timeLastPub_poses = rospy.get_time()
                    outputDict = self.camera_image_process()
                    if outputDict is not None:
                        self.outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                                 block=True,
                                                 timeout=None)
                        self.lastImageProcessed = True

        except Exception as e:
            self.logger.warning("acquisitionProcessor update failed: : %s" % str(e))
            raise Exception("acquisitionProcessor update failed: : %s" % str(e))


    def odometry_callback(self, ros_data):
        """
        Callback function that is executed upon reception of new odometry data.
        """
        try:
            # Prepare the new ROS message for the processed odometry
            odometry = TransformStamped()
            odometry.header.seq = 0
            odometry.header = ros_data.header
            odometry.header.frame_id = self.ACQ_DEVICE_NAME
            odometry.child_frame_id = self.ACQ_DEVICE_NAME

            # Transform the incoming data to quaternions
            transform_current = np.array([[math.cos(ros_data.theta), -1.0 * math.sin(ros_data.theta), ros_data.x],
                                          [math.sin(ros_data.theta), math.cos(ros_data.theta), ros_data.y],
                                          [0.0, 0.0, 1.0]])

            transform_previous = np.array([[math.cos(self.previousOdometry.theta), -1.0 * math.sin(self.previousOdometry.theta), self.previousOdometry.x],
                                               [math.sin(self.previousOdometry.theta), math.cos(self.previousOdometry.theta), self.previousOdometry.y],
                                               [0.0, 0.0, 1.0]])

            transform_previous_inv = np.linalg.inv(transform_previous)

            self.previousOdometry = ros_data

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


            # Save the resuts to the new odometry relative pose message
            odometry.transform.translation.x = x
            odometry.transform.translation.y = y
            odometry.transform.translation.z = 0
            odometry.transform.rotation.x = q_x
            odometry.transform.rotation.y = q_y
            odometry.transform.rotation.z = q_z
            odometry.transform.rotation.w = q_w

            # Add the new data to the queue so that the server side publisher can publish them on the other ROS Master
            self.outputDictQueue.put(obj=pickle.dumps({"odometry": odometry}, protocol=-1),
                                     block=True,
                                     timeout=None)

            # Update the last odometry time
            self.timeLastPub_odometry = rospy.get_time()

            self.logger.warning('ODO CALLBACK')

        except Exception as e:
            self.logger.warning("acquisitionProcessor odometry_callback failed: : %s" % str(e))
            pass

    def camera_image_process(self):
        """
        Contains the camera image processing necessary.
        """

        outputDict = None
        # If there is new data to process
        if self.lastCameraInfo is not None and self.lastCameraImage is not None:
            # Collect latest ros_data
            currRawImage = self.lastCameraImage
            currCameraInfo = self.lastCameraInfo

            # Convert from ROS image message to numpy array
            cv_image = self.bridge.compressed_imgmsg_to_cv2(currRawImage, desired_encoding="mono8")

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
            dsp_options={"beautify": self.ACQ_BEAUTIFY, "tag_size": self.ACQ_TAG_SIZE}
            dsp = deviceSideProcessor(dsp_options)
            outputDict = dsp.process(cv_image,  (np.array(currCameraInfo.K)*scale_matrix).reshape((3,3)), currCameraInfo.D)

            # Add the time stamp and source of the input image to the output
            for idx in range(len(outputDict["apriltags"])):
                outputDict["apriltags"][idx]["timestamp_secs"] = currRawImage.header.stamp.secs
                outputDict["apriltags"][idx]["timestamp_nsecs"] = currRawImage.header.stamp.nsecs
                outputDict["apriltags"][idx]["source"] = self.ACQ_DEVICE_NAME

            # Generate a diagnostic image
            if self.ACQ_TEST_STREAM==1:
                try:
                    image = np.copy(outputDict['rect_image'])

                    # Put the AprilTag bound boxes and numbers to the image
                    for tag in outputDict["apriltags"]:
                        for idx in range(len(tag["corners"])):
                            cv2.line(image, tuple(tag["corners"][idx-1, :].astype(int)), tuple(tag["corners"][idx, :].astype(int)), (0, 255, 0))
                            # cv2.rectangle(image, (tag["corners"][0, 0].astype(int)-10,tag["corners"][0, 1].astype(int)-10), (tag["corners"][0, 0].astype(int)+15,tag["corners"][0, 1].astype(int)+15), (0, 0, 255), cv2.FILLED)
                        cv2.putText(image,str(tag["tag_id"]),
                                    org=(tag["corners"][0, 0].astype(int)+10,tag["corners"][0, 1].astype(int)+10),
                                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                    fontScale=0.4,
                                    color=(255, 0, 0))

                    # Put device and timestamp to the image
                    cv2.putText(image,'device: '+ self.ACQ_DEVICE_NAME +', timestamp: '+str(currRawImage.header.stamp.secs)+"+"+str(currRawImage.header.stamp.nsecs),
                                org=(30,30),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.6,
                                color=(0, 255, 0))

                    # Add the original and diagnostic info to the outputDict
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

                    #THIS DOESN'T WORK YET
                    # print(outputDict["newCameraMatrix"])
                    # outputDict["rectified_camera_info"] = self.lastCameraInfo
                    # outputDict["rectified_camera_info"].K = list(np.array(outputDict["newCameraMatrix"].flatten(order="C")))


                except Exception as e:
                    self.logger.warning("acquisitionProcessor odometry_callback failed: : %s" % str(e))
                    pass
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
        self.lastCameraImage = ros_data
        self.lastImageProcessed = False


class deviceSideProcessor():
    """
    Packages the image rectification and AprilTag detection for images.
    """
    def __init__(self, options):
        self.ImageRectifier = None
        self.opt_beautify = options.get("beautify", False)
        self.tag_size = options.get("tag_size", 0.065)
        self.aprilTagProcessor = apriltags3.Detector(searchpath=[ACQ_APRILTAG_SO],
                               families='tag36h11',
                               nthreads=14,
                               quad_decimate=1.0,
                               quad_sigma=0.0,
                               refine_edges=1,
                               decode_sharpening=0.25,
                               debug=0)

    def process(self, raw_image, cameraMatrix, distCoeffs):
        """
        Processes an image.
        """

        try:
            # 0. Initialize the image rectifier if it hasn't been already (that's done so that we don't recompute the remapping)
            if self.ImageRectifier is None:
                self.ImageRectifier = ImageRectifier(raw_image, cameraMatrix, distCoeffs)

            # 1. Rectify the raw image and get the new camera cameraMatrix
            rect_image, newCameraMatrix = self.ImageRectifier.rectify(raw_image)

            # 2. Extract april tags data
            if len(rect_image.shape) == 3:
                rect_image_gray = cv2.cvtColor(rect_image, cv2.COLOR_BGR2GRAY)
            else:
                rect_image_gray = rect_image

            # Beautify if wanted:
            if self.opt_beautify and self.ImageRectifier:
                raw_image = self.ImageRectifier.beautify(raw_image)

            # 3. Extract poses from april tags data
            camera_params = ( newCameraMatrix[0,0], newCameraMatrix[1,1], newCameraMatrix[0,2], newCameraMatrix[1,2] )
            tags = self.aprilTagProcessor.detect(rect_image_gray, True, camera_params, self.tag_size)

            # 4. Package output
            outputDict = dict()
            outputDict["rect_image"] = rect_image
            outputDict["new_camera_matrix"] = newCameraMatrix
            outputDict["apriltags"] = list()
            for atag in tags:
                outputDict["apriltags"].append({'tag_id': atag.tag_id,
                                                'corners': atag.corners,
                                                'qvec': self.mat2quat(atag.pose_R),
                                                'tvec': atag.pose_t })
            return outputDict

        except Exception as e:
            self.logger.warning("deviceSideProcessor process failed: : %s" % str(e))
            pass

    def mat2quat(self, M):
        """
        Helper function that converts rotation matrices to quaternions.
        """
        try:
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
        except:
            self.logger.warning("deviceSideProcessor process failed: : %s" % str(e))
            pass
