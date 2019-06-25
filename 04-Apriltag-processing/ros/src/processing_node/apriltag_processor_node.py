#!/usr/bin/env python

import logging
import math
import multiprocessing
import os
import sys

import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image,
                             RegionOfInterest)
from std_msgs.msg import Header

import apriltags3
import cv2
from duckietown_msgs.msg import AprilTagDetection, Pose2DStamped
from image_rectifier import ImageRectifier

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO')
sys.path.append(ACQ_APRILTAG_LIB)


class ApriltagProcessorNode():
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
        self.ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
        self.ACQ_BEAUTIFY = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
        self.ACQ_TAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))
        self.ACQ_POSES_TOPIC = os.getenv('ACQ_POSES_TOPIC', "poses")

        self.ACQ_APRILTAG_QUAD_DECIMATE = float(
            os.getenv('ACQ_APRILTAG_QUAD_DECIMATE', 1.0))

        self.aprilTagProcessor = apriltags3.Detector(searchpath=[ACQ_APRILTAG_SO],
                                                     families='tag36h11',
                                                     nthreads=4,
                                                     quad_decimate=1.0,
                                                     quad_sigma=0.0,
                                                     refine_edges=1,
                                                     decode_sharpening=0.25,
                                                     debug=0)
        self.ACQ_POSES_UPDATE_RATE = float(
            os.getenv('ACQ_POSES_UPDATE_RATE', 10))  # Hz

        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('apriltag_processor_node',
                        anonymous=True, disable_signals=True)
        self.subscriberRawImage = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW, CompressedImage,
                                                   self.camera_image_callback,  queue_size=50)
        self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                     self.camera_info_callback,  queue_size=50)

        self.publishers = {}
        self.publishers["apriltags"] = rospy.Publisher(
            "/poses_acquisition/" + self.ACQ_POSES_TOPIC, AprilTagDetection, queue_size=20)

        # If the test stream is requested
        if self.ACQ_TEST_STREAM:
            self.publishers["test_stream_image"] = rospy.Publisher(
                "/poses_acquisition/test_video/"+self.ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
            self.publishers["raw_image"] = rospy.Publisher(
                "/poses_acquisition/raw_video/"+self.ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
            self.publishers["rectified_image"] = rospy.Publisher(
                "/poses_acquisition/rectified_video/"+self.ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
            self.publishers["raw_camera_info"] = rospy.Publisher(
                "/poses_acquisition/camera_info_raw/"+self.ACQ_DEVICE_NAME, CameraInfo, queue_size=1)
            self.publishers["new_camera_matrix"] = rospy.Publisher(
                "/poses_acquisition/camera_info_rectified/"+self.ACQ_DEVICE_NAME, CameraInfo, queue_size=1)

        # Initialize attributes
        self.lastCameraInfo = None

        self.logger = logger
        self.logger.info('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW)

        self.seq_stamper = 0
        self.image_processor_list = []

        # Initialize the device side processor
        self.imageprocessor_options = {'beautify': self.ACQ_BEAUTIFY,
                                       'tag_size': self.ACQ_TAG_SIZE}
        rospy.on_shutdown(self.on_shutdown)

        self.logger.info('Apriltag processor node is set up.')

    def camera_info_callback(self, ros_data):
        """
        Callback function that is executed upon reception of new camera info data.
        """
        self.lastCameraInfo = ros_data

    def camera_image_callback(self, ros_data):
        """
        Callback function that is executed upon reception of a new camera image.
        """
        self.logger.info("Got image")
        if self.lastCameraInfo is not None:
            # Collect latest ros_data
            new_image_processor = ImageProcessor(
                self.imageprocessor_options, self.logger, ros_data, self.lastCameraInfo, self.publishers, self.seq_stamper, self.aprilTagProcessor)
            self.seq_stamper += 1
            # self.image_processor_list.append(
            #     new_image_processor)
            new_image_processor.run()

        else:
            self.logger.warning("No camera info")

    def on_shutdown(self):
        self.logger.info("Waiting for all apriltag image processors to end")
        for process in self.image_processor_list:
            process.join()
        self.logger.info("apriltag processor node shutting down now")


class ImageProcessor():
    """
    Packages the image rectification and AprilTag detection for images.
    """

    def __init__(self, options, logger, raw_image, camera_info, publishers, seq_stamper, aprilTagProcessor):
        # super(ImageProcessor, self).__init__()
        self.logger = logger
        self.ImageRectifier = None
        self.bridge = CvBridge()
        self.aprilTagProcessor = aprilTagProcessor
        self.seq_stamper = seq_stamper
        self.publishers = publishers
        self.opt_beautify = options.get('beautify', False)
        self.tag_size = options.get('tag_size', 0.065)
        self.raw_image = raw_image
        self.camera_info = camera_info
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', 'watchtower10')
        self.ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))

    def run(self):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(
            self.raw_image, desired_encoding='mono8')

        # Scale the K matrix if the image resolution is not the same as in the calibration
        currRawImage_height = cv_image.shape[0]
        currRawImage_width = cv_image.shape[1]

        scale_matrix = np.ones(9)
        if self.camera_info.height != currRawImage_height or self.camera_info.width != currRawImage_width:
            scale_width = float(currRawImage_width) / self.camera_info.width
            scale_height = float(currRawImage_height) / self.camera_info.height

            scale_matrix[0] *= scale_width
            scale_matrix[2] *= scale_width
            scale_matrix[4] *= scale_height
            scale_matrix[5] *= scale_height

        outputDict = dict()

        # Process the image and extract the apriltags
        outputDict = self.process(cv_image,  (np.array(
            self.camera_info.K)*scale_matrix).reshape((3, 3)), self.camera_info.D)
        outputDict['header'] = self.raw_image.header

        # Add the time stamp and source of the input image to the output
        for idx in range(len(outputDict['apriltags'])):
            outputDict['apriltags'][idx]['timestamp_secs'] = self.raw_image.header.stamp.secs
            outputDict['apriltags'][idx]['timestamp_nsecs'] = self.raw_image.header.stamp.nsecs
            outputDict['apriltags'][idx]['source'] = self.ACQ_DEVICE_NAME

        # Generate a diagnostic image
        if self.ACQ_TEST_STREAM == 1:
            image = np.copy(outputDict['rect_image'])

            # Put the AprilTag bound boxes and numbers to the image
            for tag in outputDict['apriltags']:
                for idx in range(len(tag['corners'])):
                    cv2.line(image, tuple(tag['corners'][idx-1, :].astype(int)),
                             tuple(tag['corners'][idx, :].astype(int)), (0, 255, 0))
                    # cv2.rectangle(image, (tag['corners'][0, 0].astype(int)-10,tag['corners'][0, 1].astype(int)-10), (tag['corners'][0, 0].astype(int)+15,tag['corners'][0, 1].astype(int)+15), (0, 0, 255), cv2.FILLED)
                cv2.putText(image, str(tag['tag_id']),
                            org=(tag['corners'][0, 0].astype(int)+10,
                                 tag['corners'][0, 1].astype(int)+10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.0,
                            thickness=2,
                            color=(255, 0, 0))

            # Put device and timestamp to the image
            cv2.putText(image, 'device: ' + self.ACQ_DEVICE_NAME + ', timestamp: '+str(self.raw_image.header.stamp.secs)+'+'+str(self.raw_image.header.stamp.nsecs),
                        org=(30, 30),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1.0,
                        thickness=2,
                        color=(255, 0, 0))

            # Add the original and diagnostic info to the outputDict
            outputDict['test_stream_image'] = self.bridge.cv2_to_compressed_imgmsg(
                image, dst_format='png')
            outputDict['test_stream_image'].header.stamp.secs = self.raw_image.header.stamp.secs
            outputDict['test_stream_image'].header.stamp.nsecs = self.raw_image.header.stamp.nsecs
            outputDict['test_stream_image'].header.frame_id = self.ACQ_DEVICE_NAME

            outputDict['raw_image'] = self.raw_image
            outputDict['raw_camera_info'] = self.camera_info

            outputDict['rectified_image'] = self.bridge.cv2_to_compressed_imgmsg(
                outputDict['rect_image'], dst_format='png')
            outputDict['rectified_image'].header.stamp.secs = self.raw_image.header.stamp.secs
            outputDict['rectified_image'].header.stamp.nsecs = self.raw_image.header.stamp.nsecs
            outputDict['rectified_image'].header.frame_id = self.ACQ_DEVICE_NAME

        # PUBLISH HERE
        self.publish(outputDict)

    def process(self, raw_image, cameraMatrix, distCoeffs):
        """
        Processes an image.
        """

        try:
            # 0. Initialize the image rectifier if it hasn't been already (that's done so that we don't recompute the remapping)
            if self.ImageRectifier is None:
                self.ImageRectifier = ImageRectifier(
                    raw_image, cameraMatrix, distCoeffs)

            # 1. Rectify the raw image and get the new camera cameraMatrix
            rect_image, newCameraMatrix = self.ImageRectifier.rectify(
                raw_image)

            # 2. Extract april tags data
            if len(rect_image.shape) == 3:
                rect_image_gray = cv2.cvtColor(rect_image, cv2.COLOR_BGR2GRAY)
            else:
                rect_image_gray = rect_image

            # Beautify if wanted:
            if self.opt_beautify and self.ImageRectifier:
                raw_image = self.ImageRectifier.beautify(raw_image)

            # 3. Extract poses from april tags data
            camera_params = (
                newCameraMatrix[0, 0], newCameraMatrix[1, 1], newCameraMatrix[0, 2], newCameraMatrix[1, 2])
            tags = self.aprilTagProcessor.detect(
                rect_image_gray, True, camera_params, self.tag_size)

            # 4. Package output
            outputDict = dict()
            outputDict['rect_image'] = rect_image
            outputDict['new_camera_matrix'] = newCameraMatrix
            outputDict['apriltags'] = list()
            for atag in tags:
                outputDict['apriltags'].append({'tag_id': atag.tag_id,
                                                'corners': atag.corners,
                                                'qvec': self.mat2quat(atag.pose_R),
                                                'tvec': atag.pose_t,
                                                'tag_family': atag.tag_family,
                                                'hamming': atag.hamming,
                                                'decision_margin': atag.decision_margin,
                                                'homography': atag.homography,
                                                'center': atag.center,
                                                'pose_error': atag.pose_err})
            return outputDict

        except Exception as e:
            self.logger.warning(
                'ImageProcessor process failed: : %s' % str(e))
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
        except Exception as e:
            self.logger.warning(
                'ImageProcessor process failed: : %s' % str(e))
            pass

    def publish(self, incomingData):
        if "apriltags" in incomingData:
            for tag in incomingData["apriltags"]:
                # Publish the relative pose
                newApriltagDetectionMsg = AprilTagDetection()
                newApriltagDetectionMsg.header.stamp.secs = int(
                    tag["timestamp_secs"])
                newApriltagDetectionMsg.header.stamp.nsecs = int(
                    tag["timestamp_nsecs"])
                newApriltagDetectionMsg.header.frame_id = str(
                    tag["source"])
                newApriltagDetectionMsg.transform.translation.x = float(
                    tag["tvec"][0])
                newApriltagDetectionMsg.transform.translation.y = float(
                    tag["tvec"][1])
                newApriltagDetectionMsg.transform.translation.z = float(
                    tag["tvec"][2])
                newApriltagDetectionMsg.transform.rotation.x = float(
                    tag["qvec"][0])
                newApriltagDetectionMsg.transform.rotation.y = float(
                    tag["qvec"][1])
                newApriltagDetectionMsg.transform.rotation.z = float(
                    tag["qvec"][2])
                newApriltagDetectionMsg.transform.rotation.w = float(
                    tag["qvec"][3])
                newApriltagDetectionMsg.tag_id = int(tag["tag_id"])
                newApriltagDetectionMsg.tag_family = tag["tag_family"]
                newApriltagDetectionMsg.hamming = int(tag["hamming"])
                newApriltagDetectionMsg.decision_margin = float(
                    tag["decision_margin"])
                newApriltagDetectionMsg.homography = tag["homography"].flatten(
                )
                newApriltagDetectionMsg.center = tag["center"]
                newApriltagDetectionMsg.corners = tag["corners"].flatten()
                newApriltagDetectionMsg.pose_error = tag["pose_error"]

                self.publishers["apriltags"].publish(newApriltagDetectionMsg)
                self.logger.info("Published pose for tag %d in sequence %d" % (
                    tag["tag_id"], self.seq_stamper))

        # Publish the test and raw data if submitted and requested:
        if self.ACQ_TEST_STREAM:
            if "test_stream_image" in incomingData:
                imgMsg = incomingData["test_stream_image"]
                imgMsg.header.seq = self.seq_stamper
                self.publishers["test_stream_image"].publish(imgMsg)

            if "raw_image" in incomingData:
                imgMsg = incomingData["raw_image"]
                imgMsg.header.seq = self.seq_stamper
                self.publishers["raw_image"].publish(imgMsg)

            if "rectified_image" in incomingData:
                imgMsg = incomingData["rectified_image"]
                imgMsg.header.seq = self.seq_stamper
                self.publishers["rectified_image"].publish(imgMsg)

            if "raw_camera_info" in incomingData:
                self.publishers["raw_camera_info"].publish(
                    incomingData["raw_camera_info"])

            if "new_camera_matrix" in incomingData:
                new_camera_info = CameraInfo()
                try:
                    new_camera_info.header = incomingData["raw_camera_info"].header
                    new_camera_info.height = incomingData["raw_image"].shape[0]
                    new_camera_info.width = incomingData["raw_image"].shape[1]
                    new_camera_info.distortion_model = incomingData["raw_camera_info"].distortion_model
                    new_camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
                except:
                    pass
                new_camera_info.K = incomingData["new_camera_matrix"].flatten(
                )
                self.publishers["new_camera_matrix"].publish(new_camera_info)


def main():
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    logger.info('Device side processor starting in LIVE mode')
    ap = ApriltagProcessorNode(logger)
    rospy.spin()


if __name__ == "__main__":
    main()
