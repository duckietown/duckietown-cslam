#!/usr/bin/env python

import logging
import math
import multiprocessing
import os
import pprint
import copy
import subprocess
import sys
import time
from multiprocessing import Manager

import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import (AprilTagExtended, Pose2DStamped,
                                 WheelsCmdStamped)
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image,
                             RegionOfInterest)
from std_msgs.msg import Header

import apriltags3
import cv2
from image_rectifier import ImageRectifier

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO')
sys.path.append(ACQ_APRILTAG_LIB)


class PostProcessor():
    def __init__(self, logger, general_config, manager):
        self.logger = logger
        self.manager = manager
        self.general_config = general_config
        self.ACQ_TOPIC_WHEEL_COMMAND = self.general_config["ACQ_TOPIC_WHEEL_COMMAND"]
        self.ACQ_ODOMETRY_TOPIC = self.general_config['ACQ_ODOMETRY_TOPIC']
        self.INPUT_BAG_PATH = self.general_config['INPUT_BAG_PATH']
        self.OUTPUT_BAG_PATH = self.general_config['OUTPUT_BAG_PATH']
        self.ACQ_TOPIC_RAW = self.general_config['ACQ_TOPIC_RAW']
        self.ACQ_TOPIC_CAMERAINFO = self.general_config['ACQ_TOPIC_CAMERAINFO']
        self.ACQ_TEST_STREAM = self.general_config['ACQ_TEST_STREAM']
        self.ACQ_BEAUTIFY = self.general_config['ACQ_BEAUTIFY']
        self.ACQ_TAG_SIZE = self.general_config['ACQ_TAG_SIZE']
        self.ACQ_POSES_TOPIC = self.general_config['ACQ_POSES_TOPIC']
        self.aprilTagProcessor = apriltags3.Detector(searchpath=[ACQ_APRILTAG_SO],
                                                     families='tag36h11',
                                                     nthreads=4,
                                                     quad_decimate=1.0,
                                                     quad_sigma=0.0,
                                                     refine_edges=1,
                                                     decode_sharpening=0.25,
                                                     debug=0)
        self.apriltag_pose_topic = str(
            "/poses_acquisition/" + self.ACQ_POSES_TOPIC)
        self.odometry_topic = str(
            "/poses_acquisition/" + self.ACQ_ODOMETRY_TOPIC)
        self.list_of_agent_apriltag = []
        self.list_of_agent_odometry = []

        self.sub_config = {}
        self.sub_config["ACQ_TOPIC_RAW"] = self.general_config['ACQ_TOPIC_RAW'] + "/remapped"
        self.sub_config["ACQ_TOPIC_CAMERAINFO"] = self.general_config['ACQ_TOPIC_CAMERAINFO'] + "/remapped"
        self.sub_config["ACQ_TOPIC_WHEEL_COMMAND"] = self.general_config['ACQ_TOPIC_WHEEL_COMMAND'] + "/remapped"
        self.sub_config["ACQ_TEST_STREAM"] = self.general_config['ACQ_TEST_STREAM']
        self.sub_config["ACQ_BEAUTIFY"] = self.general_config['ACQ_BEAUTIFY']
        self.sub_config["ACQ_TAG_SIZE"] = self.general_config['ACQ_TAG_SIZE']
        self.sub_config["INPUT_BAG_PATH"] = None
        self.sub_config["OUTPUT_BAG_PATH"] = self.general_config['OUTPUT_BAG_PATH']
        self.sub_config["ACQ_POSES_TOPIC"] = self.general_config['ACQ_POSES_TOPIC']
        self.sub_config["ACQ_ODOMETRY_TOPIC"] = self.general_config['ACQ_ODOMETRY_TOPIC']

        self.set_list_of_agent()
        # self.logger.info(self.list_of_agent_apriltag)
        self.node_list = []
        pass

    def set_list_of_agent(self):
        bag = rosbag.Bag(self.INPUT_BAG_PATH + ".bag", 'r')
        for topic, _, _ in bag.read_messages():
            topic_parts = topic.split("/")
            for part in topic_parts:
                if "watchtower" in part:
                    if part not in self.list_of_agent_apriltag:
                        self.list_of_agent_apriltag.append(part)
                if "autobot" in part or "duckiebot" in part:
                    if part not in self.list_of_agent_apriltag:
                        self.list_of_agent_apriltag.append(part)
                        self.list_of_agent_odometry.append(part)

    def launch_all(self):
        number_of_agent = len(self.list_of_agent_apriltag)
        number_of_processes = max(2, int(8.0/number_of_agent))
        topic_remapping = {}
        for agent in self.list_of_agent_apriltag:
            config = self.sub_config
            config["ACQ_DEVICE_NAME"] = agent
            camera_topic = "%s/%s" % (agent,
                                      self.general_config["ACQ_TOPIC_RAW"])
            camera_remap = "/%s/%s" % (agent, self.sub_config["ACQ_TOPIC_RAW"])
            camera_info_topic = "%s/%s" % (agent,
                                           self.general_config["ACQ_TOPIC_CAMERAINFO"])
            camera_info_remap = "/%s/%s" % (agent,
                                            self.sub_config["ACQ_TOPIC_CAMERAINFO"])
            topic_remapping[camera_topic] = camera_remap
            topic_remapping[camera_info_topic] = camera_info_remap

            apriltag_processor_node = ApriltagProcessorNode(
                self.logger, config, number_of_processes, self.aprilTagProcessor, self.manager)
            self.node_list.append(apriltag_processor_node)

        for agent in self.list_of_agent_odometry:
            config = self.sub_config
            config["ACQ_DEVICE_NAME"] = agent
            odometry_topic = "%s/%s" % (agent,
                                        self.general_config["ACQ_TOPIC_WHEEL_COMMAND"])
            odometry_remap = "/%s/%s" % (agent,
                                         self.sub_config["ACQ_TOPIC_WHEEL_COMMAND"])

            topic_remapping[odometry_topic] = odometry_remap

            odometry_processor_node = OdometryProcessor(self.logger, config)
            self.node_list.append(odometry_processor_node)

        cmd = "rosbag play %s.bag" % self.INPUT_BAG_PATH
        for topic, remap in topic_remapping.iteritems():
            cmd += " %s:=%s" % (topic, remap)
        try:
            subprocess.check_output(cmd, shell=True)
        except subprocess.CalledProcessError:
            return "Error"

        for node in self.node_list:
            node.signal_end()

        return 0

    def get_data_and_create_bag(self):
        if os.path.isfile(self.OUTPUT_BAG_PATH):
            outputbag = rosbag.Bag(self.OUTPUT_BAG_PATH, 'a')
        else:
            outputbag = rosbag.Bag(self.OUTPUT_BAG_PATH, 'w')

        for node in self.node_list:
            data = node.get_data()
            for message_dict in data:
                if message_dict["topic"] == "apriltags":
                    outputbag.write(self.apriltag_pose_topic,
                                    message_dict["message"], message_dict["message"].header.stamp)
                if message_dict["topic"] == "odometry":
                    outputbag.write(self.odometry_topic,
                                    message_dict["message"], message_dict["message"].header.stamp)

        outputbag.close()

        del self.aprilTagProcessor

        while len(self.node_list) != 0:
            node = self.node_list[0]
            self.node_list.remove(node)
            self.logger.info("removing nodes!")
        return True


class ApriltagProcessorNode():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger, config, number_of_processors, aprilTagProcessor, manager):
        self.number_of_processors = number_of_processors
        self.config = config
        self.logger = logger
        self.manager = manager
        self.ACQ_DEVICE_NAME = self.config['ACQ_DEVICE_NAME']
        self.ACQ_TOPIC_RAW = self.config['ACQ_TOPIC_RAW']
        self.ACQ_TOPIC_CAMERAINFO = self.config['ACQ_TOPIC_CAMERAINFO']
        self.ACQ_TEST_STREAM = self.config['ACQ_TEST_STREAM']
        self.ACQ_BEAUTIFY = self.config['ACQ_BEAUTIFY']
        self.ACQ_TAG_SIZE = self.config['ACQ_TAG_SIZE']
        self.ACQ_POSES_TOPIC = self.config['ACQ_POSES_TOPIC']
        self.INPUT_BAG_PATH = self.config['INPUT_BAG_PATH']
        self.OUTPUT_BAG_PATH = self.config['OUTPUT_BAG_PATH']
        self.aprilTagProcessor = aprilTagProcessor
        # Initialize ROS nodes and subscribe to topics

        self.publish_queue = self.manager.Queue()
        self.image_queue = self.manager.Queue()
        self.publishers = {}
        self.apriltag_pose_topic = str(
            "/poses_acquisition/" + self.ACQ_POSES_TOPIC)
        self.publishers["apriltags"] = rospy.Publisher(
            self.apriltag_pose_topic, AprilTagExtended, queue_size=20)

        # Initialize attributes
        self.lastCameraInfo = None

        # self.logger.info('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW)

        self.seq_stamper = 0
        self.image_processor_list = []

        self.death_signal = (42, 42, 42)

        # Initialize the device side processor
        self.imageprocessor_options = {'beautify': self.ACQ_BEAUTIFY,
                                       'tag_size': self.ACQ_TAG_SIZE}
        # rospy.on_shutdown(self.on_shutdown)
        for i in range(self.number_of_processors):
            new_image_processor = ImageProcessor(
                self.imageprocessor_options, self.logger, self.publishers, self.aprilTagProcessor, self.publish_queue, self.config, self.image_queue, self.death_signal)
            self.image_processor_list.append(
                new_image_processor)
            new_image_processor.start()

        self.camera_topic = str(
            '/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW)
        self.camera_info_topic = str(
            '/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_CAMERAINFO)

        # if self.INPUT_BAG_PATH is None:
        self.subscriberImage = rospy.Subscriber(self.camera_topic, CompressedImage,
                                                self.camera_image_callback,  queue_size=100)
        self.subscriberCameraInfo = rospy.Subscriber(self.camera_info_topic, CameraInfo,
                                                     self.camera_info_callback,  queue_size=100)
        self.last_call_back_time = rospy.get_time()

        self.end_reached = False

        self.logger.info('Apriltag processor node for %s is set up.' %
                         self.ACQ_DEVICE_NAME)

    def are_all_processors_finished(self):
        for process in self.image_processor_list:
            process.join(timeout=0)
            if process.is_alive():
                return False
        return True

    def get_data(self):
        while not self.end_reached:
            rospy.sleep(0.1)
        data = []
        for process in self.image_processor_list:
            process.join()
        while not self.publish_queue.empty():
            try:
                message_dict = self.publish_queue.get(block=True, timeout=5.0)
                data.append(message_dict)
            except:
                pass
        return data

    def signal_end(self):
        self.logger.info("giving death signal")

        while rospy.get_time() - self.last_call_back_time < 2.0:
            rospy.sleep(1)

        for process in self.image_processor_list:
            self.image_queue.put(self.death_signal)

        self.end_reached = True

    def camera_info_callback(self, ros_data):
        """
        Callback function that is executed upon reception of new camera info data.
        """
        self.last_call_back_time = rospy.get_time()

        self.lastCameraInfo = ros_data

    def camera_image_callback(self, ros_data):
        """
        Callback function that is executed upon reception of a new camera image.
        """
        self.last_call_back_time = rospy.get_time()
        # print(type(ros_data))
        # print(type(self.lastCameraInfo))
        # print(type(self.seq_stamper))
        # self.logger.info("Got image")
        if self.lastCameraInfo is not None:
            # Collect latest ros_data
            self.image_queue.put(
                (ros_data, self.lastCameraInfo, self.seq_stamper), block=True)
            self.seq_stamper += 1

            # self.logger.warning(str(len(multiprocessing.active_children())))
        else:
            self.logger.warning("No camera info")


class ImageProcessor(multiprocessing.Process):
    """
    Packages the image rectification and AprilTag detection for images.
    """

    def __init__(self,  options, logger, publishers, aprilTagProcessor, publish_queue, config, image_queue, death_signal):
        super(ImageProcessor, self).__init__()
        self.logger = logger
        self.ImageRectifier = None
        self.publish_queue = publish_queue
        self.image_queue = image_queue
        self.bridge = CvBridge()
        self.aprilTagProcessor = aprilTagProcessor
        self.publishers = publishers
        self.opt_beautify = options.get('beautify', False)
        self.tag_size = options.get('tag_size', 0.065)
        self.death_signal = death_signal
        self.ACQ_DEVICE_NAME = config['ACQ_DEVICE_NAME']
        self.ACQ_TEST_STREAM = config['ACQ_TEST_STREAM']

        self.raw_image = None
        self.camera_info = None
        self.seq_stamper = None

    def run(self):
        while True:

            (self.raw_image, self.camera_info,
                self.seq_stamper) = self.image_queue.get(block=True)
            if (self.raw_image, self.camera_info,
                    self.seq_stamper) == self.death_signal:
                break
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                self.raw_image, desired_encoding='mono8')

            # Scale the K matrix if the image resolution is not the same as in the calibration
            currRawImage_height = cv_image.shape[0]
            currRawImage_width = cv_image.shape[1]

            scale_matrix = np.ones(9)
            if self.camera_info.height != currRawImage_height or self.camera_info.width != currRawImage_width:
                scale_width = float(currRawImage_width) / \
                    self.camera_info.width
                scale_height = float(currRawImage_height) / \
                    self.camera_info.height

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
                newApriltagDetectionMsg = AprilTagExtended()
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

                self.publish_queue.put(
                    {"topic": "apriltags", "message": newApriltagDetectionMsg})
                # self.publishers["apriltags"].publish(newApriltagDetectionMsg)
                # self.logger.info("Published pose for tag %d in sequence %d" % (
                #     tag["tag_id"], self.seq_stamper))

        # Publish the test and raw data if submitted and requested:
        if self.ACQ_TEST_STREAM:
            if "test_stream_image" in incomingData:
                imgMsg = incomingData["test_stream_image"]
                imgMsg.header.seq = self.seq_stamper
                self.publish_queue.put(
                    {"topic": "test_stream_image", "message": imgMsg})
                # self.publishers["test_stream_image"].publish(imgMsg)

            if "raw_image" in incomingData:
                imgMsg = incomingData["raw_image"]
                imgMsg.header.seq = self.seq_stamper
                self.publish_queue.put(
                    {"topic": "raw_image", "message": imgMsg})
                # self.publishers["raw_image"].publish(imgMsg)

            if "rectified_image" in incomingData:
                imgMsg = incomingData["rectified_image"]
                imgMsg.header.seq = self.seq_stamper
                self.publish_queue.put(
                    {"topic": "rectified_image", "message": imgMsg})
                # self.publishers["rectified_image"].publish(imgMsg)

            if "raw_camera_info" in incomingData:
                self.publish_queue.put(
                    {"topic": "raw_camera_info", "message": incomingData["raw_camera_info"]})

                # self.publishers["raw_camera_info"].publish(
                #     incomingData["raw_camera_info"])

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
                self.publish_queue.put(
                    {"topic": "new_camera_matrix", "message": new_camera_info})

                # self.publishers["new_camera_matrix"].publish(new_camera_info)


class OdometryProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger, config):

        self.config = config
        self.logger = logger

        self.odometry_list = []
        # Get the environment variables
        self.ACQ_DEVICE_NAME = self.config['ACQ_DEVICE_NAME']
        self.ACQ_TOPIC_WHEEL_COMMAND = self.config["ACQ_TOPIC_WHEEL_COMMAND"]
        self.ACQ_ODOMETRY_TOPIC = self.config['ACQ_ODOMETRY_TOPIC']

        self.last_call_back_time = rospy.get_time()

        self.subscriber = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_WHEEL_COMMAND, WheelsCmdStamped,
                                           self.wheels_cmd_callback,  queue_size=150)

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

        self.last_header_stamp = None
        self.odometry_topic = str(
            "/poses_acquisition/"+self.ACQ_ODOMETRY_TOPIC)

        self.end_reached = False
        self.odometry_msg = TransformStamped()
        self.odometry_msg.header.frame_id = self.ACQ_DEVICE_NAME
        self.odometry_msg.child_frame_id = self.ACQ_DEVICE_NAME
        self.odometry_msg.transform.translation.z = 0.0
        self.odometry_msg.transform.rotation.x = 0.0
        self.odometry_msg.transform.rotation.y = 0.0

        self.logger.info('Acquisition processor is set up.')

    def signal_end(self):
        self.logger.info("giving death signal")

        while rospy.get_time() - self.last_call_back_time < 2.0:
            rospy.sleep(1)

        self.end_reached = True

    def get_data(self):
        while not self.end_reached:
            rospy.sleep(0.1)
        return self.odometry_list

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
            if(time_diff > 0.0):
                self.compute_and_publish_odometry(time_diff)

        self.last_header_stamp = wheels_msg.header.stamp
        self.last_linear_velocity = self.linear_velocity_factor * \
            (wheels_msg.vel_left + wheels_msg.vel_right)
        self.last_angular_velocity = self.angular_velocity_factor * \
            (wheels_msg.vel_right - wheels_msg.vel_left)

        self.last_message_time = current_time
        # self.logger.info("processed wheel command")

    def compute_and_publish_odometry(self, time_diff):
        self.odometry_msg.header.stamp = self.last_header_stamp
        self.odometry_msg.header.frame_id = self.ACQ_DEVICE_NAME

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
        self.odometry_msg.transform.translation.y = 0.0
        # self.odometry_msg.transform.translation.z = 0
        # self.odometry_msg.transform.rotation.x = q_x
        # self.odometry_msg.transform.rotation.y = q_y
        self.odometry_msg.transform.rotation.z = q_z
        self.odometry_msg.transform.rotation.w = q_w
        # self.logger.info("publishing odometry")
        # if self.OUTPUT_BAG_PATH is None:
        #     self.odometry_publisher.publish(self.odometry_msg)
        odometry_msg = copy.deepcopy(self.odometry_msg)
        message_dict = {"topic": "odometry",
                        "message": odometry_msg}
        self.odometry_list.append(message_dict)
        # self.outputbag.write(self.odometry_topic, self.odometry_msg)


def get_environment_variables():
    config = dict()

    config['ACQ_DEVICE_NAME'] = os.getenv('ACQ_DEVICE_NAME', 'watchtower10')
    config['ACQ_TOPIC_RAW'] = os.getenv(
        'ACQ_TOPIC_RAW', 'camera_node/image/compressed')
    config['ACQ_TOPIC_CAMERAINFO'] = os.getenv(
        'ACQ_TOPIC_CAMERAINFO', 'camera_node/camera_info')
    config['ACQ_TEST_STREAM'] = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
    config['ACQ_BEAUTIFY'] = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
    config['ACQ_TAG_SIZE'] = float(os.getenv('ACQ_TAG_SIZE', 0.065))
    config['ACQ_POSES_TOPIC'] = os.getenv('ACQ_POSES_TOPIC', "poses")
    config["ACQ_TOPIC_WHEEL_COMMAND"] = os.getenv(
        'ACQ_TOPIC_WHEEL_COMMAND', "wheels_driver_node/wheels_cmd_decalibrated")
    config["ACQ_ODOMETRY_TOPIC"] = os.getenv(
        'ACQ_ODOMETRY_TOPIC', 'odometry')
    config['INPUT_BAG_PATH'] = os.getenv('INPUT_BAG_PATH')
    config['OUTPUT_BAG_PATH'] = os.getenv('OUTPUT_BAG_PATH')

    return config


def main():
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    logger.info('Post processor starting')
    rospy.init_node('post_processor_node',
                    anonymous=True, disable_signals=True)
    config = get_environment_variables()
    manager = Manager()
    post_processor = PostProcessor(logger, config, manager)

    post_processor.launch_all()
    post_processor.get_data_and_create_bag()
    rospy.sleep(1)
    del post_processor
    rospy.signal_shutdown("End of things")
    logger.info("should die now")
    exit(0)


if __name__ == "__main__":
    main()
    exit()
