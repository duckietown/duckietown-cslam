#!/usr/bin/env python3

from py_MVO import VisualOdometry
from Common_Modules import *
import apriltags3
import rospy
import rosbag
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import sys
import pickle as pickle
import numpy as np
from pathos.multiprocessing import ProcessingPool

import math

from image_rectifier import ImageRectifier

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO')
sys.path.append(ACQ_APRILTAG_LIB)

ACQ_VISUAL_ODOMETRY_LIB = os.getenv('ACQ_VISUAL_ODOMETRY_LIB')
sys.path.append(ACQ_VISUAL_ODOMETRY_LIB)

dsp = None


class acquisitionProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger, mode='live'):

        self.mode = mode
        if self.mode != 'live' and self.mode != 'postprocessing':
            raise Exception(
                "The argument mode should be 'live' or 'postprocessing'. Received %s instead." % self.mode)

        # Get the environment variables
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', 'watchtower10')
        self.ACQ_TOPIC_RAW = os.getenv(
            'ACQ_TOPIC_RAW', 'camera_node/image/compressed')
        self.ACQ_TOPIC_CAMERAINFO = os.getenv(
            'ACQ_TOPIC_CAMERAINFO', 'camera_node/camera_info')
        self.ACQ_TOPIC_VELOCITY_TO_POSE = os.getenv(
            'ACQ_TOPIC_VELOCITY_TO_POSE', None)
        self.ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
        self.ACQ_BEAUTIFY = bool(int(os.getenv('ACQ_BEAUTIFY', 1)))
        self.ACQ_TAG_SIZE = float(os.getenv('ACQ_TAG_SIZE', 0.065))
        self.ACQ_STATIONARY_ODOMETRY = bool(
            int(os.getenv('ACQ_STATIONARY_ODOMETRY', 0)))
        self.ACQ_APRILTAG_QUAD_DECIMATE = float(
            os.getenv('ACQ_APRILTAG_QUAD_DECIMATE', 1.0))
        self.ACQ_ODOMETRY_POST_VISUAL_ODOMETRY = bool(
            int(os.getenv('ACQ_ODOMETRY_POST_VISUAL_ODOMETRY', 0)))
        self.ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES = os.getenv(
            'ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES', 'SURF')

        if self.mode == 'live':
            self.ACQ_POSES_UPDATE_RATE = float(
                os.getenv('ACQ_POSES_UPDATE_RATE', 10))  # Hz
            self.ACQ_ODOMETRY_UPDATE_RATE = float(
                os.getenv('ACQ_ODOMETRY_UPDATE_RATE', 30))  # Hz
        elif self.mode == 'postprocessing':
            pass

        # Initialize ROS nodes and subscribe to topics
        if self.mode == 'live':
            rospy.init_node('acquisition_processor',
                            anonymous=True, disable_signals=True)
            self.subscriberRawImage = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW, CompressedImage,
                                                       self.camera_image_callback,  queue_size=1)
            self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_CAMERAINFO, CameraInfo,
                                                         self.camera_info_callback,  queue_size=1)

            # Only if set (probably not for watchtowers)
            if self.ACQ_TOPIC_VELOCITY_TO_POSE and self.ACQ_ODOMETRY_UPDATE_RATE > 0:
                self.subscriberCameraInfo = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_VELOCITY_TO_POSE, Pose2DStamped,
                                                             self.odometry_callback,  queue_size=1)
        elif self.mode == 'postprocessing':
            self.bag_topics = {'raw_image': '/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW,
                               'camera_info': '/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_CAMERAINFO}
            # Only if set (probably not for watchtowers)
            if self.ACQ_TOPIC_VELOCITY_TO_POSE:
                self.bag_topics['odometry'] = '/'+self.ACQ_DEVICE_NAME + \
                    '/'+self.ACQ_TOPIC_VELOCITY_TO_POSE

        # CvBridge is neccessary for the image processing
        self.bridge = CvBridge()

        # Initialize atributes
        self.lastCameraInfo = None
        self.lastCameraImage = None
        self.lastImageProcessed = True
        self.lastOdometry = None
        self.lastOdometryProcessed = True

        self.previousOdometry = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        if self.mode == 'live':
            self.timeLastPub_odometry = rospy.get_time()
            self.timeLastPub_poses = rospy.get_time()

        self.logger = logger

        # Initialize the device side processor
        self.dsp_options = {'beautify': self.ACQ_BEAUTIFY,
                            'tag_size': self.ACQ_TAG_SIZE}
        self.dsp = None

        self.logger.info('Acquisition processor is set up.')

    def liveUpdate(self, outputDictQueue, quitEvent):
        """
        Runs constantly and processes new data as it comes.
        """

        assert(self.mode == 'live')

        while not quitEvent.is_set():
            # check if we want odometry:
            if self.ACQ_TOPIC_VELOCITY_TO_POSE and self.ACQ_POSES_UPDATE_RATE > 0 and rospy.get_time() - self.timeLastPub_poses >= 1.0/self.ACQ_POSES_UPDATE_RATE:
                odometry = None
                # If we have a new actual message, use it
                if not self.lastOdometryProcessed:

                    odometry = self.odometry_process(
                        self.lastOdometry.x, self.lastOdometry.y, self.lastOdometry.theta, self.lastOdometry.header)
                    self.lastOdometryProcessed = True

                # Else send a dummy message if requested and if live (only if no other message has been sent for the last 0.2 secs)
                elif self.ACQ_STATIONARY_ODOMETRY:
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

                # Do these only if an odometry message was generated
                if odometry is not None:
                    # Update the last odometry time
                    if self.mode == 'live':
                        self.timeLastPub_odometry = rospy.get_time()

                    outputDictQueue.put(obj=pickle.dumps({'odometry': odometry}, protocol=-1),
                                        block=True,
                                        timeout=None)

            # Check if the last image data was not yet processed and if it's time to process it (in order to sustain the deisred update rate)
            if rospy.get_time() - self.timeLastPub_poses >= 1.0/self.ACQ_POSES_UPDATE_RATE and not self.lastImageProcessed:
                self.timeLastPub_poses = rospy.get_time()

                if self.lastCameraInfo is not None and self.lastCameraImage is not None:
                    # Collect latest ros_data
                    currRawImage = self.lastCameraImage
                    currCameraInfo = self.lastCameraInfo
                    outputDict = self.camera_image_process(
                        currRawImage, currCameraInfo)
                    if outputDict is not None:
                        outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                            block=True,
                                            timeout=None)
                        self.lastImageProcessed = True

    def postprocess(self, outputDictQueue, bag, n_threads=8):
        """
        Processes the data from a ROS bag.
        IMPORTANT: This will load all the bag contents in RAM! Make sure that
        the bag size is not too big and that you have sufficient RAM memory.
        """

        assert(self.mode == 'postprocessing')

        self.logger.info('Postprocessing started')

        # Setup the ProcessingPool. We use pathos as it uses dill instead of pickle which packeges more classes

        p = ProcessingPool(n_threads)
        self.logger.info('Bag loaded')

        # We process each topic separately.

        # [CAMERA INFO] The images can be processed in parallel. We assume the camera info does not change so we simply take the first one
        self.logger.info('Getting the camera info')
        for (topic, msg, t) in bag.read_messages(topics=self.bag_topics['camera_info']):
            cameraInfo = self.debagify_msg(msg)
            break
        self.logger.info('Camera info obtained')

        # [APRIL TAGS]

        def full_process(msg):
            return self.camera_image_process(msg, cameraInfo)

        self.logger.info('Loading the camera images')
        msgs_to_process = [self.debagify_msg(msg) for (
            topic, msg, t) in bag.read_messages(topics=self.bag_topics['raw_image'])]

        self.logger.info('Processing the camera images')
        results = p.map(full_process, msgs_to_process)
        for result in results:
            if result is not None:
                outputDictQueue.put(obj=pickle.dumps(result, protocol=-1),
                                    block=True,
                                    timeout=None)
        self.logger.info('Finished processing the camera images')

        # [ODOMETRY]  Odometry needs to be processed sequentially as it has to subtract the poses of subsequent messages
        if self.ACQ_ODOMETRY_POST_VISUAL_ODOMETRY:  # If visual odometry was requested
            self.logger.info(
                'Odometry processing starting. Using Visual Odometry (VO). This will be slow.')

            # We reuse the rectified images and the new camera matrix from results
            K = results[0]['new_camera_matrix'].reshape((3, 3))

            # Setting up the visual odometry
            vo = VisualOdometry(
                K, self.ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES, pitch_adjust=np.deg2rad(10.0))
            clahe = cv2.createCLAHE(clipLimit=5.0)

            # Iterating through the images one by one
            for img_id, res in enumerate(results):

                img = res['rect_image']
                img = clahe.apply(img)

                if vo.update(img, img_id):
                    # The scaling is rather arbitrary, it seems that VO gives centimeters, but in general the scaling is fishy...
                    odometry = TransformStamped()
                    odometry.header.seq = 0
                    odometry.header = res['header']
                    odometry.header.frame_id = self.ACQ_DEVICE_NAME
                    odometry.child_frame_id = self.ACQ_DEVICE_NAME

                    cy = math.cos(vo.relative_pose_theta * 0.5)
                    sy = math.sin(vo.relative_pose_theta * 0.5)
                    cp = 1.0
                    sp = 0.0
                    cr = 1.0
                    sr = 0.0

                    q_w = cy * cp * cr + sy * sp * sr
                    q_x = cy * cp * sr - sy * sp * cr
                    q_y = sy * cp * sr + cy * sp * cr
                    q_z = sy * cp * cr - cy * sp * sr

                    # Save the resuts to the new odometry relative pose message
                    odometry.transform.translation.x = vo.relative_pose_x
                    odometry.transform.translation.y = vo.relative_pose_y
                    odometry.transform.translation.z = 0
                    odometry.transform.rotation.x = q_x
                    odometry.transform.rotation.y = q_y
                    odometry.transform.rotation.z = q_z
                    odometry.transform.rotation.w = q_w

                    outputDictQueue.put(obj=pickle.dumps({'odometry': odometry}, protocol=-1),
                                        block=True,
                                        timeout=None)

                if img_id % 100 == 0:
                    self.logger.info('VO: %d/%d frames processed.' %
                                     (img_id+1, len(results)))

            self.logger.info('Odometry processing finished')

        # If VO was not requested and only if set (probably not for watchtowers)
        elif self.ACQ_TOPIC_VELOCITY_TO_POSE:
            self.logger.info(
                'Odometry processing starting. Using topic %s' % self.ACQ_TOPIC_VELOCITY_TO_POSE)
            for (topic, msg, t) in bag.read_messages(topics=self.bag_topics['odometry']):
                odometry = self.odometry_process(
                    msg.x, msg.y, msg.theta, msg.header)
                outputDictQueue.put(obj=pickle.dumps({'odometry': odometry}, protocol=-1),
                                    block=True,
                                    timeout=None)
            self.logger.info('Odometry processing finished')

        bag.close()

    def odometry_process(self, x, y, theta, header):
        """
        Callback function that is executed upon reception of new odometry data.
        """

        # Prepare the new ROS message for the processed odometry
        odometry = TransformStamped()
        odometry.header.seq = 0
        odometry.header = header
        odometry.header.frame_id = self.ACQ_DEVICE_NAME
        odometry.child_frame_id = self.ACQ_DEVICE_NAME

        # Transform the incoming data to quaternions
        transform_current = np.array([[math.cos(theta), -1.0 * math.sin(theta), x],
                                      [math.sin(theta), math.cos(theta), y],
                                      [0.0, 0.0, 1.0]])

        transform_previous = np.array([[math.cos(self.previousOdometry["theta"]), -1.0 * math.sin(self.previousOdometry["theta"]), self.previousOdometry["x"]],
                                       [math.sin(self.previousOdometry["theta"]), math.cos(
                                           self.previousOdometry["theta"]), self.previousOdometry["y"]],
                                       [0.0, 0.0, 1.0]])

        transform_previous_inv = np.linalg.inv(transform_previous)

        self.previousOdometry = {'x': x, 'y': y, 'theta': theta}

        transform_relative = np.matmul(
            transform_previous_inv, transform_current)

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

        return odometry

    def camera_image_process(self, currRawImage, currCameraInfo):
        """
        Contains the necessary camera image processing.
        """

        outputDict = None

        # Use the right dsp, depending on the mode
        if self.mode == 'live':
            if self.dsp == None:
                self.dsp = deviceSideProcessor(self.dsp_options, self.logger)
            dsp = self.dsp
        if self.mode == 'postprocessing':
            global dsp
            if dsp == None:
                dsp = deviceSideProcessor(self.dsp_options, self.logger)
        assert(dsp is not None)

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
        outputDict = dsp.process(cv_image,  (np.array(
            currCameraInfo.K)*scale_matrix).reshape((3, 3)), currCameraInfo.D)
        outputDict['header'] = currRawImage.header

        # Add the time stamp and source of the input image to the output
        for idx in range(len(outputDict['apriltags'])):
            outputDict['apriltags'][idx]['timestamp_secs'] = currRawImage.header.stamp.secs
            outputDict['apriltags'][idx]['timestamp_nsecs'] = currRawImage.header.stamp.nsecs
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
            cv2.putText(image, 'device: ' + self.ACQ_DEVICE_NAME + ', timestamp: '+str(currRawImage.header.stamp.secs)+'+'+str(currRawImage.header.stamp.nsecs),
                        org=(30, 30),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1.0,
                        thickness=2,
                        color=(255, 0, 0))

            # Add the original and diagnostic info to the outputDict
            outputDict['test_stream_image'] = self.bridge.cv2_to_compressed_imgmsg(
                image, dst_format='png')
            outputDict['test_stream_image'].header.stamp.secs = currRawImage.header.stamp.secs
            outputDict['test_stream_image'].header.stamp.nsecs = currRawImage.header.stamp.nsecs
            outputDict['test_stream_image'].header.frame_id = self.ACQ_DEVICE_NAME

            outputDict['raw_image'] = currRawImage
            outputDict['raw_camera_info'] = currCameraInfo

            outputDict['rectified_image'] = self.bridge.cv2_to_compressed_imgmsg(
                outputDict['rect_image'], dst_format='png')
            outputDict['rectified_image'].header.stamp.secs = currRawImage.header.stamp.secs
            outputDict['rectified_image'].header.stamp.nsecs = currRawImage.header.stamp.nsecs
            outputDict['rectified_image'].header.frame_id = self.ACQ_DEVICE_NAME

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

    def odometry_callback(self, ros_data):
        """
        Callback function that is executed upon reception of a new odometry message.
        """
        self.lastOdometry = ros_data
        self.lastOdometryProcessed = False

    def debagify_msg(self, msg):
        """
        The messages read from the rosbags do not have the correct classes so we need to reinitialize themself.
        """
        if 'Header' in str(type(msg)):
            newMsg = Header()
            newMsg.seq = msg.seq
            newMsg.stamp.secs = msg.stamp.secs
            newMsg.stamp.nsecs = msg.stamp.nsecs
            newMsg.frame_id = msg.frame_id
            return newMsg
        if 'RegionOfInterest' in str(type(msg)):
            newMsg = RegionOfInterest()
            newMsg.x_offset = msg.x_offset
            newMsg.y_offset = msg.y_offset
            newMsg.height = msg.height
            newMsg.width = msg.width
            newMsg.do_rectify = msg.do_rectify
            return newMsg
        if '_sensor_msgs__CompressedImage' in str(type(msg)):
            im = self.bridge.compressed_imgmsg_to_cv2(msg).copy()
            newMsg = self.bridge.cv2_to_compressed_imgmsg(im)
            newMsg.header = self.debagify_msg(msg.header)
            return newMsg
        if '_sensor_msgs__CameraInfo' in str(type(msg)):
            newMsg = CameraInfo()
            newMsg.header = self.debagify_msg(msg.header)
            newMsg.height = msg.height
            newMsg.width = msg.width
            newMsg.distortion_model = msg.distortion_model
            newMsg.D = msg.D
            newMsg.P = msg.P
            newMsg.R = msg.R
            newMsg.K = msg.K
            newMsg.binning_x = msg.binning_x
            newMsg.binning_y = msg.binning_y
            newMsg.roi = self.debagify_msg(msg.roi)
            return newMsg
        raise Exception("Message type %s could not be debagified" %
                        str(type(msg)))


class deviceSideProcessor():
    """
    Packages the image rectification and AprilTag detection for images.
    """

    def __init__(self, options, logger):
        self.logger = logger
        self.ImageRectifier = None
        self.opt_beautify = options.get('beautify', False)
        self.tag_size = options.get('tag_size', 0.065)
        self.aprilTagProcessor = apriltags3.Detector(searchpath=[ACQ_APRILTAG_SO],
                                                     families='tag36h11',
                                                     nthreads=4,
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
                'deviceSideProcessor process failed: : %s' % str(e))
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
            self.logger.warning(
                'deviceSideProcessor process failed: : %s' % str(e))
            pass
