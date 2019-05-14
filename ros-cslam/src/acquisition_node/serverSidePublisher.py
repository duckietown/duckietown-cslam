#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from duckietown_msgs.msg import AprilTagDetection
import cPickle as pickle
import os
import numpy as np
import Queue
import collections
import yaml

# A mockup class that behaves as rospy.Publish but instead saves to a rosbag
class MockupPublisher():
    def __init__(self, bag, topic):
        self.bag = bag
        self.topic = topic
    def publish(self, msg):
        self.bag.write(self.topic, msg, msg.header.stamp)

def publishOnServer(outputDictQueue, quitEvent, logger, mode='live'):
    """
    Publishes the processed data on the ROS Master that the graph optimizer uses.
    """

    assert(mode=='live' or mode=='postprocessing')

    logger.info("Setting up the server side process")

    # Get the environment variables
    ACQ_POSES_TOPIC = os.getenv('ACQ_POSES_TOPIC', "poses")
    ACQ_ODOMETRY_TOPIC = os.getenv('ACQ_ODOMETRY_TOPIC', "odometry")
    ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
    ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))
    ACQ_OBSERVATIONS_STATISTICS_OUTPUT = os.getenv('ACQ_OBSERVATIONS_STATISTICS_OUTPUT', None)

    seq_stamper = 0
    counts = collections.Counter()

    # Different initialization of the topics for live and postprocessing modes
    if mode=='live':
        # Setup the topics
        publisherPoses = rospy.Publisher("/poses_acquisition/"+ACQ_POSES_TOPIC, AprilTagDetection, queue_size=1)
        publisherOdometry = rospy.Publisher("/poses_acquisition/"+ACQ_ODOMETRY_TOPIC, TransformStamped, queue_size=1)

        # If the test stream is requested
        if ACQ_TEST_STREAM:
            publisherTestImages = rospy.Publisher("/poses_acquisition/test_video/"+ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
            publisherRawImages = rospy.Publisher("/poses_acquisition/raw_video/"+ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
            publisherRectifiedImages = rospy.Publisher("/poses_acquisition/rectified_video/"+ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
            publisherCameraInfoRaw = rospy.Publisher("/poses_acquisition/camera_info_raw/"+ACQ_DEVICE_NAME, CameraInfo, queue_size=1)
            publisherCameraInfoRectified = rospy.Publisher("/poses_acquisition/camera_info_rectified/"+ACQ_DEVICE_NAME, CameraInfo, queue_size=1)

        # Init the node (live mode only)
        rospy.init_node('acquisition_node_'+ACQ_DEVICE_NAME)

    elif mode=='postprocessing':

        # Open the bag (postprocessing mode only)
        ACQ_OUTPUT_BAG = os.getenv('ACQ_OUTPUT_BAG', None)
        if ACQ_OUTPUT_BAG == None:
            raise Exception("ACQ_OUTPUT_BAG must be set if the server processor is in postprocessing mode!")
        if os.path.exists(ACQ_OUTPUT_BAG):
            bag = rosbag.Bag(ACQ_OUTPUT_BAG, 'a')
        else:
            bag = rosbag.Bag(ACQ_OUTPUT_BAG, 'w')


        # Setup the topics
        publisherPoses = MockupPublisher(bag, "/poses_acquisition/"+ACQ_POSES_TOPIC)
        publisherOdometry = MockupPublisher(bag, "/poses_acquisition/"+ACQ_ODOMETRY_TOPIC)

        # If the test stream is requested
        if ACQ_TEST_STREAM:
            publisherTestImages = MockupPublisher(bag, "/poses_acquisition/test_video/"+ACQ_DEVICE_NAME+"/compressed")
            publisherRawImages = MockupPublisher(bag, "/poses_acquisition/raw_video/"+ACQ_DEVICE_NAME+"/compressed")
            publisherRectifiedImages = MockupPublisher(bag, "/poses_acquisition/rectified_video/"+ACQ_DEVICE_NAME+"/compressed")
            publisherCameraInfoRaw = MockupPublisher(bag, "/poses_acquisition/camera_info_raw/"+ACQ_DEVICE_NAME)
            publisherCameraInfoRectified = MockupPublisher(bag, "/poses_acquisition/camera_info_rectified/"+ACQ_DEVICE_NAME)

    logger.info("Setting up the server side process completed. Waiting for messages...")

    # Run continuously, check for new data arriving from the acquisitionProcessor and processed it when it arrives
    while not quitEvent.is_set():
        try:
            newQueueData = outputDictQueue.get(block=True, timeout=5)
            # logger.info("Backlog of messages to send: %d messages" % outputDictQueue.qsize())
            incomingData = pickle.loads(newQueueData)

            if "apriltags" in incomingData:
                for tag in incomingData["apriltags"]:
                    # Publish the relative pose
                    newApriltagDetectionMsg = AprilTagDetection()
                    newApriltagDetectionMsg.header.seq = seq_stamper
                    newApriltagDetectionMsg.header.stamp.secs = int(tag["timestamp_secs"])
                    newApriltagDetectionMsg.header.stamp.nsecs = int(tag["timestamp_nsecs"])
                    newApriltagDetectionMsg.header.frame_id = str(tag["source"])
                    newApriltagDetectionMsg.transform.translation.x = float(tag["tvec"][0])
                    newApriltagDetectionMsg.transform.translation.y = float(tag["tvec"][1])
                    newApriltagDetectionMsg.transform.translation.z = float(tag["tvec"][2])
                    newApriltagDetectionMsg.transform.rotation.x = float(tag["qvec"][0])
                    newApriltagDetectionMsg.transform.rotation.y = float(tag["qvec"][1])
                    newApriltagDetectionMsg.transform.rotation.z = float(tag["qvec"][2])
                    newApriltagDetectionMsg.transform.rotation.w = float(tag["qvec"][3])
                    newApriltagDetectionMsg.tag_id = int(tag["tag_id"])
                    newApriltagDetectionMsg.tag_family = tag["tag_family"]
                    newApriltagDetectionMsg.hamming = int(tag["hamming"])
                    newApriltagDetectionMsg.decision_margin = float(tag["decision_margin"])
                    newApriltagDetectionMsg.homography = tag["homography"].flatten()
                    newApriltagDetectionMsg.center = tag["center"]
                    newApriltagDetectionMsg.corners = tag["corners"].flatten()
                    newApriltagDetectionMsg.pose_error = tag["pose_error"]

                    publisherPoses.publish(newApriltagDetectionMsg)
                    if mode=="live":
                        logger.info("Published pose for tag %d in sequence %d" % (tag["tag_id"], seq_stamper))

                    counts[newApriltagDetectionMsg.tag_id] += 1

            if "odometry" in incomingData:
                publisherOdometry.publish(incomingData["odometry"])
                counts["odometry"] += 1

            # Publish the test and raw data if submitted and requested:
            if ACQ_TEST_STREAM:
                if "test_stream_image" in incomingData:
                    imgMsg = incomingData["test_stream_image"]
                    imgMsg.header.seq = seq_stamper
                    publisherTestImages.publish(imgMsg)

                if "raw_image" in incomingData:
                    imgMsg = incomingData["raw_image"]
                    imgMsg.header.seq = seq_stamper
                    publisherRawImages.publish(imgMsg)

                if "rectified_image" in incomingData:
                    imgMsg = incomingData["rectified_image"]
                    imgMsg.header.seq = seq_stamper
                    publisherRectifiedImages.publish(imgMsg)

                if "raw_camera_info" in incomingData:
                    publisherCameraInfoRaw.publish(incomingData["raw_camera_info"])

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
                    new_camera_info.K = incomingData["new_camera_matrix"].flatten()
                    publisherCameraInfoRectified.publish(new_camera_info)

            seq_stamper+=1

        except KeyboardInterrupt:
            raise( Exception("Exiting") )
        except Queue.Empty:
            if os.getenv('ACQ_DEVICE_MODE', 'live') == 'live':
                logger.warning("No messages received in the last 5 seconds!")
        except Exception as e:
            logger.warning("Exception: %s" % str(e))
            pass

    # Close the bag (postprocessing mode only)
    if mode=='postprocessing':
        bag.close()

    # Save or append to an existing file:
    if ACQ_OBSERVATIONS_STATISTICS_OUTPUT:
        logger.info("Saving statistics to %s", ACQ_OBSERVATIONS_STATISTICS_OUTPUT)
        new_stats = dict()
        for id, count in counts.iteritems():
            new_stats[id] = count

        yaml.dump({ACQ_DEVICE_NAME: new_stats}, open(ACQ_OBSERVATIONS_STATISTICS_OUTPUT,'a'))

    # print the observation statistics in the terminal
    logger.info("\n\n")
    logger.info("----------------------------------------------------")
    logger.info("COUNTS OF OBSERVED APRIL TAGS AND ODOMETRY MESSAGES:")
    logger.info("Device: %s" % ACQ_DEVICE_NAME)
    for id, count in counts.iteritems():
        logger.info("tag: {0}\t{1} observations".format(id, count))
