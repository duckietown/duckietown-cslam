#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import cPickle as pickle
import os

def publishOnServer(outputDictQueue, quitEvent):
    """
    Publishes the processed data on the ROS Master that the graph optimizer uses.
    """

    # Get the environment variables
    ACQ_POSES_TOPIC = os.getenv('ACQ_POSES_TOPIC', "poses")
    ACQ_ODOMETRY_TOPIC = os.getenv('ACQ_ODOMETRY_TOPIC', "odometry")
    ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
    ACQ_TEST_STREAM = bool(int(os.getenv('ACQ_TEST_STREAM', 1)))

    seq_stamper = 0

    publisherPoses = rospy.Publisher("/poses_acquisition/"+ACQ_POSES_TOPIC, TransformStamped, queue_size=1)
    publisherOdometry = rospy.Publisher("/poses_acquisition/"+ACQ_ODOMETRY_TOPIC, TransformStamped, queue_size=1)

    # If the test stream is requested
    if ACQ_TEST_STREAM:
        publisherTestImages = rospy.Publisher("/poses_acquisition/test_video/"+ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
        publisherRawImages = rospy.Publisher("/poses_acquisition/raw_video/"+ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
        publisherRectifiedImages = rospy.Publisher("/poses_acquisition/rectified_video/"+ACQ_DEVICE_NAME+"/compressed", CompressedImage, queue_size=1)
        publisherCameraInfoRaw = rospy.Publisher("/poses_acquisition/camera_info_raw/"+ACQ_DEVICE_NAME, CameraInfo, queue_size=1)
        publisherCameraInfoRectified = rospy.Publisher("/poses_acquisition/camera_info_rectified/"+ACQ_DEVICE_NAME, CameraInfo, queue_size=1)

    rospy.init_node('acquisition_node_'+ACQ_DEVICE_NAME)

    # Run continuously, check for new data arriving from the acquisitionProcessor and processed it when it arrives
    while not quitEvent.is_set():
        try:
            newQueueData = outputDictQueue.get(block=True, timeout=1)
            incomingData = pickle.loads(newQueueData)

            print("Backlog of messages to send: %d messages" % outputDictQueue.qsize())

            if "apriltags" in incomingData:
                for tag in incomingData["apriltags"]:
                    # Publish the relative pose
                    newTransformStamped = TransformStamped()
                    newTransformStamped.header.seq = seq_stamper
                    newTransformStamped.header.stamp.secs = int(tag["timestamp_secs"])
                    newTransformStamped.header.stamp.nsecs = int(tag["timestamp_nsecs"])
                    newTransformStamped.header.frame_id = str(tag["source"])
                    newTransformStamped.child_frame_id = str(tag["tag_id"])
                    newTransformStamped.transform.translation.x = float(tag["tvec"][0])
                    newTransformStamped.transform.translation.y = float(tag["tvec"][1])
                    newTransformStamped.transform.translation.z = float(tag["tvec"][2])
                    newTransformStamped.transform.rotation.x = float(tag["qvec"][0])
                    newTransformStamped.transform.rotation.y = float(tag["qvec"][1])
                    newTransformStamped.transform.rotation.z = float(tag["qvec"][2])
                    newTransformStamped.transform.rotation.w = float(tag["qvec"][3])

                    publisherPoses.publish(newTransformStamped)
                    print("Published pose for tag %d in sequence %d" % (tag["tag_id"], seq_stamper))

            if "odometry" in incomingData:
                publisherOdometry.publish(incomingData["odometry"])

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

                if "rectified_camera_info" in incomingData:
                    publisherCameraInfoRectified.publish(incomingData["rectified_camera_info"])

            seq_stamper+=1

        except KeyboardInterrupt:
            raise( Exception("Exiting") )
        except Exception as e:
            print("Exception: %s", str(e))
            pass
