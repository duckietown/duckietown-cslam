#!/usr/bin/env python

from interprocess_communication import socketServer
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cPickle as pickle
import os

ACQ_SOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
ACQ_SOCKET_PORT = int(os.getenv('ACQ_SOCKET_PORT', 65432))
ACQ_POSES_TOPIC = os.getenv('ACQ_POSES_TOPIC', "poses")
ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower10")
ACQ_TEST_STREAM = bool(os.getenv('ACQ_TEST_STREAM', True))

server = socketServer(ACQ_SOCKET_HOST, ACQ_SOCKET_PORT)

seq_stamper = 0

publisherPoses = rospy.Publisher("/poses_acquisition/"+ACQ_POSES_TOPIC, TransformStamped, queue_size=1)
if ACQ_TEST_STREAM: publisherImages = rospy.Publisher("/poses_acquisition/video/"+ACQ_DEVICE_NAME, Image, queue_size=1)
bridge = CvBridge()
rospy.init_node('acquisition_node_'+ACQ_DEVICE_NAME)

while True:
    try:
        incomingData = pickle.loads(server.waitAndGetData())

        for tag in incomingData:
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

            # Publish the overlayed image if submitted:
            if ACQ_TEST_STREAM and tag["overlayed_image"] is not None:
                imgMsg = bridge.cv2_to_imgmsg(tag["overlayed_image"], encoding="rgb8")
                imgMsg.header.seq = seq_stamper
                imgMsg.header.stamp.secs = int(tag["timestamp_secs"])
                imgMsg.header.stamp.nsecs = int(tag["timestamp_nsecs"])
                imgMsg.header.frame_id = str(tag["source"])
                publisherImages.publish(imgMsg)

        seq_stamper+=1

    except KeyboardInterrupt:
        raise( Exception("Exiting") )
    except Exception as e:
        print("Exception: %s", str(e))
        pass
