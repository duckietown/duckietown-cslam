#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import multiprocessing
import cPickle as pickle

import numpy as np
import math

from acquisitionProcessor import acquisitionProcessor
from serverSidePublisher import publishOnServer

# IMPORT THE ENVIRONMENT VARIABLES
ACQ_SOCKET_HOST = os.getenv('ACQ_SOCKET_HOST', '127.0.0.1')
ACQ_ROS_MASTER_URI_SERVER = os.getenv('ACQ_ROS_MASTER_URI_SERVER', "")
ACQ_ROS_MASTER_URI_SERVER_PORT = os.getenv('ACQ_ROS_MASTER_URI_SERVER_PORT', "")
ACQ_ROS_MASTER_URI_DEVICE = os.getenv('ACQ_ROS_MASTER_URI_DEVICE', "")
ACQ_ROS_MASTER_URI_DEVICE_PORT = os.getenv('ACQ_ROS_MASTER_URI_DEVICE_PORT', "")


#Set up the two processes:
def runDeviceSideProcess(ROS_MASTER_URI, outputDictQueue, quitEvent):
    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    ap = acquisitionProcessor(outputDictQueue)
    ap.update(quitEvent)

def runServerSideProcess(ROS_MASTER_URI, outpuDictQueue, quitEvent):
    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    publishOnServer(outputDictQueue, quitEvent)

if __name__ == '__main__':
    print("Image and odometry acquision, processing and publishing setting up")

    # Event to terminate the two processes
    quitEvent = multiprocessing.Event()

    ros_master_uri_server = "http://"+ACQ_ROS_MASTER_URI_SERVER+":"+ACQ_ROS_MASTER_URI_SERVER_PORT
    ros_master_uri_device = "http://"+ACQ_ROS_MASTER_URI_DEVICE+":"+ACQ_ROS_MASTER_URI_DEVICE_PORT

    outputDictQueue = multiprocessing.Queue(maxsize=20)

    deviceSideProcess = multiprocessing.Process(target=runDeviceSideProcess,
                                                args=(ros_master_uri_device,outputDictQueue,quitEvent,),
                                                name="deviceSideProcess")
    serverSideProcess = multiprocessing.Process(target=runServerSideProcess,
                                                args=(ros_master_uri_server,outputDictQueue,quitEvent,),
                                                name="serverSideProcess")

    deviceSideProcess.start()
    serverSideProcess.start()

    # Exit if any of the two processes exits:
    while True:
        if not deviceSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            serverSideProcess.terminate()
            outputDictQueue.close()
            raise Exception("The device side process exited for some reason. Stopping everything.")
            sys.exit()

        if not serverSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            serverSideProcess.terminate()
            outputDictQueue.close()
            raise Exception("The server side process exited for some reason. Stopping everything.")
            sys.exit()
