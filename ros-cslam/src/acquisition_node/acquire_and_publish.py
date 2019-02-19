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

import multiprocessing
import logging
import traceback
logger = multiprocessing.log_to_stderr()
logger.setLevel(logging.INFO)

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


# Define the two concurrent processes:
def runDeviceSideProcess(ROS_MASTER_URI, outputDictQueue, quitEvent):
    """
    Receive and process data from the remote device (Duckiebot or watchtower).
    """

    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    ap = acquisitionProcessor(outputDictQueue, logger)
    ap.update(quitEvent)
def runServerSideProcess(ROS_MASTER_URI, outpuDictQueue, quitEvent):
    """
    Publush the processed data to the ROS Master that the graph optimizer uses.
    """
    try:
        os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
        publishOnServer(outputDictQueue, quitEvent)
    except Exception as error:
        serverSideProcessException.put(obj=traceback.format_exc(),
                                       block=True,
                                       timeout=None)

if __name__ == '__main__':
    """
    Starts the two processes and sets up their termination.
    """

    print("Image and odometry acquision, processing and publishing setting up")

    # Event to terminate the two processes
    quitEvent = multiprocessing.Event()

    ros_master_uri_server = "http://"+ACQ_ROS_MASTER_URI_SERVER+":"+ACQ_ROS_MASTER_URI_SERVER_PORT
    ros_master_uri_device = "http://"+ACQ_ROS_MASTER_URI_DEVICE+":"+ACQ_ROS_MASTER_URI_DEVICE_PORT

    # outputDictQueue is used to pass data between the two processes
    outputDictQueue = multiprocessing.Queue(maxsize=20)

    # Sharing excepton states
    deviceSideProcessException = multiprocessing.Queue()
    serverSideProcessException = multiprocessing.Queue()

    deviceSideProcess = multiprocessing.Process(target=runDeviceSideProcess,
                                                args=(ros_master_uri_device,outputDictQueue,quitEvent),
                                                name="deviceSideProcess")
    serverSideProcess = multiprocessing.Process(target=runServerSideProcess,
                                                args=(ros_master_uri_server,outputDictQueue,quitEvent),
                                                name="serverSideProcess")

    deviceSideProcess.start()
    serverSideProcess.start()

    # Exit if any of the two processes exits:
    while True:
        if not deviceSideProcess.is_alive():
            quitEvent.set()
            exc_info = to_child.recv()
            deviceSideProcess.terminate()
            serverSideProcess.terminate()
            outputDictQueue.close()
            print("The device side process exited. Stopping everything.")
            while not deviceSideProcessException.empty():
                print('a')
                print(deviceSideProcessException.get(block=False, timeout=1))
            deviceSideProcessException.close()
            sys.exit()

        if not serverSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            serverSideProcess.terminate()
            outputDictQueue.close()
            print("The server side process exited. Stopping everything.")
            e = serverSideProcessException.get(block=False, timeout=1)
            raise e
            sys.exit()
