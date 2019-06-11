#!/usr/bin/env python

from serverSidePublisher import publishOnServer
from acquisitionProcessor import acquisitionProcessor
import math
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import Pose2DStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import time

import multiprocessing
import logging
import traceback
logger = multiprocessing.log_to_stderr()
logger.setLevel(logging.INFO)
# logger=None


# IMPORT THE ENVIRONMENT VARIABLES (DEPENDING ON THE MODE)
ACQ_DEVICE_MODE = os.getenv('ACQ_DEVICE_MODE', 'live')
if ACQ_DEVICE_MODE != 'live' and ACQ_DEVICE_MODE != 'postprocessing':
    raise Exception(
        "The environment variable ACQ_DEVICE_MODE should be 'live' or 'postprocessing'. Received %s instead." % ACQ_MODE)
ACQ_SERVER_MODE = os.getenv('ACQ_SERVER_MODE', 'live')
if ACQ_SERVER_MODE != 'live' and ACQ_SERVER_MODE != 'postprocessing':
    raise Exception(
        "The environment variable ACQ_SERVER_MODE should be 'live' or 'postprocessing'. Received %s instead." % ACQ_MODE)

if ACQ_DEVICE_MODE == 'live':
    ACQ_ROS_MASTER_URI_DEVICE = os.getenv('ACQ_ROS_MASTER_URI_DEVICE', "")
    ACQ_ROS_MASTER_URI_DEVICE_PORT = os.getenv(
        'ACQ_ROS_MASTER_URI_DEVICE_PORT', "")
elif ACQ_DEVICE_MODE == 'postprocessing':
    ACQ_DEVICE_THREADS = int(os.getenv('ACQ_DEVICE_THREADS', 8))
    ACQ_DEVICE_BAG = os.getenv('ACQ_DEVICE_BAG', None)
    if ACQ_DEVICE_BAG == None:
        raise Exception('Env. variable ACQ_DEVICE_BAG should be defined!')

if ACQ_SERVER_MODE == 'live':
    ACQ_ROS_MASTER_URI_SERVER = os.getenv('ACQ_ROS_MASTER_URI_SERVER', "")
    ACQ_ROS_MASTER_URI_SERVER_PORT = os.getenv(
        'ACQ_ROS_MASTER_URI_SERVER_PORT', "")
elif ACQ_SERVER_MODE == 'postprocessing':
    pass

# Define the two concurrent processes:


def runDeviceSideProcess(ROS_MASTER_URI, outputDictQueue, quitEvent):
    """
    Receive and process data from the remote device (Duckiebot or watchtower).
    """
    if ACQ_DEVICE_MODE == 'live':
        logger.info('Device side processor starting in LIVE mode')
        os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
        ap = acquisitionProcessor(logger, mode="live")
        ap.liveUpdate(outputDictQueue, quitEvent)
    elif ACQ_DEVICE_MODE == 'postprocessing':
        logger.info('Device side processor starting in POSTPROCESSING mode')
        ap = acquisitionProcessor(logger, mode="postprocessing")
        ap.postprocess(outputDictQueue, bag=rosbag.Bag(
            ACQ_DEVICE_BAG), n_threads=ACQ_DEVICE_THREADS)


def runServerSideProcess(ROS_MASTER_URI, outpuDictQueue, quitEvent):
    """
    Publish the processed data to the ROS Master that the graph optimizer uses.
    """
    if ACQ_SERVER_MODE == 'live':
        logger.info('Server side processor starting in LIVE mode')
        os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
        publishOnServer(outputDictQueue, quitEvent, logger, mode='live')
    elif ACQ_SERVER_MODE == 'postprocessing':
        logger.info('Server side processor starting in POSTPROCESSING mode')
        publishOnServer(outputDictQueue, quitEvent,
                        logger, mode='postprocessing')


if __name__ == '__main__':
    """
    Starts the two processes and sets up their termination.
    """

    print("Image and odometry acquision, processing and publishing setting up")

    # Event to terminate the two processes
    quitEvent = multiprocessing.Event()

    if ACQ_SERVER_MODE == 'live':
        ros_master_uri_server = "http://"+ACQ_ROS_MASTER_URI_SERVER + \
            ":"+ACQ_ROS_MASTER_URI_SERVER_PORT
    else:
        ros_master_uri_server = None
    if ACQ_DEVICE_MODE == 'live':
        ros_master_uri_device = "http://"+ACQ_ROS_MASTER_URI_DEVICE + \
            ":"+ACQ_ROS_MASTER_URI_DEVICE_PORT
    else:
        ros_master_uri_device = None

    # outputDictQueue is used to pass data between the two processes
    if ACQ_SERVER_MODE == 'live':
        outputDictQueue = multiprocessing.Queue(maxsize=20)
    else:
        outputDictQueue = multiprocessing.Queue(maxsize=0)

    # Start the processes

    deviceSideProcess = multiprocessing.Process(target=runDeviceSideProcess,
                                                args=(ros_master_uri_device,
                                                      outputDictQueue, quitEvent),
                                                name="deviceSideProcess")

    deviceSideProcess.start()
    # publishOnServer(outputDictQueue, quitEvent, logger)

    # ap = acquisitionProcessor(outputDictQueue, logger, mode="postprocessing")

    serverSideProcess = multiprocessing.Process(target=runServerSideProcess,
                                                args=(ros_master_uri_server,
                                                      outputDictQueue, quitEvent),
                                                name="serverSideProcess")

    serverSideProcess.start()

    # Exit if any of the two processes exits:
    while True:
        # print("Current approx. size %d " % outputDictQueue.qsize())

        if not deviceSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            # Wait until the queue is processed
            while not outputDictQueue.empty():
                time.sleep(0.1)
            outputDictQueue.close()
            # Give time for submitting the last message to the server
            serverSideProcess.join()
            time.sleep(0.5)
            serverSideProcess.terminate()
            print("The device side process exited. Stopping everything.")
            sys.exit()

        if not serverSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            serverSideProcess.terminate()
            outputDictQueue.close()
            print("The server side process exited. Stopping everything.")
            sys.exit()

        time.sleep(0.2)
