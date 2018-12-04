#!/bin/bash

export ROS_MASTER_URI=http://${ACQ_ROS_MASTER_URI_SERVER}:${ACQ_ROS_MASTER_URI_SERVER_PORT}
python serverSidePublisher.py
