#!/bin/bash

export ROS_MASTER_URI=http://${ACQ_ROS_MASTER_URI_DEVICE}:${ACQ_ROS_MASTER_URI_DEVICE_PORT}
./deviceSideSubscriber.py
