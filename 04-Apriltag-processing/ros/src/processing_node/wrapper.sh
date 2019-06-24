#!/bin/bash

# Ensure that the hostnames and ip address are setup in /etc/hosts
echo "${ACQ_ROS_MASTER_URI_DEVICE_IP} ${ACQ_ROS_MASTER_URI_DEVICE}" >> /etc/hosts
echo "${ACQ_ROS_MASTER_URI_SERVER_IP} ${ACQ_ROS_MASTER_URI_SERVER}" >> /etc/hosts

# Start the two processes
python apriltag_processor_node.py
