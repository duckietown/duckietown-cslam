#!/bin/bash

# Ensure that the hostnames and ip address are setup in /etc/hosts
#echo "${ROS_MASTER_URI_DEVICE_IP} ${ROS_MASTER_URI_DEVICE}" >> /etc/hosts

# Start the diagnostics
rosrun cslam_diagnostics cslam_diagnostics_node.py
