#!/bin/bash

# Ensure that the hostnames and ip address are setup in /etc/hosts
echo "${ROS_MASTER_IP} ${ROS_MASTER}" >> /etc/hosts

export ROS_MASTER_URI=http://${ROS_MASTER}:11311

# Start the diagnostics
rosrun cslam_diagnostics cslam_diagnostics_node.py
