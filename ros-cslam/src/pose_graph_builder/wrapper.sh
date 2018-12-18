#!/bin/bash

# Ensure that the hostnames and ip address are setup in /etc/hosts
echo "${ROS_MASTER_URI_DEVICE_IP} ${ROS_MASTER_URI_DEVICE}" >> /etc/hosts

export ROS_MASTER_URI=http://${ROS_MASTER_URI_DEVICE}:11311
export ROS_IP=${ROS_MASTER_URI_DEVICE_IP}

# Start the diagnostics
roslaunch pose_graph_builder graph_builder.launch

