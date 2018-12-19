#!/bin/bash

# Ensure that the hostnames and ip address are setup in /etc/hosts
echo "${ROS_MASTER_IP} ${ROS_MASTER}" >> /etc/hosts

export ROS_MASTER_URI=http://${ROS_MASTER}:11311
echo ROS_MASTER_URI

cd /graph_optimizer/catkin_ws
source /graph_optimizer/catkin_ws/devel/setup.bash
roslaunch pose_graph_builder graph_builder.launch &
rosrun cslam_visualization publish_markers.py
