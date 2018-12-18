#!/bin/bash

# ALWAYS SET THESE TWO:
SERVER_HOSTNAME=ubuntu-Aleks.local
SERVER_IP=192.168.1.104

array=(demowatchtower01 demowatchtower02 demowatchtower03 demowatchtower04 demowatchtower05 demowatchtower06 demowatchtower07 demowatchtower08 demowatchtower09 demowatchtower10 demowatchtower11 demowatchtower12 demowatchtower13 demowatchtower14 demowatchtower15 )

echo "We are setting up ${#array[*]} watchtowers"

for index in ${!array[*]}
do
    printf "\nSetting up %s\n" ${array[$index]}
    docker -H ${array[$index]}.local start roscore || echo "ERROR: Starting roscore on ${array[$index]} failed. Probably this watchtower wasn't configured properly or we can't connect via the network."
    docker -H ${array[$index]}.local start ros-picam || echo "ERROR: Starting ros-picam on ${array[$index]} failed. Probably this watchtower wasn't configured properly or we can't connect via the network."
    docker -H ${array[$index]}.local stop cslam-aquisition || echo "Didn't stop older cslam-aquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local rm cslam-aquisition || echo "Didn't remove older cslam-aquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local run -d --name cslam-aquisition --restart always --network=host -e ACQ_ROS_MASTER_URI_DEVICE=${array[$index]}.local -e ACQ_ROS_MASTER_URI_DEVICE_IP=127.0.0.1 -e ACQ_ROS_MASTER_URI_SERVER=${SERVER_HOSTNAME} -e ACQ_ROS_MASTER_URI_SERVER_IP=${SERVER_IP}  -e ACQ_DEVICE_NAME=${array[$index]} -e ACQ_BEAUTIFY=0 -e ACQ_STATIONARY_ODOMETRY=0 -e ACQ_ODOMETRY_UPDATE_RATE=0 -e ACQ_POSES_UPDATE_RATE=15 -e ACQ_TEST_STREAM=1 -e ACQ_TOPIC_VELOCITY_TO_POSE=velocity_to_pose_node/pose -e ACQ_TOPIC_RAW=camera_node/image/compressed duckietown/cslam-aquisition:rpi || echo "ERROR: Starting cslam-acquisition on ${array[$index]} failed. Probably this watchtower wasn't configured properly or we can't connect via the network."
done
