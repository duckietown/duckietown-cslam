#!/bin/bash

# ALWAYS SET THESE TWO:
ROS_MASTER_HOSTNAME=duckietown3
ROS_MASTER_IP=192.168.1.15

if (( $#==2 )); then
  printf "Using the COMMAND LINE hostname and ip address\n"
  ROS_MASTER_HOSTNAME=$1
  ROS_MASTER_IP=$2
else
  printf "Using the DEFAULT hostname and ip address\n"
fi

array=(watchtower21 watchtower22 watchtower23 watchtower24 watchtower25 watchtower26 watchtower27 watchtower28 watchtower29 watchtower30 watchtower31 watchtower32 watchtower33 watchtower34 watchtower35)

echo "Setting up watchtowers with ROS_MASTER as $ROS_MASTER_HOSTNAME and IP as $ROS_MASTER_IP"
echo "We are setting up ${#array[*]} watchtowers"
echo "If any one of them fails or complains about incompatible Docker version, just run it again"

for index in ${!array[*]}
do
    printf "\nSetting up %s\n" ${array[$index]}
    docker -H ${array[$index]}.local start roscore || echo "ERROR: Starting roscore on ${array[$index]} failed. Probably this watchtower wasn't configured properly or we can't connect via the network."
    docker -H ${array[$index]}.local stop ros_picam
    docker -H ${array[$index]}.local rm ros_picam
    docker -H ${array[$index]}.local run -d --name ros_picam --network=host --device /dev/vchiq -v /data:/data aleksandarpetrov/rpi-duckiebot-ros-picam:highresshutter
    # docker -H ${array[$index]}.local start ros-picam || echo "ERROR: Starting ros-picam on ${array[$index]} failed. Probably this watchtower wasn't configured properly or we can't connect via the network."
    docker -H ${array[$index]}.local stop cslam-acquisition || echo "Didn't stop older cslam-acquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local rm cslam-acquisition || echo "Didn't remove older cslam-acquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local pull duckietown/cslam-acquisition:rpi-ATmsg
    docker -H ${array[$index]}.local run -d --name cslam-acquisition --restart always --network=host -e ACQ_ROS_MASTER_URI_DEVICE=${array[$index]} -e ACQ_ROS_MASTER_URI_DEVICE_IP=127.0.0.1 -e ACQ_ROS_MASTER_URI_SERVER=${ROS_MASTER_HOSTNAME} -e ACQ_ROS_MASTER_URI_ROS_MASTER_IP=${ROS_MASTER_IP}  -e ACQ_DEVICE_NAME=${array[$index]} -e ACQ_BEAUTIFY=1 -e ACQ_STATIONARY_ODOMETRY=0 -e ACQ_ODOMETRY_UPDATE_RATE=0 -e ACQ_POSES_UPDATE_RATE=15 -e ACQ_TEST_STREAM=1 -e ACQ_TOPIC_VELOCITY_TO_POSE=velocity_to_pose_node/pose -e ACQ_TOPIC_RAW=camera_node/image/compressed duckietown/cslam-acquisition:rpi-ATmsg || echo "ERROR: Starting cslam-acquisition on ${array[$index]} failed. Probably this watchtower wasn't configured properly or we can't connect via the network."
done
