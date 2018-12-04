#!/bin/bash

# This script ensures that both deviceSideSubscriber.py and serverSidePublisher.py are always running.

# Ensure that the hostnames and ip address are setup in /etc/hosts
echo "${ACQ_ROS_MASTER_URI_DEVICE_IP} ${ACQ_ROS_MASTER_URI_DEVICE}" >> /etc/hosts
echo "${ACQ_ROS_MASTER_URI_SERVER_IP} ${ACQ_ROS_MASTER_URI_SERVER}" >> /etc/hosts

# Start the first process
./startServerSide.sh &
# status=$?
# if [ $status -ne 0 ]; then
#   echo "Failed to start serverSidePublisher.py: $status"
#   exit $status
# else
#   echo "Started serverSidePublisher.py!"
# fi

# Start the second process
./startDeviceSide.sh &
# status=$?
# if [ $status -ne 0 ]; then
#   echo "Failed to start deviceSideProcessor.py: $status"
#   exit $status
# else
#   echo "Started deviceSideProcessor.py!"
# fi

# Naive check runs checks once a second to see if either of the processes exited.
# If one has, it kills the container

while sleep 1; do
  ps aux |grep deviceSideSubscriber |grep -q -v grep
  PROCESS_1_STATUS=$?
  ps aux |grep serverSidePublisher |grep -q -v grep
  PROCESS_2_STATUS=$?
  # If the greps above find anything, they exit with 0 status
  # If they are not both 0, then something is wrong
  if [ $PROCESS_1_STATUS -ne 0 ]; then
    echo "deviceSideSubscriber.py has exited."
    ps aux
    cat startServerSide.out
    cat startDeviceSide.out
    exit 1
  fi
  if [ $PROCESS_2_STATUS -ne 0 ]; then
    echo "serverSidePublisher.py has exited."
    exit 1
  fi
done

# while sleep 1; do
#   ps aux |grep deviceSideProcessor |grep -q -v grep
#   PROCESS_STATUS=$?
#   if [ $PROCESS_STATUS -ne 0 ]; then
#     /bin/bash startDeviceSide.sh &
#     status=$?
#     if [ $status -ne 0 ]; then
#       echo "Failed to restart deviceSideProcessor.py: $status"
#     else
#       echo "Restarted deviceSideProcessor.py!"
#     fi
#   fi
#
#   ps aux |grep serverSidePublisher |grep -q -v grep
#   PROCESS_STATUS=$?
#   if [ $PROCESS_STATUS -ne 0 ]; then
#     /bin/bash startServerSide.sh &
#     status=$?
#     if [ $status -ne 0 ]; then
#       echo "Failed to restart serverSidePublisher.py: $status"
#     else
#       echo "Restarted serverSidePublisher.py!"
#     fi
#   fi
# done
