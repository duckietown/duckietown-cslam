FROM ros:kinetic-ros-core

MAINTAINER Amaury Camus <camusam@ethz.ch>


# Install OpenCV and CvBridge
RUN apt-get update && apt-get install -f -y --allow-unauthenticated python-opencv ros-kinetic-cv-bridge ros-kinetic-rospy python-pip && apt-get clean

RUN pip install pathos multiprocessing-logging

# YOU ALWAYS NEED TO SPECIFY:
# ACQ_ROS_MASTER_URI_DEVICE
# ACQ_ROS_MASTER_URI_DEVICE_IP
# ACQ_ROS_MASTER_URI_SERVER
# ACQ_ROS_MASTER_URI_SERVER_IP
# ACQ_DEVICE_NAME # should match the topic, e.g. watchtower10





# Argumens for where the different places from which we will get files from are
ARG ros_src_dir=ros/src

# Create a directory to store the Python files and the visual odometry library
RUN mkdir /processing_node


# Setup the duckietown_msgs
RUN mkdir -p /catkin-ws/src
COPY ${ros_src_dir}/duckietown_msgs /catkin-ws/src/duckietown_msgs
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin-ws/; catkin_make"


# Copy the Python files
COPY ${ros_src_dir}/processing_node/odometry_processor_node.py /processing_node

# Copy the bash wrapper
COPY ${ros_src_dir}/processing_node/wrapper.sh /processing_node
RUN chmod +x /processing_node/*.sh
RUN chmod +x /processing_node/*.py

# OPTIONALLY YOU CAN ALSO SPECIFY + DEFAULT VALUES
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
# ENV ACQ_TOPIC_VELOCITY_TO_POSE velocity_to_pose_node/pose
ENV ACQ_ODOMETRY_TOPIC odometry



# Start the processes
CMD /bin/bash -c "cd /processing_node; source /catkin-ws/devel/setup.bash; /processing_node/wrapper.sh"
