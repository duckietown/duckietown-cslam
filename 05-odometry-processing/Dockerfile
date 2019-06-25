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
ARG visual_odometry_dir=lib/src/visual_odometry
ARG ros_src_dir=ros/src
ARG algo_dir=lib/src/duckietown_cslam/algo

# Create a directory to store the Python files and the visual odometry library
RUN mkdir /processing_node



# Copy the Visual Odometry library
RUN mkdir /processing_node/visual_odometry
COPY ${visual_odometry_dir} /processing_node/visual_odometry
ENV ACQ_VISUAL_ODOMETRY_LIB /processing_node/visual_odometry/src/

# Setup the duckietown_msgs
RUN mkdir -p /catkin-ws/src
COPY ${ros_src_dir}/duckietown_msgs /catkin-ws/src/duckietown_msgs
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin-ws/; catkin_make"


# Copy the Python files
COPY ${ros_src_dir}/processing_node/odometry_processor_node.py /processing_node
COPY ${algo_dir}/image_rectifier.py /processing_node

# Copy the bash wrapper
COPY ${ros_src_dir}/processing_node/wrapper.sh /processing_node
RUN chmod +x /processing_node/*.sh

# OPTIONALLY YOU CAN ALSO SPECIFY + DEFAULT VALUES
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_TOPIC_RAW camera_node/image/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
# ENV ACQ_TOPIC_VELOCITY_TO_POSE velocity_to_pose_node/pose
ENV ACQ_POSES_TOPIC poses
ENV ACQ_ODOMETRY_TOPIC odometry
ENV ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES SURF

# Start the processes
CMD /bin/bash -c "cd /processing_node; source /catkin-ws/devel/setup.bash; /processing_node/wrapper.sh"
