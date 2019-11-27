FROM duckietown/dt-ros-commons:daffy-amd64

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
ARG apriltag_dir=lib/src/apriltags3-py
ARG post_processor_src_dir=ros/src
ARG algo_dir=lib/src/duckietown_cslam/algo

# Create a directory to store the Python files and the April tag library
RUN mkdir /post_processor_node


# Copy and install the April tag library and configure the environment variables for it
RUN mkdir /post_processor_node/apriltag
COPY ${apriltag_dir}/apriltags /post_processor_node/apriltag
RUN  /bin/bash -c "mkdir /post_processor_node/apriltag/build; cd /post_processor_node/apriltag/build; cmake ..; make install"
COPY ${apriltag_dir}/apriltags3.py /post_processor_node/apriltags3.py
ENV ACQ_APRILTAG_LIB /post_processor_node/apriltag/
ENV ACQ_APRILTAG_SO /post_processor_node/apriltag/build/lib

# Copy the Python files
COPY ${post_processor_src_dir}/processing_node/post_processor_node.py /post_processor_node
COPY ${algo_dir}/image_rectifier.py /post_processor_node

# Copy the bash wrapper
COPY ${post_processor_src_dir}/processing_node/wrapper.sh /post_processor_node
RUN chmod +x /post_processor_node/*.sh

# OPTIONALLY YOU CAN ALSO SPECIFY + DEFAULT VALUES
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_TOPIC_RAW imageSparse/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
ENV ACQ_TAG_SIZE 0.065
ENV ACQ_POSES_TOPIC poses
ENV ACQ_TEST_STREAM 0
ENV ACQ_BEAUTIFY 0
ENV ACQ_APRILTAG_QUAD_DECIMATE 1.0
ENV ACQ_ODOMETRY_TOPIC odometry

# Start the processes
CMD /bin/bash -c "cd /post_processor_node; source /opt/ros/kinetic/setup.bash; source /code/catkin_ws/devel/setup.bash; /post_processor_node/wrapper.sh"
