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


RUN apt-get install -y wget unzip
RUN cd / && wget "https://github.com/opencv/opencv/archive/4.3.0.zip" && unzip 4.3.0.zip && rm -r 4.3.0.zip
RUN mkdir /opencv-4.3.0/build && cd /opencv-4.3.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && make -j$(nproc) && make install



# Argumens for where the different places from which we will get files from are
ARG aruco_lib_dir=lib/src/aruco
ARG aruco_detector_src_dir=ros/src
ARG algo_dir=lib/src/duckietown_cslam/algo

# Create a directory to store the Python files and the Aruco library
RUN mkdir /aruco_detector_node


# Copy and install the April tag library and configure the environment variables for it
RUN mkdir /aruco_detector_node/aruco/
RUN cd /aruco_detector_node/aruco/ && wget "https://sourceforge.net/projects/aruco/files/3.1.12/aruco-3.1.12.zip" && \
    unzip aruco-3.1.12.zip && rm -r aruco-3.1.12.zip
COPY ${aruco_lib_dir}/aruco_caller.cpp /aruco_detector_node/aruco/
COPY ${aruco_lib_dir}/CMakeLists.txt /aruco_detector_node/aruco/
RUN  /bin/bash -c "mkdir /aruco_detector_node/aruco/build; cd /aruco_detector_node/aruco/build; cmake ..; make"
COPY ${aruco_lib_dir}/aruco_lib_adapter.py /aruco_detector_node/aruco/
#ENV ACQ_APRILTAG_LIB /aruco_detector_node/aruco
ENV ACQ_APRILTAG_SO /aruco_detector_node/aruco/build/libaruco_lib.so

# Copy the Python files
COPY ${aruco_detector_src_dir}/aruco_detector_node/aruco_detector_node.py /aruco_detector_node
COPY ${algo_dir}/image_rectifier.py /aruco_detector_node

# Copy the bash wrapper
COPY ${aruco_detector_src_dir}/aruco_detector_node/wrapper.sh /aruco_detector_node
RUN chmod +x /aruco_detector_node/wrapper.sh

ENV PYTHONPATH="$PYTHONPATH:/aruco_detector_node/aruco/"

# OPTIONALLY YOU CAN ALSO SPECIFY + DEFAULT VALUES
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_TOPIC_RAW camera_image/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_image_info
ENV ACQ_TAG_SIZE 0.065
ENV ACQ_POSES_TOPIC poses
ENV ACQ_TEST_STREAM 0
ENV ACQ_BEAUTIFY 0

# Start the processes
CMD /bin/bash -c "cd /aruco_detector_node; source /opt/ros/kinetic/setup.bash; source /code/catkin_ws/devel/setup.bash; /aruco_detector_node/wrapper.sh"
