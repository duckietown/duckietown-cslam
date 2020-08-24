FROM duckietown/dt-ros-commons:daffy-amd64

# Install OpenCV and CvBridge
RUN apt-get update && apt-get install -f -y --allow-unauthenticated \
                      python-opencv ros-kinetic-cv-bridge ros-kinetic-rospy python-pip && apt-get clean
RUN pip install pathos multiprocessing-logging

# You always need to specify:
# ACQ_ROS_MASTER_URI_DEVICE
# ACQ_ROS_MASTER_URI_DEVICE_IP
# ACQ_ROS_MASTER_URI_SERVER
# ACQ_ROS_MASTER_URI_SERVER_IP
# ACQ_DEVICE_NAME # should match the topic, e.g. watchtower10

# Download and install opencv-4.3.0 for the ArUco library
RUN apt-get install -y wget unzip
RUN cd / && wget https://github.com/opencv/opencv/archive/4.3.0.zip && unzip 4.3.0.zip && rm -r 4.3.0.zip
RUN mkdir /opencv-4.3.0/build && cd /opencv-4.3.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && make -j$(nproc) && make install

# Download and install Boost.NumPy for python-boost binding
RUN apt-get install -y git
RUN cd / && git clone https://github.com/ndarray/Boost.NumPy.git && cd Boost.NumPy && \
    mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && make install

# Argumens for where the different places from which we will get files from are
ARG boost_numpy_dir=/Boost.NumPy/build/lib/
ARG aruco_lib_dir=lib/src/aruco
ARG apriltag_processor_src_dir=ros/src/processing_node
ARG algo_dir=lib/src/duckietown_cslam/algo

# Create a directory to store the Python files and the ArUco library
RUN mkdir /apriltag_processor_node

# Download the ArUco library and fix the bug in it
COPY ${aruco_lib_dir}/aruco_bug_fixer.py /apriltag_processor_node/
RUN cd /apriltag_processor_node && wget https://sourceforge.net/projects/aruco/files/3.1.12/aruco-3.1.12.zip && \
    unzip aruco-3.1.12.zip && rm -r aruco-3.1.12.zip && python /apriltag_processor_node/aruco_bug_fixer.py

# Copy the AprilTagExtended message files
COPY /ros/src/duckietown_msgs /ros/src/duckietown_msgs
RUN /bin/bash -c "cd /ros/src && source /opt/ros/kinetic/setup.bash && catkin_init_workspace && cd .. && catkin_make"

# Copy and build the ArUco specific program files and configure the environment variables
COPY ${aruco_lib_dir}/aruco_caller.cpp /apriltag_processor_node/
COPY ${aruco_lib_dir}/CMakeLists.txt /apriltag_processor_node/
RUN mkdir /apriltag_processor_node/build && cd /apriltag_processor_node/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BOOST_NUMPY_SO=${boost_numpy_dir}/libboost_numpy.so .. && \
    make -j$(nproc) aruco_caller
COPY ${aruco_lib_dir}/config.yml /apriltag_processor_node/
COPY ${aruco_lib_dir}/aruco_lib_adapter.py /apriltag_processor_node/
ENV ACQ_ARUCO_CALLER_DIR /apriltag_processor_node/build
ENV ACQ_CONFIG_YML /apriltag_processor_node/config.yml

# Copy the Python files
COPY ${apriltag_processor_src_dir}/apriltag_processor_node.py /apriltag_processor_node
COPY ${algo_dir}/image_rectifier.py /apriltag_processor_node

# Copy the bash wrapper
COPY ${apriltag_processor_src_dir}/wrapper.sh /apriltag_processor_node
RUN chmod +x /apriltag_processor_node/wrapper.sh

# Set optional environment variables
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_TOPIC_RAW camera_node/image/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
ENV ACQ_TAG_SIZE 0.065
ENV ACQ_POSES_TOPIC poses
ENV ACQ_TEST_STREAM 0
ENV ACQ_BEAUTIFY 0
ENV ACQ_IMAGE_PROCESSORS 3
ENV ACQ_DETECTOR_TYPE DFC

# Start the processes
CMD /bin/bash -c "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${boost_numpy_dir} && \
                  source /opt/ros/kinetic/setup.bash && source /ros/devel/setup.bash && \
                  cd /apriltag_processor_node && ./wrapper.sh"