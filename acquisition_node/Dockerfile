FROM ros:kinetic-ros-core

MAINTAINER Aleksandar Petrov <alpetrov@ethz.ch>

# YOU ALWAYS NEED TO SPECIFY:
# ACQ_ROS_MASTER_URI_DEVICE
# ACQ_ROS_MASTER_URI_DEVICE_IP
# ACQ_ROS_MASTER_URI_SERVER
# ACQ_ROS_MASTER_URI_SERVER_IP
# ACQ_DEVICE_NAME # should match the topic, e.g. watchtower10


# OPTIONALLY YOU CAN ALSO SPECIFY + DEFAULT VALUES
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_UPDATE_RATE 2
ENV ACQ_SOCKET_HOST 127.0.0.1
ENV ACQ_SOCKET_PORT 65432
ENV ACQ_TOPIC_RAW camera_node/image/raw
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
ENV ACQ_TOPIC_VELOCITY_TO_POSE velocity_to_pose_node/pose
ENV ACQ_TAG_SIZE 0.065
ENV ACQ_POSES_TOPIC poses
ENV ACQ_ODOMETRY_TOPIC odometry
ENV ACQ_TEST_STREAM 1
ENV ACQ_BEAUTIFY 1

# Create a directory to store the Python files and the April tag library
RUN mkdir /acquisition_node

# Install OpenCV and CvBridge
RUN apt update
RUN apt install -y python-opencv
RUN apt install -y ros-kinetic-cv-bridge
RUN apt install -y ros-kinetic-rospy

# Copy and install the April tag library and configure the environment variables for it
RUN mkdir /acquisition_node/apriltag
COPY apriltag /acquisition_node/apriltag
RUN cd /acquisition_node/apriltag ; mkdir build ; cd build ; cmake .. -DCMAKE_BUILD_TYPE=Release ; make -j4
ENV ACQ_APRILTAG_LIB /acquisition_node/apriltag/python
ENV ACQ_APRILTAG_SO /acquisition_node/apriltag/build/lib

# Setup the duckietown_msgs
RUN mkdir -p /catkin-ws/src
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin-ws/; catkin_make"
COPY duckietown_msgs /catkin-ws/src/duckietown_msgs
RUN /bin/bash -c "source /catkin-ws/devel/setup.bash; cd /catkin-ws/; catkin_make"

# Copy the Python files
COPY deviceSideProcessor.py /acquisition_node
COPY deviceSideSubscriber.py /acquisition_node
COPY image_rectifier.py /acquisition_node
COPY interprocess_communication.py /acquisition_node
COPY serverSidePublisher.py /acquisition_node

# Copy the bash wrappers (needed to configure the right environment
# (ROS_MASTER_URI) and to make sure the python modules
# are restarted if they exit for whatever reason)
COPY startDeviceSide.sh /acquisition_node
COPY startServerSide.sh /acquisition_node
COPY wrapper.sh /acquisition_node
RUN chmod +x /acquisition_node/*.sh

# Start the processes
CMD /bin/bash -c "cd /acquisition_node; source /catkin-ws/devel/setup.bash; /acquisition_node/wrapper.sh"
