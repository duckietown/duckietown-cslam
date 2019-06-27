FROM ros:kinetic-ros-core

MAINTAINER Aleksandar Petrov <alpetrov@ethz.ch>

# Install OpenCV and CvBridge
RUN apt-get update && apt-get install -f -y --allow-unauthenticated python-opencv ros-kinetic-cv-bridge ros-kinetic-rospy python-pip && apt-get clean
RUN pip install pathos multiprocessing-logging


# OPTIONALLY YOU CAN ALSO SPECIFY + DEFAULT VALUES
ENV OUTPUT_FRAMERATE 12
ENV MIN_SHOT_LENGTH 12
ENV ATMSGS_BAG /bags/processed.bag
ENV VIDEO_BAGS /bags
ENV POSES_TOPIC /poses_acquisition/poses
ENV VIDEO_TOPIC /camera_node/image/compressed
ENV OUTPUT_FILE /bags/overheadVideo.mp4
ENV TRACKED_AT_ID 410
ENV CAMERA_RESOLUTION_HEIGHT 1296
ENV CAMERA_RESOLUTION_WIDTH 972
ENV TITLE_CARD 1


# Setup the duckietown_msgs
RUN mkdir -p /catkin-ws/src
COPY ros/src/duckietown_msgs /catkin-ws/src/duckietown_msgs
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin-ws/; catkin_make"

# Get the actual script in
COPY WesQuackerson.py /WesQuackerson.py
COPY titlecard.png /titlecard.png

# Start the processes
CMD /bin/bash -c "cd / ; source /catkin-ws/devel/setup.bash; python WesQuackerson.py"