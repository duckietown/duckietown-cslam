FROM ros:kinetic-ros-core

MAINTAINER Amaury Camus <camusam@ethz.ch>

# Install rospy
RUN apt-get update && apt-get install -y --allow-unauthenticated --no-install-recommends ros-kinetic-rospy python-qt4 && apt-get clean


# Setup the ros-cslam
RUN mkdir -p /catkin_ws/src/cslam_diagnostics
COPY ros/src/cslam_diagnostics /catkin_ws/src/cslam_diagnostics
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin_ws/; catkin_make"
RUN chmod +x /catkin_ws/src/cslam_diagnostics/*.sh
ENV DISPLAY $DISPLAY
ENV QT_X11_NO_MITSHM 1

CMD /bin/bash -c "cd /catkin_ws; source /catkin_ws/devel/setup.bash; /catkin_ws/src/cslam_diagnostics/wrapper_diagnostics.sh"
