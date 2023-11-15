#!/bin/bash
set -e

source $ROS_ROOT/install/setup.bash
source /root/ros2_pre_installed/install/setup.bash

cd /root/colcon_ws && colcon build --symlink-install --packages-select orbslam3

exec "$@"

#RUN cd /root && mkdir -p /root/colcon_ws/src 
#COPY ./orbslam3_ros2 /root/colcon_ws/src 
#RUN SITE_PACKAGES="/opt/ros/foxy/install/lib/python3.6/site-packages/" \
#    && ORB_SLAM3_ROOT_DIR="/root/ORB_SLAM3" \
#    && cd /root/colcon_ws/src/orbslam3_ros2 \
#    && sed -i "5s#.*#set(ENV{PYTHONPATH} $SITE_PACKAGES)#" CMakeLists.txt \
#    && sed -i "8s#.*#set(ORB_SLAM3_ROOT_DIR $ORB_SLAM3_ROOT_DIR)#" ./CMakeModules/FindORB_SLAM3.cmake \
#    && cd /root/colcon_ws && . $ROS_ROOT/install/local_setup.bash && . $ROS_ROOT/install/setup.bash \
#    && colcon build --symlink-install --packages-select orbslam3

#RUN cd ~/colcon_ws/src/orbslam3_ros2/vocabulary/ && tar -xvzf ORBvoc.txt.tar.gz 

