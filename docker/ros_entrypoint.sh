#!/bin/bash
set -e

. $ROS_ROOT/install/setup.bash
#source /root/ros2_pre_installed/install/setup.bash

echo $TARGET_BUILD

if [[ "$TARGET_BUILD" == "dev" ]]
then
  export ORB_SLAM3_DIR="/root/ORB_SLAM3-dev"
  
  cd /root/ORB_SLAM3-dev && ./build.sh
  cd /root/ORB_SLAM3-dev/Thirdparty/Sophus/build && make install
  rm -rf /root/ORB_SLAM3
else
  export ORB_SLAM3_DIR="/root/ORB_SLAM3"
fi

cd /root/colcon_ws/src/orbslam3_ros2/vocabulary/ && tar -xvzf ORBvoc.txt.tar.gz 
cd /root/colcon_ws && colcon build --symlink-install

. /root/colcon_ws/install/setup.bash

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

