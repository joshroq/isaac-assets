#!/bin/bash
set +e
echo "waiting for libORB_SLAM3.so..."
for i in $(seq 1 120); do [ -f ~/rover_nav/logs/orbslam_lib.done ] && break; sleep 30; done
LIB=~/slam_source/ORB_SLAM3/lib/libORB_SLAM3.so
if [ ! -f "$LIB" ]; then echo "LIB MISSING - aborting wrapper build"; exit 1; fi
echo "lib ready, building wrapper $(date)"
source /opt/ros/humble/setup.bash
cd ~/orbslam_ws
colcon build --packages-select orbslam3_ros2_wrapper \
  --cmake-args -DORB_SLAM3_DIR=$HOME/slam_source/ORB_SLAM3 -DCMAKE_BUILD_TYPE=Release \
  > ~/rover_nav/logs/wrapper_build.log 2>&1
echo "wrapper build RC=$?"
ls ~/orbslam_ws/install/orbslam3_ros2_wrapper/lib/orbslam3_ros2_wrapper/rgbd_odom_node 2>/dev/null && echo WRAPPER_OK || echo WRAPPER_FAIL
touch ~/rover_nav/logs/wrapper.done
