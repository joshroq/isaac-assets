#!/bin/bash
# Base motors + IMU + wheel odom + LD19 lidar + rf2o + TF. Run FIRST, keep running.
source ~/rover_nav/local_ros_env.sh
source ~/rover_nav/_check_single_stack.sh
exec ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=false
