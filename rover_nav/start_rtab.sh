#!/bin/bash
# RTAB-Map RGB-D + LiDAR SLAM (headless): base + lidar + OAK-D + rtabmap -> /map + map->odom.
source ~/rover_nav/local_ros_env.sh
source ~/rover_nav/_oak_watcher.sh
exec ros2 launch ~/rover_nav/rtab_slam.launch.py
