#!/bin/bash
# LiDAR SLAM (slam_toolbox) + Nav2 (2D lidar only). Needs start_bringup.sh running.
source ~/rover_nav/local_ros_env.sh
B=~/rover_nav/param_backups/amcl_dwa.yaml.orig
[ -f "$B" ] && cp "$B" ~/ugv_ws/src/ugv_main/ugv_nav/param/amcl_dwa.yaml   # pure-lidar params
exec ros2 launch ugv_nav nav.launch.py use_slam:=true use_rviz:=false use_localplan:=dwa
