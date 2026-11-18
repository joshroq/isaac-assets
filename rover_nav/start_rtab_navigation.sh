#!/bin/bash
# Nav2 navigation servers for RTAB (rtabmap already supplies /map + map->odom).
source ~/rover_nav/local_ros_env.sh
exec ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:=$HOME/ugv_ws/src/ugv_main/ugv_nav/param/rtabmap_dwa.yaml
