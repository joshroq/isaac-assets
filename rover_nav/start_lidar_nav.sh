#!/bin/bash
# LiDAR SLAM (slam_toolbox) + Nav2 — live mapping, 2D lidar only.
# SELF-CONTAINED: this launch starts the base bringup (motors / LD19 / wheel odom /
# rf2o / TF) ITSELF. Do NOT also run start_bringup.sh — a second ugv_bringup opens
# /dev/ttyTHS1 twice and shreds the base feedback ("multiple access on port").
# slam_toolbox builds the map as you drive; send goals with send_goal.py (or teleop).
source ~/rover_nav/local_ros_env.sh
source ~/rover_nav/_check_single_stack.sh
exec ros2 launch ugv_nav slam_nav.launch.py slam:=True use_rviz:=false
