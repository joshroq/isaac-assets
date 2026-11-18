#!/bin/bash
# LiDAR SLAM + Nav2 + OAK-D depth obstacle cloud.
# SELF-CONTAINED base: nav.launch.py starts its OWN ugv_bringup (motors/LD19/odom/rf2o/TF),
# so do NOT also run start_bringup.sh (a second bringup double-opens /dev/ttyTHS1 and shreds
# the ESP32 feedback). You DO need start_oak.sh running for the depth cloud (/oak/stereo).
source ~/rover_nav/local_ros_env.sh
source ~/rover_nav/_check_single_stack.sh
P=~/ugv_ws/src/ugv_main/ugv_nav/param/amcl_dwa.yaml
B=~/rover_nav/param_backups/amcl_dwa.yaml.orig
[ -f "$B" ] || cp "$P" "$B"            # preserve pristine copy once
python3 ~/rover_nav/inject_depth.py "$B" "$P"   # regenerate augmented params from pristine
# depth image (16UC1 mm) -> XYZ cloud for the costmap
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/oak/stereo/image_raw -r camera_info:=/oak/stereo/camera_info \
  -r points:=/oak/points -p queue_size:=10 > ~/rover_nav/logs/depthcloud.log 2>&1 &
echo "depth cloud node PID $!"
exec ros2 launch ugv_nav nav.launch.py use_slam:=true use_rviz:=false use_localplan:=dwa
