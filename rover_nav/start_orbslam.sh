#!/bin/bash
# ORB-SLAM3 RGB-D visual SLAM -> map->odom. Run start_bringup.sh FIRST, then this.
# Drive through textured scenery to initialize tracking (holds map->odom until tracking OK).
source ~/rover_nav/local_ros_env.sh
source ~/orbslam_ws/install/setup.bash
source ~/rover_nav/_oak_watcher.sh
ros2 launch ugv_vision oak_d_lite.launch.py > ~/rover_nav/logs/oak.log 2>&1 &   # OAK RGBD
sleep 22
exec ros2 run orbslam3_ros2_wrapper rgbd_odom_node --ros-args \
  -p vocabulary:=$HOME/slam_source/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  -p rgb_topic:=/oak/rgb/image_rect \
  -p depth_topic:=/oak/stereo/image_raw \
  -p camera_info_topic:=/oak/rgb/camera_info \
  -p camera_optical_frame:=oak_rgb_camera_optical_frame \
  -p map_frame:=map -p odom_frame:=odom -p base_frame:=base_footprint \
  -p publish_tf:=true -p publish_pose:=false -p use_viewer:=false \
  -p camera_fps:=8 -p orb_features:=1500
