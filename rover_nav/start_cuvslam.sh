#!/bin/bash
# cuVSLAM stereo VIO -> map->odom. Run start_bringup.sh FIRST (base+lidar+TF), then this.
source ~/rover_nav/local_ros_env.sh
source ~/rover_nav/_oak_watcher.sh
# OAK in stereo mode
ros2 launch depthai_ros_driver camera.launch.py params_file:=$HOME/rover_nav/oak_stereo.yaml parent_frame:=3d_camera_link > ~/rover_nav/logs/oakstereo.log 2>&1 &
sleep 22
# rectify raw stereo -> rect (cuVSLAM needs rectified)
ros2 run image_proc rectify_node --ros-args -r image:=/oak/left/image_raw  -r camera_info:=/oak/left/camera_info  -r image_rect:=/oak/left/image_rect  > /tmp/rectL.log 2>&1 &
ros2 run image_proc rectify_node --ros-args -r image:=/oak/right/image_raw -r camera_info:=/oak/right/camera_info -r image_rect:=/oak/right/image_rect > /tmp/rectR.log 2>&1 &
sleep 4
exec ros2 launch ~/rover_nav/cuvslam.launch.py
