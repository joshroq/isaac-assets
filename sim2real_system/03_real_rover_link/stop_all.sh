#!/bin/bash
# Stop every rover ROS process started by these scripts (incl. detached node children).
for pat in 'ros2 launch' 'nav.launch' point_cloud_xyz oak_d_lite component_container \
           teleop_twist_keyboard ugv_bringup ugv_driver ldlidar_node base_node \
           rf2o_laser_odometry robot_state_publisher slam_toolbox 'controller_server' \
           'planner_server' 'bt_navigator' 'nav2_' async_slam_toolbox joint_state_publisher robot_pose_publisher; do
  pkill -f "$pat" 2>/dev/null
done
sleep 2; echo "remaining ros nodes:"; pgrep -af 'ldlidar|nav2|slam_toolbox|component_container|ugv_|oak' | grep -v pgrep | head
