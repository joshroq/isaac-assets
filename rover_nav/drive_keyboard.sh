#!/bin/bash
# Keyboard teleop -> /cmd_vel. Needs start_bringup.sh running. KEEP THIS WINDOW FOCUSED.
source ~/rover_nav/local_ros_env.sh
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
