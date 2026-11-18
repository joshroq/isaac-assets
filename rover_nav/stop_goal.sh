#!/bin/bash
# PANIC STOP for Nav2: cancel the current goal and halt the rover in place.
# Use when you sent a goal it cannot reach / it is heading somewhere bad.
# It publishes the rover's CURRENT pose as the goal, so Nav2 sees it has "arrived"
# and the controller stops commanding /cmd_vel; then a short zero burst as insurance
# (the base has NO cmd_vel deadman). Leaves the nav stack running so you can re-goal.
source ~/rover_nav/local_ros_env.sh
python3 ~/rover_nav/send_goal.py --forward 0 --seconds 1.5
python3 -c "import rclpy, time
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node('stop_goal_zero')
p = n.create_publisher(Twist, '/cmd_vel', 10)
end = time.time() + 1.0
while time.time() < end:
    p.publish(Twist()); time.sleep(0.03)
print('stopped: goal cancelled + wheels zeroed'); n.destroy_node(); rclpy.shutdown()"
