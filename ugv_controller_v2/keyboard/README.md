# Keyboard control (over ROS 2)

Drive the baked-graph rover by keyboard through ROS 2. There is **no Isaac-side
keyboard handler** in this system — the rover is driven exclusively by
`geometry_msgs/Twist` on `/cmd_vel`, so keyboard teleop is just another `/cmd_vel`
publisher. Handing control between keyboard and Nav2 is therefore automatic:
whichever one is publishing `/cmd_vel` drives the wheels (don't run both at once).

## Run

```bash
# Terminal 1 — Isaac (baked rover + warehouse)
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — keyboard teleop (keep this terminal focused while driving)
ugv_controller_v2/keyboard/keyboard_drive.sh
```

`keyboard_drive.sh` runs the **real rover's own** upstream teleop node
(`ugv_ws/.../ugv_tools/keyboard_ctrl.py`) straight from source — no `colcon build`
needed, it only imports `rclpy` + `geometry_msgs`. The controls are therefore
**one-to-one** with the hardware. This is the exact message the node prints on
start (`ugv_tools/keyboard_ctrl.py` `msg`):

```
Control Your Car!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
```

Initial speeds match the source: linear `0.2`, angular `0.5` (`speed, turn` in
`main()`), publishing `twist.linear.x = speed·x`, `twist.angular.z = turn·th` on
`/cmd_vel` — the topic the baked `ROS2SubscribeTwist` graph consumes.

## Why a shell script and not a launch file

Keyboard teleop needs a raw TTY to read single keypresses. `ros2 launch` captures
a node's stdio, so the teleop would get no keys. Run the wrapper in its own
foreground terminal instead.

## Alternative: standard teleop

If you prefer the ROS 2 standard teleop (identical effect — publishes `/cmd_vel`):

```bash
source /opt/ros/humble/setup.bash
source ~/.local/bin/ros2ddsenv
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Verify it reaches Isaac

```bash
source /opt/ros/humble/setup.bash && source ~/.local/bin/ros2ddsenv
ros2 topic echo /cmd_vel        # should show Twist while you press keys
ros2 topic echo /odom --once    # should advance after you drive forward
```
