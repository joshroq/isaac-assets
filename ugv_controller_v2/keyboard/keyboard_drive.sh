#!/usr/bin/env bash
# Drive the baked-graph Isaac rover from the KEYBOARD, over ROS 2.
#
# This runs the REAL rover's upstream keyboard teleop
# (ugv_ws/.../ugv_tools/keyboard_ctrl.py) directly from source. It publishes
# geometry_msgs/Twist on /cmd_vel — exactly the topic the baked ROS2SubscribeTwist
# graph in jetson_rover_v2.usd consumes to drive the wheels. So the same node
# that drives the real WaveShare rover drives the sim, with zero sim-specific code.
#
# Keyboard teleop needs a real terminal (raw TTY), which `ros2 launch` cannot
# provide — so this is a plain foreground shell wrapper, not a launch file. Run it
# in its OWN terminal (separate from the Isaac terminal) and keep that terminal
# focused while driving.
#
# USAGE
#   ugv_controller_v2/keyboard/keyboard_drive.sh
#
# Controls (printed on start): i/k/j/l/u/o/m/,/. to move, q/z/w/x/e/c to change
# speed, space or k to stop, CTRL-C to quit.
set -euo pipefail

REPO_ROOT="/home/joshroq/projects/isaac-assets"
KEYBOARD_CTRL="$REPO_ROOT/ugv_ws/src/ugv_main/ugv_tools/ugv_tools/keyboard_ctrl.py"

if [[ ! -f "$KEYBOARD_CTRL" ]]; then
  echo "[keyboard_drive] Upstream keyboard node not found: $KEYBOARD_CTRL" >&2
  exit 1
fi

echo "[keyboard_drive] Sourcing ROS 2 Humble..."
# ROS 2's setup scripts read unset vars (e.g. AMENT_TRACE_SETUP_FILES), which
# would trip this script's `set -u` (nounset) and abort before teleop starts.
# Relax nounset just around the ROS sourcing, then restore it.
set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

# Match the DDS settings Isaac uses so both sides discover each other.
if [[ -f "$HOME/.local/bin/ros2ddsenv" ]]; then
  # shellcheck source=/dev/null
  source "$HOME/.local/bin/ros2ddsenv"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
fi
set -u  # restore nounset now that ROS setup scripts are sourced

echo "[keyboard_drive] Publishing geometry_msgs/Twist on /cmd_vel"
echo "[keyboard_drive]   ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "[keyboard_drive] Keep THIS terminal focused while driving. CTRL-C to quit."
echo

# run_path executes the module top-level (defining main + key bindings) without
# triggering any __main__ guard, then we call main() exactly once. This drives
# the node straight from the source file with no colcon build required.
exec python3 -c "import runpy, sys; sys.argv=['keyboard_ctrl']; g=runpy.run_path('$KEYBOARD_CTRL'); g['main']()"
