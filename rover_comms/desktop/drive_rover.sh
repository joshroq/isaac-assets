#!/usr/bin/env bash
# drive_rover.sh — teleop the PHYSICAL rover from the desktop over the link.
#
# Publishes to /cmd_vel_remote (NOT /cmd_vel) so the rover-side deadman watchdog
# interposes: if this link drops mid-drive, the watchdog stops the rover. The
# watchdog passes /cmd_vel_remote -> /cmd_vel on the rover.
#
#   rover_comms/desktop/drive_rover.sh
#
# SAFETY: first remote drives should be with the rover ON BLOCKS (wheels off the
# ground). Then floor-drive only with a live camera feed and an on-campus
# spotter. Use teleop_twist_keyboard's speed keys to keep it slow to start.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/desktop_env.sh"

REMOTE_TOPIC="${REMOTE_CMD_TOPIC:-/cmd_vel_remote}"

echo
echo "Driving the REAL rover via ${REMOTE_TOPIC} (watchdog-protected)."
echo "Keep it slow. Ctrl-C to stop (watchdog also stops the rover if the link drops)."
echo

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:="${REMOTE_TOPIC}"
