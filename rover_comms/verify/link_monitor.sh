#!/usr/bin/env bash
# link_monitor.sh — desktop-side health check for the desktop<->rover link.
# Reports Tailscale path, ping RTT, and whether the rover's ROS graph is visible.
# Handy to leave running in a corner during a remote session.
#
#   rover_comms/verify/link_monitor.sh <rover-ip-or-host> [interval_s]
#   ROVER=100.x.y.z rover_comms/verify/link_monitor.sh
set -o pipefail

ROVER="${1:-${ROVER:-}}"
INTERVAL="${2:-5}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${ROVER}" ]; then
  echo "usage: link_monitor.sh <rover-ip-or-host> [interval_s]   (or set ROVER=...)" >&2
  echo "tip: 'tailscale status' lists rover peers." >&2
  exit 1
fi

# make sure we can introspect the discovery-server graph
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../desktop/desktop_env.sh" >/dev/null 2>&1 || source /opt/ros/humble/setup.bash

echo "Monitoring link to ${ROVER} every ${INTERVAL}s (Ctrl-C to stop)."
echo "time            tailscale        rtt         rover_ros_nodes"
while true; do
  ts="$(tailscale status 2>/dev/null | awk -v r="${ROVER}" '$1==r || $2==r {print ($5=="-"?"direct":$5); found=1} END{if(!found)print "?"}' | head -1)"
  rtt="$(ping -c1 -W2 "${ROVER}" 2>/dev/null | sed -n 's/.*time=\([0-9.]*\) ms/\1ms/p')"
  [ -z "${rtt}" ] && rtt="DOWN"
  nodes="$(timeout 4 ros2 node list 2>/dev/null | grep -cE '.' )"
  printf '%s   %-14s  %-9s   %s nodes visible\n' "$(date +%H:%M:%S)" "${ts:-?}" "${rtt}" "${nodes:-0}"
  sleep "${INTERVAL}"
done
