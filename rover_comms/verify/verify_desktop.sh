#!/usr/bin/env bash
# Desktop-side sanity check for the rover comms link.
# Run AFTER: (1) discovery_server.sh is running, (2) you sourced desktop_env.sh,
# and (3) the rover driver is up with rover_env.sh sourced.
set -o pipefail   # not -u: ROS setup.bash references unbound vars
source /opt/ros/humble/setup.bash

pass(){ echo "  [ OK ] $*"; }
fail(){ echo "  [FAIL] $*"; }

echo "== 1. Environment =="
[ "${ROS_DOMAIN_ID:-}" = "0" ] && pass "ROS_DOMAIN_ID=0" || fail "ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-unset}' (want 0)"
[ "${RMW_IMPLEMENTATION:-}" = "rmw_fastrtps_cpp" ] && pass "RMW=rmw_fastrtps_cpp" || fail "RMW='${RMW_IMPLEMENTATION:-unset}'"
[ -n "${ROS_DISCOVERY_SERVER:-}" ] && pass "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER" || fail "ROS_DISCOVERY_SERVER unset -- source desktop_env.sh"
[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && pass "no FASTRTPS_DEFAULT_PROFILES_FILE (good)" || fail "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE (unset it!)"

echo "== 2. Discovery server process =="
# the CLI 'fastdds discovery' spawns a child named 'fast-discovery-server'
if pgrep -af "fast-discovery-server|fastdds discovery" >/dev/null; then pass "discovery server running"; else fail "no discovery server process -- start discovery_server.sh"; fi
if ss -lun 2>/dev/null | grep -q ":11811"; then pass "UDP :11811 is listening"; else fail "nothing listening on UDP :11811"; fi

echo "== 3. Reachability to rover (set ROVER_IP env to test) =="
ROVER_IP="${ROVER_IP:-}"
if [ -n "$ROVER_IP" ]; then
  if ping -c1 -W2 "$ROVER_IP" >/dev/null 2>&1; then pass "ping $ROVER_IP"; else fail "cannot ping $ROVER_IP"; fi
else
  echo "  [skip] set ROVER_IP=<rover ip> to test ping"
fi

echo "== 4. ROS graph (needs rover driver up) =="
echo "  nodes:";  timeout 5 ros2 node list  2>/dev/null | sed 's/^/    /' || echo "    (none)"
echo "  topics:"; timeout 5 ros2 topic list 2>/dev/null | grep -iE 'cmd_vel|scan|odom|imu|tf' | sed 's/^/    /' || echo "    (none)"
echo
echo "Expected once the rover is connected: the rover's /cmd_vel subscriber and"
echo "its real /scan,/odom,/imu,/tf publishers appear above."
echo "Full check:  ros2 topic info /cmd_vel --verbose   (Subscription count >= 1)"
