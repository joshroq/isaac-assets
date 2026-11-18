#!/usr/bin/env bash
# view_rover.sh — LIVE-visualize the PHYSICAL rover on THIS desktop in RViz.
#
# The desktop twin of `navlaunch -v2 -<stack>`: same per-stack RViz view, but bound
# to the rover's real topics. The nav stack itself runs ON THE ROVER; this is viz +
# goal-setting only. See rover_comms/desktop/VISUALIZE_ROVER.md for the full recipe.
#
#   rover_comms/desktop/view_rover.sh [lidar|depth|rgb|orbslam3|cuvslam] [near|far]
#       default stack = lidar,  default mode = near
#
# TRANSPORT: the rover runs NATIVE MULTICAST (no discovery server), so its own Nav2/SLAM
# bringup is reliable. The Wi-Fi AP drops multicast between machines, so THIS desktop
# reaches the rover with a Fast DDS initial-peers profile (generated below) that unicasts
# discovery to the rover's metatraffic ports. That's entirely desktop-side — it never
# touches the rover's discovery. Override the rover IP with ROVER_IP=<ip> if yours differs.
#
# PREREQUISITES:
#   1. rover (ssh):  ~/rover_nav/rovernav <stack>     (one command; see rovernav --help)
#   2. desktop    :  this script
#
# Set a goal once RViz is up: click "2D Nav Goal" (publishes /goal_pose to the rover's
# Nav2 over the link). RViz-less fallback over ssh: rover_nav/send_goal.py.
set -eo pipefail   # not -u: ROS setup.bash references unbound vars

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# --- args: stack, optional mode (near|far), then passthrough ros2 launch args -------
raw="${1:-lidar}"; raw="${raw#-}"
case "$raw" in
  lidar|slam)             stack="lidar" ;;
  depth|icp)              stack="depth" ;;
  rgb|rtab|rtabmap)       stack="rgb" ;;
  orbslam3|orbslam|orb)   stack="orbslam3" ;;
  cuvslam|nvblox)         stack="cuvslam" ;;
  *) echo "unknown stack '$1' (want: lidar|depth|rgb|orbslam3|cuvslam)"; exit 1 ;;
esac
shift || true
mode="${ROVER_VIEW_MODE:-near}"
case "${1:-}" in near|far) mode="$1"; shift ;; esac

# Rover IPs (override via env or ~/.config/rover_comms/rover_ips.env).
_ri="${XDG_CONFIG_HOME:-$HOME/.config}/rover_comms/rover_ips.env"
[ -f "$_ri" ] && . "$_ri"
: "${ROVER_LAN_IP:=10.168.168.115}"        # rover on the ugv AP LAN (near)
: "${ROVER_TAILSCALE_IP:=100.89.177.82}"   # rover on Tailscale (far)
: "${ROVER_PEER_PARTICIPANTS:=40}"         # how many rover participant IDs to probe
if [ -z "${ROVER_IP:-}" ]; then
  case "$mode" in
    near) ROVER_IP="$ROVER_LAN_IP" ;;
    far)  ROVER_IP="$ROVER_TAILSCALE_IP" ;;
  esac
fi

# --- desktop reaches the rover via a Fast DDS initial-peers profile -----------------
# The rover uses multicast, which the AP drops between machines. So we unicast SPDP
# discovery straight to the rover's metatraffic ports. In domain 0 the metatraffic
# unicast port for participant N is 7410 + 2*N, and the rover runs ~12-15 participants
# (each nav node is one), so we list a generous range — otherwise nav2_container (a
# high participant ID) is never discovered and /map, costmaps and /goal_pose go missing.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
_cfg="${XDG_CONFIG_HOME:-$HOME/.config}/rover_comms"; mkdir -p "$_cfg"
_peers_xml="${_cfg}/view_rover_peers.xml"
{
  echo '<?xml version="1.0" encoding="UTF-8"?>'
  echo '<dds><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
  echo '  <participant profile_name="rover_peer" is_default_profile="true"><rtps><builtin><initialPeersList>'
  _n=0
  while [ "$_n" -lt "$ROVER_PEER_PARTICIPANTS" ]; do
    echo "    <locator><udpv4><address>${ROVER_IP}</address><port>$((7410 + 2*_n))</port></udpv4></locator>"
    _n=$((_n + 1))
  done
  echo '  </initialPeersList></builtin></rtps></participant>'
  echo '</profiles></dds>'
} > "$_peers_xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$_peers_xml"

# The ROS 2 daemon caches the discovery config; bounce it so the preflight below and any
# CLI in this terminal re-read the profile and see the rover (data can flow before the
# daemon's own listing catches up).
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo
echo "=== Connecting to the rover (${mode}) via initial-peers -> ${ROVER_IP} ==="
sleep 3   # let the freshly-started daemon populate the graph before the preflight
if ! ping -c1 -W2 "$ROVER_IP" >/dev/null 2>&1; then
  echo "WARN: cannot ping ${ROVER_IP}. Wrong mode/IP, or rover offline?"
  echo "      near expects the ugv AP LAN; far expects Tailscale up on both ends."
fi

# Preflight the topics this stack's RViz needs. A dead /scan (or camera, for visual
# stacks) means RViz will sit empty -- fail early with a hint instead.
need_topics="/scan /tf"
case "$stack" in
  rgb|orbslam3) need_topics="$need_topics /oak/rgb/image_raw" ;;
  depth)        need_topics="$need_topics /oak/points" ;;
esac
live_topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"
if [ -z "${live_topics// }" ]; then
  echo "WARN: no topics visible over the link yet. Is the rover up?  (ssh rover:"
  echo "      ~/rover_nav/rovernav ${stack})   — and is ${ROVER_IP} reachable for '${mode}'?"
else
  missing=""
  for t in $need_topics; do echo "$live_topics" | grep -qx "$t" || missing="$missing $t"; done
  if [ -n "${missing// }" ]; then
    echo "WARN: link up but these topics not present yet:$missing (starting anyway)"
  else
    echo "  link OK — required topics visible:$need_topics"
  fi
fi

echo
echo "=== RViz: live view of the rover ($stack) ==="
echo "  Set a goal: click '2D Nav Goal' in RViz (-> /goal_pose to the rover's Nav2)."
exec ros2 launch "${REPO_ROOT}/ugv_controller_v2/nav_systems/view_rover.launch.py" \
  stack:="$stack" "$@"
