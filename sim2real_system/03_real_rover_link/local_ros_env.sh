# Local-only ROS 2 env for the rover. Everything runs on the Jetson; no desktop link.
source /opt/ros/humble/setup.bash
[ -f "$HOME/ugv_ws/install/setup.bash" ] && source "$HOME/ugv_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# --- desktop-visibility toggle -------------------------------------------------
# By DEFAULT the rover is local-only (ROS_LOCALHOST_ONLY=1): every nav stack runs
# on the Jetson and nothing leaves it. To also stream the live graph (/scan, /map,
# /tf, costmaps, /plan, /oak/*) to the desktop RViz viewer (view_rover.sh) and let
# the desktop send /goal_pose back, set ROVER_NAV_COMMS before sourcing this:
#     export ROVER_NAV_COMMS=1     # (near|far|on all mean the same on the rover)
#
# HOW: comms mode just turns OFF localhost-only so the rover's nodes announce their
# LAN locators — the rover keeps using NATIVE MULTICAST discovery (no discovery
# server, no XML), exactly like the sim / like standalone local nav. That matters:
# routing the rover's own nodes through a Discovery Server (desktop- OR rover-hosted)
# breaks intra-rover *service* discovery under load — the Nav2/SLAM bringup then hangs
# forever on "Waiting for service map_saver/get_state" (and earlier, lifecycle bond
# timeouts). Native multicast keeps bringup reliable. The Wi-Fi AP drops multicast
# BETWEEN machines, so the DESKTOP crosses that gap on its own side, with a Fast DDS
# initial-peers profile pointing at the rover (view_rover.sh builds it) — that never
# touches the rover's discovery. Unset => unchanged local-only behavior.
case "${ROVER_NAV_COMMS:-}" in
  ""|0|local|off|no|false)
          export ROS_LOCALHOST_ONLY=1 ;;
  *)      # any truthy value (1|on|near|far|yes|...) => broadcast on the LAN (multicast)
          export ROS_LOCALHOST_ONLY=0 ;;
esac
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_SUPER_CLIENT
if [ "${ROS_LOCALHOST_ONLY}" = "0" ]; then
  echo "[local_ros_env] comms mode ON -> native multicast on the LAN (LOCALHOST_ONLY=0)"
  echo "[local_ros_env]   view on the desktop with:  rover_comms/desktop/view_rover.sh <stack>"
fi
# --- Waveshare model selection (copied from .bashrc) ---
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld19
