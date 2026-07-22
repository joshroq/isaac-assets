# Desktop ROS 2 client environment for talking to the physical rover via the
# Fast DDS Discovery Server.  SOURCE this (do not execute):
#
#     source rover_comms/desktop/desktop_env.sh
#
# Desktop ROS nodes reach the local discovery server over loopback; the server
# then relays discovery to the rover.  This works for BOTH near (ugv AP) and
# far (Tailscale) because the server listens on 0.0.0.0 -- only the ROVER has
# to pick which desktop IP to point at.
#
# Run the server first (separate terminal):  rover_comms/desktop/discovery_server.sh

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Talk to the local discovery server (id 0) over loopback.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
# SUPER_CLIENT lets ros2 CLI (topic list / node list) see ALL participants the
# server knows about, not just the ones this process needs.
export ROS_SUPER_CLIENT=TRUE

# IMPORTANT: never set FASTRTPS_DEFAULT_PROFILES_FILE here -- the old
# initial-peers profile is what isolated Isaac.  Discovery Server needs no XML.
unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true

echo "[desktop_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[desktop_env] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER  SUPER_CLIENT=$ROS_SUPER_CLIENT"
echo "[desktop_env] (start the server with: rover_comms/desktop/discovery_server.sh)"
