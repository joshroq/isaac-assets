#!/bin/bash
# start_comms_server.sh — DEPRECATED / no-op.
#
# The rover no longer runs a Fast DDS Discovery Server. It broadcasts to the desktop
# over NATIVE MULTICAST (routing the rover's own nodes through a discovery server broke
# intra-rover service discovery — Nav2/SLAM bringup hung on "Waiting for service
# map_saver/get_state"). The desktop crosses the multicast-dropping Wi-Fi AP on its own
# side with a Fast DDS initial-peers profile (rover_comms/desktop/view_rover.sh builds it).
#
# So there is nothing to start here. Just bring up a stack and view it:
#     ~/rover_nav/rovernav lidar            # (or depth)
#     # desktop:  rover_comms/desktop/view_rover.sh lidar
echo "[comms_server] Not needed anymore — the rover uses native multicast."
echo "[comms_server] Bring up a stack with:  ~/rover_nav/rovernav lidar   (or depth)"
echo "[comms_server] Then on the desktop:    rover_comms/desktop/view_rover.sh lidar"
# Clean up any leftover server from the old design so it can't interfere.
pkill -f "fast-discovery-server|fastdds discovery" 2>/dev/null && echo "[comms_server] (stopped a leftover discovery server)"
exit 0
