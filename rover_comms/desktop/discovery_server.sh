#!/usr/bin/env bash
# Fast DDS Discovery Server for desktop <-> rover ROS 2 comms.
#
# WHY THIS EXISTS
#   ROS 2 (Fast DDS) default discovery relies on UDP multicast. The WiFi AP
#   drops multicast (confirmed by packet capture: the rover's SPDP packets
#   leave its wlP1p1s0 but never reach the desktop), so participants never
#   find each other even though plain unicast pings succeed.
#
#   A Discovery Server replaces multicast discovery with a single unicast
#   rendezvous point. It does NOT override any locator lists, so unlike the
#   earlier fastdds_rover.xml attempt it will NOT break local Isaac discovery.
#
# This server runs on the DESKTOP and listens on ALL interfaces (0.0.0.0),
# so the same process serves both:
#   - near clients over the ugv AP LAN   (10.168.168.240:11811)
#   - far  clients over Tailscale        (100.107.253.10:11811)
#
# Clients (desktop nodes AND the rover) reach it via ROS_DISCOVERY_SERVER.
# See ../rover_comms/README.md.

set -eo pipefail   # not -u: ROS setup.bash references unbound vars

SERVER_ID="${DISCOVERY_SERVER_ID:-0}"
LISTEN_ADDR="${DISCOVERY_LISTEN_ADDR:-0.0.0.0}"
PORT="${DISCOVERY_SERVER_PORT:-11811}"

source /opt/ros/humble/setup.bash

echo "[discovery_server] starting Fast DDS Discovery Server"
echo "[discovery_server]   id=${SERVER_ID}  listen=${LISTEN_ADDR}:${PORT}"
echo "[discovery_server]   desktop Tailscale IP : $(tailscale ip -4 2>/dev/null | head -1 || echo '<tailscale down>')"
echo "[discovery_server]   desktop LAN IPs      : $(ip -4 -brief addr | awk '!/^(lo|docker|br-|virbr|veth|tailscale)/{print $3}' | paste -sd' ')"
echo "[discovery_server] clients must set ROS_DISCOVERY_SERVER to one of the above IPs :${PORT}"
echo "[discovery_server] Ctrl-C to stop."

# -i server id (must match position 0 in ROS_DISCOVERY_SERVER on clients)
# -l listen address, -p listen port
exec fastdds discovery -i "${SERVER_ID}" -l "${LISTEN_ADDR}" -p "${PORT}"
