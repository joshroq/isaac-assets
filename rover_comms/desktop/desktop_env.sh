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

# IMPORTANT: never set the old initial-peers profile here -- that is what
# isolated Isaac.  Discovery Server needs no XML.
#
# Optional: for far/WAN operation you can restrict DDS to loopback + this
# machine's Tailscale interface (whitelist only, safe) so it doesn't advertise
# unreachable LAN locators to the rover.  Enable with DESKTOP_DDS_WHITELIST=1.
if [ "${DESKTOP_DDS_WHITELIST:-0}" = "1" ]; then
  _de_ts_ip="$(tailscale ip -4 2>/dev/null | head -1)"
  _de_cfg="${XDG_CONFIG_HOME:-$HOME/.config}/rover_comms"
  if [ -n "${_de_ts_ip}" ]; then
    mkdir -p "${_de_cfg}"
    _de_xml="${_de_cfg}/fastdds_tailscale.xml"
    cat > "${_de_xml}" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>ts_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>127.0.0.1</address>
          <address>${_de_ts_ip}</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="ts_whitelist" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>ts_udp</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XML
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_de_xml}"
    echo "[desktop_env] DDS interface whitelist ON (127.0.0.1 + ${_de_ts_ip})"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
    echo "[desktop_env] WARN: DDS whitelist requested but no Tailscale IP; skipping"
  fi
  unset _de_ts_ip _de_cfg _de_xml
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
fi

echo "[desktop_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[desktop_env] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER  SUPER_CLIENT=$ROS_SUPER_CLIENT"
echo "[desktop_env] (start the server with: rover_comms/desktop/discovery_server.sh)"
