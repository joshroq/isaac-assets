# rover_env.sh — single source of truth for the rover's ROS 2 + Discovery Server
# environment.  SOURCE this (do not execute):
#
#     source rover_env.sh            # use persisted/default mode
#     source rover_env.sh near       # force near mode (ugv AP LAN)
#     source rover_env.sh far        # force far  mode (Tailscale)
#
# The rover is a CLIENT of the desktop's Fast DDS Discovery Server. The ONLY
# thing that changes between near and far is which desktop IP it points at.
#
#   near : desktop is on the rover's "ugv" AP at 10.168.168.240 (no internet
#          needed — battery/field demos)
#   far  : both machines on Tailscale; desktop is 100.107.253.10 (rover MUST
#          have an internet uplink — see TAILSCALE.md; AP-only won't work)
#
# Mode is chosen from, in order:
#   1. the first argument to this script (near|far)
#   2. $ROVER_COMMS_MODE
#   3. the file  ~/.config/rover_comms/mode      (set via `rover-mode near|far`)
#   4. default: far
#
# Desktop IPs can be overridden without editing this file by either:
#   - exporting DESKTOP_TAILSCALE_IP / DESKTOP_LAN_IP before sourcing, or
#   - putting them in  ~/.config/rover_comms/desktop_ips.env

# ---- defaults (override via env or ~/.config/rover_comms/desktop_ips.env) ----
: "${DESKTOP_TAILSCALE_IP:=100.107.253.10}"   # desktop `tailscale ip -4`
: "${DESKTOP_LAN_IP:=10.168.168.240}"         # desktop IP on the rover's ugv AP
: "${DISCOVERY_SERVER_PORT:=11811}"
# -----------------------------------------------------------------------------

_rc_cfg_dir="${XDG_CONFIG_HOME:-$HOME/.config}/rover_comms"
# paths.env (written by the installer) sets UGV_WS etc. for wherever the repo
# was cloned; desktop_ips.env optionally overrides the desktop IPs.
[ -f "${_rc_cfg_dir}/paths.env" ]       && . "${_rc_cfg_dir}/paths.env"
[ -f "${_rc_cfg_dir}/desktop_ips.env" ] && . "${_rc_cfg_dir}/desktop_ips.env"

# ROS 2 middleware baseline — MUST match the desktop and Isaac exactly.
if [ -f /opt/ros/humble/setup.bash ]; then
  # ROS setup.bash trips `set -u`; source with u disabled if it is on.
  case "$-" in *u*) _rc_had_u=1; set +u;; *) _rc_had_u=0;; esac
  . /opt/ros/humble/setup.bash
  [ "${_rc_had_u}" = 1 ] && set -u
  unset _rc_had_u
fi

export ROS_DOMAIN_ID=0                       # NOTE: desktop uses 0, not 42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Source the rover's built workspace if present (so ugv_bringup etc. are found).
for _rc_ws in "${UGV_WS:-$HOME/isaac-assets/ugv_ws}/install/setup.bash" \
              "$HOME/ugv_ws/install/setup.bash"; do
  if [ -f "${_rc_ws}" ]; then
    case "$-" in *u*) _rc_had_u=1; set +u;; *) _rc_had_u=0;; esac
    . "${_rc_ws}"
    [ "${_rc_had_u}" = 1 ] && set -u
    unset _rc_had_u
    break
  fi
done
unset _rc_ws

# ---- pick mode ---------------------------------------------------------------
_rc_mode="${1:-${ROVER_COMMS_MODE:-}}"
if [ -z "${_rc_mode}" ] && [ -f "${_rc_cfg_dir}/mode" ]; then
  _rc_mode="$(tr -d '[:space:]' < "${_rc_cfg_dir}/mode")"
fi
_rc_mode="${_rc_mode:-far}"

case "${_rc_mode}" in
  near) _rc_desktop_ip="${DESKTOP_LAN_IP}" ;;
  far)  _rc_desktop_ip="${DESKTOP_TAILSCALE_IP}" ;;
  *)
    echo "[rover_env] invalid mode '${_rc_mode}' (want near|far)" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac

export ROVER_COMMS_MODE="${_rc_mode}"
export ROS_DISCOVERY_SERVER="${_rc_desktop_ip}:${DISCOVERY_SERVER_PORT}"
# Let ros2 CLI (topic/node list) see everything the server knows.
export ROS_SUPER_CLIENT=TRUE

# Optional (far mode only): restrict DDS to loopback + this machine's Tailscale
# interface so participants don't advertise unreachable LAN locators over the
# WAN tunnel (symptom: discovery works but `ros2 topic echo` hangs). This is a
# WHITELIST ONLY — no locator/peer overrides — so it is safe, unlike the old
# initial-peers profile. Enable with ROVER_COMMS_DDS_WHITELIST=1.
if [ "${_rc_mode}" = "far" ] && [ "${ROVER_COMMS_DDS_WHITELIST:-0}" = "1" ]; then
  _rc_ts_ip="$(tailscale ip -4 2>/dev/null | head -1)"
  if [ -n "${_rc_ts_ip}" ]; then
    mkdir -p "${_rc_cfg_dir}"
    _rc_xml="${_rc_cfg_dir}/fastdds_tailscale.xml"
    cat > "${_rc_xml}" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>ts_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>127.0.0.1</address>
          <address>${_rc_ts_ip}</address>
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
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_rc_xml}"
    echo "[rover_env] DDS interface whitelist ON (127.0.0.1 + ${_rc_ts_ip})"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
    echo "[rover_env] WARN: DDS whitelist requested but no Tailscale IP; skipping"
  fi
else
  # The old initial-peers XML is what isolated nodes — make sure it is NOT set.
  unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
fi

unset _rc_cfg_dir _rc_mode _rc_desktop_ip _rc_ts_ip _rc_xml

echo "[rover_env] mode=${ROVER_COMMS_MODE}  ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}"
echo "[rover_env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}  LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
