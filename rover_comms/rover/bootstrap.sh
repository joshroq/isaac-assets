#!/usr/bin/env bash
# bootstrap.sh — one command to set up rover comms after cloning the repo.
#
#   git clone <your-repo-url> ~/isaac-assets
#   cd ~/isaac-assets/rover_comms/rover
#   ./bootstrap.sh
#
# It checks prerequisites, (optionally) builds the Waveshare workspace, installs
# the comms helpers + service, and prints the exact next steps. Idempotent.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "=========================================="
echo " rover_comms bootstrap"
echo " repo: ${REPO_ROOT}"
echo "=========================================="

# --- prerequisite: ROS 2 Humble ---------------------------------------------
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble." >&2
  echo "Install ROS 2 Humble first (or run jetson_rover_setup/install_jetson.sh" >&2
  echo "which also builds the driver workspace), then re-run this." >&2
  exit 1
fi
echo "[ok] ROS 2 Humble present."

# --- prerequisite: Fast DDS RMW ---------------------------------------------
if ! dpkg -l ros-humble-rmw-fastrtps-cpp >/dev/null 2>&1; then
  echo "[warn] ros-humble-rmw-fastrtps-cpp not installed — installing…"
  sudo apt-get update && sudo apt-get install -y ros-humble-rmw-fastrtps-cpp
fi
echo "[ok] rmw_fastrtps_cpp available."

# --- driver workspace (needed for rover-bringup) ----------------------------
if [ ! -f "${REPO_ROOT}/ugv_ws/install/setup.bash" ]; then
  echo
  echo "[note] The Waveshare driver workspace (${REPO_ROOT}/ugv_ws) is not built."
  echo "       You need it for rover-bringup (the actual robot driver)."
  if [ -x "${REPO_ROOT}/jetson_rover_setup/install_jetson.sh" ]; then
    read -r -p "Run jetson_rover_setup/install_jetson.sh now to build it? [y/N] " a || a=n
    case "$a" in
      y|Y) ( cd "${REPO_ROOT}/jetson_rover_setup" && ./install_jetson.sh ) ;;
      *)   echo "       Skipped — comms will install, but rover-bringup won't run"
           echo "       until the workspace is built." ;;
    esac
  else
    echo "       (jetson_rover_setup/install_jetson.sh not found — build ugv_ws manually.)"
  fi
else
  echo "[ok] driver workspace built."
fi

# --- install the comms layer ------------------------------------------------
echo
echo "[step] installing rover comms…"
"${SCRIPT_DIR}/install_rover_comms.sh"

echo
echo "=========================================="
echo " bootstrap complete"
echo "=========================================="
cat <<EOF
Next:
  rover-mode near         # nearby demo on the ugv AP (no internet), OR
  rover-mode far          # remote via Tailscale, then: ./tailscale_up.sh

  sudo systemctl disable --now ugv-bringup.service        # avoid double driver
  sudo systemctl enable  --now rover-comms-bringup.service

  rover-verify            # confirm the link (desktop discovery server must be up)
EOF
