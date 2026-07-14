#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ -f "${REPO_ROOT}/ugv_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ugv_ws/install/setup.bash"
fi

set -a
source "${REPO_ROOT}/jetson_rover_setup/env/ros2_rover.env"
set +a

env | sort | grep -E '^(ROS_|RMW_|CYCLONEDDS|ISAAC_ASSETS_DIR|UGV_WS)='

