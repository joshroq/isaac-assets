#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source "${SCRIPT_DIR}/ros2_shared_env.sh"
exec ros2 launch "${REPO_ROOT}/ugv_controller_v2/nav_systems/nav_lidar/isaac_nav_lidar.launch.py" "$@"

