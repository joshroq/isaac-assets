#!/usr/bin/env bash
# One-time build for the v2 nav_orbslam3 system.
#
# ORB-SLAM3 is C++ (unlike slam_toolbox / RTAB-Map, which come from apt), so — like the
# upstream ROS example — it needs a compile. This script:
#   1. builds ORB-SLAM3's own library (Thirdparty + libORB_SLAM3.so) if missing,
#   2. uncompresses the ORB vocabulary if missing,
#   3. colcon-builds the ROS 2 wrapper (orbslam3_ros2_wrapper) into an overlay HERE.
#
# The overlay lives in this folder (not ugv_ws) so the ORB-SLAM3 dependency stays
# contained in the system that needs it. `navlaunch -v2 -orbslam3` sources it for you;
# to launch by hand, source it yourself:
#   source install/setup.bash
#   ros2 launch isaac_nav_orbslam3.launch.py
#
# Prereqs (apt): build-essential cmake libeigen3-dev libopencv-dev libboost-all-dev
#                libpangolin-dev  (+ ros-humble-navigation2 ros-humble-nav2-bringup
#                ros-humble-cv-bridge ros-humble-tf2-eigen for the wrapper)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORB_SLAM3_DIR="${ORB_SLAM3_DIR:-/home/joshroq/projects/slam_source/ORB_SLAM3}"

echo "== nav_orbslam3 (v2) build =="
echo "ORB_SLAM3_DIR = ${ORB_SLAM3_DIR}"

if [[ ! -d "${ORB_SLAM3_DIR}" ]]; then
  echo "ERROR: ORB-SLAM3 source not found at ${ORB_SLAM3_DIR}" >&2
  echo "Set ORB_SLAM3_DIR=/path/to/ORB_SLAM3 and re-run." >&2
  exit 1
fi

# ── 1. ORB-SLAM3 library ────────────────────────────────────────────────────────
if [[ ! -f "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so" ]]; then
  echo "-- Building ORB-SLAM3 (this takes a while)..."
  ( cd "${ORB_SLAM3_DIR}" && chmod +x build.sh && ./build.sh )
else
  echo "-- libORB_SLAM3.so already present; skipping ORB-SLAM3 build."
fi

# ── 2. Vocabulary ───────────────────────────────────────────────────────────────
if [[ ! -f "${ORB_SLAM3_DIR}/Vocabulary/ORBvoc.txt" ]]; then
  echo "-- Uncompressing ORB vocabulary..."
  ( cd "${ORB_SLAM3_DIR}/Vocabulary" && tar -xf ORBvoc.txt.tar.gz )
else
  echo "-- ORBvoc.txt already present; skipping."
fi

# ── 3. ROS 2 wrapper (colcon overlay in this folder) ────────────────────────────
echo "-- Building orbslam3_ros2_wrapper..."
cd "${HERE}"
colcon build --paths orbslam3_ros2_wrapper \
  --cmake-args "-DORB_SLAM3_DIR=${ORB_SLAM3_DIR}"

echo
echo "Done. Now run the system with:"
echo "  navlaunch -v2 -orbslam3"
echo
echo "or by hand:"
echo "  source ${HERE}/install/setup.bash"
echo "  ros2 launch ${HERE}/isaac_nav_orbslam3.launch.py"
