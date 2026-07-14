#!/usr/bin/env bash
# Launch Isaac Sim's bundled Python (python.sh) in a shell scrubbed of the
# *system* ROS 2 (Humble / Python 3.10) environment.
#
# WHY THIS EXISTS
# ~/.bashrc sources /opt/ros/humble/setup.bash into every terminal, which puts
# Python-3.10 rclpy on PYTHONPATH / AMENT_PREFIX_PATH. Isaac Sim runs Python
# 3.12 and re-adds those paths at startup, so `import rclpy` resolves to the
# 3.10 build and fails:
#     ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# That makes the Nav2 /cmd_vel driver blow up and Isaac shut down before it
# finishes setup ("Simulation App Shutting Down" with no "Setup complete.").
#
# THE FIX
# Launch Isaac from an environment WITHOUT system ROS sourced, so Isaac uses its
# OWN bundled Python-3.12 rclpy (the internal 'humble' distro shipped in
# exts/isaacsim.ros2.core/humble). DDS discovery still reaches Nav2 running in a
# normal, ROS-sourced terminal because both sides share ROS_DOMAIN_ID and
# RMW_IMPLEMENTATION over the wire.
#
# USAGE (drop-in for `isaacpy`):
#   ./isaacpy_clean.sh ugv_controller/common/nav_systems/nav_lidar/run_small_warehouse_nav.py
#   ./isaacpy_clean.sh run_small_warehouse_nav.py --headless
# Optional: alias isaacros="$HOME/projects/isaac-assets/isaacpy_clean.sh"
set -euo pipefail

ISAAC_PY="${ISAACSIM_PYTHON:-$HOME/Applications/isaacsim/python.sh}"
if [[ ! -x "$ISAAC_PY" ]]; then
  echo "[isaacpy_clean] Isaac python not found/executable: $ISAAC_PY" >&2
  echo "[isaacpy_clean] Set ISAACSIM_PYTHON to your isaacsim/python.sh path." >&2
  exit 1
fi

# Drop path entries that point at system ROS or the ROS 2 workspace overlay.
_strip_ros() {
  local out="" part
  local IFS=':'
  for part in $1; do
    case "$part" in
      */opt/ros/*|*/ros2_ws/*) ;;   # skip system ROS + workspace overlay
      "") ;;
      *) out="${out:+$out:}$part" ;;
    esac
  done
  printf '%s' "$out"
}

# Isaac's bundled ROS 2 (matched to its Python 3.12): the Python packages and the
# C libraries the C++ bridge dlopens (librcutils.so, librmw_fastrtps_cpp.so, ...).
ROS_DISTRO="${ISAAC_ROS_DISTRO:-humble}"
ROS_CORE="$(dirname "$ISAAC_PY")/exts/isaacsim.ros2.core"
INTERNAL_LIB="$ROS_CORE/$ROS_DISTRO/lib"
INTERNAL_RCLPY="$ROS_CORE/$ROS_DISTRO/rclpy"
if [[ ! -d "$INTERNAL_LIB" ]]; then
  echo "[isaacpy_clean] WARNING: bundled ROS libs not found at $INTERNAL_LIB" >&2
fi
if [[ ! -d "$INTERNAL_RCLPY" ]]; then
  echo "[isaacpy_clean] WARNING: bundled ROS Python packages not found at $INTERNAL_RCLPY" >&2
fi

# (1) Remove system ROS from the PYTHON path so Isaac's bundled py3.12 rclpy wins
#     (system rclpy is built for py3.10 -> 'No module named rclpy._rclpy_pybind11').
unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export PYTHONPATH="$INTERNAL_RCLPY"

# (2) Put Isaac's bundled ROS C libs FIRST on the loader path (and drop system ROS
#     so the C++ bridge can't half-load mismatched libs). This is what the bridge's
#     own "To use the internal libraries..." message asks for.
export LD_LIBRARY_PATH="${INTERNAL_LIB}$( s="$(_strip_ros "${LD_LIBRARY_PATH:-}")"; [[ -n "$s" ]] && printf ':%s' "$s" )"
export PATH="$(_strip_ros "${PATH:-}")"

# Keep DDS interop with the Nav2 side (must match the Nav2 terminal).
export ROS_DISTRO
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[isaacpy_clean] system ROS scrubbed from PYTHONPATH; using Isaac's bundled py3.12 rclpy."
echo "[isaacpy_clean] bundled ROS Python packages on PYTHONPATH: $INTERNAL_RCLPY"
echo "[isaacpy_clean] bundled ROS libs on LD_LIBRARY_PATH: $INTERNAL_LIB"
echo "[isaacpy_clean] ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
exec "$ISAAC_PY" "$@"
