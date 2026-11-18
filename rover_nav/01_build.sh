#!/bin/bash
set +e
echo "START build $(date)"
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
# Build the WHOLE workspace. The start_*.sh scripts need ugv_bringup (base+lidar
# bringup + ugv_driver), and emcl2/ugv_base_node/ugv_description/ugv_interface/
# ugv_tools are transitive deps — the old --packages-select subset (6 pkgs) was
# NOT enough to reproduce a working install on a fresh rover.
colcon build --symlink-install --parallel-workers 3 \
  --event-handlers console_cohesion+
echo "BUILD RC=$?"
echo "DONE build $(date)"; touch ~/rover_nav/logs/build.done
