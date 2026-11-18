#!/bin/bash
set +e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_sudo.sh"
echo "START apt $(date)"
rsudo apt-get update -y
PKGS="ros-humble-robot-localization ros-humble-depth-image-proc ros-humble-image-proc \
ros-humble-rtabmap-slam ros-humble-rtabmap-sync ros-humble-rtabmap-util ros-humble-rtabmap-odom \
ros-humble-depthai-ros-driver ros-humble-depthai"
for p in $PKGS; do
  echo "=== installing $p ==="
  rsudo apt-get install -y "$p" && echo "OK $p" || echo "FAILED $p"
done
echo "DONE apt $(date)"; touch ~/rover_nav/logs/apt.done
