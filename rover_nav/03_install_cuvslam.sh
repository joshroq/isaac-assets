#!/bin/bash
set +e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_sudo.sh"
echo "START cuvslam/nvblox install $(date)"
rsudo apt-get install -y ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-nvblox
echo "RC=$?"
echo "DONE $(date)"; touch ~/rover_nav/logs/cuvslam_dl.done
