#!/bin/bash
set +e
cd ~/slam_source
echo "START orbslam clone $(date)"
[ -d ORB_SLAM3 ] || git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git
[ -d Pangolin ] || git clone --depth 1 --recursive https://github.com/stevenlovegrove/Pangolin.git
# uncompress the DBoW2 vocabulary ORB-SLAM3 ships compressed
[ -f ORB_SLAM3/Vocabulary/ORBvoc.txt ] || { cd ORB_SLAM3/Vocabulary && tar -xf ORBvoc.txt.tar.gz && cd ~/slam_source; }
du -sh ORB_SLAM3 Pangolin 2>/dev/null
echo "DONE orbslam clone $(date)"; touch ~/rover_nav/logs/orbslam_dl.done
