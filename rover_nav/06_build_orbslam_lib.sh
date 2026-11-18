#!/bin/bash
set +e
echo "START orbslam lib build $(date)"
cd ~/slam_source/ORB_SLAM3
# Limit build parallelism: default `make -j` OOM-kills on the Jetson's ~7.4GB RAM.
sed -i -E "s/make -j[0-9]*/make -j${ORB_SLAM3_JOBS:-2}/g" build.sh
chmod +x build.sh
./build.sh > ~/rover_nav/logs/orbslam_lib_inner.log 2>&1
echo "build.sh RC=$?"
ls -la lib/libORB_SLAM3.so 2>/dev/null && echo "LIB OK" || echo "LIB MISSING"
echo "DONE $(date)"; touch ~/rover_nav/logs/orbslam_lib.done
