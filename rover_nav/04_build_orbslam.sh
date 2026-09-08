#!/bin/bash
set +e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_sudo.sh"
echo "START orbslam build $(date)"
rsudo apt-get install -y libeigen3-dev libopencv-dev libboost-all-dev \
  libglew-dev libgl1-mesa-dev libssl-dev cmake g++ ninja-build libepoxy-dev >/dev/null 2>&1
cd ~/slam_source/Pangolin
[ -f build/libpango_core.so ] || { cmake -B build -GNinja >/dev/null 2>&1; cmake --build build -j4 >/dev/null 2>&1; rsudo cmake --install build >/dev/null 2>&1; }
echo "Pangolin done rc=$?"
cd ~/slam_source/ORB_SLAM3
sed -i 's/++11/++14/g' CMakeLists.txt 2>/dev/null   # common Humble/gcc11 fix
# Limit build parallelism: the default `make -j` OOM-kills compiling Optimizer.cc
# on the Jetson's ~7.4GB RAM. Override with ORB_SLAM3_JOBS if you add swap.
sed -i -E "s/make -j[0-9]*/make -j${ORB_SLAM3_JOBS:-2}/g" build.sh
chmod +x build.sh; ./build.sh > ~/rover_nav/logs/orbslam_build_inner.log 2>&1
echo "ORB_SLAM3 build rc=$?"; ls -la lib/libORB_SLAM3.so 2>/dev/null
echo "DONE orbslam build $(date)"; touch ~/rover_nav/logs/orbslam_build.done
