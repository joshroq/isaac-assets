#!/bin/bash
# OAK-D (RGB + depth + IMU). Includes a chmod watcher because the depthai udev rule
# does not reliably apply to the on-the-fly re-enumeration on this box.
source ~/rover_nav/local_ros_env.sh
source ~/rover_nav/_check_single_oak.sh
source ~/rover_nav/_sudo.sh
( rsudo bash -c '
for i in $(seq 1 300); do
  for d in /sys/bus/usb/devices/*/idVendor; do
    if [ "$(cat "$d" 2>/dev/null)" = "03e7" ]; then
      dev=$(dirname "$d"); B=$(cat "$dev/busnum" 2>/dev/null); D=$(cat "$dev/devnum" 2>/dev/null)
      [ -n "$B" ] && [ -n "$D" ] && chmod 666 "$(printf /dev/bus/usb/%03d/%03d "$B" "$D")" 2>/dev/null
    fi
  done; sleep 0.2
done' ) &
exec ros2 launch ugv_vision oak_d_lite.launch.py
