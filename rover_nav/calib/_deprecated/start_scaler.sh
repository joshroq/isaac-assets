#!/bin/bash
# Run the calibrated cmd_vel scaler: /cmd_vel_raw -> (scaled) -> /cmd_vel.
source ~/rover_nav/local_ros_env.sh
eval $(python3 -c "import yaml;d=yaml.safe_load(open('$HOME/rover_nav/calib/calibration.yaml'));print(f\"L={d['linear_scale']};A={d['angular_scale']}\")")
exec python3 ~/rover_nav/calib/cmd_vel_scaler.py --ros-args -p linear_scale:=$L -p angular_scale:=$A
