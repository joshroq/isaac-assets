#!/bin/bash
# One-command calibration run for the physical rover.
# Drives a fixed motion, records a rosbag, captures odom + battery voltage, then prompts
# you for the tape/grid measurements and appends ONE row to calibration_runs.csv.
#
# Base bringup must be running first (in another terminal):   ~/rover_nav/start_bringup.sh
# SAFE: _calib_drive.py zeros /cmd_vel at the end (the base has no cmd_vel deadman).
#
# Usage:
#   calib_run.sh <surface> <label> straight <vx_mps> <time_s>
#   calib_run.sh <surface> <label> spin     <wz_rps> <time_s>
#   calib_run.sh <surface> <label> custom   <vx_mps> <wz_rps> <time_s>
# Examples:
#   calib_run.sh vinyl t1a straight 0.15 3      # T1 straight-line
#   calib_run.sh vinyl t2a spin 0.5 3           # T2 spin-in-place
#   calib_run.sh vinyl t4a straight 0.20 2      # then measure coast for T4
#
# For T3 (square) run four straight + four spin legs and record the final grid error
# in the 'notes'. For T5 (accel) read the bag; for T6 (deadband) use 'custom' with
# small vx values and note where motion begins.

source ~/rover_nav/local_ros_env.sh
CAL=~/rover_nav/calib
CSV="$CAL/calibration_runs.csv"
BAGDIR="$CAL/bags"
mkdir -p "$BAGDIR"

surface="${1:?need <surface> (e.g. vinyl, carpet, tile)}"
label="${2:?need <label> (e.g. t1a)}"
motion="${3:?need motion: straight|spin|custom}"
case "$motion" in
  straight) vx="${4:?need vx}"; wz=0;            T="${5:?need time}";;
  spin)     vx=0;              wz="${4:?need wz}"; T="${5:?need time}";;
  custom)   vx="${4:?need vx}"; wz="${5:?need wz}"; T="${6:?need time}";;
  *) echo "motion must be straight|spin|custom"; exit 1;;
esac

if ! ros2 topic info /cmd_vel 2>/dev/null | grep -q "Subscription count: [1-9]"; then
  echo "WARNING: no /cmd_vel subscriber found -- is start_bringup.sh running?"
  echo "(waiting 3s, Ctrl-C to abort)"; sleep 3
fi

ts=$(date +%Y%m%d-%H%M%S)
run_id="${surface}_${label}_${motion}_${ts}"
bag="$BAGDIR/$run_id"
cmd_dist=$(python3 -c "print(abs($vx)*$T)")
cmd_ang=$(python3 -c "import math; print(round(math.degrees(abs($wz)*$T),2))")

echo "=== RUN $run_id ==="
echo "vx=$vx m/s  wz=$wz rad/s  T=${T}s  -> commanded dist=$cmd_dist m, angle=$cmd_ang deg"
echo "Place the rover on the START mark, aligned to +x. Press ENTER to drive..."
read _

ros2 bag record -o "$bag" /odom /odom/odom_raw /imu/data_raw /voltage /cmd_vel /scan >/dev/null 2>&1 &
BAGPID=$!
sleep 1.5

res=$(python3 "$CAL/_calib_drive.py" --vx "$vx" --wz "$wz" --time "$T")

kill -INT "$BAGPID" 2>/dev/null; sleep 1
echo "$res"

odom_dist=$(echo "$res" | grep -oP 'odom_dist=\K[-0-9.]+')
odom_dyaw=$(echo "$res" | grep -oP 'odom_dyaw_deg=\K[-0-9.]+')
vs=$(echo "$res" | grep -oP 'volt_start=\K[-0-9.]+')
ve=$(echo "$res" | grep -oP 'volt_end=\K[-0-9.]+')
echo "commanded: dist=$cmd_dist m  angle=$cmd_ang deg   |   odom: dist=$odom_dist m  dyaw=$odom_dyaw deg"

echo "--- measure on the GRID with your tape/meter-stick, enter values (blank = skip) ---"
read -p "tape forward distance (m): " tape_dist
read -p "lateral drift dy (m, +left / -right): " tape_dy
read -p "heading change (deg, +left / -right): " tape_dhead
read -p "spin center drift dx (m): " cdx
read -p "spin center drift dy (m): " cdy
read -p "coast distance past stop (m, T4): " coast
read -p "move-threshold v (m/s, T6): " thresh
read -p "notes (e.g. T3 square CW err=..): " notes

if [ ! -f "$CSV" ]; then
  echo "run_id,surface,label,motion,vx_cmd,wz_cmd,time_s,cmd_dist_m,cmd_angle_deg,odom_dist_m,odom_dyaw_deg,volt_start,volt_end,tape_dist_m,tape_dy_m,tape_dheading_deg,center_dx_m,center_dy_m,coast_dist_m,move_thresh_mps,notes,bag" > "$CSV"
fi
echo "$run_id,$surface,$label,$motion,$vx,$wz,$T,$cmd_dist,$cmd_ang,$odom_dist,$odom_dyaw,$vs,$ve,$tape_dist,$tape_dy,$tape_dhead,$cdx,$cdy,$coast,$thresh,\"$notes\",$bag" >> "$CSV"
echo "appended -> $CSV   (bag: $bag)"
