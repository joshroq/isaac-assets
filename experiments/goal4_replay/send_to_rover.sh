#!/usr/bin/env bash
# Replay a sim goal-4 /cmd_vel command stream on the REAL rover (open-loop sim->real check).
#   experiments/goal4_replay/send_to_rover.sh <file.csv> [ssh-alias]
# The rover's sensor/driver bringup must be running; put the rover ON BLOCKS first, then
# floor with a spotter. Publishes straight to /cmd_vel (calibration is baked into the sim
# USD, so no runtime scaler). The Waveshare driver has no deadman — stay ready to stop it.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CMD="${1:?usage: send_to_rover.sh <goal4_replay/xxx.csv> [ssh-alias]}"
[ -f "$CMD" ] || CMD="$HERE/$CMD"
[ -f "$CMD" ] || { echo "no such command file: $1"; exit 1; }
ROVER="${2:-rover}"
BN="$(basename "$CMD")"

echo "==> copying $BN to $ROVER:~/rover_nav/goal4_cmds/"
ssh "$ROVER" 'mkdir -p ~/rover_nav/goal4_cmds'
scp "$CMD" "$ROVER:~/rover_nav/goal4_cmds/$BN"

echo "==> replaying on $ROVER  (rover on blocks/clear? Ctrl-C to abort)"
read -r -p "    press ENTER to run, or Ctrl-C to cancel... " _
ssh -t "$ROVER" "source ~/rover_nav/local_ros_env.sh && \
  python3 ~/rover_nav/calib/replay_cmdvel.py ~/rover_nav/goal4_cmds/$BN --topic /cmd_vel"
echo "==> done. Measure the real stop pose vs the sim trial's final_x/final_y (results.csv)."
