#!/usr/bin/env bash
# REAL trial: drive the physical rover through a sequence (over SSH, locally on the
# rover) and record its ground-truth path from the overhead camera.
#
# Prereqs:
#   - Camera streaming (UDP 5000 free), grid calibrated, sticker geometry measured
#   - the rover's base driver running:  ssh rover 'bash ~/rover_nav/start_bringup.sh'
#     (keep it running in its own window)
#
# Driving happens ENTIRELY on the rover (native multicast, no DDS over the network):
# we SSH in and run the same sequence against the rover's local /cmd_vel. cmd_sequence
# zeroes the motors on finish AND on SIGTERM/SIGHUP (SSH drop) - important since the
# Waveshare base has no cmd_vel timeout and no watchdog is deployed here.
#
#   ./trial_real.sh [sequence.yaml] [out.csv]
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROVER="${ROVER_HOST:-rover}"
SEQ="${1:-$HERE/../parallel/sequences/square.yaml}"
OUT="${2:-$HERE/real_run.csv}"
SSHOPTS="-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"

# Verify the new camera reference before running a motion trial.
python3 "$HERE/../calib/sticker_tracker.py" --check-config

echo "[real] verifying the rover base driver is LISTENING on /cmd_vel ..."
# A launch process can exist while the base node has failed (e.g. wrong serial
# port) - so check the thing that actually matters: a live /cmd_vel subscriber,
# plus whether /odom is publishing (base <-> motor-controller link alive).
PRE=$(ssh $SSHOPTS "$ROVER" 'source ~/rover_nav/local_ros_env.sh >/dev/null 2>&1
  subs=$(timeout 6 ros2 topic info /cmd_vel 2>/dev/null | sed -n "s/.*Subscription count: *//p")
  od=$(timeout 6 ros2 topic echo /odom --once --field header.stamp.sec 2>/dev/null | head -1)
  echo "subs=${subs:-0};odom=${od:-none}"' 2>/dev/null || echo "subs=0;odom=none")
echo "  rover reports: $PRE"
SUBS=$(printf '%s' "$PRE" | sed -n 's/.*subs=\([0-9][0-9]*\).*/\1/p'); SUBS=${SUBS:-0}
if [ "$SUBS" -lt 1 ]; then
  echo "ERROR: nothing is subscribed to /cmd_vel - the base driver is not driving the motors." >&2
  echo "  (A bringup_lidar launch process can be up while the base node has failed.)" >&2
  echo "  Start it in a terminal and KEEP it running (now defaults to /dev/ttyTHS1):" >&2
  echo "    ssh $ROVER 'bash ~/rover_nav/start_bringup.sh'" >&2
  echo "  Then confirm it's alive:" >&2
  echo "    ssh $ROVER 'source ~/rover_nav/local_ros_env.sh; ros2 topic echo /odom --once'" >&2
  exit 1
fi
case "$PRE" in
  *odom=none*) echo "  WARN: /odom not publishing - base node may not be talking to the motors"\
                    "(serial must be the full path /dev/ttyTHS1)." >&2 ;;
esac

echo "[real] staging sequence player on the rover ..."
ssh $SSHOPTS "$ROVER" 'mkdir -p ~/sim2real_run'
scp $SSHOPTS "$HERE/../parallel/cmd_sequence.py" "$SEQ" "$ROVER:~/sim2real_run/" >/dev/null
SEQBASE="$(basename "$SEQ")"

# record for: 3s countdown + sequence duration + 4s buffer, so the recorder
# self-terminates (no reliance on a kill signal, which was hanging trials).
DUR=$(python3 -c "import yaml;print(sum(s.get('duration',0) for s in yaml.safe_load(open('$SEQ'))['steps']))" 2>/dev/null || echo 10)
RECDUR=$(python3 -c "print(int(3+$DUR+4))")
echo "[real] recording camera ground truth -> $OUT  (duration ${RECDUR}s)"
python3 "$HERE/record_traj.py" --source camera --out "$OUT" --duration "$RECDUR" &
REC=$!
sleep 1
if ! kill -0 "$REC" 2>/dev/null; then
  wait "$REC" || true
  echo "ERROR: camera recorder failed before the sequence started." >&2
  exit 1
fi

echo "[real] ROVER WILL DRIVE in 3s - clear the 1x1 m grid. Ctrl-C to abort."
sleep 3
echo "[real] driving sequence on the rover: $SEQBASE"
ssh $SSHOPTS "$ROVER" "source ~/rover_nav/local_ros_env.sh && \
  python3 ~/sim2real_run/cmd_sequence.py --sequence ~/sim2real_run/$SEQBASE \
  --no-sim --real-topic /cmd_vel" || true

if ! wait "$REC"; then
  echo "ERROR: camera trajectory recording failed." >&2
  exit 1
fi
kill -INT "$REC" 2>/dev/null || true # backup
echo "[real] done -> $OUT"
