#!/usr/bin/env bash
# One-command launch of the overhead metric visualization on the desktop.
#   1. start the Pi camera stream (scoped passwordless sudo)
#   2. calibrate the grid (click BL,BR,TR,TL; every tape crossing is then found
#      automatically) if no calibration exists yet
#   3. open the live metric overlay window
# The Pi stream is stopped again when you close the overlay (protects the 2A supply).
#
#   ./run_viz.sh                # normal launch
#   ./run_viz.sh --recalibrate  # re-click the grid corners first
#   ./run_viz.sh --recalibrate --size 2x2 --spacing 0.5   # a different grid (metres)
#   ./run_viz.sh --no-stream    # don't touch the Pi (stream already running)
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
PI="joshu@pi-cam.local"
SSHOPTS="-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
CALIB="$HERE/calib/grid_calib.json"

RECAL=0; DO_STREAM=1; GRID_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --recalibrate) RECAL=1 ;;
    --no-stream)   DO_STREAM=0 ;;
    --size|--spacing) GRID_ARGS+=("$1" "${2:?$1 needs a value}"); shift ;;
    -h|--help)     grep -E '^#( |$)' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown arg: $1 (see --help)"; exit 2 ;;
  esac
  shift
done

# Only one consumer of UDP 5000 at a time - drop any plain receiver.
pkill -f 'sim2real_calib/capture/receive.py' 2>/dev/null || true

cleanup() {
  if [ "$DO_STREAM" = 1 ]; then
    echo "[*] stopping Pi stream ..."
    ssh $SSHOPTS "$PI" 'sudo -n systemctl stop sim2real-camera.service' 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [ "$DO_STREAM" = 1 ]; then
  echo "[1/3] starting Pi camera stream ..."
  if ssh $SSHOPTS "$PI" 'sudo -n systemctl start sim2real-camera.service'; then
    echo "      stream running (watch the Pi's red LED on 2A power)"
  else
    echo "      WARN: couldn't start the stream over SSH; start it manually and re-run with --no-stream" >&2
  fi
fi

if [ "$RECAL" = 1 ] || [ ! -f "$CALIB" ]; then
  echo "[2/3] grid calibration - click BL, BR, TR, TL, then ENTER to save ..."
  python3 "$HERE/calib/calibrate_grid.py" --source stream "${GRID_ARGS[@]}"
else
  echo "[2/3] using existing calibration: $CALIB  (--recalibrate to redo)"
fi

echo "[3/3] launching live metric overlay - q to quit ..."
python3 "$HERE/viz/metric_overlay.py"
