#!/usr/bin/env bash
# Generalized single-trial replay capture using Isaac Stage Recorder.
# Parameterized version of run_lidar_stage_record_trial.sh: works for any nav system
# (lidar, depth, rtab, cuvslam, orbslam), producing the SAME deliverable layout the
# lidar capture did, so every stack can be stored side by side.
#
#   NAV=depth  ./run_stage_record_trial.sh
#   NAV=rtab   ./run_stage_record_trial.sh
#   NAV=cuvslam ./run_stage_record_trial.sh
#   NAV=orbslam ./run_stage_record_trial.sh
#
# Output layout (mirrors the lidar capture):
#   experiments/runs/replaytest_1trial/stage_recorder_<NAV>/
#     cold_start_<NAV>_G4_center/    results.csv + routes.png + map.npz + paths/ + cmds/
#     replay_usd/                    Stage Recorder take + sampled fallback USD
#     gates/                         record.start/record.stop/record.done
#     logs/                          Isaac/Nav2/trial logs
set -euo pipefail

NAV="${NAV:?set NAV=lidar|depth|rtab|cuvslam|orbslam}"

ROOT="${ROOT:-/home/joshroq/projects/isaac-assets}"
EXP="$ROOT/experiments"
RUN_ROOT="${RUN_ROOT:-$EXP/runs/replaytest_1trial/stage_recorder_${NAV}}"
LOGS="$RUN_ROOT/logs"
GATES="$RUN_ROOT/gates"
REPLAY_USD="$RUN_ROOT/replay_usd"
CELL="cold_start_${NAV}_G4_center"
CELL_OUT="$RUN_ROOT/$CELL"
TAKE_NAME="${TAKE_NAME:-${CELL}_trial01_stage_record}"

# nav system -> navlaunch tag, Isaac camera flag, extra launch args.
case "$NAV" in
  lidar)   TAG=lidar;    CAM="--no-camera-topics";        EXTRA="" ;;
  depth)   TAG=depth;    CAM="";                          EXTRA="" ;;
  rtab)    TAG=rgb;      CAM="";                          EXTRA="" ;;
  # 16 GB GPU now: cuVSLAM's old VRAM heap-abort is gone, but keep the proven
  # 640x480 camera + trimmed nvblox footprint (framerate + safety headroom).
  cuvslam) TAG=cuvslam;  CAM="--camera-res 640x480";      EXTRA="" ;;
  orbslam) TAG=orbslam3; CAM="--camera-res 640x480";      EXTRA="orbslam_viewer:=false" ;;
  *) echo "unknown NAV=$NAV" >&2; exit 2 ;;
esac

mkdir -p "$LOGS" "$GATES" "$REPLAY_USD" "$CELL_OUT"
rm -f "$GATES"/record.start "$GATES"/record.stop "$GATES"/record.done

set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ugv_ws/install/setup.bash" 2>/dev/null || true
# ORB-SLAM3 wrapper overlay (orbslam3_ros2_wrapper); harmless for other stacks.
source "$ROOT/ugv_controller_v2/nav_systems/nav_orbslam3/install/setup.bash" 2>/dev/null || true
source "$HOME/.local/bin/ros2ddsenv" >/dev/null 2>&1 || true
set -u

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER

stamp() { date '+%H:%M:%S'; }
say() { echo "[$(stamp)] $*"; }

teardown() {
  for p in 'run_nav_trial' 'ros2 launch' 'nav_systems' 'navlaunch' slam_toolbox \
           rtabmap component_container visual_slam rgbd_odom_node point_cloud \
           robot_state_publisher map_saver lifecycle_manager ekf_node rf2o \
           'run_obstacle_warehouse' 'isaac-sim' /kit/; do
    pkill -f "$p" 2>/dev/null || true
  done
  ros2 daemon stop >/dev/null 2>&1 || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* /dev/shm/fast_datasharing* 2>/dev/null || true
  sleep 5
}

wait_topic() {
  local topic="$1" timeout_s="$2"
  local end=$((SECONDS + timeout_s))
  while [ "$SECONDS" -lt "$end" ]; do
    timeout 10 ros2 topic echo "$topic" --once >/dev/null 2>&1 && return 0
    sleep 3
  done
  return 1
}

wait_nav() {
  local timeout_s="$1"
  local end=$((SECONDS + timeout_s))
  while [ "$SECONDS" -lt "$end" ]; do
    timeout 6 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
      | grep -q "label='active'" && return 0
    sleep 4
  done
  return 1
}

finish() {
  local code=$?
  teardown
  exit "$code"
}
trap finish EXIT

say "fresh $NAV Stage Recorder trial (tag=$TAG cam='$CAM' extra='$EXTRA') -> $RUN_ROOT"
say "ROS: domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION localhost_only=$ROS_LOCALHOST_ONLY"
teardown

nohup isaacpy "$ROOT/ugv_controller_v2/run_obstacle_warehouse.py" \
  --headless $CAM \
  --stage-record-dir "$REPLAY_USD" \
  --stage-record-name "$TAKE_NAME" \
  --stage-record-gate-dir "$GATES" \
  > "$LOGS/isaac.log" 2>&1 &
ISAAC_PID=$!
say "isaac PID $ISAAC_PID"

wait_topic /clock 300 || { say "ERROR: Isaac did not publish /clock"; exit 1; }

# shellcheck disable=SC2086
nohup navlaunch -v2 -"$TAG" use_rviz:=false $EXTRA > "$LOGS/${NAV}.nav.log" 2>&1 &
NAV_PID=$!
say "nav PID $NAV_PID"

wait_nav 360 || { say "ERROR: Nav2 did not become active"; exit 1; }

python3 "$EXP/run_nav_trial.py" \
  --nav "$NAV" \
  --condition cold_start \
  --goal 4 \
  --trials 1 \
  --init-maneuver sweep \
  --out "$CELL_OUT" \
  --stage-record-gate-dir "$GATES" \
  > "$LOGS/${NAV}.trial.log" 2>&1

say "trial complete; waiting for recorder finalization"
for _ in $(seq 1 60); do
  [ -f "$GATES/record.done" ] && break
  sleep 2
done

if [ ! -f "$GATES/record.done" ]; then
  say "WARN: recorder did not write record.done before timeout"
else
  cat "$GATES/record.done"
fi

say "results: $CELL_OUT/results.csv"
say "replay usd dir: $REPLAY_USD"
