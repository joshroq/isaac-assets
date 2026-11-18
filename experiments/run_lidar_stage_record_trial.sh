#!/usr/bin/env bash
# Fresh lidar replay capture using Isaac Stage Recorder.
#
# Output layout:
#   experiments/runs/replaytest_1trial/stage_recorder_lidar/
#     cold_start_lidar_G4_center/        results.csv + paths/ + cmds/
#     replay_usd/                        Stage Recorder take + sampled fallback USD
#     gates/                             record.start/record.stop/record.done
#     logs/                              Isaac/Nav2/trial logs
set -euo pipefail

ROOT="${ROOT:-/home/joshroq/projects/isaac-assets}"
EXP="$ROOT/experiments"
RUN_ROOT="${RUN_ROOT:-$EXP/runs/replaytest_1trial/stage_recorder_lidar}"
LOGS="$RUN_ROOT/logs"
GATES="$RUN_ROOT/gates"
REPLAY_USD="$RUN_ROOT/replay_usd"
CELL_OUT="$RUN_ROOT/cold_start_lidar_G4_center"
TAKE_NAME="${TAKE_NAME:-cold_start_lidar_G4_center_trial01_stage_record}"

mkdir -p "$LOGS" "$GATES" "$REPLAY_USD" "$CELL_OUT"
rm -f "$GATES"/record.start "$GATES"/record.stop "$GATES"/record.done

set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ugv_ws/install/setup.bash" 2>/dev/null || true
source "$HOME/.local/bin/ros2ddsenv" >/dev/null 2>&1 || true
set -u

# Keep the shell-side ROS probes/Nav2 processes on the same DDS scope as Isaac.
# run_baked_rover.py forces these values internally; exporting them here prevents
# a caller's terminal environment from making wait_topic look in a different graph.
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER

stamp() { date '+%H:%M:%S'; }
say() { echo "[$(stamp)] $*"; }

teardown() {
  for p in 'run_nav_trial' 'ros2 launch' 'nav_systems' 'navlaunch' slam_toolbox \
           component_container robot_state_publisher map_saver lifecycle_manager \
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

say "fresh lidar Stage Recorder trial -> $RUN_ROOT"
say "ROS: domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION localhost_only=$ROS_LOCALHOST_ONLY"
teardown

nohup isaacpy "$ROOT/ugv_controller_v2/run_obstacle_warehouse.py" \
  --headless --no-camera-topics \
  --stage-record-dir "$REPLAY_USD" \
  --stage-record-name "$TAKE_NAME" \
  --stage-record-gate-dir "$GATES" \
  > "$LOGS/isaac.log" 2>&1 &
ISAAC_PID=$!
say "isaac PID $ISAAC_PID"

wait_topic /clock 260 || { say "ERROR: Isaac did not publish /clock"; exit 1; }

nohup navlaunch -v2 -lidar use_rviz:=false > "$LOGS/lidar.nav.log" 2>&1 &
NAV_PID=$!
say "nav PID $NAV_PID"

wait_nav 300 || { say "ERROR: Nav2 did not become active"; exit 1; }

python3 "$EXP/run_nav_trial.py" \
  --nav lidar \
  --condition cold_start \
  --goal 4 \
  --trials 1 \
  --init-maneuver sweep \
  --out "$CELL_OUT" \
  --stage-record-gate-dir "$GATES" \
  > "$LOGS/lidar.trial.log" 2>&1

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
