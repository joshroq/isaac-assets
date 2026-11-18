#!/usr/bin/env bash
# Full SIM experiment matrix: each nav system x {cold_start, known_map}, N trials to a
# goal, generating data (results.csv + paths + cmds) AND a routes-over-map visual per cell,
# fully headless. Continues past any cell's failure; logs everything. Run DETACHED so it
# survives terminal/session teardown:
#     setsid nohup experiments/run_matrix.sh > experiments/data/_matrix_logs/matrix.log 2>&1 &
# then watch:  tail -f experiments/data/_matrix_logs/matrix.log
#
# Order is chosen to front-load the achievable cells: all cold_start first (no prior map
# needed), then lidar/depth known_map (uses the make_occupancy_map ground-truth map), then
# the visual stacks' known_map (needs a map-prep pass first).
# Env overrides: GOAL(4) TRIALS(5) NAVS(...) CONDS(...).
set +u
ROOT=/home/joshroq/projects/isaac-assets
EXP="$ROOT/experiments"
MAPS="$EXP/maps/warehouse"
GOAL="${GOAL:-4}"; TRIALS="${TRIALS:-5}"
NAVS="${NAVS:-lidar depth rtab cuvslam orbslam}"
CONDS="${CONDS:-cold_start known_map}"
LOGS="$EXP/data/_matrix_logs"; mkdir -p "$LOGS" "$MAPS"
STAMP() { date '+%H:%M:%S'; }
say() { echo "[$(STAMP)] $*"; }

source /opt/ros/humble/setup.bash
source "$ROOT/ugv_ws/install/setup.bash" 2>/dev/null
source ~/.local/bin/ros2ddsenv 2>/dev/null
# ORB-SLAM3 wrapper overlay (if built)
source "$EXP/../ugv_controller_v2/nav_systems/nav_orbslam3/install/setup.bash" 2>/dev/null

navtag() { case "$1" in rtab) echo rgb;; orbslam) echo orbslam3;; *) echo "$1";; esac; }
# Per-nav Isaac launch flags. lidar: no camera at all. Visual-SLAM stacks (cuvslam,
# orbslam) render the baked cameras at a reduced resolution so the 8 GB RTX 5060 Ti does
# not exhaust VRAM — the documented cuVSLAM+nvblox container heap-abort (exit -6) that
# strands nav on dead-reckoned odom — and so the sim camera keeps a usable, low-jitter
# frame rate for visual tracking. depth/rtab worked at native and are left unchanged.
VISUAL_CAM_RES="${VISUAL_CAM_RES:-640x480}"
cam_flag() {
  case "$1" in
    lidar)           echo "--no-camera-topics" ;;
    cuvslam|orbslam) echo "--camera-res $VISUAL_CAM_RES" ;;
    *)               echo "" ;;
  esac
}

teardown() {
  for p in 'run_nav_trial' 'ros2 launch' 'nav_systems' 'navlaunch' slam_toolbox \
           rtabmap component_container visual_slam rgbd_odom_node point_cloud \
           run_obstacle_warehouse 'isaac-sim' /kit/ ; do pkill -f "$p" 2>/dev/null; done
  ros2 daemon stop >/dev/null 2>&1   # clear stale DDS discovery so the next cell's Nav2 bringup isn't slowed
  sleep 3
  # Clear the dead cell's Fast DDS shared-memory segments. Leftover /dev/shm/fastrtps_*
  # files from killed processes cause "open_and_lock_file failed" on the next launch, which
  # silently breaks DDS transport so Isaac's /tf,/odom,/clock never reach Nav2 and the
  # costmaps hang on "Invalid frame odom" -> Nav2 never activates. Clearing them each cell
  # keeps DDS healthy across the many launch/teardown cycles.
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* /dev/shm/fast_datasharing* 2>/dev/null
  sleep 5
}

# Absolute path to a visual stack's saved prior (the file/folder that known_map loads).
prior_path() {
  case "$1" in
    cuvslam) echo "$MAPS/cuvslam_map" ;;      # cuVSLAM map folder
    orbslam) echo "$MAPS/orbslam3_atlas.osa" ;;  # ORB-SLAM3 atlas (save_atlas base + .osa)
    rtab)    echo "$MAPS/rtabmap.db" ;;        # RTAB-Map database
    *)       echo "" ;;
  esac
}
# True iff the prior exists (and, for the cuVSLAM folder, is non-empty).
prior_ok() {
  local t; t="$(prior_path "$1")"; [ -z "$t" ] && return 0
  if [ "$1" = cuvslam ]; then [ -d "$t" ] && [ -n "$(ls -A "$t" 2>/dev/null)" ]; else [ -f "$t" ]; fi
}
# Gracefully stop ONLY the SLAM node with SIGINT (Ctrl-C) so its on-shutdown map/atlas save
# actually runs, then wait for the prior to be serialised. The stock teardown() uses SIGTERM
# via pkill, which kills the node before the destructor/shutdown-hook writes the map — that
# is why the first matrix produced no cuvslam_map and no .osa atlas. Call this BEFORE teardown.
graceful_slam_save() {  # $1=nav
  local nav="$1" pat="" target; target="$(prior_path "$nav")"
  case "$nav" in
    cuvslam) pat="component_container_isolated" ;;
    orbslam) pat="rgbd_odom_node" ;;
    rtab)    pat="rtabmap" ;;
    *) return 0 ;;
  esac
  say "  map-prep: SIGINT $pat for graceful save -> $target"
  pkill -INT -f "$pat" 2>/dev/null
  local end=$((SECONDS+45))
  while [ $SECONDS -lt $end ]; do prior_ok "$nav" && break; sleep 3; done
  sleep 5   # let the final flush complete
  prior_ok "$nav" && say "  map-prep: prior SAVED ok ($target)" \
                  || say "  map-prep: WARN prior NOT saved ($target)"
}

wait_topic() { local t="$1" to="$2"; local end=$((SECONDS+to)); while [ $SECONDS -lt $end ]; do
  timeout 5 ros2 topic list 2>/dev/null | grep -q "$t" && return 0; sleep 3; done; return 1; }
wait_nav() { local to="$1"; local end=$((SECONDS+to)); while [ $SECONDS -lt $end ]; do
  timeout 6 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
    | grep -q "label='active'" && return 0; sleep 4; done; return 1; }

launch_isaac() {  # $1 = extra flags
  nohup isaacpy "$ROOT/ugv_controller_v2/run_obstacle_warehouse.py" --headless $1 \
    > "$LOGS/isaac.log" 2>&1 &
  say "  isaac PID $! (headless $1)"
}

# Build the shared ground-truth occupancy map (for lidar/depth known_map) if missing.
ensure_shared_map() {
  [ -f "$MAPS/map.yaml" ] && { say "shared map present"; return 0; }
  say "building shared occupancy map (make_occupancy_map.py)..."
  "$HOME/Applications/isaacsim/python.sh" "$EXP/make_occupancy_map.py" \
    --env warehouse_basic_obstacles.usd --out "$MAPS" > "$LOGS/make_map.log" 2>&1
  [ -f "$MAPS/map.yaml" ] && say "shared map built" || say "WARN shared map build FAILED (see make_map.log)"
}

# Map-prep pass for a visual stack: cold-start WITH its save flag, drive a few goals to
# cover the map, then kill so it saves. Best-effort.
prep_visual_map() {  # $1=nav
  local nav="$1" tag; tag=$(navtag "$nav"); local save=""
  case "$nav" in
    rtab)    save=("localization:=false" "database_path:=$MAPS/rtabmap.db") ;;
    cuvslam) save=("map_save_path:=$MAPS/cuvslam_map") ;;
    orbslam) save=("save_atlas:=$MAPS/orbslam3_atlas") ;;
    *) return 0 ;;
  esac
  rm -rf "$(prior_path "$nav")"   # clear any stale/partial prior so a failed save can't masquerade as success
  say "  map-prep: mapping run for $nav (saving prior map)"
  teardown; launch_isaac "$(cam_flag "$nav")"
  wait_topic '/clock' 260 || { say "  map-prep: isaac /clock timeout"; return 1; }
  nohup navlaunch -v2 -"$tag" use_rviz:=false "${save[@]}" > "$LOGS/${nav}_mapprep.nav.log" 2>&1 &
  wait_nav 300 || { say "  map-prep: Nav2 not active"; teardown; return 1; }
  # drive coverage: goals 3 then 4 (add 2 if reachable) to populate the map
  for g in 3 4 2; do
    python3 "$EXP/run_nav_trial.py" --nav "$nav" --condition cold_start --goal "$g" \
      --trials 1 --init-maneuver sweep --no-plot --out "$LOGS/mapprep_${nav}_g${g}" \
      >> "$LOGS/${nav}_mapprep.trial.log" 2>&1
  done
  graceful_slam_save "$nav"   # SIGINT the SLAM node so it actually writes the prior, THEN hard teardown
  teardown
  say "  map-prep done for $nav"
}

run_cell() {  # $1=nav $2=cond
  local nav="$1" cond="$2" tag; tag=$(navtag "$nav")
  local cell="${cond}_${nav}"
  say "=== CELL $cell (goal $GOAL, $TRIALS trials) ==="
  teardown; launch_isaac "$(cam_flag "$nav")"
  if ! wait_topic '/clock' 260; then say "  $cell: ISAAC FAILED (no /clock) — skip"; return 1; fi
  local km=""; [ "$cond" = known_map ] && km="-known-map"
  nohup navlaunch -v2 -"$tag" $km use_rviz:=false > "$LOGS/$cell.nav.log" 2>&1 &
  if ! wait_nav 300; then say "  $cell: Nav2 not ACTIVE — skip (see $cell.nav.log)"; teardown; return 1; fi
  python3 "$EXP/run_nav_trial.py" --nav "$nav" --condition "$cond" --goal "$GOAL" \
    --trials "$TRIALS" --return-home > "$LOGS/$cell.trial.log" 2>&1
  say "  $cell: trials done -> data/${cell}_*/"
  teardown
}

say "MATRIX START  navs=[$NAVS] conds=[$CONDS] goal=$GOAL trials=$TRIALS"
ensure_shared_map

# Phase 1: all cold_start cells (no prior map needed)
if echo "$CONDS" | grep -qw cold_start; then
  for nav in $NAVS; do run_cell "$nav" cold_start; done
fi
# Phase 2: known_map — lidar/depth use the shared map directly; visual stacks map-prep first
if echo "$CONDS" | grep -qw known_map; then
  for nav in $NAVS; do
    case "$nav" in
      lidar|depth) : ;;                       # shared map already built
      rtab|cuvslam|orbslam)
        prep_visual_map "$nav"
        if ! prior_ok "$nav"; then
          say "  known_map_$nav: prior MISSING after map-prep -> SKIP (avoids 5 doomed trials; see ${nav}_mapprep.nav.log)"
          continue
        fi ;;
    esac
    run_cell "$nav" known_map
  done
fi
teardown
say "MATRIX COMPLETE. data in $EXP/data/  (routes.png per cell). Summary:"
python3 "$EXP/summarize.py" "$EXP"/data/*/results.csv --out "$EXP/data/summary.csv" 2>/dev/null | tail -20
touch "$LOGS/matrix.done"
