#!/usr/bin/env bash
# Improved-experiment G4 rerun (IMPROVED_EXPERIMENT.md tweaks applied):
#   * INDEPENDENT per-trial runs, NO return-home (the confirmed fix) — via run_independent.sh
#   * clean environment each cell (stale /dev/shm/fastrtps + zombie /clock break Isaac)
#   * ALL 5 stacks (lidar depth rtab cuvslam orbslam), both conditions, G4; then render the
#     chase-cam videos.
#
# cuVSLAM is back in the matrix now that the box has a 16 GB GPU (the 8 GB card heap-crashed
# it — see RESULTS_STATUS.md). Visual stacks render at 640x480 (was 640x360 on 8 GB; 640x480
# also matches ORB-SLAM3's isaac_rgbd.yaml intrinsics).
#
#   full 5-stack x 2-cond x5:   setsid nohup experiments/run_experiment_v2.sh > experiments/data/_matrix_logs/exp_v2.log 2>&1 &
#   1-trial replay test folder: DATA_ROOT=experiments/runs/replaytest_1trial CONDS=cold_start TRIALS=1 \
#                                 setsid nohup experiments/run_experiment_v2.sh > /tmp/exp_v2_test.log 2>&1 &
#
# Env knobs: GOAL(4) TRIALS(5) CONDS("cold_start known_map") NAVS("lidar depth rtab cuvslam orbslam")
#            DATA_ROOT(experiments/data) — set to an isolated dir to avoid touching the main dataset.
set +u
ROOT=/home/joshroq/projects/isaac-assets; EXP="$ROOT/experiments"; MAPS="$EXP/maps/warehouse"
DATA_ROOT="${DATA_ROOT:-$EXP/data}"; case "$DATA_ROOT" in /*) : ;; *) DATA_ROOT="$ROOT/$DATA_ROOT" ;; esac
L="$DATA_ROOT/_matrix_logs"; mkdir -p "$L"
GOAL="${GOAL:-4}"; TRIALS="${TRIALS:-5}"
CONDS="${CONDS:-cold_start known_map}"; NAVS="${NAVS:-lidar depth rtab cuvslam orbslam}"
STAMP(){ date +%H:%M:%S; }; say(){ echo "[$(STAMP)] $*"; }
source /opt/ros/humble/setup.bash; source "$ROOT/ugv_ws/install/setup.bash" 2>/dev/null; source ~/.local/bin/ros2ddsenv 2>/dev/null
source "$ROOT/ugv_controller_v2/nav_systems/nav_orbslam3/install/setup.bash" 2>/dev/null

clean(){ for p in run_nav_trial 'ros2 launch' navlaunch slam_toolbox rtabmap component_container ekf_node \
   robot_localization bt_navigator controller_server planner_server amcl point_cloud rgbd_odom_node visual_slam \
   nvblox run_obstacle_warehouse 'isaac-sim' /kit/ render_replay.py; do pkill -9 -f "$p" 2>/dev/null; done
   ros2 daemon stop >/dev/null 2>&1; sleep 4; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* /dev/shm/fast_datasharing* 2>/dev/null; sleep 5; }
navtag(){ case "$1" in rtab) echo rgb;; orbslam) echo orbslam3;; *) echo "$1";; esac; }
cam_flag(){ case "$1" in lidar) echo "--no-camera-topics";; cuvslam|orbslam) echo "--camera-res 640x480";; *) echo "";; esac; }
launch_isaac(){ nohup isaacpy "$ROOT/ugv_controller_v2/run_obstacle_warehouse.py" --headless $1 > "$L/isaac.log" 2>&1 & }
wait_topic(){ local e; e=$((SECONDS+$2)); while [ $SECONDS -lt $e ]; do timeout 15 ros2 topic echo "$1" --once >/dev/null 2>&1 && return 0; sleep 3; done; return 1; }
wait_nav(){ local e; e=$((SECONDS+$1)); while [ $SECONDS -lt $e ]; do timeout 6 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null | grep -q "label='active'" && return 0; sleep 4; done; return 1; }

# ── visual-stack known-map priors (ported from run_matrix.sh) ────────────────────────────
prior_path(){ case "$1" in cuvslam) echo "$MAPS/cuvslam_map";; orbslam) echo "$MAPS/orbslam3_atlas.osa";; rtab) echo "$MAPS/rtabmap.db";; *) echo "";; esac; }
prior_ok(){ local t; t="$(prior_path "$1")"; [ -z "$t" ] && return 0
  if [ "$1" = cuvslam ]; then [ -d "$t" ] && [ -n "$(ls -A "$t" 2>/dev/null)" ]; else [ -f "$t" ]; fi; }
graceful_slam_save(){ local nav="$1" pat="" target; target="$(prior_path "$nav")"   # SIGINT so the on-shutdown save runs
  case "$nav" in cuvslam) pat="component_container_isolated";; orbslam) pat="rgbd_odom_node";; rtab) pat="rtabmap";; *) return 0;; esac
  say "  map-prep: SIGINT $pat for graceful save -> $target"; pkill -INT -f "$pat" 2>/dev/null
  local end=$((SECONDS+45)); while [ $SECONDS -lt $end ]; do prior_ok "$nav" && break; sleep 3; done; sleep 5
  prior_ok "$nav" && say "  map-prep: prior SAVED ok ($target)" || say "  map-prep: WARN prior NOT saved ($target)"; }
prep_visual_map(){ local nav="$1" tag; tag=$(navtag "$nav"); local save=""
  case "$nav" in rtab) save=("localization:=false" "database_path:=$MAPS/rtabmap.db");;
    cuvslam) save=("map_save_path:=$MAPS/cuvslam_map");; orbslam) save=("save_atlas:=$MAPS/orbslam3_atlas");; *) return 0;; esac
  rm -rf "$(prior_path "$nav")"; say "  map-prep: mapping run for $nav (saving prior)"
  clean; launch_isaac "$(cam_flag "$nav")"; wait_topic '/clock' 280 || { say "  map-prep: no /clock"; return 1; }
  nohup navlaunch -v2 -"$tag" use_rviz:=false "${save[@]}" > "$L/${nav}_mapprep.nav.log" 2>&1 &
  wait_nav 300 || { say "  map-prep: Nav2 not active"; clean; return 1; }
  for g in 3 4 2; do python3 "$EXP/run_nav_trial.py" --nav "$nav" --condition cold_start --goal "$g" \
      --trials 1 --init-maneuver sweep --no-plot --out "$L/mapprep_${nav}_g${g}" >> "$L/${nav}_mapprep.trial.log" 2>&1; done
  graceful_slam_save "$nav"; clean; say "  map-prep done for $nav"; }

rm -f "$L"/exp_v2.done
say "IMPROVED-EXP G4 rerun START  (DATA_ROOT=$DATA_ROOT  CONDS='$CONDS'  NAVS='$NAVS'  TRIALS=$TRIALS)"; clean

# known-map priors: lidar/depth use the shared occupancy map.yaml; the visual stacks need a
# map-prep pass first. Only run prep for the visual stacks we're actually going to test.
if echo "$CONDS" | grep -qw known_map; then
  for nav in $NAVS; do case "$nav" in
    rtab|cuvslam|orbslam) prior_ok "$nav" || prep_visual_map "$nav" ;;
  esac; done
fi

# independent trials per cell (run_independent.sh handles fresh Isaac+stack per trial, no return-home)
export DATA_ROOT
for cond in $CONDS; do for nav in $NAVS; do
  if [ "$cond" = known_map ]; then case "$nav" in
    rtab|cuvslam|orbslam) prior_ok "$nav" || { say "SKIP known_map/$nav (no prior — see ${nav}_mapprep.nav.log)"; continue; } ;;
  esac; fi
  say "=== $cond / $nav — independent G$GOAL x$TRIALS ==="
  NAV="$nav" COND="$cond" GOAL="$GOAL" TRIALS="$TRIALS" bash "$EXP/run_independent.sh" > "$L/exp_${cond}_${nav}.log" 2>&1
done; done

# render MP4 replays from the fresh trajectories (the cells that actually produced paths/)
clean; sleep 2
cells=(); for d in "$DATA_ROOT"/*/; do [ -d "${d}paths" ] && cells+=("${d%/}"); done
if [ ${#cells[@]} -eq 0 ]; then say "no cells with paths/ — skipping replay render"; else
  export REPLAY_VIDEO_OUTDIR="$DATA_ROOT/replay_video"
  say "rendering in-sim videos -> $REPLAY_VIDEO_OUTDIR (headless)"; rm -f "$REPLAY_VIDEO_OUTDIR"/*.mp4 2>/dev/null
  isaacpy "$EXP/render_replay.py" "${cells[@]}" --trials 1 --fps 12 --max-frames 240 --settle 6 > "$L/exp_render.log" 2>&1
  say "  -> $(ls "$REPLAY_VIDEO_OUTDIR"/*.mp4 2>/dev/null | wc -l) replay videos"
fi
clean
say "IMPROVED-EXP G4 rerun DONE"; touch "$L/exp_v2.done"
