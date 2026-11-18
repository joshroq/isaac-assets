#!/usr/bin/env bash
# Independent-trial runner for ONE nav stack / condition / goal: each trial is a fresh Isaac +
# fresh nav stack from the identical USD start pose, ONE navigation, NO return-home. Use this
# for stacks whose between-trial `--return-home` corrupts the run (ORB-SLAM3, and rtab in
# cold-start where the live map makes the return leg fail and the next trial starts at the
# goal instead of the origin). Map-based stacks with stable localization (lidar/depth, and
# rtab known-map) keep the faster continuous protocol via run_matrix.sh.
#
# A trial is RE-ATTEMPTED only on pure infrastructure flakiness (Isaac /clock or Nav2 not up);
# the stack's actual outcome (SUCCEEDED/FAILED/CANCELED/INIT_FAILED) is recorded AS-IS.
#
#   NAV=rtab COND=cold_start GOAL=1 TRIALS=5 experiments/run_independent.sh
#
# Env: NAV(req) COND(cold_start) GOAL(4) TRIALS(5) MAX_ATTEMPTS(3)
set +u
ROOT=/home/joshroq/projects/isaac-assets; EXP="$ROOT/experiments"
NAV="${NAV:?set NAV=lidar|depth|rtab|orbslam}"; COND="${COND:-cold_start}"; GOAL="${GOAL:-4}"
TRIALS="${TRIALS:-5}"; MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"
DATA_ROOT="${DATA_ROOT:-$EXP/data}"           # override to run into an isolated folder
LOGS="$DATA_ROOT/_matrix_logs"; mkdir -p "$LOGS"
STAMP(){ date '+%H:%M:%S'; }; say(){ echo "[$(STAMP)] $*"; }

source /opt/ros/humble/setup.bash
source "$ROOT/ugv_ws/install/setup.bash" 2>/dev/null
source ~/.local/bin/ros2ddsenv 2>/dev/null
source "$EXP/../ugv_controller_v2/nav_systems/nav_orbslam3/install/setup.bash" 2>/dev/null

navtag(){ case "$1" in rtab) echo rgb;; orbslam) echo orbslam3;; *) echo "$1";; esac; }
cam_flag(){ case "$1" in lidar) echo "--no-camera-topics";; cuvslam|orbslam) echo "--camera-res 640x480";; *) echo "";; esac; }
teardown(){
  for p in run_nav_trial 'ros2 launch' nav_systems navlaunch slam_toolbox rtabmap \
           component_container visual_slam rgbd_odom_node point_cloud ekf_node robot_localization \
           bt_navigator controller_server planner_server amcl run_obstacle_warehouse 'isaac-sim' /kit/; do
    pkill -f "$p" 2>/dev/null; done
  ros2 daemon stop >/dev/null 2>&1; sleep 3
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* /dev/shm/fast_datasharing* 2>/dev/null; sleep 5
}
launch_isaac(){ nohup isaacpy "$ROOT/ugv_controller_v2/run_obstacle_warehouse.py" --headless $(cam_flag "$NAV") \
  > "$LOGS/isaac.log" 2>&1 &  say "    isaac PID $! ($(cam_flag "$NAV"))"; }
wait_topic(){ local t="$1"; local to="$2"; local end=$((SECONDS+to)); while [ $SECONDS -lt $end ]; do
  timeout 15 ros2 topic echo "$t" --once >/dev/null 2>&1 && return 0; sleep 3; done; return 1; }
wait_nav(){ local to="$1"; local end=$((SECONDS+to)); while [ $SECONDS -lt $end ]; do
  timeout 6 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
    | grep -q "label='active'" && return 0; sleep 4; done; return 1; }
trial_result(){ awk -F, 'NR==2{print $12}' "$1/results.csv" 2>/dev/null; }

tag=$(navtag "$NAV")
gname=$(python3 -c "import yaml;print(yaml.safe_load(open('$EXP/goals.yaml'))['goals'][$GOAL]['name'])")
OUT="$DATA_ROOT/${COND}_${NAV}_${gname}"; TMP="$LOGS/indep_${NAV}_${COND}_${gname}_tmp"
km=""; [ "$COND" = known_map ] && km="-known-map"

say "INDEPENDENT $NAV / $COND / $gname (goal $GOAL): $TRIALS trials, fresh stack per trial, no return-home"
rm -rf "$TMP"; mkdir -p "$TMP"   # build in TMP; only replace $OUT on success (never destroy good data on a failed run)

for t in $(seq 1 "$TRIALS"); do
  attempt=0; got=0
  while [ $attempt -lt "$MAX_ATTEMPTS" ] && [ $got -eq 0 ]; do
    attempt=$((attempt+1)); say "=== $NAV/$COND/$gname TRIAL $t/$TRIALS (attempt $attempt) ==="
    rm -rf "$TMP/trial_$t"; teardown; launch_isaac
    if ! wait_topic '/clock' 260; then say "    infra: no /clock — retry"; continue; fi
    nohup navlaunch -v2 -"$tag" $km use_rviz:=false > "$LOGS/indep_${NAV}_${COND}_${gname}_t${t}.nav.log" 2>&1 &
    if ! wait_nav 300; then say "    infra: Nav2 not ACTIVE in 300s — retry"; teardown; continue; fi
    python3 "$EXP/run_nav_trial.py" --nav "$NAV" --condition "$COND" --goal "$GOAL" \
      --trials 1 --init-timeout 120 --no-plot --out "$TMP/trial_$t" \
      > "$LOGS/indep_${NAV}_${COND}_${gname}_t${t}.trial.log" 2>&1
    teardown
    [ -f "$TMP/trial_$t/results.csv" ] && { got=1; say "  -> trial $t: $(trial_result "$TMP/trial_$t")"; } || say "    no results.csv — retry"
  done
  [ $got -eq 0 ] && say "  -> trial $t: NO valid bringup after $MAX_ATTEMPTS attempts"
done

say "aggregating -> $OUT"
python3 - "$TMP" "$OUT" "$TRIALS" <<'PY'
import csv, os, shutil, sys
tmp, out, n = sys.argv[1], sys.argv[2], int(sys.argv[3]); rows=[]
for t in range(1, n+1):
    rc=os.path.join(tmp,f"trial_{t}","results.csv")
    if not os.path.exists(rc): continue
    for row in csv.DictReader(open(rc)):
        row["trial"]=str(t); row["_t"]=t; rows.append(row)
if rows:                                   # only NOW touch $OUT — a failed run leaves old data intact
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(os.path.join(out,"paths"),exist_ok=True); os.makedirs(os.path.join(out,"cmds"),exist_ok=True)
    for row in rows:
        t=row.pop("_t")
        for kind,sub in (("path_file","paths"),("cmd_file","cmds")):
            rel=(row.get(kind) or "").strip(); src=os.path.join(tmp,f"trial_{t}",rel) if rel else ""
            if src and os.path.isfile(src):
                base=f"{os.path.basename(out)}_trial{t:02d}.csv"; shutil.copyfile(src, os.path.join(out,sub,base)); row[kind]=f"{sub}/{base}"
            else: row[kind]=""
    with open(os.path.join(out,"results.csv"),"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print(f"aggregated {len(rows)} trials, {sum(r['result']=='SUCCEEDED' for r in rows)} SUCCEEDED -> {out}")
else: print("NO trials aggregated (existing data preserved)")
PY
python3 "$EXP/plot_paths.py" "$OUT" --group-by trial --out "$OUT/routes.png" \
  --map "$EXP/maps/warehouse/map.npz" --title "$NAV / $COND / $gname (independent)" 2>/dev/null
say "INDEPENDENT $NAV/$COND/$gname COMPLETE -> $OUT"
