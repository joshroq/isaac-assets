#!/usr/bin/env bash
# Defensible ORB-SLAM3 cold-start run: each trial is INDEPENDENT — a fresh Isaac + fresh
# ORB-SLAM3 from the identical USD start pose, one navigation to the goal, NO return-home.
# This removes the harness artifact where the multi-trial `--return-home` leg (Nav2 chasing
# a drifting map-frame "home" for 240 s) corrupts ORB-SLAM3's tracking and poisons later
# trials. Map-based stacks (lidar/depth/rtab) are drift-free so their continuous results are
# unaffected; only the drift-prone VIO stack needs per-trial isolation.
#
# Robustness: a trial is RE-ATTEMPTED only on pure infrastructure flakiness (Isaac /clock or
# Nav2 lifecycle not up in time — nothing to do with ORB-SLAM3). Once the stack is up, the
# trial's ORB-SLAM3 outcome (SUCCEEDED/FAILED/CANCELED/INIT_FAILED) is recorded AS-IS — that
# is the honest navigation/initialisation measurement.
#
#   setsid nohup experiments/run_orbslam_independent.sh > experiments/data/_matrix_logs/indep_orbslam.log 2>&1 &
#
# Env overrides: GOAL(4) TRIALS(5) COND(cold_start) MAX_ATTEMPTS(3).
set +u
ROOT=/home/joshroq/projects/isaac-assets
EXP="$ROOT/experiments"
GOAL="${GOAL:-4}"; TRIALS="${TRIALS:-5}"; COND="${COND:-cold_start}"; MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"
LOGS="$EXP/data/_matrix_logs"; TMP="$LOGS/indep_orbslam_tmp"
OUT="$EXP/data/${COND}_orbslam_G4_center"
mkdir -p "$LOGS" "$TMP"
STAMP() { date '+%H:%M:%S'; }
say() { echo "[$(STAMP)] $*"; }

source /opt/ros/humble/setup.bash
source "$ROOT/ugv_ws/install/setup.bash" 2>/dev/null
source ~/.local/bin/ros2ddsenv 2>/dev/null
source "$EXP/../ugv_controller_v2/nav_systems/nav_orbslam3/install/setup.bash" 2>/dev/null

teardown() {
  for p in 'run_nav_trial' 'ros2 launch' 'nav_systems' 'navlaunch' slam_toolbox \
           rtabmap component_container rgbd_odom_node point_cloud ekf_node robot_localization \
           bt_navigator controller_server planner_server amcl \
           run_obstacle_warehouse 'isaac-sim' /kit/ ; do pkill -f "$p" 2>/dev/null; done
  ros2 daemon stop >/dev/null 2>&1   # clear stale DDS discovery between rapid relaunches
  sleep 8
}
launch_isaac() {
  nohup isaacpy "$ROOT/ugv_controller_v2/run_obstacle_warehouse.py" --headless --camera-res 640x360 \
    > "$LOGS/isaac.log" 2>&1 &
  say "    isaac PID $! (headless --camera-res 640x360)"
}
wait_topic() { local t="$1" to="$2"; local end=$((SECONDS+to)); while [ $SECONDS -lt $end ]; do
  timeout 5 ros2 topic list 2>/dev/null | grep -q "$t" && return 0; sleep 3; done; return 1; }
wait_nav() { local to="$1"; local end=$((SECONDS+to)); while [ $SECONDS -lt $end ]; do
  timeout 6 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
    | grep -q "label='active'" && return 0; sleep 4; done; return 1; }

trial_result() { awk -F, 'NR==2{print $12}' "$TMP/trial_$1/results.csv" 2>/dev/null; }

say "INDEPENDENT ORB-SLAM3 run: cond=$COND goal=$GOAL trials=$TRIALS (fresh Isaac+SLAM per trial, no return-home)"
rm -rf "$TMP"/trial_* "$OUT"
mkdir -p "$OUT/paths" "$OUT/cmds"

for t in $(seq 1 "$TRIALS"); do
  attempt=0; got=0
  while [ $attempt -lt "$MAX_ATTEMPTS" ] && [ $got -eq 0 ]; do
    attempt=$((attempt+1))
    say "=== TRIAL $t/$TRIALS (attempt $attempt/$MAX_ATTEMPTS) ==="
    rm -rf "$TMP/trial_$t"
    teardown; launch_isaac
    if ! wait_topic '/clock' 230; then say "    infra: no /clock — retry"; continue; fi
    nohup navlaunch -v2 -orbslam3 use_rviz:=false > "$LOGS/indep_orbslam_t${t}.nav.log" 2>&1 &
    if ! wait_nav 300; then say "    infra: Nav2 not ACTIVE in 300s — retry"; teardown; continue; fi
    python3 "$EXP/run_nav_trial.py" --nav orbslam --condition "$COND" --goal "$GOAL" \
      --trials 1 --init-timeout 120 --init-settle 2 --no-plot --out "$TMP/trial_$t" \
      > "$LOGS/indep_orbslam_t${t}.trial.log" 2>&1
    teardown
    if [ -f "$TMP/trial_$t/results.csv" ]; then
      got=1; say "  -> trial $t result: $(trial_result $t) (stack came up on attempt $attempt)"
    else
      say "    no results.csv (stack issue) — retry"
    fi
  done
  [ $got -eq 0 ] && say "  -> trial $t: NO valid stack bringup after $MAX_ATTEMPTS attempts"
done

say "aggregating -> $OUT"
python3 - "$TMP" "$OUT" "$TRIALS" <<'PY'
import csv, os, shutil, sys
tmp, out, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
rows = []
for t in range(1, n+1):
    rc = os.path.join(tmp, f"trial_{t}", "results.csv")
    if not os.path.exists(rc):
        continue
    with open(rc) as f:
        r = list(csv.DictReader(f))
    for row in r:
        row["trial"] = str(t)
        for kind, sub in (("path_file", "paths"), ("cmd_file", "cmds")):
            rel = (row.get(kind) or "").strip()
            src = os.path.join(tmp, f"trial_{t}", rel) if rel else ""
            if src and os.path.isfile(src):
                base = f"{os.path.basename(out)}_trial{t:02d}.csv"
                shutil.copyfile(src, os.path.join(out, sub, base))
                row[kind] = f"{sub}/{base}"
            else:
                row[kind] = ""          # INIT_FAILED rows have no path/cmd file
        rows.append(row)
if rows:
    with open(os.path.join(out, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    ok = sum(r["result"] == "SUCCEEDED" for r in rows)
    ran = sum(r["result"] != "INIT_FAILED" for r in rows)
    print(f"aggregated {len(rows)} trials: {ok} SUCCEEDED, {ran} initialised & navigated -> {out}/results.csv")
else:
    print("NO trials aggregated")
PY

python3 "$EXP/plot_paths.py" "$OUT" --group-by trial --out "$OUT/routes.png" \
    --map "$EXP/maps/warehouse/map.npz" --title "orbslam / $COND / G4_center (independent trials)" 2>/dev/null
python3 "$EXP/summarize.py" "$EXP"/data/*/results.csv --out "$EXP/data/summary.csv" 2>/dev/null | tail -12
touch "$LOGS/indep_orbslam.done"
say "INDEPENDENT ORB-SLAM3 run COMPLETE -> $OUT"
