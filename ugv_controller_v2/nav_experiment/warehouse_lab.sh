#!/usr/bin/env bash
# One-command test lab: Isaac Sim (randomized-obstacle warehouse) + a RELIABLE nav
# system (lidar / depth / rtab) + automatic Nav2 bring-up, all in one tmux window.
#
# WHY THIS EXISTS
#   Running a nav test used to mean three terminals and remembering to (a) generate
#   the randomized warehouse, (b) launch Isaac on it, (c) launch the nav stack, and
#   (d) work around the Nav2 lifecycle-bringup race that otherwise leaves the stack
#   inactive so an RViz goal does nothing. This script does all of it.
#
# SYSTEMS (the reliable ones, "for right now")
#   lidar   slam_toolbox on /scan            (navlaunch -v2 -lidar)
#   depth   slam_toolbox map + depth cloud   (navlaunch -v2 -depth)
#   rtab    RTAB-Map RGB-D                    (navlaunch -v2 -rgb)
#
# OBSTACLES ARE LIVE + FRESH BY DEFAULT
#   By default the obstacles are scattered LIVE in the rover process with NO fixed
#   seed, so every run lays them out in different places. (The pre-baked-USD path is
#   opt-in via --baked; it does not re-randomize unless you also pass --seed/--fresh,
#   and Replicator props do not always survive the USD export, which is why live is
#   the default.) Pass --seed N for a reproducible layout.
#
# USAGE
#   ugv_controller_v2/nav_experiment/warehouse_lab.sh lidar
#   ugv_controller_v2/nav_experiment/warehouse_lab.sh depth --seed 7 --count 24
#   ugv_controller_v2/nav_experiment/warehouse_lab.sh rtab  --auto 5      # auto-drive 5 goals
#   ugv_controller_v2/nav_experiment/warehouse_lab.sh lidar --baked       # load a pre-baked USD
#   ugv_controller_v2/nav_experiment/warehouse_lab.sh lidar --style primitives --headless
#
# OPTIONS
#   --seed N        Fix the obstacle layout seed (reproducible). Default: none, so
#                   every run randomizes to different positions.
#   --count N       Number of obstacles (default 18).
#   --style S       props (real warehouse assets, default) | primitives.
#   --baked         Load a pre-baked randomized USD instead of scattering live. The
#                   default is live scatter (fresh every run).
#   --live          Force live scatter (this is already the default; kept for clarity).
#   --fresh         With --baked, force-regenerate the pre-baked randomized USD.
#   --no-clean      Do NOT tear down a prior nav stack first. By default the lab kills
#                   any leftover navlaunch/Nav2/slam/DPVO/ORB-SLAM/visual-bridge/RViz
#                   processes so a stale run cannot bleed /map, TF and a ghost Nav2
#                   into this one.
#   --auto [N]      After bring-up, auto-drive N goals across the map (default 3)
#                   with nav_mission.py. Without --auto the stack is just brought
#                   up and left ACTIVE for RViz 2D-Goal-Pose clicking.
#   --headless      Run Isaac with no viewport (topics still publish).
#   --no-rviz       Launch the nav stack without RViz (dodges RViz GL/GLX crashes;
#                   recommended for automated runs — drive/observe via nav_mission).
#   --no-attach     Create the tmux session but do not attach (for remote/headless).
#   -- <args...>    Everything after -- is passed through to navlaunch.
set -euo pipefail

REPO="/home/joshroq/projects/isaac-assets"
V2="$REPO/ugv_controller_v2"
NAVEXP="$V2/nav_experiment"
UGV_WS="$REPO/ugv_ws"
DDSENV="$HOME/.local/bin/ros2ddsenv"
SESSION="warehouse_lab"

SYSTEM=""
SEED=""               # empty = no fixed seed = different layout every run
COUNT=18
STYLE="props"
LIVE=1                # live scatter is the default (see OBSTACLES note above)
FRESH=0
CLEAN=1               # tear down a prior nav stack before launching
HEADLESS=0
ATTACH=1
RVIZ=1
LOOP=0                # 0 = bring Nav2 up and leave it for RViz; >0 = auto-drive
NAV_PASSTHRU=()

# The comment header above doubles as --help (up to the OPTIONS block's last line).
usage() { sed -n '2,50p' "$0" | sed 's/^# \{0,1\}//'; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        lidar|depth|rtab) SYSTEM="$1"; shift ;;
        --seed)  SEED="$2"; shift 2 ;;
        --count) COUNT="$2"; shift 2 ;;
        --style) STYLE="$2"; shift 2 ;;
        --baked) LIVE=0; shift ;;
        --live)  LIVE=1; shift ;;
        --fresh) FRESH=1; shift ;;
        --no-clean) CLEAN=0; shift ;;
        --headless) HEADLESS=1; shift ;;
        --no-rviz) RVIZ=0; shift ;;
        --no-attach) ATTACH=0; shift ;;
        --auto)
            LOOP=3
            if [[ "${2:-}" =~ ^[0-9]+$ ]]; then LOOP="$2"; shift; fi
            shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; NAV_PASSTHRU=("$@"); break ;;
        *) echo "warehouse_lab: unknown argument '$1'" >&2; usage; exit 2 ;;
    esac
done

if [[ -z "$SYSTEM" ]]; then
    echo "warehouse_lab: pick a nav system: lidar | depth | rtab" >&2
    usage; exit 2
fi

case "$SYSTEM" in
    lidar) TAG="-lidar" ;;
    depth) TAG="-depth" ;;
    rtab)  TAG="-rgb" ;;
esac

# ── Tear down a prior nav stack so it can't bleed into this run ────────────────
# Leftover navlaunch/Nav2/slam/DPVO/ORB-SLAM/visual-bridge/RViz processes from an
# earlier (possibly manually-started, non-tmux) run keep publishing /map, TF and a
# second Nav2 on the same ROS graph — the classic "an old DPVO run is interfering
# with RViz" failure. These patterns are project-specific: the nav2/slam nodes are
# matched by their `/ugv_sim/scan` remap and the launch/bridge/RViz processes by
# their repo paths, so Isaac (no such args) and unrelated ROS nodes are left alone.
cleanup_stale_stack() {
    local patterns=(
        "navlaunch"
        "nav_systems"            # ros2 launch cmd, rviz2 -d <nav_systems/*.rviz>, wrapper nodes
        "visual_map_builder.py"
        "dpvo_bridge_node.py"
        "/ugv_sim/scan"          # every Nav2 + slam_toolbox node carries this remap
    )
    local hit=0
    for pat in "${patterns[@]}"; do
        pgrep -f "$pat" >/dev/null 2>&1 && hit=1
    done
    if [[ "$hit" == "0" ]]; then
        echo "=== no stale nav stack found ==="
        return 0
    fi
    echo "=== tearing down a prior nav stack (navlaunch/Nav2/slam/visual bridges/RViz) ==="
    echo "    (skip with --no-clean)"
    for pat in "${patterns[@]}"; do pkill -TERM -f "$pat" 2>/dev/null || true; done
    sleep 2
    for pat in "${patterns[@]}"; do pkill -KILL -f "$pat" 2>/dev/null || true; done
    sleep 1
}
[[ "$CLEAN" == "1" ]] && cleanup_stale_stack

# ── Obstacle args (seed only when explicitly fixed) ───────────────────────────
OBS_ARGS=(--random-obstacle-count "$COUNT" --obstacle-style "$STYLE")
[[ -n "$SEED" ]] && OBS_ARGS+=(--random-seed "$SEED")

ISAAC_HEADLESS=""
[[ "$HEADLESS" == "1" ]] && ISAAC_HEADLESS="--headless"
# RViz is a viewer, not part of the control loop, and its GLX context is the least
# stable process here. Drop it on request so a GL crash can't take down the run.
[[ "$RVIZ" == "0" ]] && NAV_PASSTHRU+=("use_rviz:=false")

# ── Live scatter (default) vs a pre-baked randomized USD (--baked) ────────────
if [[ "$LIVE" == "1" ]]; then
    ISAAC_ARGS=(--live-replicator "${OBS_ARGS[@]}")
    echo "=== Obstacles: LIVE scatter (fresh every run$([[ -n "$SEED" ]] && echo ", seed=$SEED")) ==="
else
    # A pre-baked layout with no fixed seed would silently reuse the same 'auto'
    # USD, so force a regenerate to actually re-randomize when --baked has no seed.
    [[ -z "$SEED" ]] && FRESH=1
    USD="$(python3 - "$NAVEXP" "$SEED" "$COUNT" "$STYLE" <<'PY'
import sys, argparse
sys.path.insert(0, sys.argv[1])
from run_random_small_warehouse import default_generated_usd
seed_arg = sys.argv[2]
ns = argparse.Namespace(
    random_seed=(int(seed_arg) if seed_arg else None),
    random_obstacle_count=int(sys.argv[3]),
    obstacle_style=sys.argv[4], random_min_spacing=0.45,
    random_x_min=-5.0, random_x_max=5.0, random_y_min=-5.0, random_y_max=5.0)
print(default_generated_usd(ns))
PY
)"
    if [[ "$FRESH" == "1" || ! -f "$USD" ]]; then
        echo "=== Generating randomized warehouse USD (seed=${SEED:-auto} count=$COUNT style=$STYLE) ==="
        echo "    $USD"
        isaacpy "$NAVEXP/run_random_small_warehouse.py" --headless "${OBS_ARGS[@]}"
    else
        echo "=== Using existing randomized warehouse USD ==="
        echo "    $USD"
    fi
    ISAAC_ARGS=("${OBS_ARGS[@]}")
fi

# ── Build per-pane scripts (temp files dodge tmux/bash quoting) ───────────────
PANE_DIR="$(mktemp -d /tmp/warehouse_lab.XXXXXX)"
trap 'rm -rf "$PANE_DIR"' EXIT

cat > "$PANE_DIR/isaac.sh" <<EOF
#!/usr/bin/env bash
echo '=== [pane 1] Isaac Sim + randomized obstacles ==='
child_pid=""
stop_child_group() {
    local sig="\${1:-TERM}"
    if [[ -n "\$child_pid" ]] && kill -0 "\$child_pid" 2>/dev/null; then
        echo "=== [pane 1] stopping Isaac process group (\$child_pid) via SIG\$sig ==="
        kill -TERM -- "-\$child_pid" 2>/dev/null || true
        for _ in {1..25}; do
            kill -0 "\$child_pid" 2>/dev/null || return 0
            sleep 0.2
        done
        kill -KILL -- "-\$child_pid" 2>/dev/null || true
    fi
}
trap 'stop_child_group INT; exit 130' INT
trap 'stop_child_group TERM; exit 143' TERM
trap 'stop_child_group HUP; exit 129' HUP
setsid isaacpy "$NAVEXP/run_small_warehouse_random_nav.py" $ISAAC_HEADLESS ${ISAAC_ARGS[*]} &
child_pid="\$!"
wait "\$child_pid"
status="\$?"
child_pid=""
exit "\$status"
EOF

cat > "$PANE_DIR/nav.sh" <<EOF
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$DDSENV"
echo '=== [pane 2] waiting for Isaac /clock before launching nav ($SYSTEM) ==='
until ros2 topic list 2>/dev/null | grep -qx /clock; do sleep 2; done
sleep 3
child_pid=""
stop_child_group() {
    local sig="\${1:-TERM}"
    if [[ -n "\$child_pid" ]] && kill -0 "\$child_pid" 2>/dev/null; then
        echo "=== [pane 2] stopping navlaunch process group (\$child_pid) via SIG\$sig ==="
        kill -TERM -- "-\$child_pid" 2>/dev/null || true
        for _ in {1..25}; do
            kill -0 "\$child_pid" 2>/dev/null || return 0
            sleep 0.2
        done
        kill -KILL -- "-\$child_pid" 2>/dev/null || true
    fi
}
trap 'stop_child_group INT; exit 130' INT
trap 'stop_child_group TERM; exit 143' TERM
trap 'stop_child_group HUP; exit 129' HUP
setsid navlaunch -v2 $TAG ${NAV_PASSTHRU[*]} &
child_pid="\$!"
wait "\$child_pid"
status="\$?"
child_pid=""
exit "\$status"
EOF

cat > "$PANE_DIR/mission.sh" <<EOF
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$UGV_WS/install/setup.bash"
source "$DDSENV"
echo '=== [pane 3] waiting for Nav2 nodes, then bring-up recovery ==='
until ros2 node list 2>/dev/null | grep -q bt_navigator; do sleep 2; done
sleep 4
# These systems localize from lidar/RGB-D immediately, so no visual priming.
# --loop 0 just beats the lifecycle race and leaves Nav2 ACTIVE for RViz;
# --loop N (from --auto) auto-drives N goals across the map.
child_pid=""
stop_child_group() {
    local sig="\${1:-TERM}"
    if [[ -n "\$child_pid" ]] && kill -0 "\$child_pid" 2>/dev/null; then
        echo "=== [pane 3] stopping mission process group (\$child_pid) via SIG\$sig ==="
        kill -TERM -- "-\$child_pid" 2>/dev/null || true
        for _ in {1..25}; do
            kill -0 "\$child_pid" 2>/dev/null || return 0
            sleep 0.2
        done
        kill -KILL -- "-\$child_pid" 2>/dev/null || true
    fi
}
trap 'stop_child_group INT; exit 130' INT
trap 'stop_child_group TERM; exit 143' TERM
trap 'stop_child_group HUP; exit 129' HUP
setsid python3 "$NAVEXP/nav_mission.py" --no-prime --loop $LOOP &
child_pid="\$!"
wait "\$child_pid"
status="\$?"
child_pid=""
exit "\$status"
EOF

chmod +x "$PANE_DIR"/*.sh

# ── Lay it out in one tmux window ─────────────────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session  -d -s "$SESSION" -n lab "$PANE_DIR/isaac.sh;   exec bash"
tmux split-window -h -t "$SESSION"        "$PANE_DIR/nav.sh;     exec bash"
tmux split-window -v -t "$SESSION"        "$PANE_DIR/mission.sh; exec bash"
tmux select-layout -t "$SESSION" tiled

echo
echo "=== warehouse_lab up: session '$SESSION' ==="
echo "    system   : $SYSTEM   (navlaunch -v2 $TAG)"
echo "    obstacles: count=$COUNT style=$STYLE seed=${SEED:-random} $([[ $LIVE == 1 ]] && echo '(live)' || echo '(baked)')"
echo "    goals    : $([[ $LOOP -gt 0 ]] && echo "auto-drive $LOOP" || echo 'RViz 2D Goal Pose (Nav2 left ACTIVE)')"
echo "    detach   : Ctrl-b d     kill: tmux kill-session -t $SESSION"

if [[ "$ATTACH" == "1" && -t 1 ]]; then
    # Keep pane scripts alive for the session's lifetime instead of the EXIT trap.
    trap - EXIT
    tmux attach -t "$SESSION"
else
    trap - EXIT
    echo "    attach   : tmux attach -t $SESSION"
    echo "(pane scripts in $PANE_DIR — removed when you kill the session)"
fi
