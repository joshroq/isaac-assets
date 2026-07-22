#!/usr/bin/env bash
# Environment CHECK for the v2 nav_dpvo system. This does NOT build anything.
#
# DPVO is torch + CUDA + three compiled CUDA extensions (cuda_corr, cuda_ba,
# lietorch_backends) plus DPViewer (Pangolin/GL). All of that already lives in the `dpvo`
# CONDA ENV, which was built by hand — DPViewer in particular needed a source fix
# (the DPVO-main zip ships an EMPTY Pangolin/ dir; the zachteed/Pangolin fork has to be
# cloned and built, plus three DPViewer/CMakeLists.txt edits and a scrubbed conda
# toolchain). Rebuilding that automatically would be fragile and could break a working
# install, so this script only VERIFIES and tells you what is wrong.
#
# The load-bearing fact this checks: the `dpvo` env is python 3.10 — the SAME minor as ROS
# Humble — so /opt/ros's rclpy/tf2_ros/message packages import straight into it. That is
# what lets dpvo_bridge_node.py be one ordinary ROS node holding conda torch AND system
# rclpy in a single process, with no IPC bridge.
#
#   ./setup.sh          # check everything
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DPVO_DIR="${DPVO_DIR:-/home/joshroq/projects/slam_source/DPVO-main}"
DPVO_PYTHON="${DPVO_PYTHON:-/home/joshroq/miniforge3/envs/dpvo/bin/python}"

fail=0
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; fail=1; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$1"; }

echo "== nav_dpvo environment check =="
echo "DPVO_DIR    = $DPVO_DIR"
echo "DPVO_PYTHON = $DPVO_PYTHON"
echo

echo "-- interpreter"
if [[ -x "$DPVO_PYTHON" ]]; then
  ok "$("$DPVO_PYTHON" --version 2>&1)"
else
  bad "not found: $DPVO_PYTHON"
  echo
  echo "  DPVO lives in the 'dpvo' conda env. If it is gone, rebuild it:"
  echo "    conda env create -f $DPVO_DIR/environment.yml   # or a py3.10 env + torch cu128"
  echo "    cd $DPVO_DIR && pip install ."
  echo "  DPViewer additionally needs the zachteed/Pangolin fork cloned into Pangolin/"
  echo "  (the release zip ships it EMPTY) and a scrubbed conda toolchain."
  echo
  echo "  NOTE: it MUST be python 3.10 so ROS Humble's rclpy imports into it."
  exit 1
fi

echo
echo "-- ROS 2 (must be sourced so rclpy is importable from the dpvo env)"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  warn "ROS not sourced in this shell; sourcing /opt/ros/humble for the check"
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
ok "ROS_DISTRO=${ROS_DISTRO:-unset}"

echo
echo "-- python imports inside the dpvo env"
"$DPVO_PYTHON" - <<'PY'
import sys
fail = 0
def chk(mod, label=None):
    global fail
    try:
        m = __import__(mod)
        v = getattr(m, "__version__", "")
        print(f"  \033[32mOK\033[0m   {label or mod:22s} {v}")
        return m
    except Exception as e:
        print(f"  \033[31mFAIL\033[0m {label or mod:22s} {type(e).__name__}: {e}")
        fail = 1
        return None

# DPVO side
torch = chk("torch")
chk("dpvo"); chk("cuda_corr"); chk("cuda_ba"); chk("lietorch_backends")
# DPVO's OWN native viewer. Optional: without it, run the launch with viz:=false.
try:
    import dpviewer  # noqa: F401
    print("  \033[32mOK\033[0m   dpviewer               (DPVO's native Pangolin viewer)")
except Exception as e:
    print(f"  \033[33mWARN\033[0m dpviewer               not importable ({type(e).__name__}). "
          "Run the launch with viz:=false and use RViz only.")

# ROS side -- the whole point: these must resolve INSIDE the conda interpreter.
chk("rclpy"); chk("tf2_ros")
for m in ("sensor_msgs.msg", "geometry_msgs.msg", "sensor_msgs_py.point_cloud2"):
    try:
        __import__(m); print(f"  \033[32mOK\033[0m   {m}")
    except Exception as e:
        print(f"  \033[31mFAIL\033[0m {m}: {type(e).__name__}: {e}"); fail = 1

if torch is not None:
    if torch.cuda.is_available():
        cc = torch.cuda.get_device_capability(0)
        print(f"  \033[32mOK\033[0m   CUDA available         {torch.cuda.get_device_name(0)} "
              f"(sm_{cc[0]}{cc[1]})")
    else:
        print("  \033[31mFAIL\033[0m CUDA NOT available — DPVO is GPU-only")
        fail = 1
sys.exit(fail)
PY
[[ $? -ne 0 ]] && fail=1

echo
echo "-- trained weights"
if [[ -f "$DPVO_DIR/dpvo.pth" ]]; then
  ok "dpvo.pth ($(du -h "$DPVO_DIR/dpvo.pth" | cut -f1))"
else
  bad "dpvo.pth missing — run: cd $DPVO_DIR && bash download_models_and_data.sh"
fi

echo
echo "-- display (needed for DPVO's native DPViewer window)"
if [[ -n "${DISPLAY:-}" ]]; then
  ok "DISPLAY=$DISPLAY"
else
  warn "no DISPLAY — run the launch with viz:=false (RViz/native viewer both need X)"
fi

echo
if [[ "$fail" == "0" ]]; then
  echo "All good. Run the system with:"
  echo "  navlaunch -v2 -dpvo"
else
  echo "Environment is NOT ready — fix the FAIL lines above."
  exit 1
fi
