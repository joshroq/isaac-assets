#!/usr/bin/env bash
# One-time setup for the v2 nav_dpvo system.
#
# DPVO is a PyTorch + CUDA codebase (unlike slam_toolbox / RTAB-Map, which come from apt,
# and unlike ORB-SLAM3, which is plain C++). It needs a Python environment with torch,
# three compiled CUDA extensions (cuda_corr, cuda_ba, lietorch_backends), the trained
# network weights, and — for its native viewer — the DPViewer/Pangolin GL module.
#
# THE VENV TRICK: the bridge node needs BOTH torch (not in system python) and rclpy
# (ROS Humble, python3.10 only). So we build the venv with --system-site-packages from
# python3.10: venv torch + system rclpy in one interpreter. isaac_nav_dpvo.launch.py runs
# the bridge with THIS interpreter, so nothing else on the machine is touched.
#
#   ./setup.sh                 # full setup
#   ./setup.sh --skip-viewer   # skip DPViewer (no native window; RViz only)
#
# Prereqs: an NVIDIA driver + the CUDA toolkit (nvcc) matching the torch build below.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DPVO_DIR="${DPVO_DIR:-/home/joshroq/projects/slam_source/DPVO-main}"
VENV="$HERE/.venv"
# Torch CUDA wheel index. cu128 covers Blackwell (RTX 50-series, sm_120); older cards can
# use cu121. Override with TORCH_INDEX=... if your toolkit differs.
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"

SKIP_VIEWER=0
[[ "${1:-}" == "--skip-viewer" ]] && SKIP_VIEWER=1

echo "== nav_dpvo (v2) setup =="
echo "DPVO_DIR = $DPVO_DIR"
echo "venv     = $VENV"

if [[ ! -d "$DPVO_DIR" ]]; then
  echo "ERROR: DPVO source not found at $DPVO_DIR" >&2
  echo "Set DPVO_DIR=/path/to/DPVO-main and re-run." >&2
  exit 1
fi

# ── 0. Pre-flight: DPVO's CUDA extensions need nvcc ─────────────────────────────
if ! command -v nvcc >/dev/null 2>&1; then
  echo >&2
  echo "ERROR: nvcc (CUDA toolkit) not found, but DPVO compiles three CUDA extensions" >&2
  echo "       (cuda_corr, cuda_ba, lietorch_backends). A driver alone is not enough." >&2
  echo >&2
  echo "Install the toolkit, e.g.:" >&2
  echo "  sudo apt update && sudo apt install -y nvidia-cuda-toolkit" >&2
  echo "or a matching toolkit from https://developer.nvidia.com/cuda-downloads" >&2
  echo "then re-run this script." >&2
  exit 1
fi
echo "-- nvcc: $(nvcc --version | tail -1)"

# ── 1. venv (system-site-packages so rclpy stays importable) ────────────────────
if [[ ! -d "$VENV" ]]; then
  echo "-- Creating venv (python3.10 + --system-site-packages, so rclpy resolves)..."
  python3 -m venv --system-site-packages "$VENV"
else
  echo "-- venv already exists; reusing."
fi
# shellcheck source=/dev/null
source "$VENV/bin/activate"
python -m pip install --upgrade pip wheel setuptools >/dev/null

python - <<'PY'
import sys
print(f"-- venv python {sys.version.split()[0]}")
try:
    import rclpy  # noqa: F401
    print("-- rclpy visible inside the venv (system-site-packages OK)")
except Exception as e:
    raise SystemExit(f"rclpy NOT visible in the venv: {e}\n"
                     "Source /opt/ros/humble/setup.bash before running setup.sh.")
PY

# ── 2. torch (CUDA build) ───────────────────────────────────────────────────────
if ! python -c "import torch" 2>/dev/null; then
  echo "-- Installing torch from $TORCH_INDEX (this is a large download)..."
  python -m pip install torch torchvision --index-url "$TORCH_INDEX"
else
  echo "-- torch already installed; skipping."
fi
python - <<'PY'
import torch
print(f"-- torch {torch.__version__}  cuda_available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"-- gpu: {torch.cuda.get_device_name(0)}")
PY

# ── 3. DPVO python deps + the package itself (compiles the CUDA extensions) ─────
echo "-- Installing DPVO python deps..."
python -m pip install numpy opencv-python yacs einops kornia plyfile evo tqdm

if ! python -c "import cuda_ba" 2>/dev/null; then
  echo "-- Building + installing DPVO (compiles cuda_corr / cuda_ba / lietorch_backends)..."
  ( cd "$DPVO_DIR" && python -m pip install . )
else
  echo "-- DPVO CUDA extensions already built; skipping."
fi

# ── 4. Trained weights ──────────────────────────────────────────────────────────
if [[ ! -f "$DPVO_DIR/dpvo.pth" ]]; then
  echo "-- Downloading DPVO weights (dpvo.pth)..."
  ( cd "$DPVO_DIR" && bash download_models_and_data.sh )
else
  echo "-- dpvo.pth already present; skipping."
fi

# ── 5. DPViewer — DPVO's OWN Pangolin visualization ─────────────────────────────
# This is what `demo.py --viz` opens: live image + tracked patches + 3D trajectory.
# Optional: without it, run the launch with viz:=false and use RViz only.
if [[ "$SKIP_VIEWER" == "1" ]]; then
  echo "-- --skip-viewer given: not building DPViewer (launch with viz:=false)."
elif python -c "import dpviewer" 2>/dev/null; then
  echo "-- dpviewer already installed; skipping."
else
  echo "-- Building DPViewer (Pangolin GL viewer)..."
  if [[ ! -f "$DPVO_DIR/Pangolin/build/libpango_core.so" ]]; then
    echo "   Building DPVO's bundled Pangolin first..."
    ( cd "$DPVO_DIR/Pangolin" && cmake -B build && cmake --build build -j"$(nproc)" )
  else
    echo "   Bundled Pangolin already built."
  fi
  if ! ( cd "$DPVO_DIR/DPViewer" && python -m pip install . ); then
    echo >&2
    echo "WARNING: DPViewer failed to build. Everything else still works —" >&2
    echo "         run the system with  viz:=false  and use RViz for visualization." >&2
  fi
fi

echo
echo "Done. Run the system with:"
echo "  navlaunch -v2 -dpvo"
echo
echo "or by hand:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash"
echo "  ros2 launch $HERE/isaac_nav_dpvo.launch.py"
