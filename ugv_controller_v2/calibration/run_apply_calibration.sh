#!/usr/bin/env bash
# Run apply_sim_calibration.py with Isaac's bundled USD libs (standalone pxr, no full app).
# Paths mirror ugv_controller_v2/tools/set_camera_fps.py's wrapper.
set -euo pipefail
PXR="${PXR:-$HOME/Applications/isaacsim/extscache/omni.usd.libs-1.0.3+6312fa25.lx64.r.cp312}"
ISAAC_PY="${ISAAC_PY:-$HOME/Applications/isaacsim/kit/python/bin/python3}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec env PYTHONPATH="$PXR" LD_LIBRARY_PATH="$PXR/bin" "$ISAAC_PY" \
  "$HERE/apply_sim_calibration.py" "$@"
