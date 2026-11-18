#!/usr/bin/env bash
# Standalone USD tooling: no GUI, ROS environment, or running Isaac app required.
set -euo pipefail
ISAAC_ROOT="${ISAAC_ROOT:-$HOME/Applications/isaacsim}"
PXR="${PXR:-$ISAAC_ROOT/extscache/omni.usd.libs-1.0.3+6312fa25.lx64.r.cp312}"
ISAAC_PY="${ISAAC_PY:-$ISAAC_ROOT/kit/python/bin/python3}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec env PYTHONPATH="$PXR" LD_LIBRARY_PATH="$PXR/bin" "$ISAAC_PY" \
  "$HERE/fix_wheel_physics.py" "$@"
