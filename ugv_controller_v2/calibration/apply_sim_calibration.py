#!/usr/bin/env python3
"""BAKE a sim->real calibration into jetson_rover_v2.usd (standalone pxr, ONE-TIME).

Reads sim_calibration.yaml and authors the DifferentialController inputs, wheel-friction
materials, corner-wheel joint drives, masses and chassis drag onto the baked v2 rover USD,
then backs up and saves. This BAKES the calibration so there is no runtime cmd_vel scaler.

All prim paths / attributes live in the single source of truth calib_params.py, which the
in-memory optimiser path (run_baked_rover.py --calib) shares — so a baked USD and a per-trial
in-memory theta apply IDENTICALLY.

FLOOR friction is NOT baked here: the rover USD has no ground, so the floor PhysicsMaterial is
authored at runtime (run_ground_plane.py --calib binds it to the live ground) or baked into an
ENVIRONMENT USD separately. GEOMETRY (wheel radius, track, wheelbase) is LOCKED: calib_params
never authors it. Masses are measured; authoring one rescales that body's inertia.

USAGE (needs Isaac's USD libs; easiest via the wrapper):
    ugv_controller_v2/calibration/run_apply_calibration.sh            # apply
    ugv_controller_v2/calibration/run_apply_calibration.sh --dry-run  # report only
"""
import argparse
import datetime
import os
import shutil
import sys

try:
    from pxr import Usd  # noqa: F401
except ImportError:
    sys.exit(
        "pxr not importable. Run with Isaac's USD libs, e.g.:\n"
        "  PXR=$HOME/Applications/isaacsim/extscache/omni.usd.libs-1.0.3+6312fa25.lx64.r.cp312\n"
        "  PYTHONPATH=$PXR LD_LIBRARY_PATH=$PXR/bin \\\n"
        "    $HOME/Applications/isaacsim/kit/python/bin/python3 apply_sim_calibration.py\n"
        "(or just use run_apply_calibration.sh)"
    )
from pxr import Usd  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import calib_params as C  # noqa: E402

DEFAULT_USD = "/home/joshroq/projects/isaac-assets/jetson_rover_v2.usd"
DEFAULT_YAML = os.path.join(HERE, "sim_calibration.yaml")


def load_flat(path):
    """Flat {key: value} from sim_calibration.yaml (nested), falling back to USD defaults."""
    try:
        import yaml
    except ImportError:
        print("  (PyYAML unavailable — using embedded defaults => no-op re-bake)")
        return {}
    if not os.path.isfile(path):
        print(f"  (no {path}; using defaults => no-op)")
        return {}
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    flat = C.flatten(cfg)
    # also accept a flat 'friction_combine_mode' convenience key
    if "friction_combine_mode" in cfg:
        flat["_combine_mode"] = cfg["friction_combine_mode"]
    return flat


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", default=DEFAULT_USD)
    ap.add_argument("--config", default=DEFAULT_YAML)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--bake-floor", action="store_true",
                    help="also author the floor material into this USD (only for ENV USDs "
                         "with a ground collider; off by default since the rover USD has none)")
    args = ap.parse_args()

    flat = load_flat(args.config)
    combine_mode = flat.pop("_combine_mode", None)
    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        sys.exit(f"could not open {args.usd}")

    # Dry-run: mutate the in-memory stage, report, discard (never Save).
    changes = C.apply_to_stage(stage, flat, author_floor_material=args.bake_floor,
                               combine_mode=combine_mode)
    sets = [c for c in changes if c[0] in ("set", "create", "bind-floor", "connect")]
    warns = [c for c in changes if str(c[0]).startswith("WARN")]
    for c in warns:
        print(f"  {c[0]} {c[1]} {c[2]}".rstrip())
    if not sets:
        print("Nothing to change (USD already matches the config).")
        return 0
    for kind, prim, attr, before, after in sets:
        tag = "DRY " if args.dry_run else kind.upper().ljust(4)
        print(f"  {tag} {prim}.{attr}: {before!r} -> {after!r}")
    if args.dry_run:
        return 0

    # Real run: the in-memory stage already carries the changes (apply_to_stage set them).
    # Back up the file on disk, then Save.
    backup = f"{args.usd}.bak-{datetime.datetime.now():%Y%m%d-%H%M%S}-calib"
    shutil.copy2(args.usd, backup)
    print(f"backup: {backup}")
    stage.GetRootLayer().Save()
    print(f"saved {args.usd}  ({len(sets)} changes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
