"""Isaac launcher — baked-graph rover on a plain GROUND PLANE.

    isaacpy ugv_controller_v2/run_ground_plane.py
    isaacpy ugv_controller_v2/run_ground_plane.py --headless
    isaacpy ugv_controller_v2/run_ground_plane.py --no-camera-topics

A minimal, environment-free launch: opens jetson_rover_v2.usd and adds a default
ground plane. Useful for smoke-testing the baked ROS 2 interface (drive by
/cmd_vel, watch /odom, /scan, /tf, /ugv_sim/oak/*) without any warehouse geometry.
"""

import argparse
import sys
from pathlib import Path

COMMON_DIR = Path(__file__).resolve().parent / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from run_baked_rover import run


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Baked-graph rover on a ground plane.")
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without the Isaac viewport (topics still publish).",
    )
    parser.add_argument(
        "--no-camera-topics",
        "--no_camera_topics",
        dest="camera_topics",
        action="store_false",
        default=True,
        help=(
            "Disable baked RGB/depth/camera_info publishers for lidar-only "
            "performance/debug sessions."
        ),
    )
    parser.add_argument(
        "--calib", default=None,
        help="sim->real calibration YAML (sim_calibration.yaml shape) applied IN-MEMORY "
             "before play (shared USD untouched). Used by the sim2real optimiser per trial.",
    )
    parser.add_argument(
        "--combine-mode", "--combine_mode", dest="combine_mode", default=None,
        choices=["min", "average", "multiply"],
        help="frictionCombineMode to author on the wheel+floor materials ('min' makes the "
             "floor material the effective per-surface friction knob).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    raise SystemExit(
        run(
            environment_usd=None,
            headless=args.headless,
            camera_topics=args.camera_topics,
            calibration=args.calib,
            friction_combine_mode=args.combine_mode,
        )
    )
