"""Isaac launcher — baked-graph rover in the SMALL WAREHOUSE.

    isaacpy ugv_controller_v2/run_small_warehouse.py
    isaacpy ugv_controller_v2/run_small_warehouse.py --headless
    isaacpy ugv_controller_v2/run_small_warehouse.py --no-camera-topics

Opens jetson_rover_v2.usd (rover + all baked ROS 2 graphs) and composes the
small warehouse into it. The rover then publishes /clock, /odom, /tf, /scan and
the camera topics on its own, and is driven ONLY by /cmd_vel — from
the ROS 2 keyboard teleop (../keyboard/) or from Nav2 (../nav_systems/).
"""

import argparse
import sys
from pathlib import Path

# Make common/ importable (this file sits at the ugv_controller_v2 root).
COMMON_DIR = Path(__file__).resolve().parent / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from run_baked_rover import run, asset_path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Baked-graph rover in the small warehouse."
    )
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
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    raise SystemExit(
        run(
            environment_usd=asset_path("small_warehouse.usd"),
            headless=args.headless,
            camera_topics=args.camera_topics,
        )
    )
