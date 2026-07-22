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

from run_baked_rover import run, asset_path, DEFAULT_CAMERA_RESOLUTION


def _parse_camera_res(value: str) -> tuple[int, int] | None:
    """Parse a --camera-res value: 'WxH' (e.g. 640x480) or 'native' (leave the
    baked render products unauthored → Isaac's 1280x720 default)."""
    if value.strip().lower() in ("native", "default", "full"):
        return None
    try:
        w, h = value.lower().split("x")
        return (int(w), int(h))
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"--camera-res must be 'WxH' (e.g. 640x480) or 'native', got '{value}'"
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Baked-graph rover in the small warehouse."
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without the Isaac viewport (topics still publish).",
    )
    parser.add_argument(
        "--camera-res", "--camera_res", dest="camera_resolution",
        type=_parse_camera_res,
        default="x".join(map(str, DEFAULT_CAMERA_RESOLUTION)),
        help=(
            "Camera render-product resolution as WxH (default "
            f"{DEFAULT_CAMERA_RESOLUTION[0]}x{DEFAULT_CAMERA_RESOLUTION[1]}, the "
            "OAK-D-Lite hardware resolution). Lower = higher RGB/depth/stereo "
            "publish rate when cuVSLAM + nvblox share the GPU. Use 'native' for "
            "Isaac's 1280x720 default."
        ),
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
            camera_resolution=args.camera_resolution,
        )
    )
