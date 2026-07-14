"""Isaac launcher — LiDAR + RGB (RTAB-Map RGB-D) nav system, small warehouse.

    python run_small_warehouse_nav.py
    python run_small_warehouse_nav.py --lidar-debug-lines
    python run_small_warehouse_nav.py --headless

Brings up the rover with ROS 2 owning the wheels and publishes the universal Nav2
data (/odom, odom->base_footprint TF, /scan) PLUS the front depth camera on the
/oak topics (RSD455 retuned to OAK-D-Lite optics). It injects setup_depth_camera
as run()'s extra_sensor_setup hook so only this system publishes those topics.

Pair with the ROS 2 side (this system's folder):
    ros2 launch isaac_nav_rgb.launch.py
"""

import argparse
import sys
from pathlib import Path

# Make common/ (shared modules) and this system folder importable.
SYSTEM_DIR = Path(__file__).resolve().parent
COMMON_DIR = SYSTEM_DIR.parents[1]  # .../common/nav_systems/nav_rgb -> .../common
for _p in (str(COMMON_DIR), str(SYSTEM_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from ugv_sim import run, asset_path
from depth_camera_graph import setup_depth_camera


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Depth-camera (RTAB-Map) nav system in the small warehouse."
    )
    parser.add_argument(
        "--lidar-debug-lines", "--debug-lidar", "--show-lidar-debug",
        action="store_true", dest="lidar_debug_lines",
        help="Show the RTX lidar debug view in the Isaac viewport.",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without the Isaac viewport or keyboard teleop.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    raise SystemExit(
        run(
            environment_usd=asset_path("small_warehouse.usd"),
            nav=True,
            headless=args.headless,
            lidar_debug_lines=args.lidar_debug_lines,
            extra_sensor_setup=setup_depth_camera,
        )
    )
