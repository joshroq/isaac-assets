"""Isaac launcher — LiDAR + DEPTH (RTAB-Map depth-only) nav system, small warehouse, HEADLESS.

Use this when a windowed launch is unreliable (e.g. a NoMachine / remote-desktop
server hijacks the display, causing Vulkan swapchain failures). The RTX lidar and
the depth camera render offscreen, so /scan, /odom, /tf, /clock and the /oak depth
topics still publish and Nav2 + RTAB-Map run; there is just no Isaac viewport or
in-window keyboard teleop.

    python run_small_warehouse_nav_headless.py
"""

import sys
from pathlib import Path

SYSTEM_DIR = Path(__file__).resolve().parent
COMMON_DIR = SYSTEM_DIR.parents[1]  # .../common/nav_systems/nav_depth -> .../common
for _p in (str(COMMON_DIR), str(SYSTEM_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from ugv_sim import run, asset_path
from depth_camera_graph import setup_depth_camera


if __name__ == "__main__":
    raise SystemExit(
        run(
            environment_usd=asset_path("small_warehouse.usd"),
            nav=True,
            headless=True,
            extra_sensor_setup=setup_depth_camera,
        )
    )
