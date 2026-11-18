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
import os
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
    _default_res = (
        "native" if DEFAULT_CAMERA_RESOLUTION is None
        else "x".join(map(str, DEFAULT_CAMERA_RESOLUTION))
    )
    parser.add_argument(
        "--camera-res", "--camera_res", dest="camera_resolution",
        type=_parse_camera_res,
        default=_default_res,
        help=(
            f"Camera render-product resolution as WxH (default '{_default_res}'; "
            "'native' = Isaac's 1280x720, correct square-pixel intrinsics). Pass a "
            "smaller WxH (e.g. 512x384) only to cut VRAM on a small GPU."
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
    parser.add_argument(
        "--stage-record-dir",
        default=None,
        help="Directory for an opt-in Isaac Stage Recorder replay take.",
    )
    parser.add_argument(
        "--stage-record-name",
        default="stage_replay",
        help="Take name for --stage-record-dir.",
    )
    parser.add_argument(
        "--stage-record-gate-dir",
        default=None,
        help="Directory containing start/stop/done gate files for recorder bracketing.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    if args.stage_record_dir:
        gate_dir = Path(args.stage_record_gate_dir or args.stage_record_dir).expanduser().resolve()
        os.environ["ISAAC_STAGE_RECORD_DIR"] = str(Path(args.stage_record_dir).expanduser().resolve())
        os.environ["ISAAC_STAGE_RECORD_NAME"] = args.stage_record_name
        os.environ["ISAAC_STAGE_RECORD_START_FILE"] = str(gate_dir / "record.start")
        os.environ["ISAAC_STAGE_RECORD_STOP_FILE"] = str(gate_dir / "record.stop")
        os.environ["ISAAC_STAGE_RECORD_DONE_FILE"] = str(gate_dir / "record.done")
        os.environ.setdefault("ISAAC_STAGE_RECORD_EXIT_ON_DONE", "1")
    raise SystemExit(
        run(
            environment_usd=asset_path("warehouse_basic_obstacles.usd"),
            headless=args.headless,
            camera_topics=args.camera_topics,
            camera_resolution=args.camera_resolution,
        )
    )
