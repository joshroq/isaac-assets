"""Isaac launcher — baked rover on a ground plane, viewed from a camera MATCHED to the
real overhead webcam, with the live path traced on the floor.

    isaacpy ugv_controller_v2/run_overhead.py                     # live matched overhead view
    isaacpy ugv_controller_v2/run_overhead.py --calib sim2real_calib/surfaces/concrete.yaml
    isaacpy ugv_controller_v2/run_overhead.py --no-path           # don't draw the path line

The camera pose/FOV come from sim2real_calib/overhead/camera_setup.yaml (run
`derive_camera_pose.py` once, or drop in measured values). This is the "see the sim run from
the same viewpoint as the webcam" view for the demo. To also get a shareable mp4 of the run,
record /odom (trials/record_traj.py --source odom) and feed it to
sim2real_calib/overhead/render_overhead.py.
"""
import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
COMMON_DIR = Path(__file__).resolve().parent / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from run_baked_rover import run

DEFAULT_SETUP = REPO / "sim2real_calib" / "overhead" / "camera_setup.yaml"


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Baked rover viewed from a webcam-matched overhead camera.")
    p.add_argument("--headless", action="store_true", help="no window (topics still publish)")
    p.add_argument("--setup", default=str(DEFAULT_SETUP), help="overhead camera_setup.yaml")
    p.add_argument("--calib", default=None, help="optional per-surface sim calibration yaml")
    p.add_argument("--combine-mode", "--combine_mode", dest="combine_mode", default=None,
                   choices=["min", "average", "multiply"])
    p.add_argument("--no-path", dest="draw_path", action="store_false", default=True,
                   help="do not trace the rover path line in the view")
    p.add_argument("--no-camera-topics", "--no_camera_topics", dest="camera_topics",
                   action="store_false", default=True)
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    if not Path(args.setup).exists():
        raise SystemExit(f"camera setup not found: {args.setup}\n"
                         "Run: python3 sim2real_calib/overhead/derive_camera_pose.py")
    raise SystemExit(
        run(
            environment_usd=None,
            headless=args.headless,
            camera_topics=args.camera_topics,
            calibration=args.calib,
            friction_combine_mode=args.combine_mode,
            overhead_camera=args.setup,
            draw_overhead_path=args.draw_path,
        )
    )
