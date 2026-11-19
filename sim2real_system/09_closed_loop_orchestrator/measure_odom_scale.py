#!/usr/bin/env python3
"""Measure fusion.odom_scale (camera distance / wheel-odometry distance) on this surface.

MOVES THE ROVER: homes to a start pose facing +y, then drives straight forward and back
`--reps` times (each leg `--speed` m/s for `--seconds` s, ~30 cm) and compares the
camera's settled displacement with the raw wheel odometry. Wheel odometry over-reads on
soft/slippy floors (carpet 0.89, EVA foam 0.93, concrete 0.96), and homing's camera +
odometry fusion uses this ratio, so measure it once per surface.

    python3 loop/measure_odom_scale.py            # measure and print
    python3 loop/measure_odom_scale.py --write    # ... and write fusion.odom_scale
"""
from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from homing import CONFIG, HomingError, go_to_pose, in_workspace, load_config   # noqa: E402


# Rewrite the `odom_scale:` value (and its trailing comment) in loop_config.yaml as
# text, so every other line and comment in the YAML file is preserved.
def write_scale(path, scale, note):
    config_path = Path(path)
    config_text = config_path.read_text()
    updated_text, replacements = re.subn(r"(\n  odom_scale:\s*)[0-9.]+[^\n]*",
                     rf"\g<1>{scale:.2f}         # {note}", config_text, count=1)
    if replacements != 1:
        raise ValueError(f"no fusion.odom_scale line in {config_path}")
    config_path.write_text(updated_text)


# 1. Refuse if one straight leg from --start would leave the workspace fence.
# 2. Home to --start, then drive forward/back legs at --speed for --seconds each.
# 3. Per leg: ratio = camera displacement / raw wheel-odometry displacement (poses are
#    (time, x, y, yaw) tuples, so [1]/[2] are x/y). Legs under 5 cm of odometry are skipped.
# 4. odom_scale = mean ratio; --write stores it in the config.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(CONFIG))
    parser.add_argument("--start", default="0.50,0.35,90", help="x,y,yaw_deg to start from")
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--reps", type=int, default=2, help="forward+back pairs")
    parser.add_argument("--write", action="store_true", help="write the result into the config")
    parser.add_argument("--keep-bringup", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)

    from camera_track import CameraTrack
    from rover_link import RoverLink
    start_x_m, start_y_m, start_yaw_deg = (float(leg_speed) for leg_speed in args.start.split(","))
    leg_length_m = args.speed * args.seconds
    end_y = start_y_m + leg_length_m * math.sin(math.radians(start_yaw_deg))
    end_x = start_x_m + leg_length_m * math.cos(math.radians(start_yaw_deg))
    if not in_workspace(config, end_x, end_y):
        sys.exit(f"a {leg_length_m:.2f} m leg from {args.start} leaves the workspace; pick --start")

    camera = CameraTrack()
    camera.latency_s = config.get("fusion", {}).get("cam_latency_s", 0.0)
    rover = RoverLink(fusion_cfg=config.get("fusion", {}), log=lambda *a: None)
    ratios = {"fwd": [], "back": []}
    try:
        rover.open()
        print(f"battery {rover.voltage:.2f} V; homing to {args.start} ...", flush=True)
        go_to_pose(rover, camera, (start_x_m, start_y_m, math.radians(start_yaw_deg)), config)
        for leg_speed in [args.speed, -args.speed] * args.reps:
            time.sleep(1.0)
            camera_pose_before, wheel_odom_before = camera.settled_pose(1.0), rover.odom
            leg_end_time = time.time() + args.seconds
            while time.time() < leg_end_time:
                rover.velocity(leg_speed, 0.0)
                time.sleep(0.05)
            rover.stop()
            camera_pose_after, wheel_odom_after = camera.settled_pose(1.0), rover.odom
            if camera_pose_before is None or camera_pose_after is None or wheel_odom_before is None or wheel_odom_after is None:
                print("  leg skipped: camera or odometry missing", flush=True)
                continue
            camera_distance_m = math.hypot(camera_pose_after[1] - camera_pose_before[1], camera_pose_after[2] - camera_pose_before[2])
            wheel_distance_m = math.hypot(wheel_odom_after[1] - wheel_odom_before[1], wheel_odom_after[2] - wheel_odom_before[2])
            if wheel_distance_m < 0.05:
                print(f"  leg skipped: wheel odometry moved only {wheel_distance_m*100:.1f} cm", flush=True)
                continue
            ratios["fwd" if leg_speed > 0 else "back"].append(camera_distance_m / wheel_distance_m)
            print(f"  {'forward' if leg_speed > 0 else 'back   '}: camera {camera_distance_m*100:5.1f} cm, "
                  f"wheel {wheel_distance_m*100:5.1f} cm, ratio {camera_distance_m/wheel_distance_m:.3f}", flush=True)
    except HomingError as exc:
        sys.exit(f"homing failed: {exc}")
    finally:
        rover.stop()
        rover.close(stop_bringup=not args.keep_bringup)
        camera.close()

    all_ratios = ratios["fwd"] + ratios["back"]
    if len(all_ratios) < 2:
        sys.exit("not enough good legs to measure the scale")
    scale = sum(all_ratios) / len(all_ratios)
    format_ratios = lambda xs: "/".join(f"{x:.3f}" for x in xs) or "-"
    print(f"odom_scale = {scale:.3f}  (forward {format_ratios(ratios['fwd'])}, back {format_ratios(ratios['back'])})")
    if args.write:
        note = (f"{time.strftime('%Y-%m-%d')} measure_odom_scale: fwd {format_ratios(ratios['fwd'])}, "
                f"back {format_ratios(ratios['back'])}")
        write_scale(args.config, scale, note)
        print(f"wrote fusion.odom_scale = {scale:.2f} to {args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
