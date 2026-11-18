#!/usr/bin/env python3
"""Record a rover trajectory to CSV for sim-vs-real comparison.

Two sources:
  camera : overhead C920e -> colored sticker triangle -> grid metres (the REAL rover's ground
           truth). Needs grid calibration and sticker_config.json. Run on the desktop.
  odom   : subscribe to an Odometry topic (the SIM rover's pose from Isaac /odom).
           Needs a ROS-sourced shell (source ros2env.sh).

CSV columns: t,x,y,yaw   (t = seconds from first sample; x,y metres; yaw radians)

Examples:
  # real rover (camera), stop with Ctrl-C or after --duration
  python3 record_traj.py --source camera --out real_run.csv --duration 30
  # sim rover (odom)
  python3 record_traj.py --source odom --topic /odom --out sim_run.csv --duration 30
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "capture"))
sys.path.insert(0, os.path.join(HERE, "..", "viz"))


def record_camera(out, duration, calib_path, sticker_config=None, allow_approximate=False):
    from metric_overlay import load_calib, detect_rover
    from sticker_tracker import StickerTracker, DEFAULT_CONFIG
    from stream import LatestFrame, STREAM_URL
    if not os.path.exists(calib_path):
        print(f"ERROR: {calib_path} missing - run calibrate_grid.py first.", file=sys.stderr)
        return 2
    try:
        tracker = StickerTracker(sticker_config or DEFAULT_CONFIG)
        if not allow_approximate:
            tracker.require_measurement_ready()
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    H_img2m, _, _ = load_calib(calib_path)
    cam = LatestFrame(STREAM_URL)
    if not cam.opened() or cam.wait_first(20) is None:
        cam.release()
        print("ERROR: no camera stream (start the Pi camera, free UDP 5000).", file=sys.stderr)
        return 3
    # Every recorded pose already refers to the configured body origin.
    # Never apply the old rear-ArUco marker_to_center offset a second time.
    with open(calib_path) as calib_file:
        grid_metadata = json.load(calib_file)
    metadata = {"tracker": "colored_sticker_triangle", "config": tracker.config,
                "grid_calibration": grid_metadata,
                "measurement_ready": tracker.measurement_ready,
                "reference_point": tracker.reference_point}
    with open(str(out) + ".tracking.json", "w") as meta:
        json.dump(metadata, meta, indent=2)
    t0 = None
    n = misses = 0
    last_frame = -1
    started = time.monotonic()
    print(f"[record camera] sticker triangle -> {out} ({tracker.reference_point})")
    if not tracker.measurement_ready:
        print("[record camera] APPROXIMATE diagnostic recording; not calibrated chassis-center ground truth")
    try:
        with open(out, "w") as f:
            f.write("t,x,y,yaw\n")
            while not duration or time.monotonic() - started < duration:
                frame, stamp, number = cam.read()
                if frame is None or number == last_frame:
                    time.sleep(.003)
                    continue
                last_frame = number
                r = detect_rover(frame, H_img2m, tracker)
                if r is None:
                    misses += 1
                    continue
                if t0 is None:
                    t0 = stamp
                x, y = r["center_m"]
                yaw = math.radians(r["heading_deg"])
                f.write(f"{stamp-t0:.4f},{x:.5f},{y:.5f},{yaw:.5f}\n")
                f.flush()
                n += 1
    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
    print(f"[record camera] wrote {n} samples ({misses} frames without a valid triangle) -> {out}")
    if n == 0:
        print("ERROR: no sticker poses recorded. " + tracker.status, file=sys.stderr)
    return 0 if n else 4


def record_odom(out, duration, topic):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    def yaw_of(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    f = open(out, "w"); f.write("t,x,y,yaw\n")
    state = {"t0": None, "n": 0}

    class Rec(Node):
        def __init__(self):
            super().__init__("record_traj_odom")
            self.create_subscription(Odometry, topic, self.cb, 50)
            self.get_logger().info(f"recording {topic} -> {out}")

        def cb(self, m):
            now = time.time()
            if state["t0"] is None:
                state["t0"] = now
            p = m.pose.pose.position
            f.write(f"{now-state['t0']:.4f},{p.x:.5f},{p.y:.5f},"
                    f"{yaw_of(m.pose.pose.orientation):.5f}\n"); f.flush()
            state["n"] += 1

    rclpy.init(); node = Rec()
    t_start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if duration and state["t0"] and (time.time() - state["t0"]) >= duration:
                break
            if not duration and time.time() - t_start > 5 and state["n"] == 0:
                print("  (no /odom yet - is Isaac publishing? check discovery)",
                      file=sys.stderr); t_start = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown(); f.close()
    print(f"[record odom] wrote {state['n']} samples -> {out}")
    return 0 if state["n"] else 4


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True, choices=["camera", "odom"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=0.0, help="auto-stop after N s (0=until Ctrl-C)")
    ap.add_argument("--topic", default="/ugv_sim/odom", help="odom topic (odom source)")
    ap.add_argument("--calib", default=os.path.join(HERE, "..", "calib", "grid_calib.json"))
    ap.add_argument("--stickers", default=None, help="sticker tracker configuration JSON")
    ap.add_argument("--allow-approximate", action="store_true",
                    help="explicit diagnostic recording with unmeasured centroid/height calibration")
    args = ap.parse_args()
    if args.source == "camera":
        return record_camera(args.out, args.duration, args.calib, args.stickers, args.allow_approximate)
    return record_odom(args.out, args.duration, args.topic)


if __name__ == "__main__":
    raise SystemExit(main())
