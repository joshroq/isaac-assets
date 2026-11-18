#!/usr/bin/env python3
"""Real-time side-by-side view of the real rover (camera) and the sim rover (/odom).

Left panel : the live camera feed with the metric grid overlay + the real rover's
             sticker-triangle pose (from grid_calib.json).
Right panel: a top-down 1x1 m metric plot showing BOTH rovers' paths, each drawn
             relative to its own start pose so the shapes overlay directly - that
             divergence is exactly the sim-vs-real gap we're after.

Run from a ROS-sourced terminal (source ros2env.sh) with the camera streaming and
Isaac (run_ground_plane.py) publishing /odom:
    python3 parallel_view.py
    python3 parallel_view.py --flip-sim-y     # if sim path is mirrored vs camera
Keys: q quit, c clear trails, r reset both start poses.
"""
import argparse
import math
import os
import sys
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "capture"))
sys.path.insert(0, os.path.join(HERE, "..", "viz"))
from metric_overlay import (load_calib, draw_metric_grid, draw_axes_and_labels,  # noqa: E402
                            detect_rover, _txt)
from sticker_tracker import StickerTracker, DEFAULT_CONFIG, draw_tracking
CALIB = os.path.join(HERE, "..", "calib", "grid_calib.json")


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SimPose:
    """Latest sim rover pose from /odom, via a background rclpy node."""

    def __init__(self, topic="/odom"):
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("parallel_view_sim")
        self.pose = None  # (x, y, yaw)
        self.node.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def rel(pose, start):
    """Pose (x,y,yaw) expressed relative to a start pose -> (rx, ry)."""
    if pose is None or start is None:
        return None
    dx, dy = pose[0] - start[0], pose[1] - start[1]
    c, s = math.cos(-start[2]), math.sin(-start[2])
    return (c * dx - s * dy, s * dx + c * dy)


def draw_topdown(size, real_trail, sim_trail, span=1.5):
    canvas = np.full((size, size, 3), 30, np.uint8)
    scale = size / span
    cx = cy = size // 2

    def to_px(rp):
        return (int(cx + rp[0] * scale), int(cy + rp[1] * scale))

    # metric grid every 0.1 m
    for i in range(-int(span / 2 / 0.1), int(span / 2 / 0.1) + 1):
        g = i * 0.1
        col = (90, 90, 90) if i % 5 else (140, 140, 140)
        cv2.line(canvas, to_px((g, -span)), to_px((g, span)), col, 1)
        cv2.line(canvas, to_px((-span, g)), to_px((span, g)), col, 1)
    cv2.drawMarker(canvas, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 14, 1)

    for trail, col in ((real_trail, (0, 0, 255)), (sim_trail, (0, 220, 0))):
        pts = [to_px(p) for p in trail if p is not None]
        for a, b in zip(pts, pts[1:]):
            cv2.line(canvas, a, b, col, 2, cv2.LINE_AA)
        if pts:
            cv2.circle(canvas, pts[-1], 6, col, -1)

    _txt(canvas, "real (camera)", (10, 22), (0, 0, 255), 0.55)
    _txt(canvas, "sim (/odom)", (10, 44), (0, 220, 0), 0.55)
    # live position error between the two aligned paths
    if real_trail and sim_trail and real_trail[-1] and sim_trail[-1]:
        e = math.hypot(real_trail[-1][0] - sim_trail[-1][0],
                       real_trail[-1][1] - sim_trail[-1][1])
        _txt(canvas, f"pos err: {e*100:.1f} cm", (10, size - 16), (255, 255, 0), 0.6)
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=CALIB)
    ap.add_argument("--sim-odom", default="/ugv_sim/odom")
    ap.add_argument("--stickers", default=str(DEFAULT_CONFIG))
    ap.add_argument("--flip-sim-y", action="store_true",
                    help="negate sim y (odom is y-left, camera grid is y-down)")
    args = ap.parse_args()

    if not os.path.exists(args.calib):
        print(f"ERROR: {args.calib} missing - run calibrate_grid.py first.",
              file=sys.stderr); return 2
    calib = load_calib(args.calib)
    H_img2m, H_m2img, grid_m = calib
    try:
        tracker = StickerTracker(args.stickers)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    from stream import LatestFrame, STREAM_URL
    cam = LatestFrame(STREAM_URL)
    if not cam.opened() or cam.wait_first(20) is None:
        print("ERROR: no camera stream. Start the Pi camera + ensure UDP 5000 free.",
              file=sys.stderr); return 3

    sim = SimPose(args.sim_odom)

    real_trail, sim_trail = deque(maxlen=2000), deque(maxlen=2000)
    real_start = {"p": None}
    sim_start = {"p": None}
    win = "parallel: real (left) | sim (right)   q quit  c clear  r reset"

    while True:
        sim.spin()
        frame, _, _ = cam.read()
        if frame is None:
            continue

        # ---- left: camera + metric overlay + real rover ----
        left = frame.copy()
        draw_metric_grid(left, H_m2img, grid_m)
        draw_axes_and_labels(left, H_m2img, grid_m)
        r = detect_rover(frame, H_img2m, tracker)
        draw_tracking(left, r, tracker)
        if r is not None:
            real_pose = (r["center_m"][0], r["center_m"][1],
                         math.radians(r["heading_deg"]))
            if real_start["p"] is None:
                real_start["p"] = real_pose
            real_trail.append(rel(real_pose, real_start["p"]))
            cv2.circle(left, tuple(np.int32(r["center_px"])), 6, (0, 0, 255), -1)
            cv2.arrowedLine(left, tuple(np.int32(r["center_px"])),
                            tuple(np.int32(r["top_px"])), (0, 0, 255), 2, tipLength=0.3)
            _txt(left, f"real ({real_pose[0]:.3f},{real_pose[1]:.3f}) m",
                 (10, 24), (0, 0, 255), 0.6)
        else:
            _txt(left, "real: sticker triangle lost", (10, 24), (0, 0, 255), 0.6)

        # ---- sim pose from /odom ----
        sp = sim.pose
        if sp is not None:
            sim_pose = (sp[0], -sp[1] if args.flip_sim_y else sp[1], sp[2])
            if sim_start["p"] is None:
                sim_start["p"] = sim_pose
            sim_trail.append(rel(sim_pose, sim_start["p"]))

        # ---- right: top-down metric comparison ----
        right = draw_topdown(left.shape[0], real_trail, sim_trail)

        combo = cv2.hconcat([left, cv2.resize(right, (left.shape[0], left.shape[0]))])
        cv2.imshow(win, combo)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("c"):
            real_trail.clear(); sim_trail.clear()
        if k == ord("r"):
            real_trail.clear(); sim_trail.clear()
            real_start["p"] = None; sim_start["p"] = None

    cam.release()
    sim.close()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
