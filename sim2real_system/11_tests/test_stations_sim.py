#!/usr/bin/env python3
"""End-to-end offline test of the real-rover side: agent + link + fusion + homing +
stations (lidar teach & repeat) + obstacle guard + latency measurement, against
fake_rover.py and a fake overhead camera that is 0.2 s late, noisy, and has a
deliberate 2 cm calibration bias (so lidar must beat the camera).

    source ros2env.sh && python3 loop/tests/test_stations_sim.py
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOOP = HERE.parent
sys.path.insert(0, str(LOOP))

import rclpy                                   # noqa: E402
from nav_msgs.msg import Odometry              # noqa: E402

import stations as ST                          # noqa: E402
from homing import load_config, go_to_pose     # noqa: E402
from localizer import FusedTracker             # noqa: E402
from rover_link import RoverLink               # noqa: E402
import selfcheck                               # noqa: E402


class FakeCamera:
    """CameraTrack-compatible. Pose = truth delayed 0.2 s + noise + a fixed 2 cm bias in
    x drifting in after teaching (simulates a slightly bumped camera)."""

    def __init__(self, latency=0.2, noise=0.003):
        self.latency, self.noise = latency, noise
        self.bias = [0.0, 0.0]
        self.buf = deque(maxlen=400)
        self.truth = None
        self.status = "3/3 stickers (fake)"
        self.n_partial = 0
        self.n_frames = self.n_detect = 0
        self.H_m2img = None
        import random
        self.rnd = random.Random(7)
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("fake_camera", context=self.ctx)
        self.node.create_subscription(Odometry, "/fake/truth", self._cb, 10)
        from rclpy.executors import SingleThreadedExecutor
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        self._run = True
        self._spin_t = threading.Thread(target=self._spin, daemon=True)
        self._spin_t.start()

    def _spin(self):
        while self._run:
            try:
                self.ex.spin_once(timeout_sec=0.05)
            except Exception:
                return

    def _cb(self, m):
        q = m.pose.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        p = m.pose.pose.position
        self.truth = (p.x, p.y, yaw)
        self.buf.append((time.time(), p.x, p.y, yaw))
        self.n_frames += 1
        self.n_detect += 1

    def pose(self, max_age=0.4):
        t_want = time.time() - self.latency
        cands = [b for b in self.buf if b[0] <= t_want]
        if not cands:
            return None
        t, x, y, yaw = cands[-1]
        g = self.rnd.gauss
        return (t + self.latency, x + self.bias[0] + g(0, self.noise),
                y + self.bias[1] + g(0, self.noise), yaw + g(0, 0.006))

    def wait_pose(self, timeout=5.0, max_age=0.4):
        t_end = time.time() + timeout
        while time.time() < t_end:
            p = self.pose()
            if p:
                return p
            time.sleep(0.03)

    def settled_pose(self, duration=0.6, max_age=0.4):
        rows, t_end = [], time.time() + duration
        while time.time() < t_end:
            p = self.pose()
            if p:
                rows.append(p)
            time.sleep(0.03)
        if not rows:
            return None
        n = len(rows)
        return (rows[-1][0], sorted(r[1] for r in rows)[n // 2], sorted(r[2] for r in rows)[n // 2],
                math.atan2(sum(math.sin(r[3]) for r in rows), sum(math.cos(r[3]) for r in rows)))

    def set_grid_homography(self, H):
        pass

    # recording / overlay api used by closed_loop.real_trial
    def start_recording(self, video_path=None):
        self._rec = []
        self._rec_run = True

        def rec():
            last = None
            while self._rec_run:
                p = self.pose()
                if p and p[0] != last:
                    self._rec.append(p)
                    last = p[0]
                time.sleep(0.03)
        self._rec_t = threading.Thread(target=rec, daemon=True)
        self._rec_t.start()

    def stop_recording(self):
        self._rec_run = False
        self._rec_t.join(timeout=1)
        return list(self._rec), []

    def overlay(self, trails=(), text=None):
        return None

    def close(self):
        self._run = False
        self._spin_t.join(timeout=1.0)
        self.node.destroy_node()
        self.ctx.try_shutdown()


def main():
    os.environ.update(ROS_DOMAIN_ID="88", ROS_LOCALHOST_ONLY="1")
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}  {detail}")
        if not cond:
            fails.append(name)

    tmp = Path(tempfile.mkdtemp(prefix="stations_test_"))
    ST.ST_DIR = tmp                                      # don't touch real stations
    cfg = load_config()
    # the fake rover's own conventions (the live config carries the REAL rover's measured
    # 90 deg lidar mount and wheel-odometry scale)
    cfg["stations"]["lidar_offset"] = [0.0, 0.0, 0.0]
    cfg["stations"]["lidar_flip"] = False
    cfg["fusion"].update(odom_source="rf2o", odom_scale=1.0, cam_latency_s=0.2)
    fake = subprocess.Popen([sys.executable, str(HERE / "fake_rover.py"), "0.5", "0.5", "1.5708"],
                            start_new_session=True)
    cam = rover = tracker = None
    try:
        time.sleep(2.0)
        cam = FakeCamera()
        rover = RoverLink(host="local", fusion_cfg=cfg["fusion"], log=print)
        rover.open()
        check("telemetry: rf2o + gyro + scan", rover.have_rf2o and rover.have_gyro and rover.have_scan)
        tracker = FusedTracker(cam, rover, cam_latency_s=0.2, gain=0.3)
        time.sleep(1.5)
        check("fusion anchored", tracker.fused)

        # teach two stations from the config poses
        for name in ("T1_straight", "T2_spin"):
            st = ST.teach(rover, cam, tracker, name, cfg)
            check(f"teach {name}", st.get("lidar", {}).get("usable", False),
                  f"self-match rms {st['lidar']['self_match']['rms']*1000:.1f} mm")
        taught_truth = {}
        for name in ("T1_straight", "T2_spin"):
            go = ST.load_station(name)["camera_pose"]
            taught_truth[name] = go

        # the camera now picks up a 2 cm bias: lidar must still bring it to the SAME spot
        cam.bias = [0.02, 0.0]
        truth_at = {}
        rows = ST.verify(rover, cam, tracker, ["T1_straight", "T2_spin"], cfg, reps=3)
        summary = ST.report(rows)
        for name in ("T1_straight", "T2_spin"):
            acc = [r for r in rows if r["station"] == name and r.get("accepted")]
            check(f"{name}: all returns accepted", len(acc) == 3, summary.get(name, {}).get("accepted"))
            check(f"{name}: via lidar", all(str(r.get("method", "")).startswith("lidar") for r in acc))
        # ground truth: where did it actually end up vs the taught truth? (re-run one goto
        # and compare the fake truth to the truth at teach time = taught pose w/o bias)
        rec = ST.goto_station(rover, cam, tracker, "T2_spin", cfg)
        time.sleep(0.5)
        tx, ty, tyaw = cam.truth
        gx, gy, gyaw = taught_truth["T2_spin"]
        err = math.hypot(tx - gx, ty - gy)
        derr = abs(math.degrees((tyaw - gyaw + math.pi) % (2 * math.pi) - math.pi))
        # returns inside the accept band are not refined further (stations.accept_m/_deg)
        am, ad = cfg["stations"].get("accept_m", 0.025), cfg["stations"].get("accept_deg", 2.0)
        check(f"TRUE return error with biased camera <= {am*100:.0f} cm / {ad:.0f} deg",
              err <= am and derr <= ad, f"{err*100:.1f} cm / {derr:.1f} deg (camera bias 2.0 cm)")
        check("camera-vs-lidar disagreement detected", rec.get("cam_vs_lidar_m", 0) > 0.012,
              f"{rec.get('cam_vs_lidar_m', 0)*100:.1f} cm")

        # obstacle guard: face the bag at (1.45..1.65, 0.3..0.6) and drive at it
        cam.bias = [0.0, 0.0]
        go_to_pose(rover, tracker, (1.05, 0.45, 0.0), cfg, log=lambda *a: None)
        rover.guard(0.10)
        n0 = len(rover.guard_events)
        t_end = time.time() + 6
        while time.time() < t_end:
            rover.velocity(0.15, 0.0)
            time.sleep(0.05)
        rover.stop()
        time.sleep(0.5)
        front_gap = 1.45 - (cam.truth[0] + 0.13)
        check("obstacle guard stopped before the bag", len(rover.guard_events) > n0
              and front_gap > 0.03, f"gap to bag {front_gap*100:.1f} cm")

        # latency measurement
        lat = selfcheck.measure_latency(rover, cam)
        # relative to odometry RECEIPT time (what fusion compares against), so ~30-50 ms
        # of odometry delivery delay comes off the camera's 0.2 s
        check("latency measured (0.2 s camera)", lat is not None and 0.1 <= lat <= 0.25, f"{lat}")
    finally:
        try:
            if tracker:
                tracker.close()
            if rover:
                rover.close()
            if cam:
                cam.close()
        finally:
            os.killpg(fake.pid, 15)
            shutil.rmtree(tmp, ignore_errors=True)
    print("ALL PASS" if not fails else f"FAILED: {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
