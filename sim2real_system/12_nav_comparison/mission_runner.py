#!/usr/bin/env python3
"""Drive one course with Nav2 and record everything needed to compare rovers.

The SAME file runs on the real rover (copied to ~/navcmp_run/, system ROS Humble) and on
the desktop next to Isaac, so both rovers are measured by identical code. It only talks
to Nav2 through the NavigateToPose action, the way RViz's 2D Nav Goal does.

Per session:  wait for /odom, /scan, map->base_footprint TF and an ACTIVE Nav2 (re-issues
lifecycle STARTUP if bringup aborted, the known race under load), register the COURSE
frame to the SLAM map frame from the rover's pose at the start mark, then for every
trial drive every leg of the course (goals, then the return leg) and record.

Outputs (--out):
  meta.json                  registration, platform, args, per-trial summary, map info
  legs.csv                   one row per leg: result, times, recoveries, final pose
  trial_NN/trace.csv         10 Hz: map + course pose, odom pose/twist, last cmd, scan min,
                             Nav2 feedback (distance remaining, recoveries), battery
  trial_NN/cmd.csv           every /cmd_vel message the base received
  map.json                   final slam_toolbox /map (for the report background)
stdout carries one JSON event per line for the desktop orchestrator; logs go to stderr.

SAFETY (the Waveshare base has NO cmd_vel deadman: the ESP32 holds the last command):
leaving the course fence, a leg timeout, SIGINT/SIGTERM, low battery, or - with
--watch-stdin - the controlling SSH pipe closing all cancel the goal and publish zero
/cmd_vel for 1.5 s before anything exits.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import course as C  # noqa: E402

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav2_msgs.srv import ManageLifecycleNodes  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

RESULT = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}
SCAN_SELF_M = 0.12          # LD19 sees the rover's own mast at 3-9 cm: ignore below this
LEG_FIELDS = ["trial", "leg", "leg_index", "goal_x", "goal_y", "goal_yaw", "result",
              "reason", "t_start", "duration_s", "wall_s", "recoveries", "final_x",
              "final_y", "final_yaw", "final_err_m", "final_yaw_err_deg", "start_x",
              "start_y", "start_yaw", "n_samples"]
TRACE_FIELDS = ["t", "wall", "leg", "map_x", "map_y", "map_yaw", "x", "y", "yaw",
                "odom_x", "odom_y", "odom_yaw", "odom_v", "odom_w", "cmd_v", "cmd_w",
                "scan_min", "dist_remaining", "recoveries", "voltage"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def emit(event, **kw):
    kw["ev"] = event
    kw["wall"] = round(time.time(), 3)
    sys.stdout.write(json.dumps(kw) + "\n")
    sys.stdout.flush()


def log(msg):
    sys.stderr.write(f"[mission_runner] {msg}\n")
    sys.stderr.flush()


class Abort(Exception):
    """Session-ending stop (fence, signal, link loss, battery). rec = the interrupted leg's
    row (result FENCE = the rover left the arena, a navigation failure; STOPPED = operator /
    link / battery, not the rover's fault - the metrics drop STOPPED legs)."""

    def __init__(self, reason, rec=None):
        super().__init__(reason)
        self.rec = rec


class Runner(Node):
    def __init__(self, a):
        super().__init__("navcmp_mission_runner")
        self.a = a
        if a.use_sim_time:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=False)
        self.odom = None            # (x, y, yaw, v, w)
        self.cmd = (0.0, 0.0)
        self.scan_min = float("nan")
        self.voltage = float("nan")
        self.map_msg = None
        self.last_scan = None
        self.n_odom = self.n_scan = 0
        self.cmd_log = None
        self.cmd_t0 = 0.0
        self.abort_reason = None
        self.fence_hit = False
        self.create_subscription(Odometry, a.odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, a.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 50)
        self.create_subscription(Float32, "/voltage", lambda m: setattr(self, "voltage", float(m.data)), 10)
        self.create_subscription(
            OccupancyGrid, "/map", lambda m: setattr(self, "map_msg", m),
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.n_depth, self.depth_t0 = 0, None
        if a.depth_topic:
            # raw=True: count clouds without deserialising ~3.7 MB each in Python (which
            # throttled this node and under-read the rate 3x)
            self.create_subscription(PointCloud2, a.depth_topic, self._on_depth,
                                     qos_profile_sensor_data, raw=True)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.feedback = None

    # ── callbacks ────────────────────────────────────────────────────────────
    def _on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation),
                     m.twist.twist.linear.x, m.twist.twist.angular.z)
        self.n_odom += 1

    def _on_scan(self, m):
        lo = max(m.range_min, SCAN_SELF_M)
        valid = [r for r in m.ranges if math.isfinite(r) and lo <= r <= m.range_max]
        self.scan_min = min(valid) if valid else float("nan")
        self.n_scan += 1
        self.last_scan = m

    def _on_depth(self, _m):
        if self.depth_t0 is None:
            self.depth_t0 = time.monotonic()
        self.n_depth += 1

    def depth_hz(self):
        if self.depth_t0 is None or self.n_depth < 2:
            return 0.0
        return (self.n_depth - 1) / max(time.monotonic() - self.depth_t0, 1e-3)

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        if self.cmd_log is not None:
            self.cmd_log.append((round(self.now() - self.cmd_t0, 3), round(time.time(), 3),
                                 round(m.linear.x, 4), round(m.angular.z, 4)))

    # ── helpers ──────────────────────────────────────────────────────────────
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.abort_reason:
                return

    def map_pose(self):
        try:
            t = self.tf.lookup_transform("map", self.a.base_frame, Time())
        except Exception:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_of(t.transform.rotation))

    def zero(self, seconds=1.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.03)

    def cancel(self):
        gh, self.goal_handle = self.goal_handle, None
        if gh is not None:
            try:
                fut = gh.cancel_goal_async()
                end = time.monotonic() + 2.0
                while not fut.done() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.02)
            except Exception as exc:          # stopping must never raise
                log(f"cancel failed: {exc}")

    def safe_stop(self, why):
        log(f"SAFE STOP: {why}")
        self.cancel()
        self.zero(1.5)

    # ── readiness ────────────────────────────────────────────────────────────
    def _lifecycle_state(self, node):
        cli = self.create_client(GetState, f"/{node}/get_state")
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                return None
            fut = cli.call_async(GetState.Request())
            end = time.monotonic() + 4.0
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return fut.result().current_state.label if fut.done() and fut.result() else None
        finally:
            self.destroy_client(cli)

    def _startup(self):
        cli = self.create_client(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")
        try:
            if not cli.wait_for_service(timeout_sec=3.0):
                return False
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.STARTUP
            fut = cli.call_async(req)
            end = time.monotonic() + 30.0
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return bool(fut.done() and fut.result() and fut.result().success)
        finally:
            self.destroy_client(cli)

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        stage = ""

        def check(cond, name):
            nonlocal stage
            if stage != name:
                stage = name
                log(f"waiting for {name} ...")
                emit("waiting", what=name)
            if time.monotonic() > deadline:
                raise TimeoutError(f"not ready after {timeout:.0f} s: {name}")
            return cond

        if self.a.use_sim_time:
            while not check(self.now() > 0.0, "/clock"):
                self.spin(0.2)
        while not check(self.n_odom > 3, self.a.odom_topic):
            self.spin(0.2)
        while not check(self.n_scan > 3, self.a.scan_topic):
            self.spin(0.2)
        if self.a.depth_topic:      # depth stack without a cloud = silently lidar-only
            while not check(self.n_depth > 2, f"{self.a.depth_topic} (depth cloud)"):
                self.spin(0.2)
        while not check(self.map_pose() is not None, f"TF map->{self.a.base_frame} (slam_toolbox)"):
            self.spin(0.3)
        last_startup = time.monotonic()
        while True:
            state = self._lifecycle_state("bt_navigator")
            if check(state == "active", "Nav2 bt_navigator ACTIVE"):
                break
            # a bringup that ABORTED under load stays inactive: re-issue STARTUP - but only
            # after autostart has had 30 s, or the two transitions race each other
            if state in ("unconfigured", "inactive") and time.monotonic() - last_startup > 30.0:
                last_startup = time.monotonic()
                log(f"bt_navigator is {state}: re-issuing lifecycle STARTUP")
                self._startup()
            self.spin(1.0)
        while not check(self.nav.wait_for_server(timeout_sec=1.0), "navigate_to_pose action"):
            self.spin(0.2)
        log("Nav2 active, sensors live")

    def register(self, start):
        """Course->map from the rover's (stationary) pose at the start mark."""
        poses, odoms = [], []
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            self.spin(0.1)
            p = self.map_pose()
            if p is not None and self.odom is not None:
                poses.append(p)
                odoms.append(self.odom)
        if not poses:
            raise RuntimeError("no map pose to register the course frame")
        moving = (max(abs(o[3]) for o in odoms), max(abs(o[4]) for o in odoms))
        mean = _mean_pose(poses)
        omean = _mean_pose([o[:3] for o in odoms])
        T_map_course = C.compose(mean, C.inverse(start))
        T_odom_course = C.compose(omean, C.inverse(start))
        return {"map_at_start": mean, "odom_at_start": omean,
                "T_map_course": T_map_course, "T_course_map": C.inverse(T_map_course),
                "T_course_odom": C.inverse(T_odom_course),
                "moving_at_registration": {"v": moving[0], "w": moving[1]}}

    # ── one leg ──────────────────────────────────────────────────────────────
    def run_leg(self, trial, idx, leg, reg, writer, t0_trial, course):
        T_mc, T_cm, T_co = reg["T_map_course"], reg["T_course_map"], reg["T_course_odom"]
        gx, gy, gyaw = C.compose(T_mc, (leg.x, leg.y, leg.yaw))
        start = self.map_pose() or reg["map_at_start"]
        sc = C.compose(T_cm, start)
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = gx, gy
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        goal.pose = ps
        self.feedback = None
        emit("leg_start", trial=trial, leg=leg.name, index=idx, goal=[leg.x, leg.y, leg.yaw])
        log(f"trial {trial} leg {idx} {leg.name}: -> ({leg.x:.2f}, {leg.y:.2f}) course frame")
        t_start, w_start = self.now(), time.monotonic()
        fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb: setattr(self, "feedback", fb.feedback))
        while not fut.done():
            self.spin(0.02)
            if self.abort_reason:
                # the goal may still be accepted after we leave: get its handle (bounded)
                # so the outer safe stop can cancel it - never leave Nav2 driving unwatched
                end = time.monotonic() + 3.0
                while not fut.done() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.02)
                if fut.done() and fut.result() is not None and fut.result().accepted:
                    self.goal_handle = fut.result()
                raise Abort(self.abort_reason)
            if time.monotonic() - w_start > 10.0:
                raise RuntimeError("navigate_to_pose did not answer the goal in 10 s")
        self.goal_handle = fut.result()
        rows, result, reason, recov = [], "REJECTED", "", 0
        if self.goal_handle is not None and self.goal_handle.accepted:
            res_fut = self.goal_handle.get_result_async()
            period = 1.0 / self.a.sample_hz
            next_t = 0.0
            while True:
                self.spin(0.01)
                el = self.now() - t_start
                fb = self.feedback
                if fb is not None:
                    recov = max(recov, int(fb.number_of_recoveries))
                if el >= next_t:
                    next_t += period
                    p = self.map_pose()
                    if p is not None:
                        c = C.compose(T_cm, p)
                        o = self.odom or (math.nan,) * 5
                        oc = C.compose(T_co, o[:3]) if self.odom else (math.nan,) * 3
                        rows.append([round(self.now() - t0_trial, 3), round(time.time(), 3), leg.name,
                                     *[round(v, 4) for v in p], *[round(v, 4) for v in c],
                                     *[round(v, 4) for v in oc], round(o[3], 4), round(o[4], 4),
                                     round(self.cmd[0], 4), round(self.cmd[1], 4),
                                     round(self.scan_min, 3),
                                     round(fb.distance_remaining, 3) if fb is not None else "",
                                     recov, round(self.voltage, 2)])
                        if not course.inside_fence(c[0], c[1]) and not self.abort_reason:
                            self.abort_reason = (f"left the fence at course ({c[0]:.2f}, {c[1]:.2f})")
                            self.fence_hit = True
                if self.abort_reason:
                    self.safe_stop(self.abort_reason)
                    result = "FENCE" if self.fence_hit else "STOPPED"
                    reason = self.abort_reason
                    break
                if res_fut.done():
                    status = res_fut.result().status
                    result = RESULT.get(status, f"STATUS_{status}")
                    break
                if el > leg.timeout_s:
                    self.safe_stop(f"leg timeout {leg.timeout_s:.0f} s")
                    result, reason = "TIMEOUT", f"> {leg.timeout_s:.0f} s"
                    break
                # sim time stops if Isaac stalls: bound the leg in wall time too
                if time.monotonic() - w_start > leg.timeout_s * self.a.wall_factor + 30.0:
                    self.safe_stop("wall-clock timeout (is the clock advancing?)")
                    result, reason = "TIMEOUT", "wall clock"
                    break
                if (self.a.battery_min > 0 and math.isfinite(self.voltage)
                        and 1.0 < self.voltage < self.a.battery_min):
                    self.abort_reason = f"battery {self.voltage:.2f} V < {self.a.battery_min} V"
        self.goal_handle = None
        duration = self.now() - t_start
        end = self.map_pose() or start
        ec = C.compose(T_cm, end)
        for r in rows:
            writer.writerow(r)
        rec = {"trial": trial, "leg": leg.name, "leg_index": idx, "goal_x": leg.x, "goal_y": leg.y,
               "goal_yaw": round(leg.yaw, 4), "result": result, "reason": reason,
               "t_start": round(t_start - t0_trial, 3), "duration_s": round(duration, 3),
               "wall_s": round(time.monotonic() - w_start, 3), "recoveries": recov,
               "final_x": round(ec[0], 4), "final_y": round(ec[1], 4), "final_yaw": round(ec[2], 4),
               "final_err_m": round(math.hypot(ec[0] - leg.x, ec[1] - leg.y), 4),
               "final_yaw_err_deg": round(math.degrees(abs(C.wrap(ec[2] - leg.yaw))), 2),
               "start_x": round(sc[0], 4), "start_y": round(sc[1], 4), "start_yaw": round(sc[2], 4),
               "n_samples": len(rows)}
        emit("leg_end", **rec)
        log(f"  -> {result} in {duration:.1f} s, err {rec['final_err_m']:.3f} m, recoveries {recov}")
        if result in ("FENCE", "STOPPED"):
            raise Abort(reason, rec)
        return rec

    def save_scan(self, path):
        """One raw scan at the start mark: the report overlays the real and sim lidar
        views from the same pose (range limits, self-occlusion, beam count)."""
        m = self.last_scan
        if m is None:
            return False
        Path(path).write_text(json.dumps({
            "frame": m.header.frame_id, "angle_min": m.angle_min,
            "angle_increment": m.angle_increment, "range_min": m.range_min,
            "range_max": m.range_max,
            "ranges": [round(r, 4) if math.isfinite(r) else None for r in m.ranges]}))
        return True

    def save_map(self, path):
        m = self.map_msg
        if m is None:
            return False
        info = m.info
        o = info.origin
        Path(path).write_text(json.dumps({
            "resolution": info.resolution, "width": info.width, "height": info.height,
            "origin": [o.position.x, o.position.y, yaw_of(o.orientation)],
            "data": list(m.data)}))
        return True


def _mean_pose(poses):
    n = len(poses)
    x = sum(p[0] for p in poses) / n
    y = sum(p[1] for p in poses) / n
    yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
    return (x, y, yaw)


def run(a):
    course = C.load(a.course)
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=None)
    node = Runner(a)

    def on_signal(signum, _frame):
        node.abort_reason = f"signal {signal.Signals(signum).name}"
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if a.watch_stdin:
        def watch():
            for line in sys.stdin:
                if line.strip() == "stop":
                    node.abort_reason = "stop requested by the desktop"
                    return
            node.abort_reason = "control link closed (stdin EOF)"
        threading.Thread(target=watch, daemon=True).start()

    meta = {"platform": a.platform, "label": a.label, "course": course.name,
            "course_file": os.path.basename(a.course), "trials_requested": a.trials,
            "odom_topic": a.odom_topic, "scan_topic": a.scan_topic, "base_frame": a.base_frame,
            "use_sim_time": a.use_sim_time, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
            "host": os.uname().nodename, "trials": [], "status": "starting"}
    legs_path = out / "legs.csv"
    new_legs = not legs_path.exists()
    code = 0
    try:
        node.wait_ready(a.ready_timeout)
        if a.settle > 0:
            node.spin(a.settle)
        reg = node.register(course.start)
        meta["registration"] = reg
        node.save_scan(out / "start_scan.json")
        emit("ready", registration=reg)
        if reg["moving_at_registration"]["v"] > 0.02 or reg["moving_at_registration"]["w"] > 0.05:
            log("WARNING: rover was moving while the course frame was registered")
        with legs_path.open("a", newline="") as lf:
            lw = csv.DictWriter(lf, fieldnames=LEG_FIELDS)
            if new_legs:
                lw.writeheader()
            for trial in range(a.first_trial, a.first_trial + a.trials):
                tdir = out / f"trial_{trial:02d}"
                tdir.mkdir(exist_ok=True)
                emit("trial_start", trial=trial)
                t0 = node.now()
                node.cmd_t0, node.cmd_log = t0, []
                legs = []
                with (tdir / "trace.csv").open("w", newline="") as tf:
                    tw = csv.writer(tf)
                    tw.writerow(TRACE_FIELDS)
                    try:
                        for idx, leg in enumerate(course.legs()):
                            try:
                                rec = node.run_leg(trial, idx, leg, reg, tw, t0, course)
                            except Abort as exc:
                                if exc.rec:
                                    lw.writerow(exc.rec)
                                    lf.flush()
                                raise
                            lw.writerow(rec)
                            lf.flush()
                            legs.append(rec)
                            if rec["result"] != "SUCCEEDED" and a.on_fail == "stop-trial":
                                break
                            node.spin(a.leg_pause)
                    finally:
                        cmds, node.cmd_log = node.cmd_log or [], None
                        with (tdir / "cmd.csv").open("w", newline="") as cf:
                            cw = csv.writer(cf)
                            cw.writerow(["t", "wall", "v", "w"])
                            cw.writerows(cmds)
                ok = sum(r["result"] == "SUCCEEDED" for r in legs)
                meta["trials"].append({"trial": trial, "legs": len(legs), "succeeded": ok})
                emit("trial_end", trial=trial, legs=len(legs), succeeded=ok)
                (out / "meta.json").write_text(json.dumps(meta, indent=1))
        meta["status"] = "done"
    except Abort as exc:
        node.safe_stop(str(exc))           # idempotent: cancels any live goal, zeroes again
        meta["status"] = f"aborted: {exc}"
        emit("aborted", reason=str(exc))
        code = 2
    except Exception as exc:
        node.safe_stop(f"error: {exc}")
        meta["status"] = f"error: {exc}"
        emit("error", reason=str(exc))
        log(f"ERROR: {exc}")
        code = 3
    finally:
        node.zero(0.5)
        try:
            node.spin(0.5)
            meta["map_saved"] = node.save_map(out / "map.json")
        except Exception:
            meta["map_saved"] = False
        meta["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if a.depth_topic:
            meta["depth_hz_wall"] = round(node.depth_hz(), 2)
        (out / "meta.json").write_text(json.dumps(meta, indent=1))
        emit("done", status=meta["status"])
        node.destroy_node()
        rclpy.try_shutdown()
    return code


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--platform", choices=("real", "sim"), required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--first-trial", type=int, default=1, help="numbering when resuming")
    ap.add_argument("--odom-topic", default=None, help="default /odom (real), /ugv_sim/odom (sim)")
    ap.add_argument("--scan-topic", default=None, help="default /scan (real), /ugv_sim/scan (sim)")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--depth-topic", default="",
                    help="depth stack: wait for this cloud before starting, log its rate")
    ap.add_argument("--sample-hz", type=float, default=10.0)
    ap.add_argument("--ready-timeout", type=float, default=300.0)
    ap.add_argument("--settle", type=float, default=2.0, help="s to wait after ready")
    ap.add_argument("--leg-pause", type=float, default=1.0, help="s between legs")
    ap.add_argument("--wall-factor", type=float, default=4.0,
                    help="leg wall-clock limit = timeout_s x this + 30 s (sim clock stalls)")
    ap.add_argument("--on-fail", choices=("continue", "stop-trial"), default="continue")
    ap.add_argument("--battery-min", type=float, default=0.0, help="V; 0 = off (real: 10.4)")
    ap.add_argument("--watch-stdin", action="store_true",
                    help="stop safely when stdin closes (the SSH link) or reads 'stop'")
    a = ap.parse_args(argv)
    a.use_sim_time = a.platform == "sim"
    a.odom_topic = a.odom_topic or ("/ugv_sim/odom" if a.use_sim_time else "/odom")
    a.scan_topic = a.scan_topic or ("/ugv_sim/scan" if a.use_sim_time else "/scan")
    return a


if __name__ == "__main__":
    sys.exit(run(parse_args()))
