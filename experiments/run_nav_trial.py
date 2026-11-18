#!/usr/bin/env python3
"""Sim navigation trial runner — compare nav systems IN SIMULATION (thesis data).

The experiment lives entirely in the Isaac sim (warehouse_basic_obstacles.usd, which has
no real-world twin). It runs a selected nav system to a selected goal for N trials and
records, per trial: the trajectory, timing, failures/recoveries, and — importantly — the
exact /cmd_vel command stream. It does NOT drive the real rover.

The ONE real-world tie-in: the /cmd_vel commands from runs to GOAL 4 are also written to
`experiments/goal4_replay/` in the rover's replay format, so those exact commands can later
be sent to the real rover to check it ends up in the same place (open-loop sim->real).

Two conditions (bring the stack up the matching way; the runner just records the label):
  known_map   Experiment A — stack launched with its prior-map flags (localize only)
  cold_start  Experiment B — stack launched mapless (full SLAM from scratch)

INTERCHANGEABLE
  --nav {lidar,depth,rtab,cuvslam,orbslam}   which system (launch it yourself first)
  --condition {known_map,cold_start}         Experiment A or B (label + init default)
  --goal {1,2,3,4}  (or --goal-xy X Y)       goal 4 -> also exported for real-rover replay
  --trials N                                 repeat in one invocation
  --return-home                              return to start between trials

OUTPUT (experiments/data/<condition>_<nav>_<goal>/)
  results.csv                one row per trial (the data sheet)
  paths/<...>.csv            t,x,y,yaw trajectory per trial (for plot_paths.py)
  cmds/<...>.csv             t,vx,wz /cmd_vel stream per trial
  map.npz                    /map snapshot (plot background; skipped if a stack has none)
and for goal 4:  experiments/goal4_replay/<nav>_<condition>_trialNN.csv (rover replay format)

EXAMPLES
  # Experiment A, RTAB known-map, goal 3, 5 trials:
  python3 run_nav_trial.py --nav rtab --condition known_map --goal 3 --trials 5 --return-home
  # Experiment B, cuVSLAM cold-start, goal 4 (captures replay commands):
  python3 run_nav_trial.py --nav cuvslam --condition cold_start --goal 4 --trials 3
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import time
from datetime import datetime

import yaml

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_GOALS = os.path.join(HERE, "goals.yaml")
GOAL4_REPLAY_DIR = os.path.join(HERE, "goal4_replay")
REPLAY_GOAL = 4  # the goal whose command stream is exported for real-rover replay

RESULT_NAME = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED",
               TaskResult.FAILED: "FAILED", TaskResult.UNKNOWN: "UNKNOWN"}

FIELDS = [
    "timestamp", "condition", "nav_system", "goal", "goal_x", "goal_y", "trial",
    "init_maneuver", "init_time_s", "init_dist_m", "init_success", "result",
    "duration_s", "wall_duration_s", "path_length_m", "straight_line_m",
    "path_efficiency", "start_x", "start_y", "final_x", "final_y",
    "final_pos_error_m", "final_yaw_error_deg", "n_recoveries", "n_restarts",
    "avg_speed_mps", "n_samples", "path_file", "cmd_file",
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class Trial(BasicNavigator):
    def __init__(self, args):
        super().__init__("nav_trial_runner")
        self.args = args
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])  # sim only
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=False)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        # record every /cmd_vel the nav stack (or our init maneuver) emits
        self._rec = None  # list of (t, vx, wz) while recording
        self._rec_t0 = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self._map = None
        self.create_subscription(
            OccupancyGrid, "/map", lambda m: setattr(self, "_map", m),
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    # ── cmd recording ────────────────────────────────────────────────────────
    def _on_cmd(self, m):
        if self._rec is not None:
            self._rec.append((round(self._now() - self._rec_t0, 3),
                              round(m.linear.x, 4), round(m.angular.z, 4)))

    def start_rec(self):
        self._rec, self._rec_t0 = [], self._now()

    def stop_rec(self):
        r, self._rec = self._rec or [], None
        return r

    # ── helpers ──────────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self, frame="map", base="base_footprint"):
        try:
            t = self.tf.lookup_transform(frame, base, Time())
        except Exception:
            return None
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def wait_pose(self, frame="map", timeout=20.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            p = self.pose(frame)
            if p is not None:
                return p
            self.spin(0.2)
        return None

    def drive(self, vx, wz):
        m = Twist(); m.linear.x = float(vx); m.angular.z = float(wz); self.cmd.publish(m)

    def make_goal(self, x, y, ref=None):
        g = PoseStamped(); g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        yaw = math.atan2(y - ref[1], x - ref[0]) if ref else 0.0
        g.pose.orientation.z, g.pose.orientation.w = quat_from_yaw(yaw)
        return g

    # ── robust Nav2 readiness (replaces BasicNavigator.waitUntilNav2Active) ─────
    # BasicNavigator.waitUntilNav2Active() calls spin_until_future_complete() with NO
    # timeout, so if this node's get_state service client has not matched the (already
    # up) Nav2 container — a Fast-DDS discovery race under Isaac load — it hangs FOREVER.
    # This version bounds every spin, recreates the client each attempt to force fresh
    # discovery, and fails loudly on timeout instead of hanging the whole trial.
    def wait_until_active(self, nodes=("bt_navigator",), total_timeout=180.0):
        from lifecycle_msgs.srv import GetState

        # discovery warm-up: let this node match the graph before hammering services
        self.spin(3.0)
        deadline = time.monotonic() + total_timeout
        for node in nodes:
            active = False
            while rclpy.ok() and time.monotonic() < deadline:
                cli = self.create_client(GetState, f"{node}/get_state")
                try:
                    if cli.wait_for_service(timeout_sec=3.0):
                        fut = cli.call_async(GetState.Request())
                        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                        if fut.done() and fut.result() is not None:
                            label = fut.result().current_state.label
                            if label == "active":
                                active = True
                                break
                            self.info(f"  {node} lifecycle={label}; retrying...")
                finally:
                    self.destroy_client(cli)
                self.spin(1.0)
            if not active:
                raise TimeoutError(
                    f"{node} did not report 'active' within {total_timeout:.0f}s "
                    "(Nav2 not up, or discovery never matched)")
        # the navigate_to_pose action server must also be discovered before goToPose()
        remaining = max(5.0, deadline - time.monotonic())
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=remaining):
            raise TimeoutError("navigate_to_pose action server not available")
        self.info("Nav2 is ACTIVE and navigate_to_pose is available.")

    # ── standardized init: fixed maneuver + readiness gate ────────────────────
    def initialize(self):
        """Run ONCE before trials. Returns (maneuver, init_time_s, init_dist_m, success)."""
        man = self.args.init_maneuver
        p0 = self.wait_pose("odom", timeout=10.0) or (0.0, 0.0, 0.0)
        t0 = self._now()
        if man == "sweep":
            self.info("init: fixed sweep maneuver (out/back + left/right) for SLAM startup")
            # Gentle motion so visual SLAM (cuVSLAM/ORB-SLAM3) initialises without losing
            # tracking: slower rotation (0.4 rad/s, was 0.6) held longer for parallax.
            seq = [(0.12, 0.0, 2.5), (0.0, 0.4, 5.0), (0.0, -0.4, 5.0),
                   (-0.12, 0.0, 2.5), (0.0, 0.0, 0.5)]
            for vx, wz, dur in seq:
                end = time.monotonic() + dur
                while rclpy.ok() and time.monotonic() < end:
                    self.drive(vx, wz); self.spin(0.05)
            self.drive(0.0, 0.0)
        elif man != "none":
            self.warn(f"unknown --init-maneuver '{man}', treating as none")
        # readiness gate: map->base_footprint present + stable for --init-settle
        self.info("init: waiting for localization gate (map->base_footprint stable)...")
        gate_end = time.monotonic() + self.args.init_timeout
        stable_since = None
        last = None
        success = False
        while rclpy.ok() and time.monotonic() < gate_end:
            p = self.pose("map")
            if p is not None:
                if last is not None and math.hypot(p[0] - last[0], p[1] - last[1]) < 0.25:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= self.args.init_settle:
                        success = True
                        break
                else:
                    stable_since = None
                last = p
            self.spin(0.2)
        pf = self.pose("odom") or p0
        dist = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
        return man, round(self._now() - t0, 2), round(dist, 3), success

    # ── one trial ─────────────────────────────────────────────────────────────
    def run_trial(self, gx, gy, gname, trial, dirs, init):
        start = self.wait_pose("map")
        if start is None:
            self.error("no map->base TF; localization not up")
            return None
        straight = math.hypot(gx - start[0], gy - start[1])
        self.info(f"[{gname} trial {trial}] start=({start[0]:.2f},{start[1]:.2f}) "
                  f"goal=({gx:.2f},{gy:.2f}) [{straight:.2f} m]")
        path, recoveries, restarts = [], 0, 0
        self.start_rec()
        self._stage_record_signal("start")
        t0 = self.get_clock().now(); w0 = time.monotonic()
        dt = 1.0 / max(self.args.sample_hz, 1.0); nxt = 0.0
        self.goToPose(self.make_goal(gx, gy, ref=start))
        result = None
        while rclpy.ok():
            if self.isTaskComplete():
                result = self.getResult()
                elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
                if (result == TaskResult.FAILED and restarts < self.args.max_restarts
                        and elapsed < self.args.nav_timeout):
                    restarts += 1
                    self.warn(f"  FAILED — restart {restarts}/{self.args.max_restarts}")
                    self.spin(1.0)
                    self.goToPose(self.make_goal(gx, gy, ref=self.pose() or start))
                    continue
                break
            fb = self.getFeedback()
            if fb is not None:
                recoveries = max(recoveries, fb.number_of_recoveries)
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed >= nxt:
                p = self.pose("map")
                if p is not None:
                    path.append((round(elapsed, 3), round(p[0], 4), round(p[1], 4), round(p[2], 4)))
                nxt += dt
            if elapsed > self.args.nav_timeout:
                self.warn("  timeout — cancelling"); self.cancelTask()
                result = TaskResult.CANCELED; self.spin(0.5); break
            self.spin(0.05)
        self._stage_record_signal("stop")
        duration = (self.get_clock().now() - t0).nanoseconds * 1e-9
        wall = time.monotonic() - w0
        cmds = self.stop_rec()
        final = self.pose("map") or start
        if not path:
            path = [(0.0, *start), (round(duration, 3), *final)]
        length = sum(math.hypot(path[i][1] - path[i-1][1], path[i][2] - path[i-1][2])
                     for i in range(1, len(path)))
        pos_err = math.hypot(final[0] - gx, final[1] - gy)
        goal_yaw = math.atan2(gy - start[1], gx - start[0])
        yaw_err = math.degrees(abs((final[2] - goal_yaw + math.pi) % (2*math.pi) - math.pi))

        tag = f"{self.args.condition}_{self.args.nav}_{gname}_trial{trial:02d}"
        pf = os.path.join(dirs["paths"], tag + ".csv")
        with open(pf, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t", "x", "y", "yaw"]); w.writerows(path)
        cf = os.path.join(dirs["cmds"], tag + ".csv")
        with open(cf, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t", "vx", "wz"]); w.writerows(cmds)
        # goal 4: also stash the command stream for real-rover replay
        if self.args.goal == REPLAY_GOAL:
            os.makedirs(GOAL4_REPLAY_DIR, exist_ok=True)
            rep = os.path.join(GOAL4_REPLAY_DIR,
                               f"{self.args.nav}_{self.args.condition}_trial{trial:02d}.csv")
            shutil.copyfile(cf, rep)
            self.info(f"  goal-4 commands -> {os.path.relpath(rep, HERE)} (real-rover replay)")

        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "condition": self.args.condition, "nav_system": self.args.nav,
            "goal": gname, "goal_x": gx, "goal_y": gy, "trial": trial,
            "init_maneuver": init[0], "init_time_s": init[1], "init_dist_m": init[2],
            "init_success": init[3],
            "result": RESULT_NAME.get(result, "UNKNOWN"),
            "duration_s": round(duration, 3), "wall_duration_s": round(wall, 3),
            "path_length_m": round(length, 3), "straight_line_m": round(straight, 3),
            "path_efficiency": round(straight/length, 4) if length > 1e-6 else 0.0,
            "start_x": round(start[0], 3), "start_y": round(start[1], 3),
            "final_x": round(final[0], 3), "final_y": round(final[1], 3),
            "final_pos_error_m": round(pos_err, 3), "final_yaw_error_deg": round(yaw_err, 1),
            "n_recoveries": recoveries, "n_restarts": restarts,
            "avg_speed_mps": round(length/duration, 3) if duration > 1e-6 else 0.0,
            "n_samples": len(path), "path_file": os.path.relpath(pf, self.args.out),
            "cmd_file": os.path.relpath(cf, self.args.out),
        }
        self.info(f"  -> {row['result']}  t={row['duration_s']}s len={row['path_length_m']}m "
                  f"eff={row['path_efficiency']} err={row['final_pos_error_m']}m "
                  f"recov={recoveries} restarts={restarts}")
        return row

    def _stage_record_signal(self, name):
        gate_dir = getattr(self.args, "stage_record_gate_dir", None)
        if not gate_dir:
            return
        os.makedirs(gate_dir, exist_ok=True)
        path = os.path.join(gate_dir, f"record.{name}")
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"{name} {datetime.now().isoformat(timespec='seconds')}\n")
        self.info(f"  stage recorder gate -> {path}")

    def go_home(self, home):
        self.info("  returning to start...")
        self.goToPose(self.make_goal(home[0], home[1], ref=self.pose() or home))
        t0 = time.monotonic()
        while rclpy.ok() and not self.isTaskComplete():
            if time.monotonic() - t0 > self.args.nav_timeout:
                self.cancelTask(); break
            self.spin(0.1)

    def snapshot_map(self, out):
        self.spin(1.0)
        if self._map is None:
            self.warn("no /map (fine for cuvslam/nvblox); plot uses the shared maps/ or paths")
            return
        import numpy as np
        i = self._map.info
        grid = np.array(self._map.data, dtype=np.int8).reshape(i.height, i.width)
        np.savez(os.path.join(out, "map.npz"), grid=grid, resolution=i.resolution,
                 origin_x=i.origin.position.x, origin_y=i.origin.position.y)

    def run(self, gx, gy, gname):
        dirs = {k: os.path.join(self.args.out, k) for k in ("paths", "cmds")}
        for d in (self.args.out, *dirs.values()):
            os.makedirs(d, exist_ok=True)
        results_csv = os.path.join(self.args.out, "results.csv")
        new = not os.path.exists(results_csv)

        self.info("waiting for Nav2 (bt_navigator) ACTIVE...")
        self.wait_until_active(nodes=("bt_navigator",), total_timeout=self.args.nav_active_timeout)
        init = self.initialize()
        if not init[3]:
            self.error(f"init gate FAILED ({init}); recording INIT_FAILED and stopping.")
            with open(results_csv, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                if new:
                    w.writeheader()
                base = {k: "" for k in FIELDS}
                base.update({
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                    "condition": self.args.condition, "nav_system": self.args.nav,
                    "goal": gname, "goal_x": gx, "goal_y": gy, "trial": 0,
                    "init_maneuver": init[0], "init_time_s": init[1],
                    "init_dist_m": init[2], "init_success": init[3], "result": "INIT_FAILED"})
                w.writerow(base)
            return 1
        self.spin(self.args.settle)
        self.snapshot_map(self.args.out)
        home = self.wait_pose("map")
        rows = []
        with open(results_csv, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            if new:
                w.writeheader()
            for trial in range(1, self.args.trials + 1):
                row = self.run_trial(gx, gy, gname, trial, dirs, init)
                if row is None:
                    break
                w.writerow(row); f.flush(); rows.append(row)
                if self.args.return_home and trial < self.args.trials and home:
                    self.go_home(home); self.spin(1.0)
        ok = sum(r["result"] == "SUCCEEDED" for r in rows)
        self.info(f"DONE: {ok}/{len(rows)} SUCCEEDED -> {results_csv}")
        if self.args.plot and rows:
            self.make_plot(gname)
        return 0

    def make_plot(self, gname):
        """Auto-generate the routes-over-map figure for this run (routes.png/pdf)."""
        import subprocess
        import sys
        plotter = os.path.join(HERE, "plot_paths.py")
        out_png = os.path.join(self.args.out, "routes.png")
        cmd = [sys.executable, plotter, self.args.out, "--group-by", "trial",
               "--out", out_png,
               "--title", f"{self.args.nav} / {self.args.condition} / {gname}"]
        shared = os.path.join(HERE, "maps", "warehouse", "map.npz")
        if os.path.exists(shared):
            cmd += ["--map", shared]
        try:
            subprocess.run(cmd, check=False)
            self.info(f"routes visual -> {out_png}")
        except Exception as e:  # noqa: BLE001
            self.warn(f"auto-plot failed ({e}); run plot_paths.py manually on {self.args.out}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--nav", required=True,
                   choices=["lidar", "depth", "rtab", "cuvslam", "orbslam"])
    p.add_argument("--condition", required=True, choices=["known_map", "cold_start"],
                   help="Experiment A (known_map) or B (cold_start) — launch the stack to match")
    p.add_argument("--goal", type=int, choices=[1, 2, 3, 4])
    p.add_argument("--goal-xy", type=float, nargs=2, metavar=("X", "Y"))
    p.add_argument("--trials", type=int, default=1)
    p.add_argument("--return-home", action="store_true")
    p.add_argument("--init-maneuver", default=None, choices=["sweep", "none"],
                   help="default: none for known_map, sweep for cold_start")
    p.add_argument("--init-settle", type=float, default=2.0,
                   help="s the map->base pose must stay stable to pass the readiness gate")
    p.add_argument("--init-timeout", type=float, default=60.0)
    p.add_argument("--goals-file", default=DEFAULT_GOALS)
    p.add_argument("--out", default=None)
    p.add_argument("--nav-timeout", type=float, default=240.0)
    p.add_argument("--nav-active-timeout", type=float, default=180.0,
                   help="max seconds to wait for bt_navigator ACTIVE before failing "
                        "(bounded; never hangs forever like BasicNavigator's default wait)")
    p.add_argument("--max-restarts", type=int, default=2)
    p.add_argument("--settle", type=float, default=3.0)
    p.add_argument("--sample-hz", type=float, default=5.0)
    p.add_argument("--stage-record-gate-dir", default=None,
                   help="write record.start/record.stop files around each trial for Isaac Stage Recorder")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip auto-generating routes.png after the trials")
    p.set_defaults(plot=True)
    p.add_argument("--prime", type=float, default=None,
                   help="DEPRECATED (ignored) — use --init-maneuver")
    a = p.parse_args()
    if a.prime is not None:
        print("WARNING: --prime is deprecated and ignored; use --init-maneuver.")
    if a.init_maneuver is None:
        a.init_maneuver = "none" if a.condition == "known_map" else "sweep"
    return a


def main():
    a = parse_args()
    if a.goal_xy:
        gx, gy, gname = a.goal_xy[0], a.goal_xy[1], f"custom_{a.goal_xy[0]}_{a.goal_xy[1]}"
        a.goal = None
    elif a.goal:
        g = yaml.safe_load(open(a.goals_file))["goals"][a.goal]
        gx, gy, gname = g["x"], g["y"], g["name"]
    else:
        raise SystemExit("give --goal 1..4 or --goal-xy X Y")
    if a.out is None:
        a.out = os.path.join(HERE, "data", f"{a.condition}_{a.nav}_{gname}")
    rclpy.init()
    t = Trial(a)
    try:
        return t.run(gx, gy, gname)
    except KeyboardInterrupt:
        t.cmd.publish(Twist()); return 130
    finally:
        t.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
