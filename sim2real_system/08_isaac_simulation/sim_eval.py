#!/usr/bin/env python3
"""Run a calibrated Isaac trial through the baked v2 ROS 2 interface.

System Humble drives /cmd_vel and records /ugv_sim/odom. Commands and CSV time
use odometry's simulation timestamps, so slow rendering does not shorten a test.
Isaac uses its bundled Python/ROS libraries via isaacpy_clean.sh. Calibration is
applied in memory; logs and inputs are retained in each trial directory.
"""
from __future__ import annotations

import csv
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "ugv_controller_v2" / "calibration"))
import calib_params as C  # noqa: E402

ISAACPY = os.environ.get("ISAACPY", str(REPO / "isaacpy_clean.sh"))
RUN_GROUND = REPO / "ugv_controller_v2" / "run_ground_plane.py"
ODOM_TOPIC = os.environ.get("CALIB_ODOM_TOPIC", "/ugv_sim/odom")


# Write a dict as YAML (key order kept).
def _write_yaml(path, data):
    import yaml
    Path(path).write_text(yaml.safe_dump(data, sort_keys=False))


# Remove stale Fast DDS shared-memory segments (see docstring). The fastdds
# script has no shebang, so it must be run through `sh`.
def _clean_dds_shm():
    """Drop Fast DDS shared-memory ports left by dead processes. A publisher blocks on a
    zombie reader's port, which hung Isaac's odometry mid-battery (2026-11-03). Only
    segments whose owner is gone are removed, so live ROS processes are unaffected."""
    # /opt/ros/humble/bin/fastdds is a shell script with no shebang: exec'ing it directly
    # fails with ENOEXEC (silently, before 2026-11-05), so run it through sh.
    fastdds_path = shutil.which("fastdds") or "/opt/ros/humble/bin/fastdds"
    try:
        result = subprocess.run(["sh", fastdds_path, "shm", "clean"], capture_output=True, text=True, timeout=15)
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


# Terminate (then kill) the Isaac process group so no orphan keeps the GPU.
def _stop_isaac(proc):
    """Stop only this trial's process group, including python.sh's child."""
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    # python.sh exits on SIGTERM, but a HUNG Isaac child (e.g. blocked in DDS) ignores it
    # and survived as an orphan holding the GPU (2026-11-03): always sweep the group.
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


# Drive the running Isaac sim over ROS 2 and record its odometry: publish zero
# /cmd_vel until odometry advances and the drive graph subscribes, then hold each step's
# command for its duration in SIM time (odometry stamps), recording (t, x, y, yaw) with
# t = 0 at the first step. Writes out_csv; raises if Isaac dies, stalls or overruns.
def _record_ros_trial(proc, steps, out_csv, odom_topic, deadline):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data

    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node("calib_sim_trial", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"sample": None, "previous_stamp": None, "advancing": False,
             "progress_at": time.monotonic()}
    rows = []
    start_stamp = None

    # Odometry callback: keep the newest pose (yaw from the quaternion) and record it
    # once the sequence has started; ignore repeated or older stamps.
    def on_odom(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if state["previous_stamp"] is not None:
            if stamp <= state["previous_stamp"]:
                return
            state["advancing"] = True
        state["previous_stamp"] = stamp
        state["progress_at"] = time.monotonic()
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                         1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))
        state["sample"] = (stamp, position.x, position.y, yaw)
        if start_stamp is not None:
            rows.append((stamp - start_stamp, position.x, position.y, yaw))

    node.create_subscription(Odometry, odom_topic, on_odom, qos_profile_sensor_data)

    # Raise if Isaac exited or the overall deadline passed.
    def check_alive():
        if proc.poll() is not None:
            raise RuntimeError(f"Isaac exited with code {proc.returncode}")
        if time.monotonic() >= deadline:
            raise RuntimeError("Isaac trial exceeded startup_timeout (startup + sequence)")

    try:
        # Topic discovery alone is insufficient: require advancing samples and
        # the baked command subscriber before beginning the sequence.
        while not (state["advancing"] and cmd_vel_pub.get_subscription_count()):
            check_alive()
            cmd_vel_pub.publish(Twist())
            executor.spin_once(timeout_sec=0.05)
        start_stamp, x, y, yaw = state["sample"]
        rows.append((0.0, x, y, yaw))
        boundary = 0.0
        last_publish = 0.0
        for step in steps:
            boundary += step["duration"]
            command = Twist()
            command.linear.x = step["linear"]
            command.angular.z = step["angular"]
            cmd_vel_pub.publish(command)
            while state["sample"][0] - start_stamp < boundary:
                check_alive()
                now = time.monotonic()
                if now - state["progress_at"] > 10:
                    raise RuntimeError(f"{odom_topic} stopped advancing for 10 seconds")
                if now - last_publish >= 0.05:
                    cmd_vel_pub.publish(command)
                    last_publish = now
                executor.spin_once(timeout_sec=0.01)
        with open(out_csv, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(("t", "x", "y", "yaw"))
            writer.writerows(rows)
    finally:
        # Give the subscriber time to consume a stop before terminating Isaac.
        try:
            stop_deadline = time.monotonic() + 0.3
            while time.monotonic() < stop_deadline:
                cmd_vel_pub.publish(Twist())
                executor.spin_once(timeout_sec=0.02)
        finally:
            executor.shutdown()
            node.destroy_node()
            context.shutdown()


# One isolated Isaac launch of run_ground_plane.py with the parameter set `theta`
# (calib.yaml) running `sequence_steps`; returns the trajectory CSV path. All inputs,
# the CSV and isaac.log stay in workdir. ROS networking env vars are set for the launch
# and restored afterwards; the process group is always stopped.
def run_trial(theta: dict, sequence_steps: list, surface: str = "surface",
              combine_mode: str = "min", workdir: str | None = None,
              odom_topic: str | None = None, startup_timeout: float = 240.0,
              mode: str = "ros") -> str:
    """Return a t,x,y,yaw CSV from one bounded, isolated Isaac launch.

    The default 'ros' mode uses the native baked drive and odometry graphs.
    Explicit mode='direct' retains SIM_TRIAL for offline physics diagnostics.
    startup_timeout bounds the entire launch and sequence, in wall seconds.
    """
    if mode not in ("ros", "direct"):
        raise ValueError("mode must be 'ros' or 'direct'")
    steps = [{"linear": float(step.get("linear", 0)),
              "angular": float(step.get("angular", 0)),
              "duration": float(step.get("duration", 0))} for step in sequence_steps]
    if not steps or any(not math.isfinite(value) for step in steps for value in step.values()):
        raise ValueError("sequence must contain finite commands and durations")
    if any(step["duration"] <= 0 for step in steps):
        raise ValueError("sequence durations must be positive")
    if not math.isfinite(startup_timeout) or startup_timeout <= 0:
        raise ValueError("startup_timeout must be positive and finite")
    work_dir = Path(workdir or tempfile.mkdtemp(prefix=f"calib_{surface}_")).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    calib_yaml, seq_yaml = work_dir / "calib.yaml", work_dir / "seq.yaml"
    out_csv, log_path = work_dir / "sim_run.csv", work_dir / "isaac.log"
    _write_yaml(calib_yaml, C.nest(theta))
    _write_yaml(seq_yaml, {"name": surface, "steps": steps})
    out_csv.unlink(missing_ok=True)

    # Use the same DDS mode in system rclpy and the Isaac process. The user's
    # general shell can still use localhost-only for unrelated ROS sessions.
    networking = {"ROS_DOMAIN_ID": str(os.environ.get("ROS_DOMAIN_ID", "0")),
                  "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_LOCALHOST_ONLY": "0"}
    saved_env = {env_name: os.environ.get(env_name) for env_name in (*networking, "ROS_DISCOVERY_SERVER")}
    os.environ.update(networking)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    isaac_env = dict(os.environ, ISAAC_ROS_LOCALHOST_ONLY="0", SIM_REALTIME="1")
    for key in ("SIM_ROS", "SIM_TRIAL_SEQ", "SIM_TRIAL_OUT", "SIM_DIAG"):
        isaac_env.pop(key, None)
    if mode == "direct":
        isaac_env.update(SIM_TRIAL_SEQ=str(seq_yaml), SIM_TRIAL_OUT=str(out_csv))
    _clean_dds_shm()
    isaac_proc = None
    try:
        deadline = time.monotonic() + startup_timeout
        with log_path.open("w") as log:
            isaac_proc = subprocess.Popen(
                [ISAACPY, str(RUN_GROUND), "--headless", "--no-camera-topics",
                 "--calib", str(calib_yaml), "--combine-mode", combine_mode],
                env=isaac_env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            if mode == "ros":
                _record_ros_trial(isaac_proc, steps, out_csv, odom_topic or ODOM_TOPIC, deadline)
            else:
                exit_code = isaac_proc.wait(timeout=startup_timeout)
                if exit_code:
                    raise RuntimeError(f"Isaac exited with code {exit_code}")
        if not out_csv.exists() or out_csv.stat().st_size < 20:
            raise RuntimeError("Isaac produced no trajectory samples")
        return str(out_csv)
    except Exception as exc:
        raise RuntimeError(f"Isaac trial failed: {exc}; inspect {log_path}") from exc
    finally:
        try:
            if isaac_proc is not None:
                _stop_isaac(isaac_proc)
        finally:
            for key, value in saved_env.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


if __name__ == "__main__":
    import argparse
    import shutil
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", help="YAML command sequence; default is a smoke test")
    parser.add_argument("--out", help="Copy the completed CSV to this path")
    parser.add_argument("--calib", help="Nested sim_calibration.yaml applied in memory")
    parser.add_argument("--workdir", help="Directory for inputs, CSV and Isaac log")
    parser.add_argument("--mode", choices=("ros", "direct"), default="ros")
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--combine-mode", choices=("min", "average", "multiply"), default="min")
    args = parser.parse_args()
    steps = [{"linear": 0, "angular": 0, "duration": 1.0},
             {"linear": 0.2, "angular": 0, "duration": 3.0},
             {"linear": 0, "angular": 0, "duration": 2.0},
             {"linear": 0, "angular": 0.5, "duration": 3.14}]
    theta = C.defaults()
    theta.update({"floor_mu_dynamic": 0.6, "floor_mu_static": 0.7})
    if args.sequence:
        steps = yaml.safe_load(Path(args.sequence).read_text())["steps"]
        theta = {}
    if args.calib:
        theta = C.flatten(yaml.safe_load(Path(args.calib).read_text()) or {})
    result = run_trial(theta, steps, surface="smoke", combine_mode=args.combine_mode,
                       workdir=args.workdir, odom_topic=args.odom_topic,
                       startup_timeout=args.timeout, mode=args.mode)
    if args.out:
        target = Path(args.out).resolve()
        target.parent.mkdir(parents=True, exist_ok=True)
        if target != Path(result):
            shutil.copyfile(result, target)
        result = str(target)
    print("wrote", result)
