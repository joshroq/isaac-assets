#!/usr/bin/env python3
"""REAL-TIME sim-vs-real demo: drive BOTH rovers at once, draw both paths, compare.

Unlike the sequential trial flow (trial_real.sh then trial_sim.sh), this runs the
real rover and the Isaac sim rover on the SAME wall clock, simultaneously:

  sim  : a persistent Isaac (run_ground_plane.py, native ROS bridge, SIM_REALTIME)
         subscribes /cmd_vel and publishes /ugv_sim/odom. We publish the sequence
         to /cmd_vel locally on the desktop, timed from the moment the real rover
         reports that it started.
  real : the physical rover runs the sequence timed on the Jetson through the
         persistent rover link (loop/rover_link.py + rover_agent.py - the same path
         the closed loop uses). The overhead camera's sticker triangle gives its
         ground-truth path. Between tests the rover DRIVES ITSELF back to the next
         test's start station (loop/stations.py: camera homing + lidar fine
         correction), so the battery runs unattended.

One window stays open for the whole battery (viz/dashboard.py): live camera with both
paths, the current test top-down, and a gallery of every finished test (real vs sim +
metrics) that fills in as the battery progresses. No optimizer; this quantifies
today's sim-vs-real gap, live.

Run from a ROS-sourced shell (source ros2env.sh) on the desktop:
    python3 demo/realtime_demo.py --tests T1,T2,T4 --surface heavy_carpet
    python3 demo/realtime_demo.py --sequence parallel/sequences/T1_straight.yaml
    python3 demo/realtime_demo.py --tests T1,T2 --no-real            # sim-only rehearsal
    python3 demo/realtime_demo.py --tests T1,T2 --return-to-start prompt   # place by hand

Prereqs: camera streaming (UDP 5000), grid + stickers calibrated. The rover base
driver is started over SSH if it is not running. Start poses come from taught
stations (loop/stations.py teach <test>) or loop/loop_config.yaml start_poses.
Keys in the window: q / Esc stops both rovers and ends the run.
"""
from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import signal
import subprocess
import sys
import time
from contextlib import ExitStack, contextmanager
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SC = HERE.parent                       # sim2real_calib/
REPO = SC.parent
sys.path.insert(0, str(SC / "calib"))
sys.path.insert(0, str(SC / "capture"))
sys.path.insert(0, str(SC / "viz"))
sys.path.insert(0, str(SC / "parallel"))
sys.path.insert(0, str(SC / "overhead"))
sys.path.insert(0, str(SC / "loop"))
sys.path.insert(0, str(SC / "optimize"))
sys.path.insert(0, str(REPO / "ugv_controller_v2" / "calibration"))

from metric_overlay import load_calib  # noqa: E402
from sticker_tracker import DEFAULT_CONFIG  # noqa: E402
from parallel_view import yaw_from_quat  # noqa: E402
from dashboard import Dashboard, Track, REAL, SIM, place_at, metric_lines  # noqa: E402
import param_sheet as PS  # noqa: E402
from session_index import Session  # noqa: E402
from camera_track import CameraTrack, CameraError  # noqa: E402
from homing import load_config, start_pose, in_workspace, go_to_pose, CONFIG  # noqa: E402
import stations as ST  # noqa: E402
import calib_params as C  # noqa: E402

ISAACPY = os.environ.get("ISAACPY", str(REPO / "isaacpy_clean.sh"))
RUN_GROUND = REPO / "ugv_controller_v2" / "run_ground_plane.py"
SEQ_DIR = SC / "parallel" / "sequences"
TRIALS = SC / "trials"
CALIB = SC / "calib" / "grid_calib.json"
ODOM_TOPIC = os.environ.get("CALIB_ODOM_TOPIC", "/ugv_sim/odom")


class Reporter:
    """print() + the dashboard's log ticker."""

    def __init__(self, dash=None):
        self.dash = dash

    def __call__(self, *parts):
        line = " ".join(str(p) for p in parts)
        print(line, flush=True)
        if self.dash is not None:
            for ln in line.strip("\n").splitlines():
                self.dash.log(ln)


say = Reporter()


def load_sequence(path):
    import yaml
    data = yaml.safe_load(Path(path).read_text())
    steps = data["steps"]
    for s in steps:
        s.setdefault("linear", 0.0)
        s.setdefault("angular", 0.0)
        s.setdefault("duration", 0.0)
    return data.get("name", Path(path).stem), steps


def seq_for(test):
    hits = sorted(glob.glob(str(SEQ_DIR / f"{test}_*.yaml")))
    if not hits:
        sys.exit(f"no sequence file for {test} in {SEQ_DIR}")
    return hits[0]


@contextmanager
def sim_ros_environment():
    """Match desktop DDS discovery to Isaac, even in a localhost-only shell."""
    networking = {"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
                  "ROS_LOCALHOST_ONLY": "0", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}
    saved = {key: os.environ.get(key) for key in (*networking, "ROS_DISCOVERY_SERVER")}
    os.environ.update(networking)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    try:
        say(f"[sim] ROS_DOMAIN_ID={networking['ROS_DOMAIN_ID']} "
              "ROS_LOCALHOST_ONLY=0 RMW=rmw_fastrtps_cpp")
        yield
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


class Isaac:
    """A persistent headless Isaac publishing /ugv_sim/odom and taking /cmd_vel."""

    def __init__(self, calib_theta, logdir, combine_mode="min"):
        import yaml
        self.log_path = Path(logdir) / "isaac.log"
        calib_yaml = Path(logdir) / "calib.yaml"
        calib_yaml.write_text(yaml.safe_dump(C.nest(calib_theta), sort_keys=False))
        # unbuffered: a startup stall must be visible in isaac.log, not lost in a buffer
        env = dict(os.environ, ISAAC_ROS_LOCALHOST_ONLY="0", SIM_REALTIME="1",
                   ROS_LOCALHOST_ONLY="0", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                   PYTHONUNBUFFERED="1")
        for key in ("SIM_ROS", "SIM_TRIAL_SEQ", "SIM_TRIAL_OUT", "SIM_DIAG",
                    "ROS_DISCOVERY_SERVER"):
            env.pop(key, None)
        cmd = [ISAACPY, str(RUN_GROUND), "--headless", "--no-camera-topics",
               "--calib", str(calib_yaml), "--combine-mode", combine_mode]
        say(f"[isaac] launching: {' '.join(cmd)}")
        say(f"[isaac] log: {self.log_path}")
        self.log = open(self.log_path, "w")
        try:
            self.proc = subprocess.Popen(cmd, env=env, stdout=self.log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            self.log.close()
            raise

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait(timeout=5)
        finally:
            self.log.close()


class Ros:
    """One rclpy node: publishes sim /cmd_vel, subscribes /ugv_sim/odom."""

    def __init__(self, odom_topic=ODOM_TOPIC, sim_topic="/cmd_vel"):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        self.rclpy = rclpy
        self.Twist = Twist
        self.odom_topic, self.sim_topic = odom_topic, sim_topic
        self.context = Context()
        self.node = self.executor = None
        self.pose = None            # (x, y, yaw)
        self.stamp = None           # sim time of last odom
        self.advancing = False
        self._prev = None
        # Keep ROS alive during Python's KeyboardInterrupt cleanup so a stop can
        # still be published. The default rclpy SIGINT handler shuts it down first.
        try:
            rclpy.init(args=[], context=self.context,
                       signal_handler_options=SignalHandlerOptions.NO)
            self.node = Node("realtime_demo", context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.pub = self.node.create_publisher(Twist, sim_topic, 10)
            self.node.create_subscription(Odometry, odom_topic, self._cb,
                                          qos_profile_sensor_data)
        except BaseException:
            self.close()
            raise

    def _cb(self, msg):
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._prev is not None and s > self._prev:
            self.advancing = True
        self._prev = s
        self.stamp = s
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def spin(self):
        self.executor.spin_once(timeout_sec=0.0)

    def publish(self, lin, ang):
        m = self.Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub.publish(m)

    def sub_count(self):
        return self.pub.get_subscription_count()

    def close(self):
        try:
            if self.executor is not None:
                self.executor.shutdown()
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
                    self.node = None
            finally:
                self.context.try_shutdown()


def wait_sim_ready(ros, isaac, timeout):
    say(f"[sim] waiting for Isaac: {ros.odom_topic} advancing "
          f"+ {ros.sim_topic} subscriber ...")
    t0 = time.monotonic()
    last_status = t0
    while time.monotonic() - t0 < timeout:
        if not isaac.alive():
            raise RuntimeError(f"Isaac exited during startup (code {isaac.proc.returncode}) "
                               f"- see {isaac.log_path}")
        ros.publish(0.0, 0.0)
        ros.spin()
        if ros.advancing and ros.sub_count() >= 1:
            say(f"[sim] ready after {time.monotonic()-t0:.0f}s "
                  f"(odom advancing, {ros.sub_count()} cmd_vel sub)")
            return True
        now = time.monotonic()
        if now - last_status >= 10:
            say(f"[sim] waiting {now-t0:.0f}s: odom "
                  f"{'advancing' if ros.advancing else 'not advancing'}, "
                  f"{ros.sub_count()} cmd_vel subscribers; log: {isaac.log_path}")
            last_status = now
        time.sleep(0.05)
    raise RuntimeError(f"Isaac not ready within {timeout}s - see {isaac.log_path}")


def active_command(steps, elapsed):
    """Return (lin, ang, done) for the sequence at `elapsed` seconds."""
    t = 0.0
    for s in steps:
        t += s["duration"]
        if elapsed < t:
            return s["linear"], s["angular"], False
    return 0.0, 0.0, True


def place_point(p, sim0, anchor):
    """One sim pose (x, y, yaw) re-expressed relative to sim0 and placed at anchor."""
    dx, dy = p[0] - sim0[0], p[1] - sim0[1]
    c, s = math.cos(-sim0[2]), math.sin(-sim0[2])
    rx, ry = c * dx - s * dy, s * dx + c * dy
    C0, S0 = math.cos(anchor[2]), math.sin(anchor[2])
    return (anchor[0] + C0 * rx - S0 * ry, anchor[1] + S0 * rx + C0 * ry)


def return_to_start(rover, cam, tracker, name, stem, cfg, args):
    """Put the real rover on this test's start pose before it runs."""
    if not args.real or args.return_to_start == "off":
        return None
    if args.return_to_start == "prompt":
        say(f"[demo] place the rover on the {name} start pose, then press Enter here")
        args.dash.status(f"{name}: place the rover on its start pose - press Enter in the terminal")
        input()
        return None
    args.dash.status(f"{name}: rover returning to its start station")
    for attempt in range(1, 3):
        try:
            return ST.goto_station(rover, cam, tracker, stem, cfg, log=say)
        except Exception as exc:
            rover.stop()
            if not rover.alive or "motors" in str(exc) or attempt == 2:
                raise
            say(f"[station] {stem}: attempt {attempt} failed ({exc}); retrying")


def run_one(name, steps, outdir, ros, cam, args, rover=None, anchor=None, cfg=None):
    """Drive both rovers through one sequence in real time; write real/sim CSVs.

    Real: the rover runs the sequence timed on the Jetson; everything here is clocked
    from its reported start, so the sim starts on the same instant. The camera path
    is re-stamped by the camera latency so both CSVs share one time base.
    -> (real_csv, sim_csv, n_real, n_sim, real_start_pose | None)"""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    real_csv, sim_csv = outdir / "real.csv", outdir / "sim.csv"
    total = sum(s["duration"] for s in steps)
    dash = getattr(args, "dash", None)
    say(f"\n===== {name}: {len(steps)} steps, {total:.1f}s  -> {outdir} =====")

    p0 = cam.settled_pose(0.5) if (cam is not None and args.real) else (
        cam.pose(1.0) if cam is not None else None)
    real_start = tuple(p0[1:4]) if p0 is not None else None
    if anchor is None:
        anchor = real_start or (0.0, 0.0, 0.0)
    if cam is not None and real_start is None:
        say(f"[warn] {name}: sticker triangle not visible right now ({cam.status})")

    sim_rows, real_live, sim_live = [], [], []
    st = {"t0": None, "sim0": None, "last_pub": 0.0, "last_log": 0.0, "last_view": 0.0,
          "lost": None, "lin": 0.0, "ang": 0.0}
    latency = float(getattr(cam, "latency_s", 0.0) or 0.0) if cam is not None else 0.0

    def view(elapsed):
        if dash is None:
            return
        re = real_live[-1] if real_live else None
        se = sim_live[-1] if sim_live else None
        err = (f"live gap {math.hypot(re[0]-se[0], re[1]-se[1])*100:.1f} cm"
               if re and se else "live gap n/a")
        dash.current(f"{name}  real t+{elapsed:4.1f}  sim t+{st.get('sim_el', 0.0):4.1f} / {total:.1f}s",
                     [Track("real (camera)", real_live[:], REAL),
                      Track("sim (/odom, placed at real start)", sim_live[:], SIM)],
                     [f"cmd lin {st['lin']:+.2f} m/s  ang {st['ang']:+.2f} rad/s", err])

    def sim_elapsed():
        """Sim time since this sequence started (odometry stamps). Isaac can run slower
        than real time under load (~0.56x seen in the demo), so the sim is commanded and
        recorded on ITS clock: it executes the full sequence, just later in wall time."""
        if ros.stamp is None:
            return 0.0
        if st.get("sim_t0") is None:
            st["sim_t0"] = ros.stamp
        return ros.stamp - st["sim_t0"]

    def tick(elapsed):
        now = time.time()
        if st["t0"] is None:
            st["t0"] = now - elapsed
        lin, ang, _done = active_command(steps, elapsed)     # what the REAL rover is doing
        st["lin"], st["ang"] = lin, ang
        ros.spin()
        s_el = sim_elapsed()
        st["sim_el"] = s_el
        if now - st["last_pub"] >= 0.05:          # 20 Hz to the sim, on sim time
            ros.publish(*active_command(steps, s_el)[:2])
            st["last_pub"] = now
        if ros.pose is not None:
            sp = (ros.pose[0], -ros.pose[1] if args.flip_sim_y else ros.pose[1], ros.pose[2])
            if st["sim0"] is None:
                st["sim0"] = sp
            if ros.stamp != st.get("sim_stamp"):     # one row per new odom message
                st["sim_stamp"] = ros.stamp
                sim_rows.append((s_el, *sp))
                sim_live.append(place_point(sp, st["sim0"], anchor))
        p = cam.pose(0.5) if cam is not None else None
        if p is not None and (not real_live or real_live[-1] != tuple(p[1:3])):
            real_live.append(tuple(p[1:3]))
        if now - st["last_view"] >= 0.1:
            st["last_view"] = now
            view(elapsed)
        if now - st["last_log"] >= 1.0:           # 1 Hz progress line
            st["last_log"] = now
            re = real_live[-1] if real_live else None
            se = sim_live[-1] if sim_live else None
            say(f"  t+{elapsed:4.1f}s cmd(lin={lin:+.2f},ang={ang:+.2f}) "
                f"real={'(%.3f,%.3f)' % re if re else '----'} "
                f"sim={'(%.3f,%.3f)' % se if se else '----'} "
                f"[{cam.status if cam is not None else 'no camera'}]")
        if args.real and cfg is not None:         # safety: same geofence as the closed loop
            if p is None:
                st["lost"] = st["lost"] or now
                if now - st["lost"] > 1.5:
                    return "camera lost the rover"
            else:
                st["lost"] = None
                if not in_workspace(cfg, p[1], p[2]):
                    return f"left the workspace at ({p[1]:.2f},{p[2]:.2f})"
        return None

    def run_free(t0, until):
        """Until BOTH clocks are past `until`: real (wall) and sim (odometry stamps)."""
        while True:
            elapsed = time.time() - t0
            if elapsed > until and st.get("sim_el", 0.0) > until:
                return
            if elapsed > 4.0 * until + 30.0:
                raise RuntimeError(f"{name}: the sim clock stalled "
                                   f"({st.get('sim_el', 0.0):.1f} of {until:.1f} s in {elapsed:.0f} s)")
            why = tick(elapsed)
            if why:
                raise RuntimeError(f"{name} aborted: {why}")
            time.sleep(0.01)

    if dash is not None:
        dash.camera_source(lambda: cam.overlay(trails=[(real_live[:], REAL), (sim_live[:], SIM)])
                           if cam is not None else None)
        dash.status(f"{name}: driving real + sim  ({total:.1f}s)" if args.real
                    else f"{name}: driving sim  ({total:.1f}s)")
    # Start the sim clock from a FRESH odometry stamp: ROS is not spun while the rover
    # returns to its station, so the last stamp seen can be tens of seconds old.
    stale, t_sync = ros.stamp, time.time() + 3.0
    while time.time() < t_sync:
        ros.spin()
        if ros.stamp is not None and ros.stamp != stale:
            break
        time.sleep(0.005)
    st["sim_t0"] = ros.stamp
    video = outdir / "real_camera.mp4"
    if cam is not None:
        cam.start_recording(video_path=video if args.real else None)
    rows, frame_ts = [], []
    t_start = None
    try:
        if args.real:
            say(f"[real] rover running '{name}' (timed on the Jetson); sim follows its start")
            t_start = rover.run_sequence(steps, on_tick=tick)
        else:
            t_start = time.time()
            say(f"[sim] driving '{name}' on /cmd_vel")
            run_free(t_start, total)
        run_free(t_start, total + args.tail)       # coast-down / settle on both
    except KeyboardInterrupt:
        say("\n[demo] interrupted - stopping both rovers")
        raise
    finally:
        try:
            if rover is not None:
                rover.stop()
        finally:
            for _ in range(10):
                if not ros.context.ok():
                    break
                ros.publish(0.0, 0.0); ros.spin(); time.sleep(0.02)
            if cam is not None:
                rows, frame_ts = cam.stop_recording()
    t_start = t_start or st["t0"] or time.time()
    with open(real_csv, "w") as f:
        f.write("t,x,y,yaw\n")
        for t, x, y, yaw in rows:
            tt = t - latency - t_start
            if tt >= -0.5:
                f.write(f"{tt:.4f},{x:.5f},{y:.5f},{yaw:.5f}\n")
    with open(sim_csv, "w") as f:
        f.write("t,x,y,yaw\n")
        for t, x, y, yaw in sim_rows:
            f.write(f"{t:.4f},{x:.5f},{y:.5f},{yaw:.5f}\n")
    if frame_ts:                     # per-frame times of real_camera.mp4 on the CSV time base
        with open(outdir / "real_frames.csv", "w") as f:
            f.write("t\n")
            for t in frame_ts:
                f.write(f"{t - latency - t_start:.4f}\n")
    wall = max(time.time() - t_start, 1e-6)
    say(f"[demo] {name}: real {len(rows)} camera samples, sim {len(sim_rows)} samples; "
        f"Isaac ran at {st.get('sim_el', 0.0) / wall:.2f}x real time (sim commanded and "
        f"recorded on sim time)")
    return real_csv, sim_csv, len(rows), len(sim_rows), real_start


def extra_graphics(name, seqfile, real_csv, sim_csv, outdir, H_m2img):
    """Paths drawn on the real camera video (paths_map.png/.mp4) + the command->response
    map (response_map.png/.json). Best effort: a failure only skips that graphic."""
    outdir = Path(outdir)
    video, frames = outdir / "real_camera.mp4", outdir / "real_frames.csv"
    if H_m2img is not None and video.exists() and Path(real_csv).exists() and Path(sim_csv).exists():
        try:
            import closed_loop as CL
            CL.path_maps(name, [{"csv": str(real_csv), "video": str(video), "frames": str(frames)}],
                         str(sim_csv), None, outdir, H_m2img, say,
                         labels={"before": "sim (same commands, same start)"})
        except Exception as exc:
            say(f"[demo] {name}: path map skipped ({exc})")
    try:
        import response_map as RM
        steps = load_sequence(seqfile)[1]
        real = RM.summarise(RM.step_responses(steps, [real_csv])) if load_rows(real_csv).shape[0] > 5 else []
        sim = RM.summarise(RM.step_responses(steps, [sim_csv])) if load_rows(sim_csv).shape[0] > 5 else []
        if real or sim:
            rows = RM.merge(real, sim)
            RM.plot(rows, str(outdir / "response_map.png"), f"{name}: command -> response")
            (outdir / "response_map.json").write_text(json.dumps(
                {"sequence": name, "rows": rows, "gap": RM.gap(rows)}, indent=1))
    except Exception as exc:
        say(f"[demo] {name}: response map skipped ({exc})")


def headline_metrics(m):
    """compare_paths metrics -> the readable subset for the index."""
    if not m:
        return {"compared": "no (too few samples)"}
    cm = lambda k: None if m.get(k) is None else round(m[k] * 100, 1)
    return {"endpoint error [cm]": cm("endpoint_err_m"), "mean error [cm]": cm("mean_err_m"),
            "max error [cm]": cm("max_err_m"), "heading error [deg]": m.get("heading_err_deg"),
            "path length real [m]": m.get("path_len_real_m"),
            "path length sim [m]": m.get("path_len_sim_m"),
            "samples real / sim": f"{m.get('n_real')} / {m.get('n_sim')}"}


def test_files(outdir):
    o = Path(outdir)
    return {"paths on the real camera (still)": o / "paths_map.png",
            "comparison plot": o / "compare.png", "command -> response map": o / "response_map.png",
            "paths on the real camera (video)": o / "paths_map.mp4",
            "real camera (raw video)": o / "real_camera.mp4",
            "webcam-matched overhead render": o / "overhead.mp4",
            "real path csv": o / "real.csv", "sim path csv": o / "sim.csv",
            "real frame times": o / "real_frames.csv", "metrics json": o / "metrics.json",
            "response map json": o / "response_map.json"}


def finalize(name, real_csv, sim_csv, outdir, setup):
    outdir = Path(outdir)
    metrics = None
    mj = outdir / "metrics.json"
    if Path(real_csv).exists() and Path(sim_csv).exists() \
            and sum(1 for _ in open(real_csv)) > 1 and sum(1 for _ in open(sim_csv)) > 1:
        subprocess.run([sys.executable, str(TRIALS / "compare_paths.py"),
                        str(real_csv), str(sim_csv), "--out", str(outdir / "compare.png"),
                        "--json", str(mj), "--label", name], check=False,
                       stdout=subprocess.DEVNULL)
        if mj.exists():
            metrics = json.load(open(mj)); metrics["test"] = name
        if setup is not None:
            try:
                import render_overhead as ro
                tracks = [ro.load_track(str(real_csv)), ro.load_track(str(sim_csv))]
                ro.render(tracks, setup, out=str(outdir / "overhead.mp4"), fps=30.0,
                          labels=["real (camera)", "sim (/odom)"])
                say(f"[demo] overhead video -> {outdir/'overhead.mp4'}")
            except Exception as e:
                say(f"[demo] overhead render skipped ({e})")
    else:
        say(f"[demo] {name}: insufficient samples to compare "
              f"(real/sim CSV too short) - skipping metrics.")
    return metrics


def load_rows(path):
    try:
        a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    except (OSError, ValueError):
        return np.zeros((0, 4))
    return a if a.size else np.zeros((0, 4))


def show_result(dash, name, real_csv, sim_csv, metrics, start):
    """Gallery tile: real path vs sim path placed at `start` (the real start pose, or
    the configured start in a sim-only rehearsal), + metrics."""
    real, sim = load_rows(real_csv), load_rows(sim_csv)
    if start is None and len(real):
        start = tuple(real[0, 1:4])
    tracks = [Track("real", real, REAL)]
    if len(sim) and start is not None:
        tracks.append(Track("sim", place_at(sim, start), SIM))
    dash.result(name, name, tracks, metric_lines(metrics))
    dash.current(f"{name} (finished)", tracks, metric_lines(metrics))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sequence", help="single sequence YAML")
    g.add_argument("--tests", help="comma list e.g. T1,T2,T4 (battery)")
    ap.add_argument("--surface", default="live")
    ap.add_argument("--outroot", default=str(HERE / "runs"))
    ap.add_argument("--calib", default=str(CALIB))
    ap.add_argument("--stickers", default=str(DEFAULT_CONFIG))
    ap.add_argument("--config", default=str(CONFIG),
                    help="loop_config.yaml: start poses, workspace geofence, homing, battery")
    ap.add_argument("--sim-odom", default=ODOM_TOPIC)
    ap.add_argument("--no-real", dest="real", action="store_false", default=True,
                    help="sim-only rehearsal (safe: rover not driven)")
    ap.add_argument("--allow-approximate", action="store_true",
                    help="record even if stickers lack verified projection calibration")
    ap.add_argument("--flip-sim-y", action="store_true")
    ap.add_argument("--show", dest="show", action="store_true", default=True,
                    help="persistent dashboard window (default when $DISPLAY is set)")
    ap.add_argument("--no-show", dest="show", action="store_false",
                    help="no window (live.png snapshot is still written)")
    ap.add_argument("--no-hold", action="store_true",
                    help="exit when the battery finishes instead of keeping the window up")
    ap.add_argument("--no-countdown", action="store_true")
    ap.add_argument("--tail", type=float, default=2.0,
                    help="seconds to keep recording both rovers after the sequence ends")
    ap.add_argument("--return-to-start", choices=("station", "prompt", "off"), default="station",
                    help="before each test: the rover drives itself to the test's start "
                         "station (default), you place it and press Enter, or it starts "
                         "wherever it is")
    ap.add_argument("--no-pause", action="store_true",
                    help="old name for --return-to-start off")
    ap.add_argument("--keep-bringup", action="store_true",
                    help="leave a rover base driver we started running at the end")
    ap.add_argument("--startup-timeout", type=float, default=300.0)
    ap.add_argument("--surface-calib", default=None,
                    help="optional nested sim calibration yaml (else defaults)")
    args = ap.parse_args()
    if args.no_pause:
        args.return_to_start = "off"

    # sequences
    if args.sequence:
        name, steps = load_sequence(args.sequence)
        plan = [(name, steps, args.sequence)]
    else:
        plan = []
        for t in [x.strip() for x in args.tests.split(",") if x.strip()]:
            sf = seq_for(t)
            nm, st = load_sequence(sf)
            plan.append((t, st, sf))
    args.seqfile_for = {nm: sf for nm, _, sf in plan}

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = Path(args.outroot) / f"{args.surface}_{stamp}"       # never overwrite a past run
    base.mkdir(parents=True, exist_ok=True)
    latest = Path(args.outroot) / f"{args.surface}_latest"
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(base.name)
    except OSError:
        pass

    if not os.path.exists(args.calib):
        sys.exit(f"ERROR: {args.calib} missing - run calibrate_grid.py first.")
    calib = load_calib(args.calib)
    cfg = load_config(args.config)

    rows = []
    try:
        # Register each cleanup as soon as its resource exists. ExitStack runs
        # every callback even if another fails, including on partial startup.
        with ExitStack() as cleanup:
            ws = cfg.get("workspace")
            dash = Dashboard(f"sim2real demo [{args.surface}]", grid_dims=calib[2],
                             show=args.show, snapshot=base / "live.png",
                             record=base / "session_video.mp4",
                             workspace=(ws["x"], ws["y"]) if ws and args.real else None)
            cleanup.callback(dash.close)
            args.dash = dash
            say.dash = dash
            session = Session(base, "demo", f"Sim vs real demo - {args.surface} ({stamp})", meta={
                "surface": args.surface, "real rover": "yes" if args.real else "no (sim rehearsal)",
                "tests": ", ".join(n for n, _, _ in plan),
                "drive": "wheel contact (PhysX wheel drives)",
                "sim parameters": args.surface_calib or "schema defaults",
                "return to start": args.return_to_start})
            session.snapshot_config({
                "grid calibration": args.calib, "sticker config": args.stickers,
                "loop config (start poses, workspace)": args.config,
                "sim parameters": args.surface_calib,
                "overhead camera": SC / "overhead" / "camera_setup.yaml",
                **{f"sequence {n}": sf for n, _, sf in plan}})
            session.add_video("whole session (dashboard recording)", base / "session_video.mp4")
            session.write()
            dash.status("opening the overhead camera")

            cam = None
            try:
                cam = CameraTrack(grid_calib=args.calib, stickers=args.stickers,
                                  allow_approximate=args.allow_approximate)
            except (CameraError, ValueError) as exc:
                if args.real:
                    sys.exit(f"ERROR: {exc}")
                say(f"[camera] unavailable ({exc}) - sim-only run without the camera panel")
            if cam is not None:
                cleanup.callback(cam.close)
                cam.latency_s = float(cfg.get("fusion", {}).get("cam_latency_s", 0.0))
                dash.camera_source(cam.overlay)
                say("[camera] decoded first frame; starting the simulator next.")
                if not cam.tracker.measurement_ready:
                    say("[demo] NOTE: sticker projection not yet verified - the real path is "
                        "an approximate (parallax-corrected) estimate.")

            # overhead render setup (optional)
            setup = None
            try:
                import render_overhead as ro
                setup = ro.load_setup(str(SC / "overhead" / "camera_setup.yaml"))
            except Exception as e:
                say(f"[demo] overhead setup unavailable ({e}); skipping mp4s.")

            theta = C.defaults()
            if args.surface_calib:
                import yaml
                theta = C.flatten(yaml.safe_load(Path(args.surface_calib).read_text()) or {})
            dash.params(PS.build(theta, (), args.surface, header=(
                f"no optimiser: {Path(args.surface_calib).name if args.surface_calib else 'schema defaults'}")))

            rover = tracker = None
            if args.real:
                dash.status("connecting to the rover")
                from rover_link import RoverLink
                rover = RoverLink(log=say, logfile=str(base / "rover_agent.log"),
                                  fusion_cfg=cfg.get("fusion", {}))
                cleanup.callback(lambda: rover.close(stop_bringup=not args.keep_bringup))
                rover.open()
                ST.set_guard(rover, cfg)
                tracker = cam
                fz = cfg.get("fusion", {})
                if fz.get("enabled", True):
                    from localizer import FusedTracker
                    tracker = FusedTracker(cam, rover, cam_latency_s=fz.get("cam_latency_s", 0.2),
                                           gain=fz.get("gain", 0.3))
                    cleanup.callback(tracker.close)
                taught = ST.list_stations()
                stems = [Path(sf).stem for _, _, sf in plan]
                say(f"[demo] start stations taught: {taught or 'none'}"
                    + ("" if all(t in taught for t in stems) else
                       f"  (untaught -> loop_config start pose: "
                       f"{[t for t in stems if t not in taught]})"))

            dash.status("starting Isaac (first launch takes a minute or two)")
            cleanup.enter_context(sim_ros_environment())
            isaac = Isaac(theta, base)
            cleanup.callback(isaac.stop)
            ros = Ros(odom_topic=args.sim_odom)
            cleanup.callback(ros.close)
            wait_sim_ready(ros, isaac, args.startup_timeout)
            bmin = float(cfg.get("battery_min_v", 0) or 0)
            for i, (name, steps, sf) in enumerate(plan):
                stem = Path(sf).stem
                outdir = base / name
                if rover is not None and bmin and rover.voltage == rover.voltage \
                        and rover.voltage < bmin:
                    say(f"[demo] battery {rover.voltage:.2f} V < {bmin} V - stopping the "
                        f"battery here (charge the rover)")
                    break
                dash.status(f"{name} ({i + 1}/{len(plan)}): getting ready")
                try:
                    return_to_start(rover, cam, tracker, name, stem, cfg, args)
                except Exception as exc:
                    say(f"[demo] {name}: could not reach the start pose ({exc}) - skipped")
                    dash.result(name, f"{name} (skipped)", [], ["could not reach start pose"])
                    session.add_test(name, note=f"skipped: could not reach the start pose ({exc})")
                    session.write()
                    continue
                if args.real and not args.no_countdown:
                    dash.status(f"{name}: ROVER DRIVES in 3 s")
                    say("[real] ROVER WILL DRIVE in 3s - Ctrl-C / q to abort.")
                    time.sleep(3)
                anchor = None if args.real else start_pose(cfg, stem)
                try:
                    real_csv, sim_csv, nr, ns, real_start = run_one(
                        name, steps, outdir, ros, cam, args, rover=rover, anchor=anchor, cfg=cfg)
                except RuntimeError as exc:           # geofence / camera loss: next test
                    if rover is not None and not rover.alive:
                        raise
                    say(f"[demo] {name}: {exc}")
                    dash.result(name, f"{name} (aborted)", [], [str(exc)[:60]])
                    session.add_test(name, note=f"aborted: {exc}")
                    session.write()
                    continue
                dash.status(f"{name}: comparing paths + rendering")
                m = finalize(name, real_csv, sim_csv, outdir, setup if args.real else None)
                extra_graphics(name, sf, real_csv, sim_csv, outdir, calib[1])
                show_result(dash, name, real_csv, sim_csv, m, real_start if args.real else anchor)
                if m:
                    rows.append(m)
                session.add_test(name, metrics=headline_metrics(m), files=test_files(outdir))
                session.write()
            if rover is not None and args.return_to_start == "station" and cfg.get("park"):
                p = cfg["park"]
                dash.status("parking the rover")
                try:
                    go_to_pose(rover, tracker, (p["x"], p["y"], math.radians(p["yaw"])), cfg,
                               log=say)
                except Exception as exc:
                    say(f"[real] park failed: {exc}")
            write_summary(rows, base, args)
            session.set_summary([f"{r['test']}: endpoint {r['endpoint_err_m'] * 100:.1f} cm, "
                                 f"mean {r['mean_err_m'] * 100:.1f} cm, heading "
                                 f"{r['heading_err_deg']:.1f} deg" for r in rows] or ["no comparable results"])
            dash.save_frame(base / "final_view.png")
            dash.stop_recording()                  # finalise the video before teardown
            session.add_video("final dashboard view", base / "final_view.png")
            session.write(finished=True)
            say(f"[demo] results: {base / 'index.md'}  (catalog: {SC / 'RESULTS.md'})")
            if not args.no_hold:
                dash.hold(f"battery done ({len(rows)}/{len(plan)} compared) - "
                          "press q or close the window to exit")
    except KeyboardInterrupt:
        print("\n[demo] interrupted; rover, camera, ROS and Isaac stopped.")
        return 130
    return 0


def write_summary(rows, base, args):
    if not rows:
        say("\n[demo] no comparable results produced.")
        return
    say("\n==================== SUMMARY ====================")
    say(f"surface={args.surface}   {datetime.datetime.now():%Y-%m-%d %H:%M}")
    say("test | endpoint | mean | max | heading | real_len | sim_len")
    for r in rows:
        say(f"{r['test']:>4} | {r['endpoint_err_m']*100:6.1f}cm | "
            f"{r['mean_err_m']*100:5.1f} | {r['max_err_m']*100:5.1f} | "
            f"{r['heading_err_deg']:6.1f}d | {r['path_len_real_m']:.3f} | "
            f"{r['path_len_sim_m']:.3f}")
    with open(base / "summary.json", "w") as f:
        json.dump(rows, f, indent=2)
    say(f"[demo] summary -> {base/'summary.json'}")


if __name__ == "__main__":
    raise SystemExit(main())
