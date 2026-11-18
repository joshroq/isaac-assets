#!/usr/bin/env python3
"""Desktop side of the rover command link (pairs with rover_agent.py).

RoverLink exposes the rover agent as a small Python API:

    with RoverLink() as rover:          # stages the agent, starts the base driver if needed
        rover.velocity(0.1, 0.0)        # deadman'd setpoint - resend at >= 5 Hz
        rover.run_sequence(steps, on_tick=...)   # timed locally on the Jetson
        rover.stop()

LINK-TOLERANT DESIGN (2026-11-11; weak garage Wi-Fi, and the rover must also work from
ODU campus over Tailscale):
  * The agent runs PERSISTENTLY on the rover (`rover_agent.py --serve`, started detached)
    and the base driver (start_bringup.sh) is started detached too, so a network drop no
    longer kills either (the old held-open `ssh -tt` bringup died with the ssh session).
  * The desktop talks to the agent through `ssh -W 127.0.0.1:PORT host` and RECONNECTS by
    itself when that drops, trying every route in ROVER_HOST (default "rover-lan,rover":
    the LAN alias, then Tailscale - so the same code works at home and on campus).
  * A trial (`seq`) already loaded on the rover finishes there even if the link drops; the
    agent buffers what it emits and replays it on reconnect, so run_sequence() returns
    normally, just later. Velocity setpoints (homing) stop at once on a drop.
  * Telemetry times are the ROVER's clock mapped to the desktop clock (min-filtered offset,
    i.e. rover time + the link's best one-way delay), so replayed or jittery lines keep
    their true times.

A heartbeat pings every 0.25 s; rx_age is the time since the agent last said anything.

    python3 loop/rover_link.py --check      # connect, report route/subs/odom/voltage, exit
    python3 loop/rover_link.py --link-test  # measure the link: RTT, jitter, reconnect time
"""
from __future__ import annotations

import hashlib
import json
import os
import queue
import signal
import subprocess
import threading
import sys
import time
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
# Routes to try, in order (comma list of ssh aliases). LAN first (lowest latency, and
# Tailscale to the rover dropped while the LAN worked on 2026-10-20), then Tailscale,
# which works from anywhere (ODU campus) - see rover_comms memory / docs.
ROVER = os.environ.get("ROVER_HOST", "rover-lan,rover")
AGENT_PORT = int(os.environ.get("ROVER_AGENT_PORT", "7801"))
SSHOPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
           "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=5",
           "-o", "ServerAliveCountMax=3"]
REMOTE_DIR = "sim2real_run"                  # relative to the rover's $HOME
ENV = "source ~/rover_nav/local_ros_env.sh >/dev/null 2>&1"
EVENT_KINDS = ("ready", "seq_start", "seq_step", "seq_done", "seq_abort", "err", "bye",
               "link_lost")
TIMED_EVENTS = {"seq_start": 0, "seq_step": 1, "seq_done": 0, "link_lost": 0}  # arg idx of T


class RoverLinkError(RuntimeError):
    pass


class RoverLinkDown(RoverLinkError):
    """The link is (temporarily) down; it reconnects by itself."""


def _odom_source(cfg):
    return (cfg or {}).get("odom_source", "rf2o")


class ClockMap:
    """rover time -> desktop time. offset = min over a sliding window of (arrival - rover
    stamp) = clock skew + the link's best one-way delay; the min ignores queueing jitter
    and replayed backlog lines (which arrive late)."""

    def __init__(self, window_s=30.0):
        self.window_s = window_s
        self.samples = deque()                   # (arrival, arrival - stamp)
        self.offset = None

    def observe(self, arrival, stamp):
        d = arrival - stamp
        self.samples.append((arrival, d))
        while self.samples and arrival - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        self.offset = min(s[1] for s in self.samples)

    def to_desktop(self, stamp, arrival):
        if self.offset is None:
            return arrival
        return min(stamp + self.offset, arrival)  # never later than it actually arrived


class RoverLink:
    def __init__(self, host=ROVER, start_bringup=True, log=print, logfile=None,
                 fusion_cfg=None, reconnect_timeout=900.0):
        self.hosts = [h.strip() for h in str(host).split(",") if h.strip()]
        self.host = self.hosts[0]
        self.want_bringup = start_bringup
        self.log = log
        self.logfile = open(logfile, "a") if logfile else None
        self.reconnect_timeout = reconnect_timeout
        self.proc = None                # current transport (ssh -W, or the local agent)
        self.bringup_pid = None         # rover-side pid of a base driver WE started
        self.events = queue.Queue()
        self.odom = None            # (t, x, y, yaw, vx, wz) wheel odom
        self.odom2 = None           # same, lidar odometry (rf2o)
        self.odom_log = None        # list while recording (wheel odom)
        self.subs = 0
        self.have_odom = self.have_rf2o = self.have_gyro = self.have_scan = False
        self.gyro_bias = 0.0
        self.voltage = float("nan")
        self.guard_events = []      # (t, gap) obstacle stops
        self._scans = queue.Queue()
        self.odom_src = None        # 'odom2' (rf2o) or 'odom' (wheel), fixed at open()
        self.rosenv = None
        from fusion import OdomIntegrator
        self.fusion_cfg = fusion_cfg or {}
        self.integrator = OdomIntegrator(use_gyro=self.fusion_cfg.get("yaw_source", "gyro") == "gyro",
                                         scale=float(self.fusion_cfg.get("odom_scale", 1.0)))
        self.clock = ClockMap()
        self.status_t = 0.0
        self.last_rx = None
        self.last_pong = None
        self._wlock = threading.Lock()
        self._alive = False         # True from open() until close() / reconnect gave up
        self._up = False            # transport currently connected
        self._closing = False
        self._reconnecting = threading.Lock()
        self._replay_left = 0       # backlog lines still to come after 'resume N'
        self.link_drops = 0
        self.local = self.host == "local"

    # ---------------- lifecycle ----------------
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def _ssh(self, cmd, host=None, **kw):
        return subprocess.run(["ssh", *SSHOPTS, host or self.host, cmd], capture_output=True,
                              text=True, timeout=kw.pop("timeout", 30), **kw)

    def _pick_host(self):
        errors = []
        for h in self.hosts:
            try:
                r = self._ssh("true", host=h, timeout=20)
                if r.returncode == 0:
                    return h
                errors.append(f"{h}: {r.stderr.strip()[:120]}")
            except subprocess.TimeoutExpired:
                errors.append(f"{h}: timed out")
        raise RoverLinkError("cannot reach the rover by any route (" + "; ".join(errors) + ")")

    def _ensure_agent(self, host):
        """Stage rover_agent.py and make sure exactly THIS version is serving."""
        src = (HERE / "rover_agent.py").read_bytes()
        sha = hashlib.sha256(src).hexdigest()[:16]
        # Liveness by PID FILE: a `pgrep -f rover_agent.py` would match this very ssh
        # command line (it contains the launch text) and report a dead agent as running.
        alive = ("{ [ -f agent.pid ] && grep -q rover_agent /proc/$(cat agent.pid)/cmdline "
                 "2>/dev/null; }")
        script = (
            f"mkdir -p {REMOTE_DIR} && cd {REMOTE_DIR} && "
            f"if [ \"$(cat agent.sha 2>/dev/null)\" != {sha} ] || ! {alive}; then "
            f"  {alive} && {{ kill -INT $(cat agent.pid); sleep 1.5; }}; "
            f"  cat > rover_agent.py.new && mv rover_agent.py.new rover_agent.py && "
            f"  echo {sha} > agent.sha && "
            f"  ({ENV}; setsid nohup python3 -u rover_agent.py --serve {AGENT_PORT} "
            f"     > agent.log 2>&1 < /dev/null & echo $! > agent.pid) ; echo started; "
            f"else cat > /dev/null; echo running; fi")
        r = subprocess.run(["ssh", *SSHOPTS, host, script], input=src, capture_output=True,
                           timeout=60)
        if r.returncode:
            raise RoverLinkError(f"could not start the agent on {host}: "
                                 f"{r.stderr.decode(errors='replace').strip()[:200]}")
        return r.stdout.decode().strip().splitlines()[-1:] == ["started"]

    def _connect(self, host, ready_timeout=25.0):
        """Open one ssh -W transport to the agent and wait for its 'ready'."""
        proc = subprocess.Popen(["ssh", *SSHOPTS, "-W", f"127.0.0.1:{AGENT_PORT}", host],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True, bufsize=1,
                                start_new_session=True)
        ready = threading.Event()
        threading.Thread(target=self._reader, args=(proc, ready), daemon=True).start()
        if not ready.wait(ready_timeout):
            _kill(proc)
            raise RoverLinkError(f"agent on {host} did not answer within {ready_timeout:.0f}s")
        old, self.proc, self.host = self.proc, proc, host
        self._up = True
        if old is not None and old is not proc:
            _kill(old)
        return proc

    def open(self, ready_timeout=60.0):
        if self.local:                           # offline tests: stdio agent on this machine
            return self._open_local(ready_timeout)
        host = self._pick_host()
        started = self._ensure_agent(host)
        for attempt in range(3):                 # a freshly started agent needs ROS init time
            try:
                self._connect(host, ready_timeout=25.0 if attempt else (30.0 if started else 15.0))
                break
            except RoverLinkError:
                if attempt == 2:
                    raise
                time.sleep(2.0)
        self._alive = True
        threading.Thread(target=self._heartbeat, daemon=True).start()
        self.log(f"[rover] agent up on {self.host} (persistent, port {AGENT_PORT}; "
                 f"{'new' if started else 'already running'})")
        self._check_isolation()
        # base driver: need a real /cmd_vel subscriber + live odometry
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, 4.0):
            if not self.want_bringup:
                raise RoverLinkError("rover base driver not running (/cmd_vel has no "
                                     "subscriber) and --no-bringup was given")
            self._start_bringup()
        want_rf2o = _odom_source(self.fusion_cfg) == "rf2o"
        if want_rf2o:
            self._wait_status(lambda: self.have_rf2o, 4.0)
        self.odom_src = "odom2" if (want_rf2o and self.have_rf2o) else "odom"
        self._wait_status(lambda: self.have_gyro and self.have_scan, 3.0)
        self.log(f"[rover] base ready: /cmd_vel subs={self.subs}, battery {self.voltage:.2f} V, "
                 f"odometry for fusion = {'rf2o lidar' if self.odom_src == 'odom2' else 'wheel'}"
                 f"{'' if self.odom_src == 'odom2' or not want_rf2o else ' (rf2o not publishing!)'}"
                 f", gyro {'yes' if self.have_gyro else 'NO'}, lidar {'yes' if self.have_scan else 'NO'}")

    def _open_local(self, ready_timeout):
        proc = subprocess.Popen([sys.executable, "-u", str(HERE / "rover_agent.py")],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1,
                                start_new_session=True)
        ready = threading.Event()
        self.proc = proc
        threading.Thread(target=self._reader, args=(proc, ready), daemon=True).start()
        if not ready.wait(ready_timeout):
            raise RoverLinkError("local agent did not start")
        self._alive = self._up = True
        threading.Thread(target=self._heartbeat, daemon=True).start()
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, 15.0):
            raise RoverLinkError("local agent: no /cmd_vel subscriber or odom (fake rover?)")
        self._wait_status(lambda: self.have_rf2o and self.have_gyro and self.have_scan, 4.0)
        want = _odom_source(self.fusion_cfg) == "rf2o"
        self.odom_src = "odom2" if (want and self.have_rf2o) else "odom"
        self.log(f"[rover] local agent ready (odometry {self.odom_src})")

    def _check_isolation(self):
        """The rover's ROS must be private to the Jetson. If it were networked on the
        desktop's domain, the Isaac sim's /cmd_vel (published during every sim
        evaluation) would ALSO drive the physical rover."""
        env = self.rosenv or {}
        isolated = env.get("localhost_only") == "1" and not env.get("discovery_server")
        if not isolated and not os.environ.get("LOOP_ALLOW_NETWORKED_ROVER"):
            self.close(stop_bringup=False)
            raise RoverLinkError(
                f"rover ROS is not isolated ({env}): Isaac's /cmd_vel could drive the real "
                "rover. Set ROS_LOCALHOST_ONLY=1 / unset ROS_DISCOVERY_SERVER on the rover "
                "(rover_nav/local_ros_env.sh), or export LOOP_ALLOW_NETWORKED_ROVER=1 if the "
                "rover is on a different ROS_DOMAIN_ID than the desktop.")

    @property
    def base_alive(self):
        """The base driver is up: /cmd_vel has a subscriber per the agent's last status.
        (The driver now runs detached on the rover, so a network drop can't kill it.)"""
        return self.subs >= 1

    # ---------------- reconnect ----------------
    def _on_transport_lost(self, proc):
        if self._closing or proc is not self.proc:
            return
        self._up = False
        self.link_drops += 1
        self.events.put((time.time(), "link_down", []))
        self.log(f"[rover] link to {self.host} dropped - reconnecting (a running trial "
                 f"finishes on the rover)")
        threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def _reconnect_loop(self):
        if not self._reconnecting.acquire(blocking=False):
            return
        try:
            t0 = time.time()
            deadline = t0 + self.reconnect_timeout
            rounds = 0
            while not self._closing and time.time() < deadline:
                order = [self.host] + [h for h in self.hosts if h != self.host]
                for h in order:
                    if self._closing:
                        return
                    try:
                        if rounds % 3 == 2:          # agent itself gone? (rover rebooted)
                            self._ensure_agent(h)
                        self._connect(h)
                        self.log(f"[rover] link back via {h} after {time.time() - t0:.1f}s")
                        self.events.put((time.time(), "link_up", []))
                        return
                    except (RoverLinkError, subprocess.TimeoutExpired, OSError):
                        continue
                rounds += 1
                time.sleep(3.0)
            if not self._closing:
                self._alive = False
                self.events.put((time.time(), "eof", []))
                self.log(f"[rover] no link for {self.reconnect_timeout:.0f}s - giving up")
        finally:
            self._reconnecting.release()

    def reopen(self, wait_s=600.0, poll_s=10.0):
        """The link gave up (rover rebooted / lost power / long Wi-Fi outage): wait until
        the rover answers again, then rebuild the agent link and the base driver."""
        if self.local:
            _kill(self.proc)
            self.proc = None
            self.subs = 0
            self.have_odom = self.have_rf2o = self.have_gyro = self.have_scan = False
            while True:
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    break
            return self.open()
        self.subs = 0
        self.have_odom = False
        deadline = time.monotonic() + wait_s
        while True:
            try:
                self.host = self._pick_host()
                break
            except RoverLinkError:
                if time.monotonic() > deadline:
                    raise RoverLinkError(f"rover unreachable for {wait_s:.0f}s - check its "
                                         f"power/WiFi, then --resume")
                time.sleep(poll_s)
        self.log(f"[rover] {self.host} reachable again -> reconnecting the agent")
        while True:
            try:
                self.events.get_nowait()
            except queue.Empty:
                break
        self._closing = False
        self._ensure_agent(self.host)
        self._connect(self.host, ready_timeout=40.0)
        if not self._alive:
            self._alive = True
            threading.Thread(target=self._heartbeat, daemon=True).start()
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, 6.0):
            self._start_bringup()

    # ---------------- base driver ----------------
    def restart_bringup(self, timeout=90.0):
        """Stop whatever is left of the base stack on the rover, then start it again."""
        self.log("[rover] base driver lost -> restarting it")
        self._stop_bringup_remote(all_stacks=True)
        self._wait_status(lambda: self.subs == 0, 10.0)
        self._start_bringup(timeout)
        self.log(f"[rover] base driver back: /cmd_vel subs={self.subs}, battery {self.voltage:.2f} V")

    def _start_bringup(self, timeout=90.0):
        self.log("[rover] base driver not running -> starting start_bringup.sh (detached on "
                 "the rover: survives network drops)")
        r = self._ssh(f"mkdir -p {REMOTE_DIR}; setsid nohup bash ~/rover_nav/start_bringup.sh "
                      f"> {REMOTE_DIR}/bringup.log 2>&1 < /dev/null & echo $!", timeout=40)
        try:
            self.bringup_pid = int(r.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise RoverLinkError(f"could not start the base driver: {r.stderr.strip()[:200]}")
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, timeout):
            tail = self._ssh(f"tail -5 {REMOTE_DIR}/bringup.log", timeout=20).stdout
            raise RoverLinkError(f"base driver did not come up in {timeout:.0f}s "
                                 f"(subs={self.subs}, odom={self.have_odom}): {tail.strip()}")

    def _stop_bringup_remote(self, all_stacks=False):
        """Stop the base driver we started (its process group); all_stacks: whatever runs."""
        cmd = []
        if self.bringup_pid:
            cmd.append(f"kill -INT -{self.bringup_pid} 2>/dev/null; sleep 3; "
                       f"kill -TERM -{self.bringup_pid} 2>/dev/null")
        if all_stacks:
            cmd.append("bash ~/rover_nav/stop_all.sh >/dev/null 2>&1")
        if cmd:
            try:
                self._ssh("; ".join(cmd) + "; true", timeout=40)
            except subprocess.TimeoutExpired:
                pass
        self.bringup_pid = None

    def close(self, stop_bringup=True, stop_agent=True):
        """Stop the rover, end the agent (it publishes /cmd_vel zeros while alive, which
        would fight Nav2/teleop) and the base driver if we started it."""
        self._closing = True                    # the agent's exit must not look like a drop
        if self.proc is not None and self._up:
            try:
                self.stop()
                if stop_agent:
                    self._send("quit")
                time.sleep(0.5)
            except Exception:
                pass
        self._alive = self._up = False
        if self.local:
            if self.proc is not None:
                try:
                    self.proc.stdin.close()
                    self.proc.wait(timeout=5)
                except Exception:
                    _kill(self.proc)
        else:
            _kill(self.proc)
            if stop_agent:                       # quit may not have arrived (link down)
                try:
                    self._ssh(f"cd {REMOTE_DIR} && [ -f agent.pid ] && grep -q rover_agent "
                              f"/proc/$(cat agent.pid)/cmdline 2>/dev/null && "
                              f"kill -INT $(cat agent.pid); true", timeout=30)
                except subprocess.TimeoutExpired:
                    pass
            if stop_bringup and self.bringup_pid:
                # the agent zeroed /cmd_vel above, so stopping the driver can't run away
                self._stop_bringup_remote()
                self.log("[rover] base driver stopped")
        self.proc = None
        if self.logfile:
            self.logfile.close()
            self.logfile = None

    # ---------------- io ----------------
    def _send(self, line):
        proc = self.proc
        if proc is None or proc.poll() is not None or not self._up:
            if self._alive:
                raise RoverLinkDown("rover link is down (reconnecting)")
            raise RoverLinkError("rover agent is not running")
        with self._wlock:
            try:
                proc.stdin.write(line + "\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                raise RoverLinkDown("rover link is down (reconnecting)")

    def _reader(self, proc, ready):
        replay = 0
        for raw in proc.stdout:
            now = time.time()
            self.last_rx = now
            line = raw.strip()
            parts = line.split()
            if not parts:
                continue
            kind = parts[0]
            live = replay == 0
            if not live:
                replay -= 1
            if kind == "resume" and len(parts) == 2:
                replay = int(parts[1])
                if replay:
                    self.log(f"[rover] replaying {replay} lines buffered while the link was down")
                continue
            if kind == "dropped":
                self.log(f"[rover] WARNING: the rover dropped {parts[1]} buffered lines")
                continue
            if kind in ("odom", "odom2") and len(parts) == 7:
                stamp = float(parts[1])
                if live:
                    self.clock.observe(now, stamp)
                t = self.clock.to_desktop(stamp, now)
                vals = [float(v) for v in parts[2:]]
                rec = (t, *vals)
                if kind == "odom":
                    if live:
                        self.odom = rec
                    if self.odom_log is not None:
                        self.odom_log.append(rec)
                elif live:
                    self.odom2 = rec
                if live and kind == self.odom_src:     # ONE source only (frames differ)
                    self.integrator.odom(t, *vals[:3])
                continue
            if kind == "gyro" and len(parts) == 3:
                stamp = float(parts[1])
                if live:
                    self.clock.observe(now, stamp)
                    self.integrator.gyro(self.clock.to_desktop(stamp, now), float(parts[2]))
                continue
            if kind == "scan" and len(parts) >= 5:
                t = self.clock.to_desktop(float(parts[1]), now)
                self._scans.put((t, float(parts[2]), float(parts[3]),
                                 [int(v) / 1000.0 for v in parts[4:]]))
                continue
            if kind == "status" and len(parts) >= 4:
                if live:
                    self.subs, self.have_odom = int(parts[1]), parts[2] == "1"
                    self.voltage, self.status_t = float(parts[3]), now
                    if len(parts) >= 8:
                        self.have_rf2o, self.have_gyro = parts[4] == "1", parts[5] == "1"
                        self.have_scan, self.gyro_bias = parts[6] == "1", float(parts[7])
                continue
            if kind == "rosenv" and len(parts) == 4:
                self.rosenv = {"localhost_only": parts[1], "domain": parts[2],
                               "discovery_server": parts[3] == "1"}
                continue
            if kind == "guard_stop":
                self.guard_events.append((now, float(parts[1])))
            if kind == "pong":
                self.last_pong = now
                continue
            if self.logfile:
                self.logfile.write(f"{now:.3f} {line}{'' if live else '  (replayed)'}\n")
                self.logfile.flush()
            if kind == "ready":
                ready.set()
                continue
            if kind in EVENT_KINDS:
                t = now
                idx = TIMED_EVENTS.get(kind)
                if idx is not None and len(parts) > idx + 1:
                    try:
                        t = self.clock.to_desktop(float(parts[idx + 1]), now)
                    except ValueError:
                        pass
                self.events.put((t, kind, parts[1:]))
            else:                      # stray stderr (ROS warnings etc.)
                if kind not in ("[INFO]",):
                    self.log(f"[rover] {line}")
        if self.local:
            if proc is self.proc and not self._closing:
                self._alive = self._up = False
                self.events.put((time.time(), "eof", []))
        else:
            self._on_transport_lost(proc)

    def _heartbeat(self):
        while self._alive:
            if getattr(self, "_hb_pause", False):
                time.sleep(0.25)
                continue
            try:
                self._send("ping")
            except RoverLinkDown:
                pass
            except Exception:
                return
            time.sleep(0.25)

    def _wait_event(self, kind, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            try:
                t, k, args = self.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if k == kind:
                return t, args
            if k == "eof":
                raise RoverLinkError("rover agent exited / link gave up")
            if k == "link_down":                 # wait for the replay instead of failing
                t_end = max(t_end, time.time() + self.reconnect_timeout)
        raise RoverLinkError(f"timed out waiting for '{kind}' from rover agent")

    def _wait_status(self, cond, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if cond():
                return True
            if not self._alive and not self.local:
                raise RoverLinkError("rover link gave up")
            if self.local and (self.proc is None or self.proc.poll() is not None):
                raise RoverLinkError("rover agent exited")
            time.sleep(0.2)
        return cond()

    # ---------------- commands ----------------
    @property
    def rx_age(self):
        """Seconds since ANY line arrived from the agent (pongs come every 0.25 s), so a
        large value means the link is stalled even before ssh gives up (~15 s)."""
        t = self.last_rx
        return float("inf") if t is None else time.time() - t

    @property
    def alive(self):
        """The link is usable or reconnecting (False only after it gave up / closed)."""
        if self.local:
            return self._alive and self.proc is not None and self.proc.poll() is None
        return self._alive

    @property
    def link_up(self):
        return self._up

    def velocity(self, lin, ang):
        self._send(f"v {float(lin):.4f} {float(ang):.4f}")

    def stop(self):
        """Never raises for a dropped link: the agent already zeroed any velocity setpoint
        when the link dropped, and a loaded trial finishes on the rover by design."""
        try:
            self._send("stop")
            self._send("v 0 0")
        except RoverLinkDown:
            self.log("[rover] stop not delivered (link down; the rover already zeroed "
                     "velocity commands itself)")

    def guard(self, dist_m, yaw_offset=0.0, flip=False):
        """Lidar obstacle stop distance ahead of the moving face (0 disables), with the
        lidar's mounting yaw / mirroring so 'ahead' is really ahead."""
        self._send(f"guard {float(dist_m):.3f} {float(yaw_offset):.4f} {int(bool(flip))}")

    def scans(self, n=10, timeout=5.0):
        """Next n lidar scans -> [(t, angle_min, angle_inc, ranges_m list)]."""
        while not self._scans.empty():
            self._scans.get_nowait()
        self._send(f"scan {int(n)}")
        out, t_end = [], time.time() + timeout + 0.15 * n
        while len(out) < n and time.time() < t_end:
            try:
                out.append(self._scans.get(timeout=0.2))
            except queue.Empty:
                pass
        if not out:
            raise RoverLinkError("no lidar scans (is /scan publishing?)")
        return out

    def odometry_pose(self):
        """Latest integrated odometry pose (t, (x, y, yaw)) in the odom frame, or None."""
        return self.integrator.latest()

    def wait_link(self, timeout):
        """Block until the link is up again (True) or timeout (False)."""
        t_end = time.time() + timeout
        while time.time() < t_end and self._alive:
            if self._up:
                return True
            time.sleep(0.2)
        return self._up

    def run_sequence(self, steps, on_tick=None, tick=0.02, extra_timeout=5.0):
        """Run steps on the rover (timed locally). Returns the desktop time of seq_start
        (the rover's own stamp mapped to the desktop clock).

        on_tick(elapsed) is called every `tick` s while it runs; return a string from it
        to ABORT (e.g. geofence breach) - the rover is stopped and RoverLinkError raised.
        If the link drops mid-trial the rover finishes the sequence on its own (on_tick
        can't act then); this waits for the link and the replayed events, so it returns
        normally, later.
        """
        while not self.events.empty():
            self.events.get_nowait()
        clean = [{"linear": float(s.get("linear", 0)), "angular": float(s.get("angular", 0)),
                  "duration": float(s.get("duration", 0))} for s in steps]
        total = sum(s["duration"] for s in clean)
        if not self._up and not self.wait_link(self.reconnect_timeout):
            raise RoverLinkError("rover link down - could not start the sequence")
        self._send("seq " + json.dumps(clean, separators=(",", ":")))
        t_start, _ = self._wait_event("seq_start", 8.0)
        t_end = t_start + total + extra_timeout
        dropped = False
        while True:
            try:
                t, k, args = self.events.get(timeout=tick)
                if k == "seq_done":
                    if dropped:
                        self.log("[rover] trial finished on the rover during the dropout; "
                                 "its events arrived after reconnecting")
                    return t_start
                if k in ("seq_abort", "eof", "err"):
                    raise RoverLinkError(f"sequence aborted by rover: {k} {' '.join(args)}")
                if k == "link_down":
                    dropped = True
                    t_end = max(t_end, time.time() + self.reconnect_timeout)
            except queue.Empty:
                pass
            if on_tick is not None and self._up:
                why = on_tick(time.time() - t_start)
                if why:
                    try:
                        self.stop()
                    except RoverLinkDown:
                        pass
                    raise RoverLinkError(f"sequence aborted: {why}")
            if time.time() > t_end:
                try:
                    self.stop()
                except RoverLinkDown:
                    pass
                raise RoverLinkError("sequence never reported done")


def _kill(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


def link_test(link, seconds=20.0):
    """Ping the agent and report round-trip times (heartbeat replies) on the live link."""
    rtts, lost = [], 0
    link._hb_pause = True                        # only our pings in flight
    t_end = time.time() + seconds
    while time.time() < t_end:
        time.sleep(0.3)                          # let the heartbeat's own pong pass
        before = link.last_pong
        s = time.time()
        link._send("ping")
        while time.time() - s < 2.0 and link.last_pong == before:
            time.sleep(0.002)
        if link.last_pong != before:
            rtts.append((link.last_pong - s) * 1000)
        else:
            lost += 1
    link._hb_pause = False
    rtts.sort()
    if not rtts:
        return "no replies"
    q = lambda f: rtts[min(len(rtts) - 1, int(f * len(rtts)))]   # noqa: E731
    return (f"{len(rtts)} pings ({lost} lost) via {link.host}: median {q(0.5):.0f} ms, p90 {q(0.9):.0f} ms, "
            f"max {rtts[-1]:.0f} ms; clock offset rover->desktop "
            f"{(link.clock.offset or 0) * 1000:+.0f} ms")


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="connect, print status, exit")
    ap.add_argument("--link-test", action="store_true",
                    help="also measure round trips on the agent link for 20 s")
    ap.add_argument("--keep-bringup", action="store_true",
                    help="leave a base driver we started running on exit")
    ap.add_argument("--no-bringup", action="store_true")
    ap.add_argument("--host", default=ROVER, help="route(s) to try, comma list of ssh aliases")
    args = ap.parse_args()
    link = RoverLink(host=args.host, start_bringup=not args.no_bringup)
    try:
        link.open()
        t0 = time.time()
        while time.time() - t0 < 3:
            time.sleep(0.5)
        od = link.odom
        print(f"route={link.host} subs={link.subs} "
              f"odom={'%.3f,%.3f,%.1fdeg' % (od[1], od[2], od[3]*57.3) if od else None}"
              f" battery={link.voltage:.2f}V rf2o={link.have_rf2o} gyro={link.have_gyro} "
              f"(bias {link.gyro_bias:+.4f}) scan={link.have_scan}")
        if args.link_test:
            print(link_test(link))
    finally:
        link.close(stop_bringup=not args.keep_bringup)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
