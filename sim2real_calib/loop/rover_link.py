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


# Any failure talking to the rover (unreachable, agent died, sequence aborted ...).
class RoverLinkError(RuntimeError):
    pass


class RoverLinkDown(RoverLinkError):
    """The link is (temporarily) down; it reconnects by itself."""


# Which odometry the camera+odometry fusion uses: 'rf2o' (lidar) or 'wheel'.
def _odom_source(cfg):
    return (cfg or {}).get("odom_source", "rf2o")


# Converts rover timestamps to desktop time without synchronised clocks.
class ClockMap:
    """rover time -> desktop time. offset = min over a sliding window of (arrival - rover
    stamp) = clock skew + the link's best one-way delay; the min ignores queueing jitter
    and replayed backlog lines (which arrive late)."""

    def __init__(self, window_s=30.0):
        self.window_s = window_s
        self.samples = deque()                   # (arrival, arrival - stamp)
        self.offset = None

    # Record one (arrival time, rover stamp) pair; keep the minimum delay of the last
    # window_s seconds as the clock offset.
    def observe(self, arrival, stamp):
        apparent_delay = arrival - stamp
        self.samples.append((arrival, apparent_delay))
        while self.samples and arrival - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        self.offset = min(sample[1] for sample in self.samples)

    # Rover stamp -> desktop time (never later than when the line actually arrived).
    def to_desktop(self, stamp, arrival):
        if self.offset is None:
            return arrival
        return min(stamp + self.offset, arrival)  # never later than it actually arrived


# Desktop side of the rover connection. rover_agent.py runs detached ON the rover
# (survives Wi-Fi drops); this class reaches it through `ssh -W` to its TCP port, sends
# one-line commands ('v lin ang', 'stop', 'seq <json>', 'scan n', 'guard ...') and parses
# its telemetry lines (odom, gyro, scan, status, events) in a reader thread.
# State kept here: latest odometry, battery voltage, sensor flags, an event queue.
class RoverLink:
    # host = comma list of ssh routes tried in order (LAN then Tailscale). fusion_cfg
    # (loop_config 'fusion') sets the odometry integrator's yaw source and odom_scale.
    def __init__(self, host=ROVER, start_bringup=True, log=print, logfile=None,
                 fusion_cfg=None, reconnect_timeout=900.0):
        self.hosts = [route.strip() for route in str(host).split(",") if route.strip()]
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

    # Run one shell command on the rover over ssh (captured output).
    def _ssh(self, cmd, host=None, **kw):
        return subprocess.run(["ssh", *SSHOPTS, host or self.host, cmd], capture_output=True,
                              text=True, timeout=kw.pop("timeout", 30), **kw)

    # First route that answers `ssh true`.
    def _pick_host(self):
        errors = []
        for route in self.hosts:
            try:
                ssh_result = self._ssh("true", host=route, timeout=20)
                if ssh_result.returncode == 0:
                    return route
                errors.append(f"{route}: {ssh_result.stderr.strip()[:120]}")
            except subprocess.TimeoutExpired:
                errors.append(f"{route}: timed out")
        raise RoverLinkError("cannot reach the rover by any route (" + "; ".join(errors) + ")")

    # Copy rover_agent.py to the rover and (re)start it if the running copy differs
    # (sha256) or is dead. Returns True if a new agent was started.
    def _ensure_agent(self, host):
        """Stage rover_agent.py and make sure exactly THIS version is serving."""
        agent_source = (HERE / "rover_agent.py").read_bytes()
        agent_hash = hashlib.sha256(agent_source).hexdigest()[:16]
        # Liveness by PID FILE: a `pgrep -f rover_agent.py` would match this very ssh
        # command line (it contains the launch text) and report a dead agent as running.
        agent_alive_test = ("{ [ -f agent.pid ] && grep -q rover_agent /proc/$(cat agent.pid)/cmdline "
                 "2>/dev/null; }")
        remote_script = (
            f"mkdir -p {REMOTE_DIR} && cd {REMOTE_DIR} && "
            f"if [ \"$(cat agent.sha 2>/dev/null)\" != {agent_hash} ] || ! {agent_alive_test}; then "
            f"  {agent_alive_test} && {{ kill -INT $(cat agent.pid); sleep 1.5; }}; "
            f"  cat > rover_agent.py.new && mv rover_agent.py.new rover_agent.py && "
            f"  echo {agent_hash} > agent.sha && "
            f"  ({ENV}; setsid nohup python3 -u rover_agent.py --serve {AGENT_PORT} "
            f"     > agent.log 2>&1 < /dev/null & echo $! > agent.pid) ; echo started; "
            f"else cat > /dev/null; echo running; fi")
        ssh_result = subprocess.run(["ssh", *SSHOPTS, host, remote_script], input=agent_source, capture_output=True,
                           timeout=60)
        if ssh_result.returncode:
            raise RoverLinkError(f"could not start the agent on {host}: "
                                 f"{ssh_result.stderr.decode(errors='replace').strip()[:200]}")
        return ssh_result.stdout.decode().strip().splitlines()[-1:] == ["started"]

    # Open the `ssh -W 127.0.0.1:PORT` transport and wait for the agent's 'ready' line.
    def _connect(self, host, ready_timeout=25.0):
        """Open one ssh -W transport to the agent and wait for its 'ready'."""
        transport = subprocess.Popen(["ssh", *SSHOPTS, "-W", f"127.0.0.1:{AGENT_PORT}", host],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True, bufsize=1,
                                start_new_session=True)
        ready_event = threading.Event()
        threading.Thread(target=self._reader, args=(transport, ready_event), daemon=True).start()
        if not ready_event.wait(ready_timeout):
            _kill(transport)
            raise RoverLinkError(f"agent on {host} did not answer within {ready_timeout:.0f}s")
        old_transport, self.proc, self.host = self.proc, transport, host
        self._up = True
        if old_transport is not None and old_transport is not transport:
            _kill(old_transport)
        return transport

    # Connect: pick a route, ensure the agent, connect (3 tries), start the heartbeat,
    # refuse a networked rover ROS, start the base driver if /cmd_vel has no subscriber,
    # then choose the fusion odometry source.
    def open(self, ready_timeout=60.0):
        if self.local:                           # offline tests: stdio agent on this machine
            return self._open_local(ready_timeout)
        route = self._pick_host()
        agent_started = self._ensure_agent(route)
        for attempt in range(3):                 # a freshly started agent needs ROS init time
            try:
                self._connect(route, ready_timeout=25.0 if attempt else (30.0 if agent_started else 15.0))
                break
            except RoverLinkError:
                if attempt == 2:
                    raise
                time.sleep(2.0)
        self._alive = True
        threading.Thread(target=self._heartbeat, daemon=True).start()
        self.log(f"[rover] agent up on {self.host} (persistent, port {AGENT_PORT}; "
                 f"{'new' if agent_started else 'already running'})")
        self._check_isolation()
        # base driver: need a real /cmd_vel subscriber + live odometry
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, 4.0):
            if not self.want_bringup:
                raise RoverLinkError("rover base driver not running (/cmd_vel has no "
                                     "subscriber) and --no-bringup was given")
            self._start_bringup()
        wants_rf2o = _odom_source(self.fusion_cfg) == "rf2o"
        if wants_rf2o:
            self._wait_status(lambda: self.have_rf2o, 4.0)
        self.odom_src = "odom2" if (wants_rf2o and self.have_rf2o) else "odom"
        self._wait_status(lambda: self.have_gyro and self.have_scan, 3.0)
        self.log(f"[rover] base ready: /cmd_vel subs={self.subs}, battery {self.voltage:.2f} V, "
                 f"odometry for fusion = {'rf2o lidar' if self.odom_src == 'odom2' else 'wheel'}"
                 f"{'' if self.odom_src == 'odom2' or not wants_rf2o else ' (rf2o not publishing!)'}"
                 f", gyro {'yes' if self.have_gyro else 'NO'}, lidar {'yes' if self.have_scan else 'NO'}")

    # Offline test mode (host 'local'): run the agent on this machine over stdio.
    def _open_local(self, ready_timeout):
        agent_process = subprocess.Popen([sys.executable, "-u", str(HERE / "rover_agent.py")],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1,
                                start_new_session=True)
        ready_event = threading.Event()
        self.proc = agent_process
        threading.Thread(target=self._reader, args=(agent_process, ready_event), daemon=True).start()
        if not ready_event.wait(ready_timeout):
            raise RoverLinkError("local agent did not start")
        self._alive = self._up = True
        threading.Thread(target=self._heartbeat, daemon=True).start()
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, 15.0):
            raise RoverLinkError("local agent: no /cmd_vel subscriber or odom (fake rover?)")
        self._wait_status(lambda: self.have_rf2o and self.have_gyro and self.have_scan, 4.0)
        wants_rf2o = _odom_source(self.fusion_cfg) == "rf2o"
        self.odom_src = "odom2" if (wants_rf2o and self.have_rf2o) else "odom"
        self.log(f"[rover] local agent ready (odometry {self.odom_src})")

    # Safety: refuse to run unless the rover's ROS is local-only (see docstring).
    def _check_isolation(self):
        """The rover's ROS must be private to the Jetson. If it were networked on the
        desktop's domain, the Isaac sim's /cmd_vel (published during every sim
        evaluation) would ALSO drive the physical rover."""
        rover_ros_env = self.rosenv or {}
        ros_isolated = rover_ros_env.get("localhost_only") == "1" and not rover_ros_env.get("discovery_server")
        if not ros_isolated and not os.environ.get("LOOP_ALLOW_NETWORKED_ROVER"):
            self.close(stop_bringup=False)
            raise RoverLinkError(
                f"rover ROS is not isolated ({rover_ros_env}): Isaac's /cmd_vel could drive the real "
                "rover. Set ROS_LOCALHOST_ONLY=1 / unset ROS_DISCOVERY_SERVER on the rover "
                "(rover_nav/local_ros_env.sh), or export LOOP_ALLOW_NETWORKED_ROVER=1 if the "
                "rover is on a different ROS_DOMAIN_ID than the desktop.")

    @property
    def base_alive(self):
        """The base driver is up: /cmd_vel has a subscriber per the agent's last status.
        (The driver now runs detached on the rover, so a network drop can't kill it.)"""
        return self.subs >= 1

    # ---------------- reconnect ----------------
    # Reader thread hit end of stream: mark the link down and start reconnecting.
    def _on_transport_lost(self, proc):
        if self._closing or proc is not self.proc:
            return
        self._up = False
        self.link_drops += 1
        self.events.put((time.time(), "link_down", []))
        self.log(f"[rover] link to {self.host} dropped - reconnecting (a running trial "
                 f"finishes on the rover)")
        threading.Thread(target=self._reconnect_loop, daemon=True).start()

    # Retry every route every 3 s until reconnect_timeout (re-staging the agent every
    # third round in case the rover rebooted); posts link_up / eof events.
    def _reconnect_loop(self):
        if not self._reconnecting.acquire(blocking=False):
            return
        try:
            drop_time = time.time()
            deadline = drop_time + self.reconnect_timeout
            round_count = 0
            while not self._closing and time.time() < deadline:
                routes_in_order = [self.host] + [route for route in self.hosts if route != self.host]
                for route in routes_in_order:
                    if self._closing:
                        return
                    try:
                        if round_count % 3 == 2:          # agent itself gone? (rover rebooted)
                            self._ensure_agent(route)
                        self._connect(route)
                        self.log(f"[rover] link back via {route} after {time.time() - drop_time:.1f}s")
                        self.events.put((time.time(), "link_up", []))
                        return
                    except (RoverLinkError, subprocess.TimeoutExpired, OSError):
                        continue
                round_count += 1
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

    # Start ~/rover_nav/start_bringup.sh detached on the rover and wait until /cmd_vel
    # has a subscriber and odometry flows.
    def _start_bringup(self, timeout=90.0):
        self.log("[rover] base driver not running -> starting start_bringup.sh (detached on "
                 "the rover: survives network drops)")
        ssh_result = self._ssh(f"mkdir -p {REMOTE_DIR}; setsid nohup bash ~/rover_nav/start_bringup.sh "
                      f"> {REMOTE_DIR}/bringup.log 2>&1 < /dev/null & echo $!", timeout=40)
        try:
            self.bringup_pid = int(ssh_result.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise RoverLinkError(f"could not start the base driver: {ssh_result.stderr.strip()[:200]}")
        if not self._wait_status(lambda: self.subs >= 1 and self.have_odom, timeout):
            bringup_log_tail = self._ssh(f"tail -5 {REMOTE_DIR}/bringup.log", timeout=20).stdout
            raise RoverLinkError(f"base driver did not come up in {timeout:.0f}s "
                                 f"(subs={self.subs}, odom={self.have_odom}): {bringup_log_tail.strip()}")

    # Interrupt (then terminate) the base driver's process group; optionally run
    # stop_all.sh for any other stack.
    def _stop_bringup_remote(self, all_stacks=False):
        """Stop the base driver we started (its process group); all_stacks: whatever runs."""
        remote_commands = []
        if self.bringup_pid:
            remote_commands.append(f"kill -INT -{self.bringup_pid} 2>/dev/null; sleep 3; "
                       f"kill -TERM -{self.bringup_pid} 2>/dev/null")
        if all_stacks:
            remote_commands.append("bash ~/rover_nav/stop_all.sh >/dev/null 2>&1")
        if remote_commands:
            try:
                self._ssh("; ".join(remote_commands) + "; true", timeout=40)
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
    # Write one command line to the agent (RoverLinkDown if the link is reconnecting).
    def _send(self, line):
        transport = self.proc
        if transport is None or transport.poll() is not None or not self._up:
            if self._alive:
                raise RoverLinkDown("rover link is down (reconnecting)")
            raise RoverLinkError("rover agent is not running")
        with self._wlock:
            try:
                transport.stdin.write(line + "\n")
                transport.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                raise RoverLinkDown("rover link is down (reconnecting)")

    # Reader thread: parse every agent line. Telemetry (odom/odom2/gyro/scan/status/
    # rosenv) updates state; 'resume N' announces N replayed backlog lines after a
    # reconnect - those are logged and queued as events but don't update 'live' state;
    # other known kinds go to the event queue with rover times mapped to the desktop clock.
    def _reader(self, proc, ready):
        replay_left = 0
        for raw_line in proc.stdout:
            arrival_time = time.time()
            self.last_rx = arrival_time
            text_line = raw_line.strip()
            fields = text_line.split()
            if not fields:
                continue
            message_kind = fields[0]
            is_live = replay_left == 0
            if not is_live:
                replay_left -= 1
            if message_kind == "resume" and len(fields) == 2:
                replay_left = int(fields[1])
                if replay_left:
                    self.log(f"[rover] replaying {replay_left} lines buffered while the link was down")
                continue
            if message_kind == "dropped":
                self.log(f"[rover] WARNING: the rover dropped {fields[1]} buffered lines")
                continue
            if message_kind in ("odom", "odom2") and len(fields) == 7:
                rover_stamp = float(fields[1])
                if is_live:
                    self.clock.observe(arrival_time, rover_stamp)
                desktop_time = self.clock.to_desktop(rover_stamp, arrival_time)
                values = [float(field) for field in fields[2:]]
                odom_record = (desktop_time, *values)
                if message_kind == "odom":
                    if is_live:
                        self.odom = odom_record
                    if self.odom_log is not None:
                        self.odom_log.append(odom_record)
                elif is_live:
                    self.odom2 = odom_record
                if is_live and message_kind == self.odom_src:     # ONE source only (frames differ)
                    self.integrator.odom(desktop_time, *values[:3])
                continue
            if message_kind == "gyro" and len(fields) == 3:
                rover_stamp = float(fields[1])
                if is_live:
                    self.clock.observe(arrival_time, rover_stamp)
                    self.integrator.gyro(self.clock.to_desktop(rover_stamp, arrival_time), float(fields[2]))
                continue
            if message_kind == "scan" and len(fields) >= 5:
                desktop_time = self.clock.to_desktop(float(fields[1]), arrival_time)
                self._scans.put((desktop_time, float(fields[2]), float(fields[3]),
                                 [int(field) / 1000.0 for field in fields[4:]]))
                continue
            if message_kind == "status" and len(fields) >= 4:
                if is_live:
                    self.subs, self.have_odom = int(fields[1]), fields[2] == "1"
                    self.voltage, self.status_t = float(fields[3]), arrival_time
                    if len(fields) >= 8:
                        self.have_rf2o, self.have_gyro = fields[4] == "1", fields[5] == "1"
                        self.have_scan, self.gyro_bias = fields[6] == "1", float(fields[7])
                continue
            if message_kind == "rosenv" and len(fields) == 4:
                self.rosenv = {"localhost_only": fields[1], "domain": fields[2],
                               "discovery_server": fields[3] == "1"}
                continue
            if message_kind == "guard_stop":
                self.guard_events.append((arrival_time, float(fields[1])))
            if message_kind == "pong":
                self.last_pong = arrival_time
                continue
            if self.logfile:
                self.logfile.write(f"{arrival_time:.3f} {text_line}{'' if is_live else '  (replayed)'}\n")
                self.logfile.flush()
            if message_kind == "ready":
                ready.set()
                continue
            if message_kind in EVENT_KINDS:
                desktop_time = arrival_time
                time_field_index = TIMED_EVENTS.get(message_kind)
                if time_field_index is not None and len(fields) > time_field_index + 1:
                    try:
                        desktop_time = self.clock.to_desktop(float(fields[time_field_index + 1]), arrival_time)
                    except ValueError:
                        pass
                self.events.put((desktop_time, message_kind, fields[1:]))
            else:                      # stray stderr (ROS warnings etc.)
                if message_kind not in ("[INFO]",):
                    self.log(f"[rover] {text_line}")
        if self.local:
            if proc is self.proc and not self._closing:
                self._alive = self._up = False
                self.events.put((time.time(), "eof", []))
        else:
            self._on_transport_lost(proc)

    # Ping every 0.25 s while alive (the agent zeroes velocity if pings stop).
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

    # Wait for one event kind from the queue (extends the wait while the link is down).
    def _wait_event(self, kind, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                event_time, event_kind, args = self.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if event_kind == kind:
                return event_time, args
            if event_kind == "eof":
                raise RoverLinkError("rover agent exited / link gave up")
            if event_kind == "link_down":                 # wait for the replay instead of failing
                deadline = max(deadline, time.time() + self.reconnect_timeout)
        raise RoverLinkError(f"timed out waiting for '{kind}' from rover agent")

    # Poll a condition on the parsed status until true or timeout.
    def _wait_status(self, cond, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
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
        last_rx_time = self.last_rx
        return float("inf") if last_rx_time is None else time.time() - last_rx_time

    @property
    def alive(self):
        """The link is usable or reconnecting (False only after it gave up / closed)."""
        if self.local:
            return self._alive and self.proc is not None and self.proc.poll() is None
        return self._alive

    @property
    def link_up(self):
        return self._up

    # Command a body twist: forward speed (m/s) and turn rate (rad/s).
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
        received, deadline = [], time.time() + timeout + 0.15 * n
        while len(received) < n and time.time() < deadline:
            try:
                received.append(self._scans.get(timeout=0.2))
            except queue.Empty:
                pass
        if not received:
            raise RoverLinkError("no lidar scans (is /scan publishing?)")
        return received

    def odometry_pose(self):
        """Latest integrated odometry pose (t, (x, y, yaw)) in the odom frame, or None."""
        return self.integrator.latest()

    def wait_link(self, timeout):
        """Block until the link is up again (True) or timeout (False)."""
        deadline = time.time() + timeout
        while time.time() < deadline and self._alive:
            if self._up:
                return True
            time.sleep(0.2)
        return self._up

    # Upload a list of {linear, angular, duration} steps; the agent times them locally
    # on the rover. Waits for seq_done, calling on_tick(elapsed) so the caller can abort
    # (e.g. the rover left the workspace). Returns the sequence start in desktop time.
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
        clean_steps = [{"linear": float(step.get("linear", 0)), "angular": float(step.get("angular", 0)),
                  "duration": float(step.get("duration", 0))} for step in steps]
        total_duration = sum(step["duration"] for step in clean_steps)
        if not self._up and not self.wait_link(self.reconnect_timeout):
            raise RoverLinkError("rover link down - could not start the sequence")
        self._send("seq " + json.dumps(clean_steps, separators=(",", ":")))
        seq_start_time, _ = self._wait_event("seq_start", 8.0)
        deadline = seq_start_time + total_duration + extra_timeout
        link_dropped = False
        while True:
            try:
                event_time, event_kind, args = self.events.get(timeout=tick)
                if event_kind == "seq_done":
                    if link_dropped:
                        self.log("[rover] trial finished on the rover during the dropout; "
                                 "its events arrived after reconnecting")
                    return seq_start_time
                if event_kind in ("seq_abort", "eof", "err"):
                    raise RoverLinkError(f"sequence aborted by rover: {event_kind} {' '.join(args)}")
                if event_kind == "link_down":
                    link_dropped = True
                    deadline = max(deadline, time.time() + self.reconnect_timeout)
            except queue.Empty:
                pass
            if on_tick is not None and self._up:
                abort_reason = on_tick(time.time() - seq_start_time)
                if abort_reason:
                    try:
                        self.stop()
                    except RoverLinkDown:
                        pass
                    raise RoverLinkError(f"sequence aborted: {abort_reason}")
            if time.time() > deadline:
                try:
                    self.stop()
                except RoverLinkDown:
                    pass
                raise RoverLinkError("sequence never reported done")


# Terminate a child's whole process group (SIGTERM, then SIGKILL after 3 s).
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


# Measure ping round trips over the live link for `seconds` (median / p90 / max).
def link_test(link, seconds=20.0):
    """Ping the agent and report round-trip times (heartbeat replies) on the live link."""
    round_trips_ms, lost_pings = [], 0
    link._hb_pause = True                        # only our pings in flight
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(0.3)                          # let the heartbeat's own pong pass
        pong_before = link.last_pong
        sent_time = time.time()
        link._send("ping")
        while time.time() - sent_time < 2.0 and link.last_pong == pong_before:
            time.sleep(0.002)
        if link.last_pong != pong_before:
            round_trips_ms.append((link.last_pong - sent_time) * 1000)
        else:
            lost_pings += 1
    link._hb_pause = False
    round_trips_ms.sort()
    if not round_trips_ms:
        return "no replies"
    percentile = lambda f: round_trips_ms[min(len(round_trips_ms) - 1, int(f * len(round_trips_ms)))]   # noqa: E731
    return (f"{len(round_trips_ms)} pings ({lost_pings} lost) via {link.host}: median {percentile(0.5):.0f} ms, p90 {percentile(0.9):.0f} ms, "
            f"max {round_trips_ms[-1]:.0f} ms; clock offset rover->desktop "
            f"{(link.clock.offset or 0) * 1000:+.0f} ms")


# --check: connect, print one status line (route, subscribers, odometry, battery,
# sensors) and disconnect. surface_demo.py parses the 'battery=' field of this line.
def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="connect, print status, exit")
    parser.add_argument("--link-test", action="store_true",
                    help="also measure round trips on the agent link for 20 s")
    parser.add_argument("--keep-bringup", action="store_true",
                    help="leave a base driver we started running on exit")
    parser.add_argument("--no-bringup", action="store_true")
    parser.add_argument("--host", default=ROVER, help="route(s) to try, comma list of ssh aliases")
    args = parser.parse_args()
    rover_link = RoverLink(host=args.host, start_bringup=not args.no_bringup)
    try:
        rover_link.open()
        start_time = time.time()
        while time.time() - start_time < 3:
            time.sleep(0.5)
        wheel_odom = rover_link.odom
        print(f"route={rover_link.host} subs={rover_link.subs} "
              f"odom={'%.3f,%.3f,%.1fdeg' % (wheel_odom[1], wheel_odom[2], wheel_odom[3]*57.3) if wheel_odom else None}"
              f" battery={rover_link.voltage:.2f}V rf2o={rover_link.have_rf2o} gyro={rover_link.have_gyro} "
              f"(bias {rover_link.gyro_bias:+.4f}) scan={rover_link.have_scan}")
        if args.link_test:
            print(link_test(rover_link))
    finally:
        rover_link.close(stop_bringup=not args.keep_bringup)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
