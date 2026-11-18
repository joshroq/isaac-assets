#!/usr/bin/env python3
"""Rover-side command agent for the closed sim2real loop (runs ON the Jetson).

The real rover is driven entirely on its local /cmd_vel (native multicast, local-only ROS
- no DDS across the network). Two transports, same line protocol:

  stdio (default)   the desktop runs this inside one SSH session and talks through
                    stdin/stdout; stdin EOF (SSH dropped) zeroes and exits.
  --serve PORT      PERSISTENT (link-tolerant, 2026-11-11): started detached on the rover,
                    listens on 127.0.0.1:PORT; the desktop connects with `ssh -W
                    127.0.0.1:PORT rover` and simply reconnects after a network drop. A
                    dropped link zeroes any velocity setpoint at once, but a pre-loaded
                    `seq` (a trial) FINISHES ON THE ROVER (bounded by its own duration;
                    the lidar obstacle stop stays active) - the user's chosen policy, so
                    weak Wi-Fi or a campus relay no longer aborts trials. Everything
                    emitted while no desktop is connected is buffered and replayed (after
                    `resume N`) on reconnect, so the trial's events and odometry arrive
                    late but complete. Exits on `quit`, on signals, or after --idle-exit s
                    with no client and no running sequence (it publishes /cmd_vel zeros
                    while alive, which would fight Nav2 / teleop).

stdin (one command per line)
    v LIN ANG     velocity setpoint (deadman: zeroed if not refreshed for --deadman s)
    seq JSON      run a timed step list locally: [{"linear":..,"angular":..,"duration":..}]
    stop          abort any sequence and zero
    scan N        send the next N lidar scans (station fingerprints / lidar view)
    guard D [YAW FLIP]  lidar obstacle stop: block translation toward anything within
                  D m of the chassis edge in the direction of travel (0 = off); YAW/FLIP =
                  lidar mounting yaw (rad) and mirrored-scan flag
    ping          heartbeat (any line counts); replies 'pong'
    quit          zero and exit

stdout (one event per line, flushed)
    rosenv LOCALHOST_ONLY DOMAIN_ID DISCOVERY_SERVER(0/1)   (sent once, before ready)
    ready
    status SUBS HAVE_ODOM VOLTAGE HAVE_RF2O HAVE_GYRO HAVE_SCAN GYRO_BIAS   (1 Hz)
    odom T X Y YAW VX WZ            wheel odometry (/odom)
    odom2 T X Y YAW VX WZ           lidar odometry (/odom_rf2o)
    gyro T WZ                       IMU yaw rate, bias-corrected (bias learnt while still)
    scan T ANGLE_MIN ANGLE_INC R0 R1 ...   ranges in mm (0 = no return)
    guard_stop DIST                 translation blocked by an obstacle
    seq_start T / seq_step I T / seq_done T / seq_abort REASON
    pong / err MSG / bye
    resume N / dropped N            (--serve) N buffered lines follow / N were lost (overflow)

Safety: the Waveshare base has no cmd_vel timeout - it HOLDS the last velocity.
So this agent publishes /cmd_vel continuously (zeros when idle), zeroes when the
setpoint is stale, when the link goes silent for --link-timeout s (heartbeats
stop), on stdin EOF (SSH dropped), on any malformed command, and on
SIGTERM/SIGHUP/SIGINT. Output goes through a writer THREAD: the control loop never
blocks on the network (before 2026-11-11 a stalled SSH filled the stdout pipe in ~6 s,
blocking the loop - no zeros, no watchdog - while the base held its last command).
"""
import argparse
import collections
import json
import math
import queue
import signal
import socket
import sys
import threading
import time

OUTBOX_MAX = 120000            # lines kept while the desktop is away (~10 min of telemetry)


class Channel:
    """Line transport. The control loop only calls emit() (never blocks) and drains
    `lines`; a writer thread does the (possibly blocking) network writes."""

    def __init__(self):
        self.lines = queue.Queue()
        self.outbox = collections.deque()
        self.cv = threading.Condition()
        self.dropped = 0

    def emit(self, *parts):
        line = " ".join(str(p) for p in parts) + "\n"
        with self.cv:
            if len(self.outbox) >= OUTBOX_MAX:
                self.outbox.popleft()
                self.dropped += 1
            self.outbox.append(line)
            self.cv.notify()

    def _next(self):
        with self.cv:
            while not self.outbox:
                self.cv.wait(0.5)
            return self.outbox.popleft()


class StdioChannel(Channel):
    connected = True

    def start(self):
        def reader():
            for raw in sys.stdin:
                self.lines.put(raw.strip())
            self.lines.put(None)             # EOF: SSH dropped / desktop closed us

        def writer():
            while True:
                line = self._next()
                try:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                except (BrokenPipeError, OSError):
                    return
        threading.Thread(target=reader, daemon=True).start()
        threading.Thread(target=writer, daemon=True).start()


class ServerChannel(Channel):
    """One desktop at a time on 127.0.0.1:PORT; a new connection replaces the old one.
    While nobody is connected the outbox simply accumulates (the backlog)."""

    def __init__(self, port, header):
        super().__init__()
        self.port, self.header = port, header
        self.conn = None
        self.gen = 0
        self.connected = False
        self.last_client_t = time.time()

    def start(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.port))
        srv.listen(2)
        threading.Thread(target=self._accept, args=(srv,), daemon=True).start()
        threading.Thread(target=self._writer, daemon=True).start()

    def _accept(self, srv):
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.cv:
                old, self.conn = self.conn, conn
                self.gen += 1
                gen = self.gen
                backlog = len(self.outbox)
                head = "".join(" ".join(map(str, h)) + "\n" for h in self.header())
                if self.dropped:
                    head += f"dropped {self.dropped}\n"
                    self.dropped = 0
                head += f"resume {backlog}\n"
                self.connected = True
                self.last_client_t = time.time()
                self.cv.notify()
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
            try:
                conn.sendall(head.encode())
            except OSError:
                pass
            self.lines.put(("__connect__", gen))
            threading.Thread(target=self._reader, args=(conn, gen), daemon=True).start()

    def _reader(self, conn, gen):
        buf = b""
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    self.lines.put(raw.decode(errors="replace").strip())
        except OSError:
            pass
        with self.cv:
            if gen == self.gen:              # still the current client: it is gone
                self.conn, self.connected = None, False
                self.last_client_t = time.time()
        self.lines.put(("__disconnect__", gen))

    def _writer(self):
        while True:
            with self.cv:
                while not (self.outbox and self.conn is not None):
                    self.cv.wait(0.5)
                conn, line = self.conn, self.outbox.popleft()
            try:
                conn.sendall(line.encode())
            except OSError:
                with self.cv:                # keep it for the next client
                    self.outbox.appendleft(line)
                    if self.conn is conn:
                        self.conn, self.connected = None, False
                time.sleep(0.2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--deadman", type=float, default=0.5,
                    help="zero a 'v' setpoint not refreshed within this many seconds")
    ap.add_argument("--link-timeout", type=float, default=1.5,
                    help="zero + abort if NO line (incl. ping) arrives for this long")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--odom2-topic", default="/odom_rf2o")
    ap.add_argument("--imu-topic", default="/imu/data_raw")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--max-linear", type=float, default=0.7)
    ap.add_argument("--max-angular", type=float, default=2.5)
    # chassis box around the lidar origin (base_footprint): returns inside it are the
    # rover itself; the guard looks ahead of its front/rear face
    ap.add_argument("--half-length", type=float, default=0.13)
    ap.add_argument("--half-width", type=float, default=0.10)
    ap.add_argument("--serve", type=int, default=0, metavar="PORT",
                    help="persistent mode: listen on 127.0.0.1:PORT (see module doc)")
    ap.add_argument("--idle-exit", type=float, default=1800.0,
                    help="--serve: exit after this many s with no client and no sequence")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Float32

    rclpy.init()
    node = Node("sim2real_rover_agent")
    pub = node.create_publisher(Twist, args.cmd_topic, 10)
    st = {"odom_t": 0.0, "odom2_t": 0.0, "gyro_t": 0.0, "scan_t": 0.0,
          "volt": float("nan"), "bias": 0.0, "bias_n": 0, "scan": None,
          "scan_req": 0, "still_since": None, "commanding": False}
    out = queue.Queue()            # telemetry lines produced in callbacks

    def on_odom(msg, tag):
        p = msg.pose.pose.position
        now = time.time()
        st[tag + "_t"] = now
        out.put((tag, f"{now:.4f}", f"{p.x:.5f}", f"{p.y:.5f}",
                 f"{yaw_of(msg.pose.pose.orientation):.5f}",
                 f"{msg.twist.twist.linear.x:.4f}", f"{msg.twist.twist.angular.z:.4f}"))

    def on_imu(msg):
        now = time.time()
        wz = float(msg.angular_velocity.z)
        st["gyro_t"] = now
        # learn the gyro bias while the rover is commanded still AND has been for 1 s
        still = st["still_since"] is not None and now - st["still_since"] > 1.0
        if still and abs(wz - st["bias"]) < 0.05:
            st["bias_n"] = min(st["bias_n"] + 1, 400)
            st["bias"] += (wz - st["bias"]) / st["bias_n"]
        out.put(("gyro", f"{now:.4f}", f"{wz - st['bias']:.5f}"))

    def on_scan(msg):
        st["scan"] = msg
        st["scan_t"] = time.time()
        if st["scan_req"] > 0:
            st["scan_req"] -= 1
            rr = " ".join(str(int(r * 1000)) if math.isfinite(r) and msg.range_min <= r
                          <= msg.range_max else "0" for r in msg.ranges)
            out.put(("scan", f"{st['scan_t']:.4f}", f"{msg.angle_min:.6f}",
                     f"{msg.angle_increment:.8f}", rr))

    node.create_subscription(Odometry, args.odom_topic, lambda m: on_odom(m, "odom"),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, args.odom2_topic, lambda m: on_odom(m, "odom2"),
                             qos_profile_sensor_data)
    node.create_subscription(Imu, args.imu_topic, on_imu, qos_profile_sensor_data)
    node.create_subscription(LaserScan, args.scan_topic, on_scan, qos_profile_sensor_data)
    node.create_subscription(Float32, "/voltage", lambda m: st.update(volt=float(m.data)), 10)

    import os

    def rosenv_header():
        return [("rosenv", os.environ.get("ROS_LOCALHOST_ONLY", "unset"),
                 os.environ.get("ROS_DOMAIN_ID", "unset"),
                 "1" if os.environ.get("ROS_DISCOVERY_SERVER") else "0"), ("ready",)]

    chan = ServerChannel(args.serve, rosenv_header) if args.serve else StdioChannel()
    chan.start()
    emit = chan.emit
    lines = chan.lines

    stop_flag = {"stop": False}

    def on_signal(_s, _f):
        stop_flag["stop"] = True
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, on_signal)

    def clamp(v, lim):
        return max(-lim, min(lim, float(v)))

    def publish(lin, ang):
        m = Twist()
        m.linear.x = clamp(lin, args.max_linear)
        m.angular.z = clamp(ang, args.max_angular)
        pub.publish(m)

    def obstacle_ahead(lin, dist, yaw_off=0.0, flip=False):
        """Nearest return in the swept box ahead of the moving face, or None."""
        sc = st["scan"]
        if sc is None or time.time() - st["scan_t"] > 0.6 or dist <= 0 or lin == 0:
            return None
        sgn = 1.0 if lin > 0 else -1.0
        best = None
        a = sc.angle_min
        for r in sc.ranges:
            if math.isfinite(r) and 0.03 < r < 3.0:
                b = (-a if flip else a) + yaw_off
                x, y = r * math.cos(b) * sgn, r * math.sin(b)
                inside_self = abs(x) <= args.half_length and abs(y) <= args.half_width
                if not inside_self and abs(y) <= args.half_width + 0.03:
                    gap = x - args.half_length
                    if 0 <= gap <= dist and (best is None or gap < best):
                        best = gap
            a += sc.angle_increment
        return best

    setpoint = (0.0, 0.0)
    setpoint_t = 0.0
    last_line_t = time.time()
    seq = None                               # {"steps", "t0", "i"}
    guard_d = 0.0
    guard_yaw, guard_flip = 0.0, False      # lidar mounting yaw / mirrored scan
    last_guard_emit = 0.0
    period = 1.0 / args.rate
    last_status = 0.0
    if not args.serve:                       # (--serve sends these per connection)
        for h in rosenv_header():
            emit(*h)
    try:
        while not stop_flag["stop"] and rclpy.ok():
            now = time.time()
            # ---- drain commands ----
            while True:
                try:
                    line = lines.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    stop_flag["stop"] = True
                    break
                if isinstance(line, tuple):      # --serve connection change
                    last_line_t = now
                    if line[0] == "__disconnect__":
                        setpoint = (0.0, 0.0)    # velocity mode needs the desktop: stop
                        emit("link_lost", f"{now:.4f}", "seq_continues" if seq else "idle")
                    continue
                last_line_t = now
                if not line:
                    continue
                op, _, rest = line.partition(" ")
                try:
                    if op == "v":
                        a, b = rest.split()
                        setpoint, setpoint_t = (float(a), float(b)), now
                    elif op == "seq":
                        steps = json.loads(rest)
                        for s in steps:
                            for k in ("linear", "angular", "duration"):
                                s[k] = float(s.get(k, 0.0))
                        seq = {"steps": steps, "t0": now, "i": -1}
                        setpoint = (0.0, 0.0)
                        emit("seq_start", f"{now:.4f}")
                    elif op == "stop":
                        if seq is not None:
                            emit("seq_abort", "stop")
                        seq, setpoint = None, (0.0, 0.0)
                    elif op == "scan":
                        st["scan_req"] = max(1, min(int(rest or 1), 100))
                    elif op == "guard":
                        g = rest.split()
                        guard_d = max(0.0, float(g[0]))
                        guard_yaw = float(g[1]) if len(g) > 1 else 0.0
                        guard_flip = len(g) > 2 and g[2] == "1"
                    elif op == "ping":
                        emit("pong")
                    elif op == "quit":
                        stop_flag["stop"] = True
                    else:                    # unknown command: fail safe
                        if seq is not None:
                            emit("seq_abort", "bad_command")
                        seq, setpoint = None, (0.0, 0.0)
                        emit("err", "unknown", op)
                except Exception as exc:     # malformed line: never move on it
                    seq, setpoint = None, (0.0, 0.0)
                    emit("err", repr(exc).replace("\n", " "))
            if stop_flag["stop"]:
                break

            # ---- link watchdog ----
            if now - last_line_t > args.link_timeout:
                if seq is not None and not args.serve:   # stdio: old policy
                    emit("seq_abort", "link_timeout")
                    seq = None
                setpoint = (0.0, 0.0)            # --serve: a loaded trial finishes on-rover
            if (args.serve and seq is None and not chan.connected
                    and now - chan.last_client_t > args.idle_exit):
                break                            # abandoned: stop fighting other /cmd_vel users

            # ---- choose the command ----
            lin = ang = 0.0
            if seq is not None:
                el, acc = now - seq["t0"], 0.0
                for i, s in enumerate(seq["steps"]):
                    acc += s["duration"]
                    if el < acc:
                        if i != seq["i"]:
                            seq["i"] = i
                            emit("seq_step", i, f"{now:.4f}")
                        lin, ang = s["linear"], s["angular"]
                        break
                else:
                    emit("seq_done", f"{now:.4f}")
                    seq = None
            elif now - setpoint_t <= args.deadman:
                lin, ang = setpoint

            # ---- lidar obstacle guard (translation only; turning in place is allowed) ----
            if guard_d > 0 and lin != 0:
                gap = obstacle_ahead(lin, guard_d, guard_yaw, guard_flip)
                if gap is not None:
                    if now - last_guard_emit > 1.0:
                        emit("guard_stop", f"{gap:.3f}")
                        last_guard_emit = now
                    if seq is not None:              # a trial must not continue blocked
                        emit("seq_abort", f"obstacle_{gap:.2f}m")
                        seq, ang = None, 0.0
                    lin = 0.0                        # homing may still turn in place
            moving = bool(lin or ang)
            if moving:
                st["still_since"] = None
            elif st["still_since"] is None:
                st["still_since"] = now
            publish(lin, ang)

            # ---- telemetry ----
            rclpy.spin_once(node, timeout_sec=0.0)
            while True:
                try:
                    emit(*out.get_nowait())
                except queue.Empty:
                    break
            if now - last_status >= 1.0:
                last_status = now
                fresh = lambda k: int(now - st[k] < 1.0)   # noqa: E731
                emit("status", pub.get_subscription_count(), fresh("odom_t"),
                     f"{st['volt']:.2f}", fresh("odom2_t"), fresh("gyro_t"), fresh("scan_t"),
                     f"{st['bias']:.5f}")
            # sleep the remainder of the tick while still servicing callbacks
            end = now + period
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=max(0.0, end - time.time()))
    finally:
        for _ in range(int(0.5 * args.rate)):    # hammer zero: base holds last command
            publish(0.0, 0.0)
            time.sleep(period)
        emit("bye")
        time.sleep(0.3)                          # let the writer flush 'bye'
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
