#!/usr/bin/env python3
"""Link-tolerance regression for rover_agent.py --serve WITHOUT the rover: the agent runs
on the desktop's own ROS (isolated domain); we listen to its /cmd_vel and act as the
desktop over TCP, dropping and restoring the link the way weak Wi-Fi / a campus relay does.

    source ros2env.sh && python3 loop/test_agent_serve.py
"""
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PORT = 7891
os.environ.update(ROS_DOMAIN_ID="88", ROS_LOCALHOST_ONLY="1")

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402

rclpy.init()
node = rclpy.create_node("agent_serve_test_listener")
last = {"cmd": None}
node.create_subscription(Twist, "/cmd_vel",
                         lambda m: last.update(cmd=(m.linear.x, m.angular.z)), 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

agent = subprocess.Popen([sys.executable, "-u", str(HERE / "rover_agent.py"), "--serve", str(PORT),
                          "--idle-exit", "6"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
fails = []


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        fails.append(name)


class Client:
    def __init__(self, read=True):
        t0 = time.time()
        while True:
            try:
                self.s = socket.create_connection(("127.0.0.1", PORT), timeout=2)
                break
            except OSError:
                if time.time() - t0 > 20:
                    raise
                time.sleep(0.2)
        self.lines = []
        if read:
            threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = b""
        try:
            while True:
                c = self.s.recv(65536)
                if not c:
                    return
                buf += c
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    self.lines.append(raw.decode())
        except OSError:
            return

    def send(self, line):
        self.s.sendall((line + "\n").encode())

    def has(self, prefix):
        return any(ln.startswith(prefix) for ln in self.lines)

    def close(self):
        try:
            self.s.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.s.close()


def cmd_now():
    time.sleep(0.15)
    return last["cmd"]


def heartbeat(c, sec):
    t = time.time() + sec
    while time.time() < t:
        c.send("ping")
        time.sleep(0.2)


c = Client()
t0 = time.time()
while not c.has("ready") and time.time() - t0 < 15:
    time.sleep(0.1)
check("client 1: rosenv + ready + resume header", c.has("rosenv") and c.has("ready") and c.has("resume"))
heartbeat(c, 0.6)
check("idle publishes zero", cmd_now() == (0.0, 0.0))

c.send("v 0.2 0.5")
check("velocity setpoint applied", cmd_now() == (0.2, 0.5))
c.send("v 0.2 0.5")
c.close()                                  # link drops while in VELOCITY mode
check("link drop in velocity mode -> zero immediately", cmd_now() == (0.0, 0.0))

c = Client()
time.sleep(0.5)
c.send('seq [{"linear":0.12,"angular":0,"duration":3.0},{"linear":0,"angular":0.3,"duration":1.0}]')
time.sleep(0.4)
check("trial running", last["cmd"] == (0.12, 0.0))
c.close()                                  # link drops DURING the trial
time.sleep(1.0)
check("link drop mid-trial: trial keeps going on the rover", last["cmd"] == (0.12, 0.0))
time.sleep(2.2)
check("... and moves on to its next step", last["cmd"] == (0.0, 0.3))
time.sleep(1.2)
check("... and stops at its end", cmd_now() == (0.0, 0.0))

c = Client()                               # link comes back
t0 = time.time()
while not c.has("seq_done") and time.time() - t0 < 5:
    time.sleep(0.1)
check("reconnect replays the backlog: link_lost, seq_step 1, seq_done",
      c.has("resume") and c.has("link_lost") and any(ln.startswith("seq_step 1") for ln in c.lines)
      and c.has("seq_done"))
check("replayed telemetry keeps rover timestamps (status lines present)", c.has("status"))

# a stalled desktop that never reads: the outbox grows but the control loop must not block
stall = Client(read=False)
stall.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
stall.send('seq [{"linear":0.1,"angular":0,"duration":2.0}]')
time.sleep(1.0)
check("never-reading client: trial still timed by the rover", last["cmd"] == (0.1, 0.0))
time.sleep(1.4)
check("never-reading client: trial still stops on time", cmd_now() == (0.0, 0.0))
stall.close()

c = Client()
time.sleep(0.4)
c.send('seq [{"linear":0.1,"angular":0,"duration":5.0}]')
time.sleep(0.3)
c.send("stop")
check("explicit stop still aborts a trial", cmd_now() == (0.0, 0.0) and True)
c.close()

time.sleep(7.5)                            # no client, no sequence > --idle-exit 6 s
check("abandoned agent exits by itself (idle-exit)", agent.poll() is not None)
check("... after a final zero", last["cmd"] == (0.0, 0.0))
if agent.poll() is None:
    agent.terminate()
rclpy.shutdown()
print("ALL PASS" if not fails else f"FAILED: {fails}")
sys.exit(1 if fails else 0)
