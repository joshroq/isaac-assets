#!/usr/bin/env python3
"""Safety regression for rover_agent.py WITHOUT the rover: run the agent on the desktop's
own ROS (isolated domain), listen to its /cmd_vel, and check it zeroes correctly.

    source ros2env.sh && python3 loop/test_agent_local.py
"""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
os.environ.update(ROS_DOMAIN_ID="87", ROS_LOCALHOST_ONLY="1")

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402

rclpy.init()
node = rclpy.create_node("agent_test_listener")
last = {"cmd": None, "t": 0.0}
node.create_subscription(Twist, "/cmd_vel",
                         lambda m: last.update(cmd=(m.linear.x, m.angular.z), t=time.time()), 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

agent = subprocess.Popen([sys.executable, "-u", str(HERE / "rover_agent.py")],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
events = []
threading.Thread(target=lambda: [events.append((time.time(), l.strip())) for l in agent.stdout],
                 daemon=True).start()


def send(s):
    agent.stdin.write(s + "\n")
    agent.stdin.flush()


def heartbeat(sec):
    t = time.time() + sec
    while time.time() < t:
        send("ping")
        time.sleep(0.2)


def cmd_now():
    time.sleep(0.15)
    return last["cmd"]


fails = []


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        fails.append(name)


t0 = time.time()
while not any(e[1] == "ready" for e in events) and time.time() - t0 < 15:
    time.sleep(0.1)
check("agent ready", any(e[1] == "ready" for e in events))
heartbeat(1.0)
check("idle publishes zero", cmd_now() == (0.0, 0.0))

send("v 0.2 0.5")
check("velocity setpoint applied", cmd_now() == (0.2, 0.5))
heartbeat(0.8)                      # no refresh of 'v' for 0.8 s (> 0.5 s deadman)
check("deadman zeroes a stale setpoint", cmd_now() == (0.0, 0.0))

send('seq [{"linear":0.1,"angular":0,"duration":0.6},{"linear":0,"angular":0.4,"duration":0.6}]')
time.sleep(0.3)
check("sequence step 0", last["cmd"] == (0.1, 0.0))
send("ping")
time.sleep(0.6)
check("sequence step 1", last["cmd"] == (0.0, 0.4))
heartbeat(0.6)
check("sequence done -> zero", cmd_now() == (0.0, 0.0) and any(e[1].startswith("seq_done") for e in events))

send('seq [{"linear":0.15,"angular":0,"duration":10}]')
time.sleep(0.3)
check("long sequence running", last["cmd"] == (0.15, 0.0))
time.sleep(2.0)                     # desktop goes SILENT (no pings) > 1.5 s link timeout
check("link timeout aborts + zeroes", cmd_now() == (0.0, 0.0)
      and any("link_timeout" in e[1] for e in events))

send("v 0.2 0")
send("clearly not a command 1 2")
check("malformed line -> zero", cmd_now() == (0.0, 0.0))

send("v 0.2 0")
time.sleep(0.2)
agent.stdin.close()                 # SSH drop = stdin EOF while moving
agent.wait(timeout=5)
time.sleep(0.2)
check("EOF while moving -> final zero + exit", last["cmd"] == (0.0, 0.0) and agent.returncode == 0)

rclpy.shutdown()
print("ALL PASS" if not fails else f"FAILED: {fails}")
sys.exit(1 if fails else 0)
