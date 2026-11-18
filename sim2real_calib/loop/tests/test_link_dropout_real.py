#!/usr/bin/env python3
"""HARDWARE link test (the rover does not move: every command is zero velocity).

Cuts the desktop->rover transport in the middle of a loaded (stationary) trial, the way a
Wi-Fi dropout or a campus relay hiccup would, and checks that the trial completes on the
rover, the link reconnects by itself and run_sequence() returns normally with the rover's
own timing. Run it once per route:

    source ros2env.sh
    python3 loop/tests/test_link_dropout_real.py                    # LAN first, then Tailscale
    ROVER_HOST=rover python3 loop/tests/test_link_dropout_real.py   # Tailscale only (= campus)
"""
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rover_link import ROVER, RoverLink, _kill  # noqa: E402

fails = []


def check(name, cond, info=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}  {info}")
    if not cond:
        fails.append(name)


link = RoverLink(host=os.environ.get("ROVER_HOST", ROVER), log=lambda m: print("   ", m))
link.open()
print(f"connected via {link.host}")
steps = [{"linear": 0, "angular": 0, "duration": 3.0}, {"linear": 0, "angular": 0, "duration": 3.0}]


def cut():
    time.sleep(1.5)
    print("    -- cutting the transport (simulated dropout) --")
    _kill(link.proc)


threading.Thread(target=cut, daemon=True).start()
w0 = time.time()
try:
    t_start = link.run_sequence(steps)
    ok = True
except Exception as exc:
    ok, t_start = False, None
    print("    run_sequence raised:", exc)
dt = time.time() - w0
check("trial survived a mid-trial dropout", ok)
check("link reconnected by itself", link.link_up and link.link_drops == 1,
      f"drops={link.link_drops} route={link.host}")
check("seq_start stamped by the rover (before the cut)", t_start is not None and t_start < w0 + 1.5,
      f"seq_start {t_start - w0:+.2f}s after send" if t_start else "")
check("returned once the trial had really ended", dt >= 6.0, f"{dt:.1f}s")
link.close(stop_bringup=False)
print("ALL PASS" if not fails else f"FAILED: {fails}")
sys.exit(1 if fails else 0)
