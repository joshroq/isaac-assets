#!/usr/bin/env python3
"""RoverLink.reopen(): the agent process dies (the rover rebooted / lost power, 2026-11-05),
and the link is rebuilt so the rover obeys /cmd_vel again. Offline: fake_rover.py + a local
agent, on an isolated ROS domain.

    python3 loop/tests/test_reopen.py      -> ALL PASS
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def main():
    os.environ.update(ROS_DOMAIN_ID="89", ROS_LOCALHOST_ONLY="1")
    from rover_link import RoverLink
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}  {detail}")
        if not cond:
            fails.append(name)

    fake = subprocess.Popen([sys.executable, str(HERE / "fake_rover.py"), "0.5", "0.5", "0.0"],
                            start_new_session=True)
    rover = None
    try:
        time.sleep(2.0)
        rover = RoverLink(host="local", fusion_cfg={"odom_source": "wheel"}, log=print)
        rover.open()
        check("link up", rover.alive and rover.subs >= 1)
        rover.proc.kill()                                   # the agent dies
        rover.proc.wait(timeout=5)
        check("agent death detected", not rover.alive)
        rover.reopen(wait_s=30.0)
        check("reopened", rover.alive and rover.subs >= 1, f"subs={rover.subs}")
        x0 = rover.odom
        rover.velocity(0.1, 0.0)
        time.sleep(1.0)
        rover.stop()
        time.sleep(0.3)
        moved = x0 is not None and rover.odom is not None and abs(rover.odom[1] - x0[1]) > 0.02
        check("rover obeys /cmd_vel after reopen", moved,
              f"x {x0[1] if x0 else float('nan'):.3f} -> {rover.odom[1] if rover.odom else float('nan'):.3f}")
    finally:
        if rover is not None:
            try:
                rover.close(stop_bringup=False)
            except Exception:
                pass
        os.killpg(fake.pid, signal.SIGTERM)
    print("ALL PASS" if not fails else f"FAILED: {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
