#!/usr/bin/env python3
"""selfcheck's lidar angle-convention check vs fake_rover: a correct lidar must pass, a
mirrored one must fail and suggest lidar_flip.   source ros2env.sh && python3 loop/tests/test_lidar_convention.py"""
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE)]


def run(flip, cfg_flip=False):
    import test_stations_sim as T
    from rover_link import RoverLink
    import homing
    import selfcheck
    env = dict(os.environ, FAKE_LIDAR_FLIP="1" if flip else "0")
    fake = subprocess.Popen([sys.executable, str(HERE / "fake_rover.py"), "0.5", "0.4", "1.5708"],
                            env=env, start_new_session=True)
    cam = rover = None
    try:
        time.sleep(2)
        cam = T.FakeCamera()
        cfg = homing.load_config()
        cfg.setdefault("stations", {})["lidar_flip"] = cfg_flip
        cfg["stations"]["lidar_offset"] = [0.0, 0.0, 0.0]
        cfg["fusion"].update(odom_source="rf2o", odom_scale=1.0)
        rover = RoverLink(host="local", fusion_cfg=cfg["fusion"], log=lambda *a: None)
        rover.open()
        return selfcheck.check_lidar_convention(rover, cam, cfg)
    finally:
        if rover:
            rover.close()
        if cam:
            cam.close()
        os.killpg(fake.pid, 15)


def main():
    os.environ.update(ROS_DOMAIN_ID="91", ROS_LOCALHOST_ONLY="1")
    ok1, d1, s1 = run(False)
    print(f"  {'PASS' if ok1 and not s1 else 'FAIL'}  correct lidar accepted: {d1} {s1}")
    ok2, d2, s2 = run(True)
    print(f"  {'PASS' if not ok2 and s2.get('lidar_flip') else 'FAIL'}  mirrored lidar caught: {d2} {s2}")
    ok3, d3, s3 = run(True, cfg_flip=True)
    print(f"  {'PASS' if ok3 and not s3 else 'FAIL'}  mirrored lidar + lidar_flip config accepted: {d3} {s3}")
    good = ok1 and not s1 and not ok2 and s2.get("lidar_flip") and ok3 and not s3
    print("ALL PASS" if good else "FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    raise SystemExit(main())
