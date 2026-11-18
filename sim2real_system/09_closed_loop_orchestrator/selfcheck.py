#!/usr/bin/env python3
"""Preflight for the closed loop: every dependency of a reproducible trial, checked.

    python3 loop/selfcheck.py                 # read-only checks (rover does not move)
    python3 loop/selfcheck.py --motion        # + MOVES a little: camera latency (rocking
                                              #   in place; writes fusion.cam_latency_s) and
                                              #   lidar angle-convention check vs the camera

Checks: camera stream + tracking quality, camera hasn't moved (camera_guard), rover
link, base driver (/cmd_vel subscriber), battery, wheel odom, rf2o lidar odometry,
IMU gyro (+ bias), lidar scans, obstacle clearance, taught stations (+ lidar
self-consistency), Isaac launcher present. Exit code 0 only if nothing is FAIL.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SC = HERE.parent
REPO = SC.parent
sys.path.insert(0, str(HERE))

from homing import load_config, CONFIG, in_workspace   # noqa: E402

RESULTS = []


def res(level, name, detail=""):
    RESULTS.append((level, name, detail))
    mark = {"OK": "  ok ", "WARN": " WARN", "FAIL": " FAIL"}[level]
    print(f"[{mark}] {name:28s} {detail}")


def measure_latency(rover, cam, spin=0.8, seconds=4.0, period=1.0):
    """Rock in place (alternating spin direction every `period` s - a constant-rate spin
    makes a time lag indistinguishable from a yaw offset); the lag that best aligns
    the camera yaw track with the gyro-integrated yaw track is the camera latency."""
    gyro, camr = [], []
    t_end = time.time() + seconds
    last = None
    # re-use the integrator's gyro-driven yaw as the reference track
    t_start = time.time()
    while time.time() < t_end:
        sign = 1.0 if int((time.time() - t_start) / period) % 2 == 0 else -1.0
        rover.velocity(0.0, sign * spin)
        o = rover.odometry_pose()
        p = cam.pose(0.5)
        if o:
            gyro.append((o[0], o[1][2]))
        if p and p[0] != last:
            camr.append((p[0], p[3]))
            last = p[0]
        time.sleep(0.02)
    rover.stop()
    if len(gyro) < 20 or len(camr) < 20:
        return None
    g, c = np.array(gyro), np.array(camr)
    tg = np.arange(max(g[0, 0], c[0, 0]) + 0.6, min(g[-1, 0], c[-1, 0]) - 0.6, 0.01)
    gy = np.interp(tg, g[:, 0], np.unwrap(g[:, 1]))
    best = None
    for lag in np.arange(0.0, 1.5, 0.01):
        cy = np.interp(tg + lag, c[:, 0], np.unwrap(c[:, 1]))
        d = (cy - cy.mean()) - (gy - gy.mean())
        err = float(np.mean(d * d))
        if best is None or err < best[1]:
            best = (lag, err)
    if best[0] >= 1.45:
        return None                          # at the search edge: not trustworthy
    return round(float(best[0]), 3)


def check_lidar_convention(rover, cam, cfg, log=print):
    """Drive ~9 cm forward, then turn ~25 deg; compare lidar scan-matching with the camera.
    Detects a mirrored scan (rotation sign flips) or a mounting yaw (forward motion appears
    rotated) - either would make station corrections push the WRONG way. Returns
    (ok, detail, suggestion dict)."""
    import icp2d
    import stations as S
    from fusion import compose, inverse, wrap
    from homing import go_to_pose
    c = S.scfg(cfg)
    off, fl = tuple(c["lidar_offset"]), bool(c["lidar_flip"])

    def snap():
        rover.stop()
        S.wait_still(rover, cam)
        r, a, i, _ = S.capture_scan(rover, 6)
        return icp2d.scan_to_points(r, a, i, off, fl), cam.settled_pose(0.8)

    s0, p0 = snap()
    if p0 is None:
        return False, "camera can't see the rover", {}
    fwd = (p0[1] + 0.15 * math.cos(p0[3]), p0[2] + 0.15 * math.sin(p0[3]))
    if not in_workspace(cfg, *fwd):
        return False, "not enough room ahead (move the rover toward the grid centre)", {}
    t_end = time.time() + 1.0
    while time.time() < t_end:
        rover.velocity(0.10, 0.0)
        time.sleep(0.05)
    s1, p1 = snap()
    t_end = time.time() + 1.4
    while time.time() < t_end:
        rover.velocity(0.0, 0.8)
        time.sleep(0.05)
    s2, p2 = snap()
    try:
        go_to_pose(rover, cam, tuple(p0[1:]), cfg, log=lambda *a: None)
    except Exception as exc:
        log(f"         (could not drive back to the start: {exc})")
    if p1 is None or p2 is None:
        return False, "camera lost the rover during the check", {}
    m1, m2 = icp2d.match(s0, s1), icp2d.match(s1, s2)
    if not (m1.get("ok") and m2.get("ok")):
        return False, f"scan match failed ({m1.get('why') or m2.get('why')})", {}
    c1 = compose(inverse(tuple(p0[1:])), tuple(p1[1:]))      # camera, start body frame
    c2 = compose(inverse(tuple(p1[1:])), tuple(p2[1:]))
    dist_cam, dist_lid = math.hypot(c1[0], c1[1]), math.hypot(m1["dx"], m1["dy"])
    yaw_off_err = wrap(math.atan2(m1["dy"], m1["dx"]) - math.atan2(c1[1], c1[0]))
    mirrored = (m2["dyaw"] * c2[2]) < 0
    scale = dist_lid / max(dist_cam, 1e-3)
    detail = (f"forward: camera {dist_cam*100:.1f} cm, lidar {dist_lid*100:.1f} cm at "
              f"{math.degrees(yaw_off_err):+.0f} deg; turn: camera {math.degrees(c2[2]):+.1f}, "
              f"lidar {math.degrees(m2['dyaw']):+.1f} deg")
    ok = (abs(math.degrees(yaw_off_err)) < 15 and not mirrored and 0.75 < scale < 1.33
          and abs(abs(m2["dyaw"]) - abs(c2[2])) < max(0.25 * abs(c2[2]), math.radians(3)))
    suggestion = {}
    if mirrored:
        suggestion["lidar_flip"] = not fl
    if abs(math.degrees(yaw_off_err)) >= 15:
        # forward motion appears rotated by yaw_off_err in the lidar frame: rotate it back
        suggestion["lidar_offset"] = [off[0], off[1],
                                      round(wrap(off[2] - (-yaw_off_err if mirrored else yaw_off_err)), 3)]
    return ok, detail, suggestion


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--motion", "--latency", dest="motion", action="store_true",
                    help="MOVES the rover a little: measures camera latency (rocking in "
                         "place) and verifies the lidar angle convention (~9 cm + ~25 deg)")
    ap.add_argument("--allow-approximate", action="store_true")
    ap.add_argument("--keep-bringup", action="store_true")
    args = ap.parse_args()
    cfg = load_config(args.config)

    # ---- camera ----
    cam = rover = None
    try:
        from camera_track import CameraTrack, CameraError
        try:
            cam = CameraTrack(allow_approximate=args.allow_approximate)
            cam.latency_s = cfg.get("fusion", {}).get("cam_latency_s", 0.0)
            res("OK", "camera stream", "frames arriving on UDP 5000")
        except (CameraError, ValueError) as exc:
            res("FAIL", "camera stream", str(exc))
        if cam:
            time.sleep(1.5)
            p = cam.settled_pose(1.0)
            rate = cam.n_detect / max(cam.n_frames, 1)
            if p is None:
                res("FAIL", "rover tracking", cam.status)
            else:
                res("OK" if rate > 0.9 else "WARN", "rover tracking",
                    f"({p[1]:.3f}, {p[2]:.3f}, {math.degrees(p[3]):.1f} deg), detect rate "
                    f"{rate:.0%}, {cam.status}")
                if not in_workspace(cfg, p[1], p[2]):
                    res("WARN", "rover position", "outside the workspace box")
            try:
                import camera_guard
                g = camera_guard.CameraGuard().check(cam.cam.read()[0],
                                                     rover_xy=list(p[1:3]) if p else None)
                if not g.get("ok"):
                    res("FAIL", "camera unmoved", g.get("why", ""))
                elif g["moved"]:
                    res("WARN", "camera unmoved", f"shifted {g['shift_px']} px since reference "
                        "(sessions auto-correct; `camera_guard.py --check --write` to persist)")
                else:
                    res("OK", "camera unmoved", f"{g['shift_px']} px ({g['inliers']} inliers)")
            except FileNotFoundError:
                res("WARN", "camera unmoved", "no reference: camera_guard.py --save-reference")

        # ---- rover ----
        from rover_link import RoverLink, RoverLinkError
        try:
            rover = RoverLink(fusion_cfg=cfg.get("fusion", {}), log=lambda *a: None)
            rover.open()
            res("OK", "rover link + base driver", f"/cmd_vel subscribers {rover.subs}")
        except (RoverLinkError, Exception) as exc:
            res("FAIL", "rover link + base driver", str(exc))
            rover = None
        if rover:
            time.sleep(1.5)
            v, bmin = rover.voltage, float(cfg.get("battery_min_v", 0))
            res("OK" if v >= bmin + 0.3 else ("WARN" if v >= bmin else "FAIL"),
                "battery", f"{v:.2f} V (loop stops below {bmin} V)")
            res("OK" if rover.have_odom else "FAIL", "wheel odometry /odom")
            res("OK" if rover.have_rf2o else "WARN", "lidar odometry /odom_rf2o",
                "" if rover.have_rf2o else "fusion falls back to wheel odom (slips on spins)")
            res("OK" if rover.have_gyro else "WARN", "IMU gyro /imu/data_raw",
                f"bias {rover.gyro_bias:+.4f} rad/s" if rover.have_gyro else
                "fusion yaw falls back to odometry")
            if rover.have_scan:
                try:
                    import icp2d
                    import stations as S
                    r, a0, inc, _ = S.capture_scan(rover, 3)
                    ok = np.isfinite(r) & (r > icp2d.RANGE_MIN)
                    a = a0 + inc * np.arange(len(r)) + S.scfg(cfg)["lidar_offset"][2]
                    fwd = ok & (np.abs(((a + math.pi) % (2 * math.pi)) - math.pi) < math.radians(20))
                    near = float(np.min(r[fwd])) if fwd.any() else float("nan")
                    res("OK" if ok.sum() > 300 else "WARN", "lidar /scan",
                        f"{ok.sum()}/{len(r)} grid bins with room returns, nearest ahead "
                        f"{near:.2f} m")
                except Exception as exc:
                    res("FAIL", "lidar /scan", str(exc))
            else:
                res("WARN", "lidar /scan", "not publishing: stations fall back to camera only")

        # ---- stations ----
        import stations as S
        names = S.list_stations()
        want = [k for k in cfg["start_poses"] if k != "default"]
        if not names:
            res("WARN", "stations", "none taught - `loop/stations.py teach --all` "
                "(trials use the config poses, camera only)")
        else:
            bad = [n for n in names if not S.load_station(n).get("lidar", {}).get("usable", False)]
            res("OK" if not bad else "WARN", "stations",
                f"{len(names)} taught ({', '.join(names)})"
                + (f"; no usable lidar fingerprint: {bad}" if bad else ""))
            missing = [n for n in want if n not in names]
            if missing:
                res("WARN", "stations coverage", f"not taught: {missing}")

        # ---- sim ----
        isaac = REPO / "isaacpy_clean.sh"
        res("OK" if isaac.exists() else "FAIL", "Isaac launcher", str(isaac))

        # ---- motion checks (rover moves a little) ----
        if args.motion and cam and rover and rover.have_scan:
            import stations as S
            S.set_guard(rover, cfg)
            ok, detail, sug = check_lidar_convention(rover, cam, cfg)
            res("OK" if ok else "FAIL", "lidar angle convention", detail)
            if sug:
                print(f"         suggested loop_config.yaml stations: {sug}  (then re-teach "
                      "the stations)")
        if args.motion and cam and rover and rover.have_gyro:
            lat = measure_latency(rover, cam)
            if lat is None:
                res("WARN", "camera latency", "not enough samples")
            else:
                res("OK", "camera latency", f"{lat:.2f} s (set fusion.cam_latency_s)")
                import re
                p = Path(args.config)
                txt = p.read_text()
                txt = re.sub(r"(cam_latency_s:\s*)[0-9.]+", rf"\g<1>{lat:.2f}", txt)
                p.write_text(txt)
                print(f"         wrote fusion.cam_latency_s = {lat:.2f} to {p.name}")
    finally:
        if rover:
            rover.close(stop_bringup=not args.keep_bringup)
        if cam:
            cam.close()
    n_fail = sum(1 for r in RESULTS if r[0] == "FAIL")
    n_warn = sum(1 for r in RESULTS if r[0] == "WARN")
    print(f"\n{'READY' if not n_fail else 'NOT READY'}: {n_fail} fail, {n_warn} warn")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
