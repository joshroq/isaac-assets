#!/usr/bin/env python3
"""Teach-and-repeat start STATIONS: put the real rover back on the same spot every trial.

A station is a start pose that has been TAUGHT once and stores two independent
fingerprints of "this spot":
  * camera pose  - (x, y, yaw) in grid metres from the overhead stickers (absolute,
                   but only as good as the camera calibration / camera not moving)
  * lidar scan   - median LD19 scan of the room from that spot (camera-independent;
                   the room's walls/furniture do not move)

Going to a station (goto_station):
  1. COARSE  camera(+odometry fused) homing to the taught camera pose        (~2-4 cm)
  2. FINE    scan-match the current lidar scan to the station's scan -> the rover's
             displacement from the taught spot in the station's own frame; undo it
             with a small relative move; repeat until within lidar tolerance
             (camera only measures the RELATIVE correction here, so camera calibration
             errors cannot shift the start spot)
  3. GATE    stationary; lidar error within tolerance (primary), camera error reported,
             camera-vs-lidar consistency checked (disagreement => camera moved / grid
             calibration drifted). Everything is recorded with the trial.
If the lidar is unavailable or the match is weak, it falls back to the camera pose and
says so (the trial record carries the flag).

    python3 loop/stations.py teach T1_straight            # home to config pose, record it
    python3 loop/stations.py teach T2_spin --here         # record wherever it sits now
    python3 loop/stations.py teach --all                  # every test in loop_config
    python3 loop/stations.py goto T4_arc                  # coarse + fine + gate
    python3 loop/stations.py check                        # where am I vs each station
    python3 loop/stations.py verify --reps 5              # repeatability test (drives off
                                                          # randomly, returns, measures)
    python3 loop/stations.py list
"""
from __future__ import annotations

import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import icp2d                                            # noqa: E402
import micromove                                        # noqa: E402
from fusion import compose, inverse, wrap               # noqa: E402
from homing import (load_config, start_pose, go_to_pose, in_workspace,  # noqa: E402
                    HomingError, CONFIG)

ST_DIR = HERE / "stations"


class StationError(RuntimeError):
    pass


# ───────────────────────────── storage ─────────────────────────────
def _paths(name):
    return ST_DIR / f"{name}.json", ST_DIR / f"{name}_scan.npz"


def load_station(name):
    meta_p, scan_p = _paths(name)
    if not meta_p.exists():
        return None
    st = json.loads(meta_p.read_text())
    if scan_p.exists():
        z = np.load(scan_p)
        st["scan"] = {"ranges": z["ranges"], "angle_min": float(z["angle_min"]),
                      "angle_inc": float(z["angle_inc"])}
    return st


def list_stations():
    return sorted(p.stem for p in ST_DIR.glob("*.json")
                  if not p.stem.startswith(("camera_", "verify_")))


def scfg(cfg):
    d = {"lidar_tol_m": 0.02, "lidar_tol_deg": 1.5, "camera_tol_m": 0.04,
         "camera_tol_deg": 4.0, "accept_m": 0.04, "accept_deg": 6.0, "lateral_ignore_m": 0.03,
         "consistency_m": 0.03, "consistency_deg": 2.5,
         "fine_iterations": 3, "scans_teach": 20, "scans_check": 8,
         "lidar_offset": [0.0, 0.0, 0.0], "lidar_flip": False, "require_lidar": False}
    d.update(cfg.get("stations", {}))
    return d


def set_guard(rover, cfg):
    """Obstacle stop with the same lidar convention the station matching uses."""
    c = scfg(cfg)
    rover.guard(cfg.get("obstacle_guard_m", 0.0), c["lidar_offset"][2], c["lidar_flip"])


# ───────────────────────────── sensing ─────────────────────────────
def wait_still(rover, cam, timeout=4.0, quiet_s=0.5):
    """Wait until odometry (gyro/odom) and the camera both show no motion."""
    t_end = time.time() + timeout
    ref, ref_t = None, time.time()
    while time.time() < t_end:
        o = rover.odometry_pose()
        p = cam.pose(0.5)
        cur = (o[1] if o else (0, 0, 0)), (p[1:] if p else None)
        if ref is not None:
            (ox, oy, oa), cp = cur
            (rx, ry, ra), rp = ref
            # odometry is the precise stillness signal; the camera (sample noise ~mm,
            # occasional jumps) only vetoes gross motion odometry might miss
            moved = math.hypot(ox - rx, oy - ry) > 0.003 or abs(wrap(oa - ra)) > math.radians(0.6)
            if cp is not None and rp is not None:
                moved |= math.hypot(cp[0] - rp[0], cp[1] - rp[1]) > 0.015
            if moved:
                ref, ref_t = cur, time.time()
            elif time.time() - ref_t >= quiet_s:
                return True
        else:
            ref, ref_t = cur, time.time()
        time.sleep(0.05)
    return False


def capture_scan(rover, n):
    """Median of n scans on the fixed angular grid -> (ranges, angle_min, angle_inc, n)."""
    scans = rover.scans(n)
    grid = [icp2d.resample(r, a, i) for _, a, i, r in scans]
    ranges = icp2d.median_scan([g[0] for g in grid])
    return ranges, grid[0][1], grid[0][2], len(scans)


def lidar_error(rover, station, c, n=None):
    """Rover displacement from the taught spot, in the station's body frame."""
    if "scan" not in station:
        return {"ok": False, "why": "station has no lidar scan"}
    ranges, amin, ainc, _ = capture_scan(rover, n or c["scans_check"])
    off, fl = tuple(c["lidar_offset"]), bool(c["lidar_flip"])
    ref = icp2d.scan_to_points(station["scan"]["ranges"], station["scan"]["angle_min"],
                               station["scan"]["angle_inc"], off, fl)
    cur = icp2d.scan_to_points(ranges, amin, ainc, off, fl)
    return icp2d.match(ref, cur)


# ───────────────────────────── teach ─────────────────────────────
def teach(rover, cam, tracker, name, cfg, log=print, goal=None, here=False):
    c = scfg(cfg)
    if not here:
        goal = goal or start_pose(cfg, name)
        go_to_pose(rover, tracker, goal, cfg, log=log)
    rover.stop()
    if not wait_still(rover, cam):
        raise StationError("rover would not settle")
    p = cam.settled_pose(2.0)
    if p is None:
        raise StationError(f"camera can't see the rover ({cam.status})")
    st = {"name": name, "taught": time.strftime("%Y-%m-%d %H:%M:%S"),
          "camera_pose": [p[1], p[2], p[3]],
          "camera_pose_deg": round(math.degrees(p[3]), 2),
          "taught_from": "here" if here else "config start pose"}
    if not in_workspace(cfg, p[1], p[2]):
        raise StationError(f"taught pose ({p[1]:.2f},{p[2]:.2f}) is outside the workspace")
    ST_DIR.mkdir(exist_ok=True)
    meta_p, scan_p = _paths(name)
    if rover.have_scan:
        ranges, amin, ainc, n = capture_scan(rover, c["scans_teach"])
        # self-consistency: a second median must match the first at ~0 with a good fit
        r2, a2, i2, _ = capture_scan(rover, c["scans_check"])
        off, fl = tuple(c["lidar_offset"]), bool(c["lidar_flip"])
        chk = icp2d.match(icp2d.scan_to_points(ranges, amin, ainc, off, fl),
                          icp2d.scan_to_points(r2, a2, i2, off, fl))
        n_pts = int(np.isfinite(ranges).sum())
        st["lidar"] = {"n_scans": n, "n_points": n_pts, "self_match": {
            k: chk.get(k) for k in ("ok", "dx", "dy", "dyaw", "rms", "inlier_frac", "cond")}}
        if not chk.get("ok") or math.hypot(chk["dx"], chk["dy"]) > 0.005:
            log(f"[station] WARNING {name}: lidar fingerprint is not self-consistent "
                f"({chk.get('why') or 'offset'}); lidar refinement will be unreliable here")
            st["lidar"]["usable"] = False
        else:
            st["lidar"]["usable"] = True
        np.savez(scan_p, ranges=ranges, angle_min=amin, angle_inc=ainc)
    else:
        log(f"[station] WARNING {name}: no lidar - camera-only station")
    meta_p.write_text(json.dumps(st, indent=2, default=float))
    log(f"[station] taught {name}: camera ({p[1]:.3f}, {p[2]:.3f}, {math.degrees(p[3]):.1f} deg)"
        + (f", lidar {st['lidar']['n_points']} pts, self-match "
           f"{st['lidar']['self_match']['rms']*1000:.1f} mm rms" if "lidar" in st else ""))
    return st


# ───────────────────────────── go to a station ─────────────────────────────
def goto_station(rover, cam, tracker, name, cfg, log=print):
    """Coarse camera homing + lidar fine correction + acceptance gate -> record dict."""
    c = scfg(cfg)
    st = load_station(name)
    t0 = time.time()
    if st is None:
        log(f"[station] {name} not taught - camera-only start from loop_config "
            f"(run: loop/stations.py teach {name})")
        goal = start_pose(cfg, name)
    else:
        goal = tuple(st["camera_pose"])
    go_to_pose(rover, tracker, goal, cfg, log=log)
    rec = {"station": name, "taught": bool(st), "method": "camera", "fine_moves": 0}
    use_lidar = (st is not None and "scan" in st and st.get("lidar", {}).get("usable", True)
                 and rover.have_scan)
    lid = None
    if use_lidar:
        # Fine stage. Carpet makes small corrective turns land 4-20 deg off (stick-slip), so
        # chasing a tight tolerance can walk the rover AWAY from a good spot (seen on T2:
        # 2.6 cm/4 deg after homing -> 20 deg after four fine moves -> rejected). Rules:
        #   * within lidar_tol            -> done
        #   * within accept band          -> done (good enough; another move is coarser than
        #                                    the remaining error)
        #   * sideways offsets <= lateral_ignore_m are left alone (the back-up manoeuvre that
        #     corrects them costs far more heading error than the offset is worth)
        #   * a move that makes the error worse ends the stage (no thrashing)
        best = None
        for k in range(c["fine_iterations"] + 1):
            rover.stop()
            wait_still(rover, cam)
            lid = lidar_error(rover, st, c)
            if not lid.get("ok"):
                log(f"[station] {name}: lidar match unusable ({lid.get('why')}) - "
                    f"keeping the camera result")
                break
            e = math.hypot(lid["dx"], lid["dy"])
            de = abs(math.degrees(lid["dyaw"]))
            log(f"[station] {name}: lidar says {e*100:.1f} cm (fwd {lid['dx']*100:+.1f}, "
                f"left {lid['dy']*100:+.1f}) / {math.degrees(lid['dyaw']):+.1f} deg from the "
                f"taught spot (fit {lid['rms']*1000:.0f} mm, {lid['inlier_frac']:.0%} inliers)")
            if e <= c["lidar_tol_m"] and de <= c["lidar_tol_deg"]:
                rec["method"] = "lidar"
                break
            if e <= c["accept_m"] and de <= c["accept_deg"]:
                rec["method"] = "lidar (accept band)"
                log(f"[station] {name}: within the accept band ({c['accept_m']*100:.0f} cm / "
                    f"{c['accept_deg']:.0f} deg) - no further fine moves")
                break
            score = e / c["accept_m"] + de / c["accept_deg"]
            if best is not None and score > best + 0.25:
                log(f"[station] {name}: the last fine move made it worse - stopping the fine "
                    f"stage")
                break
            best = score if best is None else min(best, score)
            if k == c["fine_iterations"]:
                break
            # undo the lidar-measured displacement with odometry-closed micro-moves (the
            # 0.7 s camera is too late for cm-scale moves); ignore small sideways offsets
            dy = lid["dy"] if abs(lid["dy"]) > c["lateral_ignore_m"] else 0.0
            try:
                did = micromove.correct(rover, (lid["dx"], dy, lid["dyaw"]),
                                        lat_tol=c["lateral_ignore_m"])
                log(f"[station] {name}: fine move: {did}")
            except micromove.MicroMoveError as exc:
                log(f"[station] {name}: fine move failed ({exc})")
            rec["fine_moves"] += 1
    # ---- acceptance gate ----
    rover.stop()
    still = wait_still(rover, cam)
    p = cam.settled_pose(1.0)
    if p is None:
        raise StationError(f"camera lost the rover at the station ({cam.status})")
    cam_err = (math.hypot(p[1] - goal[0], p[2] - goal[1]), abs(math.degrees(wrap(p[3] - goal[2]))))
    rec.update(stationary=still, camera_pose=list(p[1:]), camera_err_m=cam_err[0],
               camera_err_deg=cam_err[1], time_s=time.time() - t0)
    if lid is not None and lid.get("ok"):
        rec.update(lidar_err_m=math.hypot(lid["dx"], lid["dy"]),
                   lidar_err_deg=abs(math.degrees(lid["dyaw"])),
                   lidar_fit_mm=lid["rms"] * 1000, lidar_inliers=lid["inlier_frac"])
        # camera's view of the displacement, in the station frame, vs the lidar's
        cam_disp = compose(inverse(goal), tuple(p[1:]))
        dd = math.hypot(cam_disp[0] - lid["dx"], cam_disp[1] - lid["dy"])
        da = abs(math.degrees(wrap(cam_disp[2] - lid["dyaw"])))
        rec.update(cam_vs_lidar_m=dd, cam_vs_lidar_deg=da)
        if dd > c["consistency_m"] or da > c["consistency_deg"]:
            log(f"[station] {name}: WARNING camera and lidar disagree by {dd*100:.1f} cm / "
                f"{da:.1f} deg - camera moved or grid calibration drifted? "
                f"(loop/camera_guard.py --check)")
        ok = (rec["lidar_err_m"] <= max(c["accept_m"], c["lidar_tol_m"] * 1.25)
              and rec["lidar_err_deg"] <= max(c["accept_deg"], c["lidar_tol_deg"] * 1.25))
    else:
        if c["require_lidar"] and st is not None:
            raise StationError(f"{name}: lidar verification required but unavailable")
        ok = cam_err[0] <= c["camera_tol_m"] and cam_err[1] <= c["camera_tol_deg"]
    rec["accepted"] = bool(ok and still)
    msg = (f"[station] {name}: {'ACCEPTED' if rec['accepted'] else 'REJECTED'} via {rec['method']} "
           f"in {rec['time_s']:.0f}s - camera {cam_err[0]*100:.1f} cm/{cam_err[1]:.1f} deg"
           + (f", lidar {rec['lidar_err_m']*100:.1f} cm/{rec['lidar_err_deg']:.1f} deg"
              if "lidar_err_m" in rec else ""))
    log(msg)
    if not rec["accepted"]:
        raise StationError(msg)
    return rec


# ───────────────────────────── CLI ─────────────────────────────
def _session(args, cfg):
    from camera_track import CameraTrack
    from rover_link import RoverLink
    from localizer import FusedTracker
    import camera_guard
    cam = CameraTrack(allow_approximate=args.allow_approximate)
    cam.latency_s = cfg.get("fusion", {}).get("cam_latency_s", 0.0)
    try:
        g = camera_guard.CameraGuard().check(cam.cam.read()[0])
        if g.get("moved") and g.get("ok"):
            print(f"[camera] moved {g['shift_px']} px since the reference - using the "
                  f"corrected homography for this session")
            cam.set_grid_homography(g["H_m2img"])
        elif not g.get("ok"):
            print(f"[camera] guard: {g.get('why')}")
    except FileNotFoundError as exc:
        print(f"[camera] {exc}")
    fz = cfg.get("fusion", {})
    rover = RoverLink(fusion_cfg=fz)
    rover.open()
    set_guard(rover, cfg)
    tracker = FusedTracker(cam, rover, cam_latency_s=fz.get("cam_latency_s", 0.2),
                           gain=fz.get("gain", 0.3)) if fz.get("enabled", True) else cam
    return cam, rover, tracker


def verify(rover, cam, tracker, names, cfg, reps, log=print, seed=0):
    """Repeatability: from a random nearby pose, return to each station `reps` times."""
    rnd = random.Random(seed)
    rows = []
    for name in names:
        st = load_station(name)
        if st is None:
            log(f"[verify] {name} not taught - skipped")
            continue
        gx, gy, ga = st["camera_pose"]
        for r in range(reps):
            for _ in range(50):              # random displaced pose inside the workspace
                d, b = rnd.uniform(0.15, 0.3), rnd.uniform(-math.pi, math.pi)
                ox, oy = gx + d * math.cos(b), gy + d * math.sin(b)
                if in_workspace(cfg, ox, oy, margin=0.08):
                    break
            off = (ox, oy, wrap(ga + rnd.uniform(-1.2, 1.2)))
            try:
                go_to_pose(rover, tracker, off, cfg, log=lambda *a: None)
                rec = goto_station(rover, cam, tracker, name, cfg, log)
            except (HomingError, StationError) as exc:
                rec = {"station": name, "accepted": False, "error": str(exc)}
                log(f"[verify] {name} rep {r+1}: FAILED {exc}")
            rec["rep"] = r + 1
            rows.append(rec)
    return rows


def report(rows, log=print):
    out = {}
    for name in sorted({r["station"] for r in rows}):
        rs = [r for r in rows if r["station"] == name and r.get("accepted")]
        n_all = sum(1 for r in rows if r["station"] == name)
        if not rs:
            log(f"[verify] {name}: 0/{n_all} accepted")
            continue
        cp = np.array([r["camera_pose"] for r in rs])
        spread = np.hypot(cp[:, 0] - cp[:, 0].mean(), cp[:, 1] - cp[:, 1].mean())
        yaw_sd = math.degrees(np.std(np.unwrap(cp[:, 2])))
        lid = [r["lidar_err_m"] for r in rs if "lidar_err_m" in r]
        out[name] = {"accepted": f"{len(rs)}/{n_all}",
                     "camera_spread_mean_cm": round(float(spread.mean()) * 100, 2),
                     "camera_spread_max_cm": round(float(spread.max()) * 100, 2),
                     "camera_yaw_sd_deg": round(yaw_sd, 2),
                     "lidar_err_mean_cm": round(float(np.mean(lid)) * 100, 2) if lid else None,
                     "lidar_err_max_cm": round(float(np.max(lid)) * 100, 2) if lid else None,
                     "mean_time_s": round(float(np.mean([r["time_s"] for r in rs])), 1)}
        log(f"[verify] {name}: {out[name]}")
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["teach", "goto", "check", "verify", "list"])
    ap.add_argument("names", nargs="*", help="station names (test stems); default all")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--here", action="store_true", help="teach: record the current pose")
    ap.add_argument("--goal", help="teach: x,y,yaw_deg instead of the config start pose")
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--allow-approximate", action="store_true")
    ap.add_argument("--keep-bringup", action="store_true")
    args = ap.parse_args()
    cfg = load_config(args.config)
    if args.cmd == "list":
        for n in list_stations():
            st = load_station(n)
            x, y, a = st["camera_pose"]
            print(f"{n:16s} ({x:.3f}, {y:.3f}, {math.degrees(a):6.1f} deg)  taught {st['taught']}"
                  f"  lidar {'yes' if 'scan' in st and st.get('lidar', {}).get('usable', True) else 'NO'}")
        return 0
    if args.cmd == "teach":
        names = args.names or ([k for k in cfg["start_poses"] if k != "default"]
                               if args.all else [])
        if not names:
            sys.exit("teach: name the station(s) or pass --all")
    else:
        names = args.names or list_stations()
    cam, rover, tracker = _session(args, cfg)
    try:
        if args.cmd == "teach":
            goal = None
            if args.goal:
                gx, gy, ga = (float(v) for v in args.goal.split(","))
                goal = (gx, gy, math.radians(ga))
            for n in names:
                teach(rover, cam, tracker, n, cfg, goal=goal, here=args.here)
        elif args.cmd == "goto":
            for n in names:
                goto_station(rover, cam, tracker, n, cfg)
        elif args.cmd == "check":
            for n in names:
                st = load_station(n)
                if st is None:
                    continue
                lid = lidar_error(rover, st, scfg(cfg)) if "scan" in st else {"ok": False}
                p = cam.settled_pose(0.8)
                ce = (math.hypot(p[1] - st["camera_pose"][0], p[2] - st["camera_pose"][1])
                      if p else float("nan"))
                print(f"{n:16s} camera {ce*100:6.1f} cm   lidar "
                      + (f"{math.hypot(lid['dx'], lid['dy'])*100:6.1f} cm "
                         f"{math.degrees(lid['dyaw']):+5.1f} deg (fit {lid['rms']*1000:.0f} mm)"
                         if lid.get("ok") else f"n/a ({lid.get('why', '')})"))
        elif args.cmd == "verify":
            rows = verify(rover, cam, tracker, names, cfg, args.reps)
            out = ST_DIR / "reports" / f"verify_{time.strftime('%Y%m%d_%H%M%S')}.json"
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps({"rows": rows, "summary": report(rows)}, indent=2,
                                      default=float))
            print(f"wrote {out}")
    finally:
        try:
            rover.close(stop_bringup=not args.keep_bringup)
        finally:
            if hasattr(tracker, "close") and tracker is not cam:
                tracker.close()
            cam.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
