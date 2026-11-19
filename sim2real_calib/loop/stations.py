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


# A station could not be taught / reached / accepted.
class StationError(RuntimeError):
    pass


# ───────────────────────────── storage ─────────────────────────────
# Files of one station: <name>.json (metadata) and <name>_scan.npz (lidar fingerprint).
def _paths(name):
    return ST_DIR / f"{name}.json", ST_DIR / f"{name}_scan.npz"


# Station dict (with its lidar scan under 'scan' if one was saved), or None.
def load_station(name):
    meta_path, scan_path = _paths(name)
    if not meta_path.exists():
        return None
    station = json.loads(meta_path.read_text())
    if scan_path.exists():
        scan_file = np.load(scan_path)
        station["scan"] = {"ranges": scan_file["ranges"], "angle_min": float(scan_file["angle_min"]),
                      "angle_inc": float(scan_file["angle_inc"])}
    return station


# Names of all taught stations (excluding the camera-guard / verify files).
def list_stations():
    return sorted(json_path.stem for json_path in ST_DIR.glob("*.json")
                  if not json_path.stem.startswith(("camera_", "verify_")))


# loop_config 'stations' settings merged over the defaults.
def scfg(cfg):
    settings = {"lidar_tol_m": 0.02, "lidar_tol_deg": 1.5, "camera_tol_m": 0.04,
         "camera_tol_deg": 4.0, "accept_m": 0.04, "accept_deg": 6.0, "lateral_ignore_m": 0.03,
         "consistency_m": 0.03, "consistency_deg": 2.5,
         "fine_iterations": 3, "scans_teach": 20, "scans_check": 8,
         "lidar_offset": [0.0, 0.0, 0.0], "lidar_flip": False, "require_lidar": False}
    settings.update(cfg.get("stations", {}))
    return settings


# Configure the rover agent's lidar obstacle stop (distance + lidar mounting).
def set_guard(rover, cfg):
    """Obstacle stop with the same lidar convention the station matching uses."""
    station_cfg = scfg(cfg)
    rover.guard(cfg.get("obstacle_guard_m", 0.0), station_cfg["lidar_offset"][2], station_cfg["lidar_flip"])


# ───────────────────────────── sensing ─────────────────────────────
# Wait until the rover has been still for quiet_s: odometry moved < 3 mm / 0.6 deg
# and the camera < 15 mm. False on timeout.
def wait_still(rover, cam, timeout=4.0, quiet_s=0.5):
    """Wait until odometry (gyro/odom) and the camera both show no motion."""
    deadline = time.time() + timeout
    reference, reference_time = None, time.time()
    while time.time() < deadline:
        odom_pose = rover.odometry_pose()
        camera_pose = cam.pose(0.5)
        current = (odom_pose[1] if odom_pose else (0, 0, 0)), (camera_pose[1:] if camera_pose else None)
        if reference is not None:
            (odom_x, odom_y, odom_yaw), camera_xy = current
            (ref_x, ref_y, ref_yaw), ref_camera_xy = reference
            # odometry is the precise stillness signal; the camera (sample noise ~mm,
            # occasional jumps) only vetoes gross motion odometry might miss
            moved = math.hypot(odom_x - ref_x, odom_y - ref_y) > 0.003 or abs(wrap(odom_yaw - ref_yaw)) > math.radians(0.6)
            if camera_xy is not None and ref_camera_xy is not None:
                moved |= math.hypot(camera_xy[0] - ref_camera_xy[0], camera_xy[1] - ref_camera_xy[1]) > 0.015
            if moved:
                reference, reference_time = current, time.time()
            elif time.time() - reference_time >= quiet_s:
                return True
        else:
            reference, reference_time = current, time.time()
        time.sleep(0.05)
    return False


# n lidar scans resampled onto the common angle grid and median-combined.
# Returns (ranges, angle_min, angle_inc, number of scans).
def capture_scan(rover, n):
    """Median of n scans on the fixed angular grid -> (ranges, angle_min, angle_inc, n)."""
    scans = rover.scans(n)
    resampled = [icp2d.resample(raw_ranges, angle_min, angle_inc) for _, angle_min, angle_inc, raw_ranges in scans]
    ranges = icp2d.median_scan([scan_bins[0] for scan_bins in resampled])
    return ranges, resampled[0][1], resampled[0][2], len(scans)


# Match a fresh median scan against the station's fingerprint (icp2d.match):
# the rover's offset from the taught spot in the station's body frame.
def lidar_error(rover, station, c, n=None):
    """Rover displacement from the taught spot, in the station's body frame."""
    if "scan" not in station:
        return {"ok": False, "why": "station has no lidar scan"}
    ranges, angle_min, angle_inc, _ = capture_scan(rover, n or c["scans_check"])
    lidar_offset, lidar_flip = tuple(c["lidar_offset"]), bool(c["lidar_flip"])
    ref_points = icp2d.scan_to_points(station["scan"]["ranges"], station["scan"]["angle_min"],
                               station["scan"]["angle_inc"], lidar_offset, lidar_flip)
    cur_points = icp2d.scan_to_points(ranges, angle_min, angle_inc, lidar_offset, lidar_flip)
    return icp2d.match(ref_points, cur_points)


# ───────────────────────────── teach ─────────────────────────────
# Teach a station: drive to the start pose (unless here=True), wait until still, store
# the camera pose and a median lidar fingerprint; a second scan must self-match within
# 5 mm or the fingerprint is marked unusable.
def teach(rover, cam, tracker, name, cfg, log=print, goal=None, here=False):
    station_cfg = scfg(cfg)
    if not here:
        goal = goal or start_pose(cfg, name)
        go_to_pose(rover, tracker, goal, cfg, log=log)
    rover.stop()
    if not wait_still(rover, cam):
        raise StationError("rover would not settle")
    camera_pose = cam.settled_pose(2.0)
    if camera_pose is None:
        raise StationError(f"camera can't see the rover ({cam.status})")
    station = {"name": name, "taught": time.strftime("%Y-%m-%d %H:%M:%S"),
          "camera_pose": [camera_pose[1], camera_pose[2], camera_pose[3]],
          "camera_pose_deg": round(math.degrees(camera_pose[3]), 2),
          "taught_from": "here" if here else "config start pose"}
    if not in_workspace(cfg, camera_pose[1], camera_pose[2]):
        raise StationError(f"taught pose ({camera_pose[1]:.2f},{camera_pose[2]:.2f}) is outside the workspace")
    ST_DIR.mkdir(exist_ok=True)
    meta_path, scan_path = _paths(name)
    if rover.have_scan:
        ranges, angle_min, angle_inc, scan_count = capture_scan(rover, station_cfg["scans_teach"])
        # self-consistency: a second median must match the first at ~0 with a good fit
        ranges_2, angle_min_2, angle_inc_2, _ = capture_scan(rover, station_cfg["scans_check"])
        lidar_offset, lidar_flip = tuple(station_cfg["lidar_offset"]), bool(station_cfg["lidar_flip"])
        self_match = icp2d.match(icp2d.scan_to_points(ranges, angle_min, angle_inc, lidar_offset, lidar_flip),
                          icp2d.scan_to_points(ranges_2, angle_min_2, angle_inc_2, lidar_offset, lidar_flip))
        point_count = int(np.isfinite(ranges).sum())
        station["lidar"] = {"n_scans": scan_count, "n_points": point_count, "self_match": {
            key: self_match.get(key) for key in ("ok", "dx", "dy", "dyaw", "rms", "inlier_frac", "cond")}}
        if not self_match.get("ok") or math.hypot(self_match["dx"], self_match["dy"]) > 0.005:
            log(f"[station] WARNING {name}: lidar fingerprint is not self-consistent "
                f"({self_match.get('why') or 'offset'}); lidar refinement will be unreliable here")
            station["lidar"]["usable"] = False
        else:
            station["lidar"]["usable"] = True
        np.savez(scan_path, ranges=ranges, angle_min=angle_min, angle_inc=angle_inc)
    else:
        log(f"[station] WARNING {name}: no lidar - camera-only station")
    meta_path.write_text(json.dumps(station, indent=2, default=float))
    log(f"[station] taught {name}: camera ({camera_pose[1]:.3f}, {camera_pose[2]:.3f}, {math.degrees(camera_pose[3]):.1f} deg)"
        + (f", lidar {station['lidar']['n_points']} pts, self-match "
           f"{station['lidar']['self_match']['rms']*1000:.1f} mm rms" if "lidar" in station else ""))
    return station


# ───────────────────────────── go to a station ─────────────────────────────
# Return to a station: camera homing to the taught pose, then (if a usable lidar
# fingerprint exists) up to fine_iterations lidar-measured odometry micro-moves, then
# the acceptance gate (stationary + lidar or camera error within tolerance). Returns the
# record dict; raises StationError when rejected.
def goto_station(rover, cam, tracker, name, cfg, log=print):
    """Coarse camera homing + lidar fine correction + acceptance gate -> record dict."""
    station_cfg = scfg(cfg)
    station = load_station(name)
    start_time = time.time()
    if station is None:
        log(f"[station] {name} not taught - camera-only start from loop_config "
            f"(run: loop/stations.py teach {name})")
        goal = start_pose(cfg, name)
    else:
        goal = tuple(station["camera_pose"])
    go_to_pose(rover, tracker, goal, cfg, log=log)
    record = {"station": name, "taught": bool(station), "method": "camera", "fine_moves": 0}
    use_lidar = (station is not None and "scan" in station and station.get("lidar", {}).get("usable", True)
                 and rover.have_scan)
    lidar_result = None
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
        best_score = None
        for fine_round in range(station_cfg["fine_iterations"] + 1):
            rover.stop()
            wait_still(rover, cam)
            lidar_result = lidar_error(rover, station, station_cfg)
            if not lidar_result.get("ok"):
                log(f"[station] {name}: lidar match unusable ({lidar_result.get('why')}) - "
                    f"keeping the camera result")
                break
            lidar_dist_m = math.hypot(lidar_result["dx"], lidar_result["dy"])
            lidar_yaw_deg = abs(math.degrees(lidar_result["dyaw"]))
            log(f"[station] {name}: lidar says {lidar_dist_m*100:.1f} cm (fwd {lidar_result['dx']*100:+.1f}, "
                f"left {lidar_result['dy']*100:+.1f}) / {math.degrees(lidar_result['dyaw']):+.1f} deg from the "
                f"taught spot (fit {lidar_result['rms']*1000:.0f} mm, {lidar_result['inlier_frac']:.0%} inliers)")
            if lidar_dist_m <= station_cfg["lidar_tol_m"] and lidar_yaw_deg <= station_cfg["lidar_tol_deg"]:
                record["method"] = "lidar"
                break
            if lidar_dist_m <= station_cfg["accept_m"] and lidar_yaw_deg <= station_cfg["accept_deg"]:
                record["method"] = "lidar (accept band)"
                log(f"[station] {name}: within the accept band ({station_cfg['accept_m']*100:.0f} cm / "
                    f"{station_cfg['accept_deg']:.0f} deg) - no further fine moves")
                break
            combined_error = lidar_dist_m / station_cfg["accept_m"] + lidar_yaw_deg / station_cfg["accept_deg"]
            if best_score is not None and combined_error > best_score + 0.25:
                log(f"[station] {name}: the last fine move made it worse - stopping the fine "
                    f"stage")
                break
            best_score = combined_error if best_score is None else min(best_score, combined_error)
            if fine_round == station_cfg["fine_iterations"]:
                break
            # undo the lidar-measured displacement with odometry-closed micro-moves (the
            # 0.7 s camera is too late for cm-scale moves); ignore small sideways offsets
            lateral_to_fix = lidar_result["dy"] if abs(lidar_result["dy"]) > station_cfg["lateral_ignore_m"] else 0.0
            try:
                move_summary = micromove.correct(rover, (lidar_result["dx"], lateral_to_fix, lidar_result["dyaw"]),
                                        lat_tol=station_cfg["lateral_ignore_m"])
                log(f"[station] {name}: fine move: {move_summary}")
            except micromove.MicroMoveError as exc:
                log(f"[station] {name}: fine move failed ({exc})")
            record["fine_moves"] += 1
    # ---- acceptance gate ----
    rover.stop()
    stationary = wait_still(rover, cam)
    camera_pose = cam.settled_pose(1.0)
    if camera_pose is None:
        raise StationError(f"camera lost the rover at the station ({cam.status})")
    camera_error = (math.hypot(camera_pose[1] - goal[0], camera_pose[2] - goal[1]), abs(math.degrees(wrap(camera_pose[3] - goal[2]))))
    record.update(stationary=stationary, camera_pose=list(camera_pose[1:]), camera_err_m=camera_error[0],
               camera_err_deg=camera_error[1], time_s=time.time() - start_time)
    if lidar_result is not None and lidar_result.get("ok"):
        record.update(lidar_err_m=math.hypot(lidar_result["dx"], lidar_result["dy"]),
                   lidar_err_deg=abs(math.degrees(lidar_result["dyaw"])),
                   lidar_fit_mm=lidar_result["rms"] * 1000, lidar_inliers=lidar_result["inlier_frac"])
        # camera's view of the displacement, in the station frame, vs the lidar's
        camera_displacement = compose(inverse(goal), tuple(camera_pose[1:]))
        disagree_m = math.hypot(camera_displacement[0] - lidar_result["dx"], camera_displacement[1] - lidar_result["dy"])
        disagree_deg = abs(math.degrees(wrap(camera_displacement[2] - lidar_result["dyaw"])))
        record.update(cam_vs_lidar_m=disagree_m, cam_vs_lidar_deg=disagree_deg)
        if disagree_m > station_cfg["consistency_m"] or disagree_deg > station_cfg["consistency_deg"]:
            log(f"[station] {name}: WARNING camera and lidar disagree by {disagree_m*100:.1f} cm / "
                f"{disagree_deg:.1f} deg - camera moved or grid calibration drifted? "
                f"(loop/camera_guard.py --check)")
        within_tolerance = (record["lidar_err_m"] <= max(station_cfg["accept_m"], station_cfg["lidar_tol_m"] * 1.25)
              and record["lidar_err_deg"] <= max(station_cfg["accept_deg"], station_cfg["lidar_tol_deg"] * 1.25))
    else:
        if station_cfg["require_lidar"] and station is not None:
            raise StationError(f"{name}: lidar verification required but unavailable")
        within_tolerance = camera_error[0] <= station_cfg["camera_tol_m"] and camera_error[1] <= station_cfg["camera_tol_deg"]
    record["accepted"] = bool(within_tolerance and stationary)
    message = (f"[station] {name}: {'ACCEPTED' if record['accepted'] else 'REJECTED'} via {record['method']} "
           f"in {record['time_s']:.0f}s - camera {camera_error[0]*100:.1f} cm/{camera_error[1]:.1f} deg"
           + (f", lidar {record['lidar_err_m']*100:.1f} cm/{record['lidar_err_deg']:.1f} deg"
              if "lidar_err_m" in record else ""))
    log(message)
    if not record["accepted"]:
        raise StationError(message)
    return record


# ───────────────────────────── CLI ─────────────────────────────
# Open camera + rover for the command line: camera-guard correction, lidar guard,
# fused tracker. Returns (camera, rover, tracker).
def _session(args, cfg):
    from camera_track import CameraTrack
    from rover_link import RoverLink
    from localizer import FusedTracker
    import camera_guard
    camera = CameraTrack(allow_approximate=args.allow_approximate)
    camera.latency_s = cfg.get("fusion", {}).get("cam_latency_s", 0.0)
    try:
        guard_result = camera_guard.CameraGuard().check(camera.cam.read()[0])
        if guard_result.get("moved") and guard_result.get("ok"):
            print(f"[camera] moved {guard_result['shift_px']} px since the reference - using the "
                  f"corrected homography for this session")
            camera.set_grid_homography(guard_result["H_m2img"])
        elif not guard_result.get("ok"):
            print(f"[camera] guard: {guard_result.get('why')}")
    except FileNotFoundError as exc:
        print(f"[camera] {exc}")
    fusion_cfg = cfg.get("fusion", {})
    rover = RoverLink(fusion_cfg=fusion_cfg)
    rover.open()
    set_guard(rover, cfg)
    tracker = FusedTracker(camera, rover, cam_latency_s=fusion_cfg.get("cam_latency_s", 0.2),
                           gain=fusion_cfg.get("gain", 0.3)) if fusion_cfg.get("enabled", True) else camera
    return camera, rover, tracker


# Repeatability test: from random poses 15-30 cm away, return to each station `reps`
# times and record each attempt.
def verify(rover, cam, tracker, names, cfg, reps, log=print, seed=0):
    """Repeatability: from a random nearby pose, return to each station `reps` times."""
    rng = random.Random(seed)
    records = []
    for name in names:
        station = load_station(name)
        if station is None:
            log(f"[verify] {name} not taught - skipped")
            continue
        station_x, station_y, station_yaw = station["camera_pose"]
        for rep_index in range(reps):
            for _ in range(50):              # random displaced pose inside the workspace
                offset_dist, offset_bearing = rng.uniform(0.15, 0.3), rng.uniform(-math.pi, math.pi)
                start_x, start_y = station_x + offset_dist * math.cos(offset_bearing), station_y + offset_dist * math.sin(offset_bearing)
                if in_workspace(cfg, start_x, start_y, margin=0.08):
                    break
            displaced_pose = (start_x, start_y, wrap(station_yaw + rng.uniform(-1.2, 1.2)))
            try:
                go_to_pose(rover, tracker, displaced_pose, cfg, log=lambda *a: None)
                record = goto_station(rover, cam, tracker, name, cfg, log)
            except (HomingError, StationError) as exc:
                record = {"station": name, "accepted": False, "error": str(exc)}
                log(f"[verify] {name} rep {rep_index+1}: FAILED {exc}")
            record["rep"] = rep_index + 1
            records.append(record)
    return records


# Per station: accepted count, camera position spread, yaw SD, lidar errors, time.
def report(rows, log=print):
    summary = {}
    for name in sorted({row["station"] for row in rows}):
        accepted_rows = [row for row in rows if row["station"] == name and row.get("accepted")]
        total_runs = sum(1 for row in rows if row["station"] == name)
        if not accepted_rows:
            log(f"[verify] {name}: 0/{total_runs} accepted")
            continue
        camera_poses = np.array([row["camera_pose"] for row in accepted_rows])
        spread_m = np.hypot(camera_poses[:, 0] - camera_poses[:, 0].mean(), camera_poses[:, 1] - camera_poses[:, 1].mean())
        yaw_sd = math.degrees(np.std(np.unwrap(camera_poses[:, 2])))
        lidar_errors = [row["lidar_err_m"] for row in accepted_rows if "lidar_err_m" in row]
        summary[name] = {"accepted": f"{len(accepted_rows)}/{total_runs}",
                     "camera_spread_mean_cm": round(float(spread_m.mean()) * 100, 2),
                     "camera_spread_max_cm": round(float(spread_m.max()) * 100, 2),
                     "camera_yaw_sd_deg": round(yaw_sd, 2),
                     "lidar_err_mean_cm": round(float(np.mean(lidar_errors)) * 100, 2) if lidar_errors else None,
                     "lidar_err_max_cm": round(float(np.max(lidar_errors)) * 100, 2) if lidar_errors else None,
                     "mean_time_s": round(float(np.mean([row["time_s"] for row in accepted_rows])), 1)}
        log(f"[verify] {name}: {summary[name]}")
    return summary


# Command line: teach | goto | check | verify | list (see the module docstring).
def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("cmd", choices=["teach", "goto", "check", "verify", "list"])
    parser.add_argument("names", nargs="*", help="station names (test stems); default all")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--here", action="store_true", help="teach: record the current pose")
    parser.add_argument("--goal", help="teach: x,y,yaw_deg instead of the config start pose")
    parser.add_argument("--reps", type=int, default=5)
    parser.add_argument("--config", default=str(CONFIG))
    parser.add_argument("--allow-approximate", action="store_true")
    parser.add_argument("--keep-bringup", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.cmd == "list":
        for station_name in list_stations():
            station = load_station(station_name)
            station_x, station_y, station_yaw = station["camera_pose"]
            print(f"{station_name:16s} ({station_x:.3f}, {station_y:.3f}, {math.degrees(station_yaw):6.1f} deg)  taught {station['taught']}"
                  f"  lidar {'yes' if 'scan' in station and station.get('lidar', {}).get('usable', True) else 'NO'}")
        return 0
    if args.cmd == "teach":
        names = args.names or ([pose_name for pose_name in config["start_poses"] if pose_name != "default"]
                               if args.all else [])
        if not names:
            sys.exit("teach: name the station(s) or pass --all")
    else:
        names = args.names or list_stations()
    camera, rover, tracker = _session(args, config)
    try:
        if args.cmd == "teach":
            goal = None
            if args.goal:
                goal_x, goal_y, goal_yaw_deg = (float(coordinate) for coordinate in args.goal.split(","))
                goal = (goal_x, goal_y, math.radians(goal_yaw_deg))
            for station_name in names:
                teach(rover, camera, tracker, station_name, config, goal=goal, here=args.here)
        elif args.cmd == "goto":
            for station_name in names:
                goto_station(rover, camera, tracker, station_name, config)
        elif args.cmd == "check":
            for station_name in names:
                station = load_station(station_name)
                if station is None:
                    continue
                lidar_result = lidar_error(rover, station, scfg(config)) if "scan" in station else {"ok": False}
                camera_pose = camera.settled_pose(0.8)
                camera_error_m = (math.hypot(camera_pose[1] - station["camera_pose"][0], camera_pose[2] - station["camera_pose"][1])
                      if camera_pose else float("nan"))
                print(f"{station_name:16s} camera {camera_error_m*100:6.1f} cm   lidar "
                      + (f"{math.hypot(lidar_result['dx'], lidar_result['dy'])*100:6.1f} cm "
                         f"{math.degrees(lidar_result['dyaw']):+5.1f} deg (fit {lidar_result['rms']*1000:.0f} mm)"
                         if lidar_result.get("ok") else f"n/a ({lidar_result.get('why', '')})"))
        elif args.cmd == "verify":
            records = verify(rover, camera, tracker, names, config, args.reps)
            report_path = ST_DIR / "reports" / f"verify_{time.strftime('%Y%m%d_%H%M%S')}.json"
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps({"rows": records, "summary": report(records)}, indent=2,
                                      default=float))
            print(f"wrote {report_path}")
    finally:
        try:
            rover.close(stop_bringup=not args.keep_bringup)
        finally:
            if hasattr(tracker, "close") and tracker is not camera:
                tracker.close()
            camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
