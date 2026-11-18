#!/usr/bin/env python3
"""Return-to-start for the REAL rover: closed-loop go-to-pose on the overhead camera.

This is what makes the real side of the loop unattended: after every trial the rover
drives itself back to the next test's start pose (x, y, yaw in grid metres), using the
camera sticker pose as feedback and the persistent rover link for commands.

Controller (20 Hz, yaw predicted ahead by latency_s to cancel camera lag):
  1. TO_PRE    - go to a pre-goal run_in_m behind the goal (rotate in place when the
                 bearing is off by > turn_in_place_deg, else drive with heading
                 correction; a close point behind is reached in REVERSE). In-place spins
                 "walk" the chassis a few cm on carpet, so they happen here, not at the goal.
  2. FACE      - rotate in place to the goal heading.
  3. APPROACH  - drive straight in along the heading, steering onto the goal line,
                 backing up if it overshoots.
  4. TRIM      - small final yaw correction, stop, median pose over settle_s, verify.
Deadband adaptation: if a command produces no motion for ~1.5 s (carpet stiction on
in-place turns), the minimum command is raised step by step up to the caps.
Safety: camera lost -> zero immediately; lost > lost_timeout_s -> give up. Goals
outside the workspace are refused. Overall timeout per attempt.

CLI (rover base driver is started automatically if needed):
  python3 loop/homing.py --goal 0.5,0.5,90          # drive to grid centre facing +y
  python3 loop/homing.py --test T1_straight         # drive to that test's start pose
  python3 loop/homing.py --where                    # just print the current pose
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

CONFIG = HERE / "loop_config.yaml"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_config(path=CONFIG):
    import yaml
    return yaml.safe_load(Path(path).read_text())


def start_pose(cfg, test):
    sp = cfg["start_poses"].get(test) or cfg["start_poses"]["default"]
    return (float(sp["x"]), float(sp["y"]), math.radians(float(sp["yaw"])))


def in_workspace(cfg, x, y, margin=0.0):
    (x0, x1), (y0, y1) = cfg["workspace"]["x"], cfg["workspace"]["y"]
    return x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin


class HomingError(RuntimeError):
    pass


def _sat(v, lo, hi):
    """Magnitude in [lo, hi], sign preserved (lo enforces the motor deadband)."""
    if v == 0:
        return 0.0
    return math.copysign(min(max(abs(v), lo), hi), v)


def go_to_pose(rover, cam, goal, cfg, log=print, on_tick=None):
    """Drive the real rover to goal=(x, y, yaw_rad). Returns a result dict; raises
    HomingError on failure (rover is always left stopped)."""
    h = dict(cfg["homing"], _cfg=cfg)
    gx, gy, gyaw = goal
    if not in_workspace(cfg, gx, gy):
        raise HomingError(f"goal ({gx:.2f},{gy:.2f}) is outside the workspace")
    pos_tol, yaw_tol = h["pos_tol_m"], math.radians(h["yaw_tol_deg"])
    last_err = None
    for attempt in range(1, int(h.get("attempts", 3)) + 1):
        try:
            res = _attempt(rover, cam, goal, h, pos_tol, yaw_tol, log, on_tick)
            if res["ok"]:
                log(f"[home] reached ({gx:.2f},{gy:.2f},{math.degrees(gyaw):.0f}deg): "
                    f"err {res['err_m']*100:.1f} cm / {res['err_deg']:.1f} deg "
                    f"in {res['time_s']:.1f}s (attempt {attempt})")
                return res
            last_err = f"timed out ({res['err_m']*100:.1f} cm, {res['err_deg']:.1f} deg off)"
        except HomingError as exc:
            last_err = str(exc)
            if "outside" in last_err:
                raise
        finally:
            rover.stop()
        log(f"[home] attempt {attempt} failed: {last_err}")
        time.sleep(1.0)
    raise HomingError(f"could not reach start pose: {last_err}")


def _attempt(rover, cam, goal, h, pos_tol, yaw_tol, log, on_tick):
    """One go-to-pose attempt as a small state machine:
         to_pre   go to PRE = goal - run_in * heading (spin walk there is harmless)
         face     rotate in place to the goal heading
         approach drive straight in along the heading, steering onto the goal,
                  backing up if overshot (no in-place spins near the goal)
         trim     final small in-place yaw correction, then settle + verify
    """
    gx, gy, gyaw = goal
    run_in = h.get("run_in_m", 0.15)
    px, py = gx - run_in * math.cos(gyaw), gy - run_in * math.sin(gyaw)
    while run_in > 0.04 and not in_workspace(h["_cfg"], px, py):   # pre-goal must be reachable
        run_in -= 0.02
        px, py = gx - run_in * math.cos(gyaw), gy - run_in * math.sin(gyaw)
    # prediction horizon: the raw camera lags ~0.25 s; a fused (odometry-propagated)
    # pose is nearly current, so only the actuation lag is left to predict over
    lat = h.get("fused_latency_s", 0.08) if getattr(cam, "fused", False) else h.get("latency_s", 0.25)
    dt = 0.05
    t0 = time.time()
    lost_since = None
    min_lin, min_ang = h["min_linear"], h["min_angular"]
    stall_ref, stall_t = None, time.time()
    last_print = 0.0
    hist = []                                    # recent (t, x, y, yaw) for prediction

    def frame(x, y, yaw):
        """along/lateral error of the GOAL in the goal-heading frame."""
        c, s = math.cos(gyaw), math.sin(gyaw)
        return (gx - x) * c + (gy - y) * s, -(gx - x) * s + (gy - y) * c

    def pick_phase(x, y, yaw):
        along, lateral = frame(x, y, yaw)
        # trim/approach cannot remove a sideways offset; only a fresh run-in can
        if abs(lateral) > 0.7 * pos_tol and abs(along) < 0.10:
            return "to_pre"
        on_line = abs(lateral) <= 2.0 * pos_tol and -0.05 <= along <= run_in + 0.10
        if on_line and abs(wrap(gyaw - yaw)) <= math.radians(25):
            return "approach" if abs(along) > 0.6 * pos_tol else "trim"
        if on_line:
            return "face"
        return "to_pre"

    phase = None
    settles = 0
    while True:
        now = time.time()
        if now - t0 > h["timeout_s"]:
            rover.stop()
            p = cam.settled_pose(0.3) or cam.pose(1.0)
            return _result(False, p, goal, now - t0)
        if not rover.alive:
            raise HomingError("rover link lost")
        p = cam.pose(max_age=0.35)
        if p is None:
            rover.velocity(0.0, 0.0)
            lost_since = lost_since or now
            if now - lost_since > h["lost_timeout_s"]:
                raise HomingError(f"camera lost the rover ({cam.status})")
            time.sleep(dt)
            continue
        lost_since = None
        ts, x, y, yaw = p
        if not hist or ts != hist[-1][0]:
            hist.append((ts, x, y, yaw))
            hist[:] = [e for e in hist if ts - e[0] <= 0.3]
        # camera+actuation latency compensation: act on where the rover WILL be (the
        # base also coasts a few cm after a stop command), not where the frame shows it
        if len(hist) >= 2 and hist[-1][0] - hist[0][0] > 0.05:
            span = hist[-1][0] - hist[0][0]
            x += (hist[-1][1] - hist[0][1]) / span * lat
            y += (hist[-1][2] - hist[0][2]) / span * lat
            yaw_pred = wrap(yaw + wrap(hist[-1][3] - hist[0][3]) / span * lat)
        else:
            yaw_pred = yaw
        if phase is None:
            phase = pick_phase(x, y, yaw)
        along, lateral = frame(x, y, yaw)
        lin = ang = 0.0
        if now - last_print > 1.0:
            last_print = now
            log(f"[home] {phase:8s} at ({x:.3f},{y:.3f},{math.degrees(yaw):6.1f}) "
                f"goal {math.hypot(gx-x, gy-y)*100:5.1f} cm (along {along*100:+.1f}, "
                f"lat {lateral*100:+.1f}), yaw err {math.degrees(wrap(gyaw-yaw)):6.1f}")

        def rotate_to(target, tol):
            err = wrap(target - yaw_pred)
            if abs(err) <= tol:
                return None
            return _sat(h["k_angular"] * err, min_ang, h["max_angular"])

        if phase == "to_pre":
            dx, dy = px - x, py - y
            dist = math.hypot(dx, dy)
            if dist <= 1.5 * pos_tol:
                phase = "face"
                continue
            err = wrap(math.atan2(dy, dx) - yaw_pred)
            direction = 1.0
            if abs(err) > math.radians(100) and dist < h["reverse_within_m"]:
                direction, err = -1.0, wrap(err + math.pi)
            if abs(err) > math.radians(h["turn_in_place_deg"]):
                ang = _sat(h["k_angular"] * err, min_ang, h["max_angular"])
            else:
                lin = direction * _sat(h["k_linear"] * dist, min_lin, h["max_linear"])
                lin *= max(math.cos(err), 0.3)
                ang = max(-h["max_angular"], min(h["max_angular"], h["k_angular"] * err))
        elif phase == "face":
            a = rotate_to(gyaw, math.radians(6))
            if a is None:
                phase = "approach"
                continue
            ang = a
        elif phase == "approach":
            if abs(lateral) > 3.0 * pos_tol or abs(wrap(gyaw - yaw)) > math.radians(35):
                phase = "to_pre"
                continue
            if abs(along) <= 0.6 * pos_tol:
                phase = "trim"
                continue
            direction = 1.0 if along > 0 else -1.0
            lin = direction * _sat(h["k_linear"] * abs(along), min_lin,
                                   min(h["max_linear"], 0.04 + 0.6 * abs(along)))
            # steer onto the goal line: heading toward a carrot 10 cm ahead on the line
            look = 0.08
            desired = gyaw + math.atan2(lateral, look) * direction
            ang = max(-0.6, min(0.6, h["k_angular"] * wrap(desired - yaw_pred)))
        elif phase == "trim":
            a = rotate_to(gyaw, yaw_tol)
            if a is None:
                rover.stop()
                time.sleep(0.15)
                sp = cam.settled_pose(h["settle_s"])
                if sp is None:
                    continue
                res = _result(True, sp, goal, time.time() - t0)
                if res["err_m"] <= pos_tol and math.radians(res["err_deg"]) <= yaw_tol * 1.5:
                    return res
                settles += 1
                if settles >= 4:
                    return dict(res, ok=False)
                phase = pick_phase(*sp[1:])
                log(f"[home] settled {res['err_m']*100:.1f} cm / {res['err_deg']:.1f} deg off "
                    f"-> {phase}")
                continue
            ang = a

        # deadband adaptation: commanded but not moving -> raise the minimum command
        if lin or ang:
            if stall_ref is None:
                stall_ref, stall_t = (x, y, yaw), now
            elif (math.hypot(x - stall_ref[0], y - stall_ref[1]) > 0.008
                  or abs(wrap(yaw - stall_ref[2])) > math.radians(1.5)):
                stall_ref, stall_t = (x, y, yaw), now
            elif now - stall_t > 1.5:
                rx = getattr(rover, "rx_age", 0.0)
                if rx > 1.0 or not getattr(rover, "alive", True):
                    # commands are not reaching the rover (its deadman stopped it): not a
                    # deadband or motor problem, so don't ramp the command or blame power
                    raise HomingError(f"link to the rover stalled (no reply for {rx:.1f}s)"
                                      " - WiFi drop? check ping rover-lan")
                if not getattr(rover, "base_alive", True):
                    # commands reach the agent but nothing drives the motors
                    raise HomingError("base driver stopped (its ssh session dropped?) - "
                                      "restart it")
                spin = bool(ang) and not lin
                floor = min_ang if spin else min_lin
                cap = h["max_angular"] if spin else h["max_linear"]
                if floor >= cap:
                    if now - stall_t > 4.0:
                        raise HomingError("rover does not move at full command - motors "
                                          "unpowered? (low battery / base driver)")
                else:
                    if spin:
                        min_ang = min(min_ang * 1.25, cap)
                    else:
                        min_lin = min(min_lin * 1.25, cap)
                    log(f"[home] stalled -> raising deadband floor (lin {min_lin:.3f}, "
                        f"ang {min_ang:.3f})")
                    stall_ref, stall_t = (x, y, yaw), now
        else:
            stall_ref = None

        rover.velocity(lin, ang)
        if on_tick:
            on_tick(p, (lin, ang), phase)
        time.sleep(dt)


def _result(ok, p, goal, elapsed):
    if p is None:
        return {"ok": False, "pose": None, "err_m": float("inf"), "err_deg": float("inf"),
                "time_s": elapsed}
    _, x, y, yaw = p
    return {"ok": ok, "pose": (x, y, yaw), "time_s": elapsed,
            "err_m": math.hypot(goal[0] - x, goal[1] - y),
            "err_deg": abs(math.degrees(wrap(goal[2] - yaw)))}


def main():
    import argparse
    from camera_track import CameraTrack
    from rover_link import RoverLink
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--goal", help="x,y,yaw_deg in grid metres/degrees")
    g.add_argument("--test", help="go to this test's start pose (loop_config.yaml)")
    g.add_argument("--where", action="store_true", help="print the camera pose and exit")
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--allow-approximate", action="store_true")
    ap.add_argument("--keep-bringup", action="store_true")
    args = ap.parse_args()
    cfg = load_config(args.config)
    cam = CameraTrack(allow_approximate=args.allow_approximate)
    cam.latency_s = cfg.get("fusion", {}).get("cam_latency_s", 0.0)
    try:
        p = cam.settled_pose(1.0)
        if p is None:
            sys.exit(f"rover not visible: {cam.status}")
        print(f"rover at x={p[1]:.3f} y={p[2]:.3f} yaw={math.degrees(p[3]):.1f}deg "
              f"(inside workspace: {in_workspace(cfg, p[1], p[2])})")
        if args.where:
            return 0
        if args.goal:
            gx, gy, gyaw = (float(v) for v in args.goal.split(","))
            goal = (gx, gy, math.radians(gyaw))
        else:
            goal = start_pose(cfg, args.test)
        link = RoverLink()
        try:
            link.open()
            go_to_pose(link, cam, goal, cfg)
        finally:
            link.close(stop_bringup=not args.keep_bringup)
    finally:
        cam.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
