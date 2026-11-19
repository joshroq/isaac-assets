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


# Wrap an angle to [-pi, pi).
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# Read loop_config.yaml.
def load_config(path=CONFIG):
    import yaml
    return yaml.safe_load(Path(path).read_text())


# A test's start pose from loop_config start_poses (x m, y m, yaw rad).
def start_pose(cfg, test):
    pose_entry = cfg["start_poses"].get(test) or cfg["start_poses"]["default"]
    return (float(pose_entry["x"]), float(pose_entry["y"]), math.radians(float(pose_entry["yaw"])))


# True if (x, y) lies inside the loop_config workspace fence, shrunk by `margin`.
def in_workspace(cfg, x, y, margin=0.0):
    (fence_x_min, fence_x_max), (fence_y_min, fence_y_max) = cfg["workspace"]["x"], cfg["workspace"]["y"]
    return fence_x_min + margin <= x <= fence_x_max - margin and fence_y_min + margin <= y <= fence_y_max - margin


# Homing gave up (camera lost, link stalled, motors not moving, goal outside ...).
class HomingError(RuntimeError):
    pass


# Clamp a command's magnitude to [lo, hi], keeping its sign; zero stays zero.
# lo = deadband: smaller commands don't move the rover on carpet/foam.
def _sat(v, lo, hi):
    """Magnitude in [lo, hi], sign preserved (lo enforces the motor deadband)."""
    if v == 0:
        return 0.0
    return math.copysign(min(max(abs(v), lo), hi), v)


# Drive to goal (x, y, yaw) using the overhead camera: up to `attempts` tries of
# _attempt(); returns the result dict of the first success or raises HomingError.
def go_to_pose(rover, cam, goal, cfg, log=print, on_tick=None):
    """Drive the real rover to goal=(x, y, yaw_rad). Returns a result dict; raises
    HomingError on failure (rover is always left stopped)."""
    h = dict(cfg["homing"], _cfg=cfg)
    goal_x, goal_y, goal_yaw = goal
    if not in_workspace(cfg, goal_x, goal_y):
        raise HomingError(f"goal ({goal_x:.2f},{goal_y:.2f}) is outside the workspace")
    pos_tol, yaw_tol = h["pos_tol_m"], math.radians(h["yaw_tol_deg"])
    last_error = None
    for attempt in range(1, int(h.get("attempts", 3)) + 1):
        try:
            result = _attempt(rover, cam, goal, h, pos_tol, yaw_tol, log, on_tick)
            if result["ok"]:
                log(f"[home] reached ({goal_x:.2f},{goal_y:.2f},{math.degrees(goal_yaw):.0f}deg): "
                    f"err {result['err_m']*100:.1f} cm / {result['err_deg']:.1f} deg "
                    f"in {result['time_s']:.1f}s (attempt {attempt})")
                return result
            last_error = f"timed out ({result['err_m']*100:.1f} cm, {result['err_deg']:.1f} deg off)"
        except HomingError as exc:
            last_error = str(exc)
            if "outside" in last_error:
                raise
        finally:
            rover.stop()
        log(f"[home] attempt {attempt} failed: {last_error}")
        time.sleep(1.0)
    raise HomingError(f"could not reach start pose: {last_error}")


# One homing attempt - a state machine run every control_period_s (see docstring).
# h = the loop_config 'homing' section. Each tick: read the camera pose, predict it
# forward by the latency, compute the goal error along / across the goal heading,
# run the current phase's controller, and adapt the deadband if the rover isn't moving.
def _attempt(rover, cam, goal, h, pos_tol, yaw_tol, log, on_tick):
    """One go-to-pose attempt as a small state machine:
         to_pre   go to PRE = goal - run_in * heading (spin walk there is harmless)
         face     rotate in place to the goal heading
         approach drive straight in along the heading, steering onto the goal,
                  backing up if overshot (no in-place spins near the goal)
         trim     final small in-place yaw correction, then settle + verify
    """
    goal_x, goal_y, goal_yaw = goal
    run_in_m = h.get("run_in_m", 0.15)
    pre_goal_x, pre_goal_y = goal_x - run_in_m * math.cos(goal_yaw), goal_y - run_in_m * math.sin(goal_yaw)
    while run_in_m > 0.04 and not in_workspace(h["_cfg"], pre_goal_x, pre_goal_y):   # pre-goal must be reachable
        run_in_m -= 0.02
        pre_goal_x, pre_goal_y = goal_x - run_in_m * math.cos(goal_yaw), goal_y - run_in_m * math.sin(goal_yaw)
    # prediction horizon: the raw camera lags ~0.25 s; a fused (odometry-propagated)
    # pose is nearly current, so only the actuation lag is left to predict over
    predict_ahead_s = h.get("fused_latency_s", 0.08) if getattr(cam, "fused", False) else h.get("latency_s", 0.25)
    control_period_s = 0.05
    start_time = time.time()
    lost_since = None
    min_lin, min_ang = h["min_linear"], h["min_angular"]
    stall_ref, stall_t = None, time.time()
    last_print = 0.0
    recent_poses = []                                    # recent (t, x, y, yaw) for prediction

    # Goal error in the goal-heading frame: `along` = distance still to drive forward,
    # `lateral` = sideways offset from the goal line.
    def frame(x, y, yaw):
        """along/lateral error of the GOAL in the goal-heading frame."""
        cos_goal, sin_goal = math.cos(goal_yaw), math.sin(goal_yaw)
        return (goal_x - x) * cos_goal + (goal_y - y) * sin_goal, -(goal_x - x) * sin_goal + (goal_y - y) * cos_goal

    # Which phase to (re)start in from a pose: off the goal line -> to_pre; on the line
    # but facing wrong -> face; on the line and facing -> approach (or trim when there).
    def pick_phase(x, y, yaw):
        along, lateral = frame(x, y, yaw)
        # trim/approach cannot remove a sideways offset; only a fresh run-in can
        if abs(lateral) > 0.7 * pos_tol and abs(along) < 0.10:
            return "to_pre"
        on_line = abs(lateral) <= 2.0 * pos_tol and -0.05 <= along <= run_in_m + 0.10
        if on_line and abs(wrap(goal_yaw - yaw)) <= math.radians(25):
            return "approach" if abs(along) > 0.6 * pos_tol else "trim"
        if on_line:
            return "face"
        return "to_pre"

    phase = None
    settles = 0
    while True:
        now = time.time()
        if now - start_time > h["timeout_s"]:
            rover.stop()
            camera_pose = cam.settled_pose(0.3) or cam.pose(1.0)
            return _result(False, camera_pose, goal, now - start_time)
        if not rover.alive:
            raise HomingError("rover link lost")
        camera_pose = cam.pose(max_age=0.35)
        if camera_pose is None:
            rover.velocity(0.0, 0.0)
            lost_since = lost_since or now
            if now - lost_since > h["lost_timeout_s"]:
                raise HomingError(f"camera lost the rover ({cam.status})")
            time.sleep(control_period_s)
            continue
        lost_since = None
        frame_time, x, y, yaw = camera_pose
        if not recent_poses or frame_time != recent_poses[-1][0]:
            recent_poses.append((frame_time, x, y, yaw))
            recent_poses[:] = [entry for entry in recent_poses if frame_time - entry[0] <= 0.3]
        # camera+actuation latency compensation: act on where the rover WILL be (the
        # base also coasts a few cm after a stop command), not where the frame shows it
        if len(recent_poses) >= 2 and recent_poses[-1][0] - recent_poses[0][0] > 0.05:
            history_span_s = recent_poses[-1][0] - recent_poses[0][0]
            x += (recent_poses[-1][1] - recent_poses[0][1]) / history_span_s * predict_ahead_s
            y += (recent_poses[-1][2] - recent_poses[0][2]) / history_span_s * predict_ahead_s
            yaw_pred = wrap(yaw + wrap(recent_poses[-1][3] - recent_poses[0][3]) / history_span_s * predict_ahead_s)
        else:
            yaw_pred = yaw
        if phase is None:
            phase = pick_phase(x, y, yaw)
        along, lateral = frame(x, y, yaw)
        linear_cmd = angular_cmd = 0.0
        if now - last_print > 1.0:
            last_print = now
            log(f"[home] {phase:8s} at ({x:.3f},{y:.3f},{math.degrees(yaw):6.1f}) "
                f"goal {math.hypot(goal_x-x, goal_y-y)*100:5.1f} cm (along {along*100:+.1f}, "
                f"lat {lateral*100:+.1f}), yaw err {math.degrees(wrap(goal_yaw-yaw)):6.1f}")

        # Turn command towards `target` heading (None once within `tol`).
        def rotate_to(target, tol):
            heading_err = wrap(target - yaw_pred)
            if abs(heading_err) <= tol:
                return None
            return _sat(h["k_angular"] * heading_err, min_ang, h["max_angular"])

        if phase == "to_pre":
            to_pre_dx, to_pre_dy = pre_goal_x - x, pre_goal_y - y
            to_pre_dist = math.hypot(to_pre_dx, to_pre_dy)
            if to_pre_dist <= 1.5 * pos_tol:
                phase = "face"
                continue
            heading_err = wrap(math.atan2(to_pre_dy, to_pre_dx) - yaw_pred)
            direction = 1.0
            if abs(heading_err) > math.radians(100) and to_pre_dist < h["reverse_within_m"]:
                direction, heading_err = -1.0, wrap(heading_err + math.pi)
            if abs(heading_err) > math.radians(h["turn_in_place_deg"]):
                angular_cmd = _sat(h["k_angular"] * heading_err, min_ang, h["max_angular"])
            else:
                linear_cmd = direction * _sat(h["k_linear"] * to_pre_dist, min_lin, h["max_linear"])
                linear_cmd *= max(math.cos(heading_err), 0.3)
                angular_cmd = max(-h["max_angular"], min(h["max_angular"], h["k_angular"] * heading_err))
        elif phase == "face":
            rotate_cmd = rotate_to(goal_yaw, math.radians(6))
            if rotate_cmd is None:
                phase = "approach"
                continue
            angular_cmd = rotate_cmd
        elif phase == "approach":
            if abs(lateral) > 3.0 * pos_tol or abs(wrap(goal_yaw - yaw)) > math.radians(35):
                phase = "to_pre"
                continue
            if abs(along) <= 0.6 * pos_tol:
                phase = "trim"
                continue
            direction = 1.0 if along > 0 else -1.0
            linear_cmd = direction * _sat(h["k_linear"] * abs(along), min_lin,
                                   min(h["max_linear"], 0.04 + 0.6 * abs(along)))
            # steer onto the goal line: heading toward a carrot 10 cm ahead on the line
            lookahead_m = 0.08
            desired_heading = goal_yaw + math.atan2(lateral, lookahead_m) * direction
            angular_cmd = max(-0.6, min(0.6, h["k_angular"] * wrap(desired_heading - yaw_pred)))
        elif phase == "trim":
            rotate_cmd = rotate_to(goal_yaw, yaw_tol)
            if rotate_cmd is None:
                rover.stop()
                time.sleep(0.15)
                settled = cam.settled_pose(h["settle_s"])
                if settled is None:
                    continue
                result = _result(True, settled, goal, time.time() - start_time)
                if result["err_m"] <= pos_tol and math.radians(result["err_deg"]) <= yaw_tol * 1.5:
                    return result
                settles += 1
                if settles >= 4:
                    return dict(result, ok=False)
                phase = pick_phase(*settled[1:])
                log(f"[home] settled {result['err_m']*100:.1f} cm / {result['err_deg']:.1f} deg off "
                    f"-> {phase}")
                continue
            angular_cmd = rotate_cmd

        # deadband adaptation: commanded but not moving -> raise the minimum command
        if linear_cmd or angular_cmd:
            if stall_ref is None:
                stall_ref, stall_t = (x, y, yaw), now
            elif (math.hypot(x - stall_ref[0], y - stall_ref[1]) > 0.008
                  or abs(wrap(yaw - stall_ref[2])) > math.radians(1.5)):
                stall_ref, stall_t = (x, y, yaw), now
            elif now - stall_t > 1.5:
                reply_age_s = getattr(rover, "rx_age", 0.0)
                if reply_age_s > 1.0 or not getattr(rover, "alive", True):
                    # commands are not reaching the rover (its deadman stopped it): not a
                    # deadband or motor problem, so don't ramp the command or blame power
                    raise HomingError(f"link to the rover stalled (no reply for {reply_age_s:.1f}s)"
                                      " - WiFi drop? check ping rover-lan")
                if not getattr(rover, "base_alive", True):
                    # commands reach the agent but nothing drives the motors
                    raise HomingError("base driver stopped (its ssh session dropped?) - "
                                      "restart it")
                is_spin = bool(angular_cmd) and not linear_cmd
                deadband_floor = min_ang if is_spin else min_lin
                command_cap = h["max_angular"] if is_spin else h["max_linear"]
                if deadband_floor >= command_cap:
                    if now - stall_t > 4.0:
                        raise HomingError("rover does not move at full command - motors "
                                          "unpowered? (low battery / base driver)")
                else:
                    if is_spin:
                        min_ang = min(min_ang * 1.25, command_cap)
                    else:
                        min_lin = min(min_lin * 1.25, command_cap)
                    log(f"[home] stalled -> raising deadband floor (lin {min_lin:.3f}, "
                        f"ang {min_ang:.3f})")
                    stall_ref, stall_t = (x, y, yaw), now
        else:
            stall_ref = None

        rover.velocity(linear_cmd, angular_cmd)
        if on_tick:
            on_tick(camera_pose, (linear_cmd, angular_cmd), phase)
        time.sleep(control_period_s)


# Homing result dict: ok, final pose, distance and heading error to the goal, time taken.
def _result(ok, p, goal, elapsed):
    if p is None:
        return {"ok": False, "pose": None, "err_m": float("inf"), "err_deg": float("inf"),
                "time_s": elapsed}
    _, x, y, yaw = p
    return {"ok": ok, "pose": (x, y, yaw), "time_s": elapsed,
            "err_m": math.hypot(goal[0] - x, goal[1] - y),
            "err_deg": abs(math.degrees(wrap(goal[2] - yaw)))}


# Command line: --where (print the camera pose) | --goal x,y,yaw_deg | --test NAME.
def main():
    import argparse
    from camera_track import CameraTrack
    from rover_link import RoverLink
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--goal", help="x,y,yaw_deg in grid metres/degrees")
    target_group.add_argument("--test", help="go to this test's start pose (loop_config.yaml)")
    target_group.add_argument("--where", action="store_true", help="print the camera pose and exit")
    parser.add_argument("--config", default=str(CONFIG))
    parser.add_argument("--allow-approximate", action="store_true")
    parser.add_argument("--keep-bringup", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    camera = CameraTrack(allow_approximate=args.allow_approximate)
    camera.latency_s = config.get("fusion", {}).get("cam_latency_s", 0.0)
    try:
        pose = camera.settled_pose(1.0)
        if pose is None:
            sys.exit(f"rover not visible: {camera.status}")
        print(f"rover at x={pose[1]:.3f} y={pose[2]:.3f} yaw={math.degrees(pose[3]):.1f}deg "
              f"(inside workspace: {in_workspace(config, pose[1], pose[2])})")
        if args.where:
            return 0
        if args.goal:
            goal_x, goal_y, goal_yaw_deg = (float(v) for v in args.goal.split(","))
            goal = (goal_x, goal_y, math.radians(goal_yaw_deg))
        else:
            goal = start_pose(config, args.test)
        rover_link = RoverLink()
        try:
            rover_link.open()
            go_to_pose(rover_link, camera, goal, config)
        finally:
            rover_link.close(stop_bringup=not args.keep_bringup)
    finally:
        camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
