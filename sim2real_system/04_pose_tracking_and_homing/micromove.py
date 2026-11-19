#!/usr/bin/env python3
"""Centimetre-scale corrective moves on the rover's OWN odometry (no camera in the loop).

Station fine correction needs moves of 1-5 cm / a few degrees. Re-running the camera
homing for that does not work on this rig: the camera is 0.7 s late and the homing
state machine backs up and re-approaches, landing about as imprecisely as it started
(measured 2026-10-20: 2.7 -> 4.5 cm over three tries). These moves close the loop on
the fused odometry instead - gyro yaw (no slip) + rf2o/wheel translation - which is
prompt and, over a few cm, accurate to millimetres. The lidar re-measures after each
correction, so any residual converges.

    correct(rover, (dx, dy, dyaw))   # rover is displaced by this from the goal, expressed
                                     # in the GOAL's body frame (as icp2d.match reports)
"""
from __future__ import annotations

import math
import time

from fusion import compose, inverse, wrap


# No fresh odometry to close the loop on.
class MicroMoveError(RuntimeError):
    pass


# Latest integrated odometry pose (x, y, yaw), refusing data older than max_age.
def _odom(rover, max_age=0.5):
    latest = rover.odometry_pose()
    if latest is None or time.time() - latest[0] > max_age:
        raise MicroMoveError("no fresh odometry")
    return latest[1]


# Proportional in-place turn on the odometry yaw (see docstring). The minimum turn
# command grows 20 % whenever the rover stalls for 0.8 s (stiction). Returns the angle
# actually turned (rad).
def turn_by(rover, angle, tol_deg=0.5, w_min=0.45, w_max=0.7, timeout=8.0, log=None,
            lead_s=0.25):
    """Rotate in place by `angle` rad (gyro-integrated yaw). Stops `lead_s` early at the
    current turn rate: the base keeps turning after a stop command (measured ~60 %
    overshoot on 4 deg turns without this)."""
    if abs(angle) < math.radians(tol_deg):
        return 0.0
    start_yaw = _odom(rover)[2]
    target_yaw = start_yaw + angle
    deadline = time.time() + timeout
    prev_yaw, last_progress_time = None, time.time()
    recent_yaw = []
    while time.time() < deadline:
        yaw = _odom(rover)[2]
        now = time.time()
        recent_yaw = [sample for sample in recent_yaw if now - sample[0] < 0.3] + [(now, yaw)]
        turn_rate = (wrap(recent_yaw[-1][1] - recent_yaw[0][1]) / (recent_yaw[-1][0] - recent_yaw[0][0])
                if len(recent_yaw) > 2 and recent_yaw[-1][0] > recent_yaw[0][0] else 0.0)
        yaw_error = wrap(target_yaw - yaw)
        if abs(yaw_error) <= math.radians(tol_deg) or (turn_rate * yaw_error > 0 and abs(yaw_error) <= abs(turn_rate) * lead_s):
            break
        turn_cmd = math.copysign(min(max(abs(yaw_error) * 1.5, w_min), w_max), yaw_error)
        rover.velocity(0.0, turn_cmd)
        if prev_yaw is not None and abs(wrap(yaw - prev_yaw)) > math.radians(0.3):
            last_progress_time = time.time()
        elif time.time() - last_progress_time > 0.8:                 # carpet stiction: push harder
            w_min = min(w_min * 1.2, w_max)
            last_progress_time = time.time()
        prev_yaw = yaw
        time.sleep(0.03)
    rover.stop()
    time.sleep(0.4)
    return wrap(_odom(rover)[2] - start_yaw)


# Proportional straight move by `dist` metres, steering to hold the start heading;
# stops early by the estimated coasting distance. Returns the distance actually driven.
def drive_by(rover, dist, tol=0.003, v_min=0.05, v_max=0.08, timeout=8.0, lead_s=0.25):
    """Drive straight by `dist` m along the current heading (holding the heading)."""
    if abs(dist) < tol:
        return 0.0
    start_x, start_y, start_heading = _odom(rover)
    deadline = time.time() + timeout
    last_along, last_time, speed_estimate = 0.0, time.time(), 0.0
    last_progress_time = time.time()
    while time.time() < deadline:
        x, y, yaw = _odom(rover)
        travelled = (x - start_x) * math.cos(start_heading) + (y - start_y) * math.sin(start_heading)
        now = time.time()
        if now - last_time > 0.05:
            speed_estimate = 0.6 * speed_estimate + 0.4 * (travelled - last_along) / (now - last_time)
            if abs(travelled - last_along) > 0.001:
                last_progress_time = now
            last_along, last_time = travelled, now
        remaining = dist - travelled
        if abs(remaining - speed_estimate * lead_s) <= tol or remaining * math.copysign(1, dist) < 0:
            break
        if now - last_progress_time > 0.8:
            v_min = min(v_min * 1.2, v_max)
            last_progress_time = now
        speed_cmd = math.copysign(min(max(abs(remaining) * 1.2, v_min), v_max), remaining)
        rover.velocity(speed_cmd, max(-0.3, min(0.3, 2.0 * wrap(start_heading - yaw))))
        time.sleep(0.03)
    rover.stop()
    time.sleep(0.4)
    x, y, _ = _odom(rover)
    return (x - start_x) * math.cos(start_heading) + (y - start_y) * math.sin(start_heading)


# Undo a small station offset. Nearly on the goal line (|lateral| <= lat_tol): turn
# towards the goal point, drive, then fix the heading. Otherwise back up `back_up` m,
# turn towards the goal point, drive to it, then fix the heading.
def correct(rover, disp, lat_tol=0.012, back_up=0.10, log=print):
    """Undo a displacement `disp` = (dx, dy, dyaw) of the rover from the goal, given in
    the goal's body frame. Returns a short description of what it did."""
    # goal pose expressed in the rover's CURRENT body frame
    goal_x, goal_y, goal_yaw = inverse(disp)
    actions = []
    if abs(goal_y) <= lat_tol:
        # nearly on the goal's line: straight move, then fix the heading
        # aim at the goal point (forward or backward, never a U-turn) if it is far enough
        aim_angle = (math.atan2(goal_y, goal_x) if goal_x > 0 else math.atan2(-goal_y, -goal_x)) if abs(goal_x) > 0.03 else 0.0
        turned = turn_by(rover, aim_angle)
        remaining = compose(inverse((0.0, 0.0, turned)), (goal_x, goal_y, goal_yaw))
        actions.append(f"drive {remaining[0]*100:+.1f} cm")
        drive_by(rover, remaining[0])
        remaining = (0.0, 0.0, remaining[2])
    else:
        # sideways offset: back up, angle toward the goal point, drive to it
        backed = drive_by(rover, -back_up)
        goal_rel = compose(inverse((backed, 0.0, 0.0)), (goal_x, goal_y, goal_yaw))      # goal from here
        turned = turn_by(rover, math.atan2(goal_rel[1], goal_rel[0]))
        goal_rel = compose(inverse((0.0, 0.0, turned)), goal_rel)
        driven = drive_by(rover, goal_rel[0])
        remaining = compose(inverse((driven, 0.0, 0.0)), goal_rel)
        actions.append(f"back {backed*100:.1f} cm, turn {math.degrees(turned):+.1f} deg, "
                     f"drive {driven*100:+.1f} cm")
    turned = turn_by(rover, remaining[2], tol_deg=0.4)
    actions.append(f"turn {math.degrees(turned):+.1f} deg")
    return "; ".join(actions)
