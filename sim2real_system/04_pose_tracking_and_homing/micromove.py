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


class MicroMoveError(RuntimeError):
    pass


def _odom(rover, max_age=0.5):
    lt = rover.odometry_pose()
    if lt is None or time.time() - lt[0] > max_age:
        raise MicroMoveError("no fresh odometry")
    return lt[1]


def turn_by(rover, angle, tol_deg=0.5, w_min=0.45, w_max=0.7, timeout=8.0, log=None,
            lead_s=0.25):
    """Rotate in place by `angle` rad (gyro-integrated yaw). Stops `lead_s` early at the
    current turn rate: the base keeps turning after a stop command (measured ~60 %
    overshoot on 4 deg turns without this)."""
    if abs(angle) < math.radians(tol_deg):
        return 0.0
    yaw0 = _odom(rover)[2]
    target = yaw0 + angle
    t_end = time.time() + timeout
    prev, stall_t = None, time.time()
    hist = []
    while time.time() < t_end:
        yaw = _odom(rover)[2]
        now = time.time()
        hist = [h for h in hist if now - h[0] < 0.3] + [(now, yaw)]
        rate = (wrap(hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0])
                if len(hist) > 2 and hist[-1][0] > hist[0][0] else 0.0)
        err = wrap(target - yaw)
        if abs(err) <= math.radians(tol_deg) or (rate * err > 0 and abs(err) <= abs(rate) * lead_s):
            break
        w = math.copysign(min(max(abs(err) * 1.5, w_min), w_max), err)
        rover.velocity(0.0, w)
        if prev is not None and abs(wrap(yaw - prev)) > math.radians(0.3):
            stall_t = time.time()
        elif time.time() - stall_t > 0.8:                 # carpet stiction: push harder
            w_min = min(w_min * 1.2, w_max)
            stall_t = time.time()
        prev = yaw
        time.sleep(0.03)
    rover.stop()
    time.sleep(0.4)
    return wrap(_odom(rover)[2] - yaw0)


def drive_by(rover, dist, tol=0.003, v_min=0.05, v_max=0.08, timeout=8.0, lead_s=0.25):
    """Drive straight by `dist` m along the current heading (holding the heading)."""
    if abs(dist) < tol:
        return 0.0
    x0, y0, h0 = _odom(rover)
    t_end = time.time() + timeout
    last, last_t, v_est = 0.0, time.time(), 0.0
    stall_t = time.time()
    while time.time() < t_end:
        x, y, yaw = _odom(rover)
        along = (x - x0) * math.cos(h0) + (y - y0) * math.sin(h0)
        now = time.time()
        if now - last_t > 0.05:
            v_est = 0.6 * v_est + 0.4 * (along - last) / (now - last_t)
            if abs(along - last) > 0.001:
                stall_t = now
            last, last_t = along, now
        err = dist - along
        if abs(err - v_est * lead_s) <= tol or err * math.copysign(1, dist) < 0:
            break
        if now - stall_t > 0.8:
            v_min = min(v_min * 1.2, v_max)
            stall_t = now
        v = math.copysign(min(max(abs(err) * 1.2, v_min), v_max), err)
        rover.velocity(v, max(-0.3, min(0.3, 2.0 * wrap(h0 - yaw))))
        time.sleep(0.03)
    rover.stop()
    time.sleep(0.4)
    x, y, _ = _odom(rover)
    return (x - x0) * math.cos(h0) + (y - y0) * math.sin(h0)


def correct(rover, disp, lat_tol=0.012, back_up=0.10, log=print):
    """Undo a displacement `disp` = (dx, dy, dyaw) of the rover from the goal, given in
    the goal's body frame. Returns a short description of what it did."""
    # goal pose expressed in the rover's CURRENT body frame
    gx, gy, gyaw = inverse(disp)
    steps = []
    if abs(gy) <= lat_tol:
        # nearly on the goal's line: straight move, then fix the heading
        # aim at the goal point (forward or backward, never a U-turn) if it is far enough
        aim = (math.atan2(gy, gx) if gx > 0 else math.atan2(-gy, -gx)) if abs(gx) > 0.03 else 0.0
        a = turn_by(rover, aim)
        rem = compose(inverse((0.0, 0.0, a)), (gx, gy, gyaw))
        steps.append(f"drive {rem[0]*100:+.1f} cm")
        drive_by(rover, rem[0])
        rem = (0.0, 0.0, rem[2])
    else:
        # sideways offset: back up, angle toward the goal point, drive to it
        b = drive_by(rover, -back_up)
        rel = compose(inverse((b, 0.0, 0.0)), (gx, gy, gyaw))      # goal from here
        a = turn_by(rover, math.atan2(rel[1], rel[0]))
        rel = compose(inverse((0.0, 0.0, a)), rel)
        d = drive_by(rover, rel[0])
        rem = compose(inverse((d, 0.0, 0.0)), rel)
        steps.append(f"back {b*100:.1f} cm, turn {math.degrees(a):+.1f} deg, "
                     f"drive {d*100:+.1f} cm")
    a = turn_by(rover, rem[2], tol_deg=0.4)
    steps.append(f"turn {math.degrees(a):+.1f} deg")
    return "; ".join(steps)
