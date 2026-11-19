#!/usr/bin/env python3
"""Camera + rover-odometry pose fusion (latency-compensated complementary filter).

The overhead camera is ABSOLUTE but late (~0.15-0.3 s decode/stream lag) and can drop
out (sticker occlusion). The rover's odometry is PROMPT and continuous but RELATIVE and
drifts (wheel slip on carpet; rf2o/gyro much less). Fusing them gives the controller a
pose that is both current and absolute:

    fused(now) = C ∘ odom(now)              C = grid <- odom correction (SE2)
    on a camera fix stamped t_cam (capture time = decode time - cam_latency):
        C_meas = cam(t_cam) ∘ odom(t_cam)^-1     odom interpolated at t_cam
        C <- blend(C, C_meas, gain)            (first fix: C = C_meas)

So camera lag never delays the control loop, and through an occlusion the pose keeps
moving with odometry. Odometry yaw can come from the IMU gyro (immune to wheel slip)
while translation comes from an odom topic, re-expressed in the gyro's heading.
"""
from __future__ import annotations

import bisect
import math
import threading


# Wrap an angle to [-pi, pi).
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# Planar (SE2) pose composition: pose b expressed in pose a's frame -> world pose.
# Poses are (x, y, yaw) tuples.
def compose(a, b):
    """SE2 a ∘ b (apply b in a's frame)."""
    cos_a, sin_a = math.cos(a[2]), math.sin(a[2])
    return (a[0] + cos_a * b[0] - sin_a * b[1], a[1] + sin_a * b[0] + cos_a * b[1], wrap(a[2] + b[2]))


# Inverse SE2 pose: compose(a, inverse(a)) = (0, 0, 0).
def inverse(a):
    cos_a, sin_a = math.cos(a[2]), math.sin(a[2])
    return (-cos_a * a[0] - sin_a * a[1], sin_a * a[0] - cos_a * a[1], wrap(-a[2]))


# Relative pose track from odometry (x, y, yaw), stamped in desktop time and kept for
# max_hist_s seconds so it can be looked up at a camera frame's (past) capture time.
class OdomIntegrator:
    """Builds a slip-robust relative track: yaw from the gyro when available, else from
    the odom message; translation from odom increments rotated into that yaw."""

    def __init__(self, use_gyro=True, max_hist_s=10.0, scale=1.0):
        self.use_gyro = use_gyro
        self.scale = scale                  # translation scale (wheel odom over-reads on carpet)
        self.max_hist = max_hist_s
        self.lock = threading.Lock()
        self.t, self.pose = [], []          # desktop-time stamped (x, y, yaw)
        self._last_odom = None              # (x, y, yaw) of the source odom
        self._gyro_yaw = 0.0
        self._gyro_t = None
        self.have_gyro = False

    # Integrate one gyro yaw-rate sample (rad/s) into the gyro heading.
    def gyro(self, t, wz):
        with self.lock:
            if self._gyro_t is not None and 0 < t - self._gyro_t < 0.5:
                self._gyro_yaw = wrap(self._gyro_yaw + wz * (t - self._gyro_t))
            self._gyro_t = t
            self.have_gyro = True

    # Add one odometry message: take its increment in the source odom's body frame, scale
    # it (odom_scale), and add it to the track rotated by the mid-step heading (gyro yaw if
    # available, else the odom's own yaw change).
    def odom(self, t, x, y, yaw):
        with self.lock:
            if self._last_odom is None:
                self._last_odom = (x, y, yaw)
                self._append(t, (0.0, 0.0, self._gyro_yaw if self._use_gyro() else yaw))
                return
            last_x, last_y, last_yaw = self._last_odom
            self._last_odom = (x, y, yaw)
            # increment in the SOURCE odom's body frame
            cos_last, sin_last = math.cos(last_yaw), math.sin(last_yaw)
            step_forward, step_left = cos_last * (x - last_x) + sin_last * (y - last_y), -sin_last * (x - last_x) + cos_last * (y - last_y)
            step_forward, step_left = step_forward * self.scale, step_left * self.scale
            prev_x, prev_y, prev_yaw = self.pose[-1]
            new_yaw = self._gyro_yaw if self._use_gyro() else wrap(prev_yaw + wrap(yaw - last_yaw))
            mid_yaw = wrap(prev_yaw + wrap(new_yaw - prev_yaw) / 2)
            cos_mid, sin_mid = math.cos(mid_yaw), math.sin(mid_yaw)
            self._append(t, (prev_x + cos_mid * step_forward - sin_mid * step_left, prev_y + sin_mid * step_forward + cos_mid * step_left, new_yaw))

    # Use the gyro heading only when asked for AND gyro samples have arrived.
    def _use_gyro(self):
        return self.use_gyro and self.have_gyro

    # Append a stamped pose and drop history older than max_hist_s.
    def _append(self, t, p):
        self.t.append(t)
        self.pose.append(p)
        while self.t and t - self.t[0] > self.max_hist:
            self.t.pop(0)
            self.pose.pop(0)

    # Odometry pose linearly interpolated at time t (yaw interpolated on the circle).
    def at(self, t):
        """Odometry pose interpolated at desktop time t (None if outside history)."""
        with self.lock:
            if not self.t or t < self.t[0] - 0.05:
                return None
            if t >= self.t[-1]:
                return self.pose[-1] if t - self.t[-1] < 0.5 else None
            index_after = bisect.bisect_right(self.t, t)
            if index_after == 0:                      # just before the first sample (within 0.05 s):
                return self.pose[0]         # i-1 would wrap to the LAST sample and extrapolate
            time_before, time_after = self.t[index_after - 1], self.t[index_after]
            pose_before, pose_after = self.pose[index_after - 1], self.pose[index_after]
            fraction = (t - time_before) / max(time_after - time_before, 1e-6)
            return (pose_before[0] + fraction * (pose_after[0] - pose_before[0]), pose_before[1] + fraction * (pose_after[1] - pose_before[1]),
                    wrap(pose_before[2] + fraction * wrap(pose_after[2] - pose_before[2])))

    # Newest (t, (x, y, yaw)) of the track, or None.
    def latest(self):
        with self.lock:
            return (self.t[-1], self.pose[-1]) if self.t else None


# Complementary filter keeping C, the correction from the odometry frame to grid
# metres (see the module docstring for the equations).
class PoseFuser:
    """fused pose = correction ∘ odom; corrections come from latency-shifted camera fixes."""

    def __init__(self, odom: OdomIntegrator, cam_latency_s=0.2, gain=0.3,
                 reject_m=0.15, reject_deg=20.0):
        self.odom = odom
        self.cam_latency = cam_latency_s
        self.gain = gain
        self.reject_m, self.reject_rad = reject_m, math.radians(reject_deg)
        self.C = None
        self.last_fix_t = 0.0
        self.n_fix = self.n_reject = 0

    # Feed one camera fix (decode time + pose): shift it back by the camera latency, look
    # the odometry up at that capture time, and move C a `gain` fraction towards the
    # measured correction. Fixes far from C are rejected as mis-detections, unless 8 in a
    # row disagree (then the odometry was reset: re-anchor).
    def camera(self, t_decode, x, y, yaw):
        capture_time = t_decode - self.cam_latency
        odom_pose = self.odom.at(capture_time)
        if odom_pose is None:
            return False
        measured_correction = compose((x, y, yaw), inverse(odom_pose))
        if self.C is None:
            self.C = measured_correction
        else:
            correction_change = compose(inverse(self.C), measured_correction)
            if math.hypot(correction_change[0], correction_change[1]) > self.reject_m or abs(correction_change[2]) > self.reject_rad:
                # a wild fix (mis-detection) - but if they KEEP disagreeing the odom is
                # what's wrong (e.g. it was reset), so re-anchor after a few in a row
                self.n_reject += 1
                if self.n_reject < 8:
                    return False
                self.C = measured_correction
            else:
                gain = self.gain
                self.C = compose(self.C, (gain * correction_change[0], gain * correction_change[1], gain * correction_change[2]))
        self.n_reject = 0
        self.n_fix += 1
        self.last_fix_t = t_decode
        return True

    # Current fused pose = C composed with the newest odometry pose.
    def pose(self, max_fix_age=4.0):
        """(t, x, y, yaw) in grid metres, or None if not anchored / anchor too old."""
        latest_odom = self.odom.latest()
        if self.C is None or latest_odom is None:
            return None
        odom_time, odom_pose = latest_odom
        if odom_time - self.last_fix_t > max_fix_age:
            return None
        return (odom_time, *compose(self.C, odom_pose))


# Simulated drive/spin with a late, noisy, briefly occluded camera and slipping wheel
# odometry; the fused position must stay within 3 cm of the truth.
def self_test():
    """Rover spins + drives; camera is 0.25 s late, noisy, drops out for 2 s."""
    import random
    rng = random.Random(1)
    integrator = OdomIntegrator(use_gyro=True)
    fuser = PoseFuser(integrator, cam_latency_s=0.25, gain=0.3)
    true_pose = [0.5, 0.2, math.pi / 2]
    wheel_odom = [0.0, 0.0, 0.0]                    # wheel odom: yaw slips (x2.5) on spins
    dt, worst_error, camera_queue = 0.02, 0.0, []
    for step in range(int(12 / dt)):
        sim_time = step * dt
        lin_speed, ang_speed = (0.1, 0.0) if sim_time < 4 else ((0.0, 0.6) if sim_time < 8 else (0.08, 0.2))
        true_pose[2] = wrap(true_pose[2] + ang_speed * dt)
        true_pose[0] += lin_speed * math.cos(true_pose[2]) * dt
        true_pose[1] += lin_speed * math.sin(true_pose[2]) * dt
        wheel_odom[2] = wrap(wheel_odom[2] + 2.5 * ang_speed * dt)                 # slipping wheel yaw
        wheel_odom[0] += lin_speed * 1.05 * math.cos(wheel_odom[2]) * dt
        wheel_odom[1] += lin_speed * 1.05 * math.sin(wheel_odom[2]) * dt
        integrator.gyro(sim_time, ang_speed + rng.gauss(0, 0.01) + 0.005)            # biased, noisy gyro
        if step % 5 == 0:
            integrator.odom(sim_time, *wheel_odom)
        camera_queue.append((sim_time, true_pose[0] + rng.gauss(0, 0.003), true_pose[1] + rng.gauss(0, 0.003),
                      true_pose[2] + rng.gauss(0, 0.01)))
        while camera_queue and sim_time - camera_queue[0][0] >= 0.25:              # arrives 0.25 s late
            cam_time, cam_x, cam_y, cam_yaw = camera_queue.pop(0)
            if not (6.0 <= cam_time < 8.0) and step % 2 == 0:          # 2 s occlusion mid-spin
                fuser.camera(sim_time, cam_x, cam_y, cam_yaw)
        fused = fuser.pose()
        if fused is not None and sim_time > 1.0:
            error = math.hypot(fused[1] - true_pose[0], fused[2] - true_pose[1])
            worst_error = max(worst_error, error)
    print(f"  worst fused position error {worst_error*100:.1f} cm (camera 0.25 s late, "
          f"2 s occlusion during a spin, slipping wheel odom)")
    # a fix stamped just before the first odom sample must not extrapolate (it once
    # anchored the fuser kilometres away: the wild first homing tick of 2026-11-03)
    integrator_b = OdomIntegrator(use_gyro=False)
    integrator_b.odom(10.00, 0.0, 0.0, 0.0)
    integrator_b.odom(10.10, 0.001, 0.0, 0.0)
    pose_before_first = integrator_b.at(9.97)
    print(f"  odom just before its first sample: {pose_before_first}")
    return worst_error if max(abs(coordinate) for coordinate in pose_before_first) < 0.01 else float("inf")


if __name__ == "__main__":
    w = self_test()
    print("SELF-TEST", "PASSED" if w < 0.03 else "FAILED")
