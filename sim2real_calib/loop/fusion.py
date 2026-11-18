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


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """SE2 a ∘ b (apply b in a's frame)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


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

    def gyro(self, t, wz):
        with self.lock:
            if self._gyro_t is not None and 0 < t - self._gyro_t < 0.5:
                self._gyro_yaw = wrap(self._gyro_yaw + wz * (t - self._gyro_t))
            self._gyro_t = t
            self.have_gyro = True

    def odom(self, t, x, y, yaw):
        with self.lock:
            if self._last_odom is None:
                self._last_odom = (x, y, yaw)
                self._append(t, (0.0, 0.0, self._gyro_yaw if self._use_gyro() else yaw))
                return
            lx, ly, lyaw = self._last_odom
            self._last_odom = (x, y, yaw)
            # increment in the SOURCE odom's body frame
            c, s = math.cos(lyaw), math.sin(lyaw)
            fx, fy = c * (x - lx) + s * (y - ly), -s * (x - lx) + c * (y - ly)
            fx, fy = fx * self.scale, fy * self.scale
            px, py, pyaw = self.pose[-1]
            nyaw = self._gyro_yaw if self._use_gyro() else wrap(pyaw + wrap(yaw - lyaw))
            mid = wrap(pyaw + wrap(nyaw - pyaw) / 2)
            cm, sm = math.cos(mid), math.sin(mid)
            self._append(t, (px + cm * fx - sm * fy, py + sm * fx + cm * fy, nyaw))

    def _use_gyro(self):
        return self.use_gyro and self.have_gyro

    def _append(self, t, p):
        self.t.append(t)
        self.pose.append(p)
        while self.t and t - self.t[0] > self.max_hist:
            self.t.pop(0)
            self.pose.pop(0)

    def at(self, t):
        """Odometry pose interpolated at desktop time t (None if outside history)."""
        with self.lock:
            if not self.t or t < self.t[0] - 0.05:
                return None
            if t >= self.t[-1]:
                return self.pose[-1] if t - self.t[-1] < 0.5 else None
            i = bisect.bisect_right(self.t, t)
            if i == 0:                      # just before the first sample (within 0.05 s):
                return self.pose[0]         # i-1 would wrap to the LAST sample and extrapolate
            t0, t1 = self.t[i - 1], self.t[i]
            a, b = self.pose[i - 1], self.pose[i]
            f = (t - t0) / max(t1 - t0, 1e-6)
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
                    wrap(a[2] + f * wrap(b[2] - a[2])))

    def latest(self):
        with self.lock:
            return (self.t[-1], self.pose[-1]) if self.t else None


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

    def camera(self, t_decode, x, y, yaw):
        t_cap = t_decode - self.cam_latency
        o = self.odom.at(t_cap)
        if o is None:
            return False
        meas = compose((x, y, yaw), inverse(o))
        if self.C is None:
            self.C = meas
        else:
            d = compose(inverse(self.C), meas)
            if math.hypot(d[0], d[1]) > self.reject_m or abs(d[2]) > self.reject_rad:
                # a wild fix (mis-detection) - but if they KEEP disagreeing the odom is
                # what's wrong (e.g. it was reset), so re-anchor after a few in a row
                self.n_reject += 1
                if self.n_reject < 8:
                    return False
                self.C = meas
            else:
                g = self.gain
                self.C = compose(self.C, (g * d[0], g * d[1], g * d[2]))
        self.n_reject = 0
        self.n_fix += 1
        self.last_fix_t = t_decode
        return True

    def pose(self, max_fix_age=4.0):
        """(t, x, y, yaw) in grid metres, or None if not anchored / anchor too old."""
        lt = self.odom.latest()
        if self.C is None or lt is None:
            return None
        t, o = lt
        if t - self.last_fix_t > max_fix_age:
            return None
        return (t, *compose(self.C, o))


def self_test():
    """Rover spins + drives; camera is 0.25 s late, noisy, drops out for 2 s."""
    import random
    rnd = random.Random(1)
    oi = OdomIntegrator(use_gyro=True)
    fz = PoseFuser(oi, cam_latency_s=0.25, gain=0.3)
    true = [0.5, 0.2, math.pi / 2]
    odo = [0.0, 0.0, 0.0]                    # wheel odom: yaw slips (x2.5) on spins
    dt, worst, cam_q = 0.02, 0.0, []
    for k in range(int(12 / dt)):
        t = k * dt
        v, w = (0.1, 0.0) if t < 4 else ((0.0, 0.6) if t < 8 else (0.08, 0.2))
        true[2] = wrap(true[2] + w * dt)
        true[0] += v * math.cos(true[2]) * dt
        true[1] += v * math.sin(true[2]) * dt
        odo[2] = wrap(odo[2] + 2.5 * w * dt)                 # slipping wheel yaw
        odo[0] += v * 1.05 * math.cos(odo[2]) * dt
        odo[1] += v * 1.05 * math.sin(odo[2]) * dt
        oi.gyro(t, w + rnd.gauss(0, 0.01) + 0.005)            # biased, noisy gyro
        if k % 5 == 0:
            oi.odom(t, *odo)
        cam_q.append((t, true[0] + rnd.gauss(0, 0.003), true[1] + rnd.gauss(0, 0.003),
                      true[2] + rnd.gauss(0, 0.01)))
        while cam_q and t - cam_q[0][0] >= 0.25:              # arrives 0.25 s late
            tc, cx, cy, cyaw = cam_q.pop(0)
            if not (6.0 <= tc < 8.0) and k % 2 == 0:          # 2 s occlusion mid-spin
                fz.camera(t, cx, cy, cyaw)
        p = fz.pose()
        if p is not None and t > 1.0:
            e = math.hypot(p[1] - true[0], p[2] - true[1])
            worst = max(worst, e)
    print(f"  worst fused position error {worst*100:.1f} cm (camera 0.25 s late, "
          f"2 s occlusion during a spin, slipping wheel odom)")
    # a fix stamped just before the first odom sample must not extrapolate (it once
    # anchored the fuser kilometres away: the wild first homing tick of 2026-11-03)
    o = OdomIntegrator(use_gyro=False)
    o.odom(10.00, 0.0, 0.0, 0.0)
    o.odom(10.10, 0.001, 0.0, 0.0)
    pre = o.at(9.97)
    print(f"  odom just before its first sample: {pre}")
    return worst if max(abs(c) for c in pre) < 0.01 else float("inf")


if __name__ == "__main__":
    w = self_test()
    print("SELF-TEST", "PASSED" if w < 0.03 else "FAILED")
