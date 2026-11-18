#!/usr/bin/env python3
"""A stand-in for the physical rover's ROS interface, for offline tests of the loop.

Subscribes /cmd_vel and simulates the REAL rover's quirks on carpet:
  * translation 0.92x command, first-order lag (coasts ~2-4 cm after a stop)
  * in-place spins slip: true yaw rate 0.45x command, deadband |w| < 0.3 rad/s
  * linear deadband |v| < 0.035 m/s
Publishes what the rover publishes:
  /odom          wheel odometry (reports COMMANDED motion -> wrong on spins)
  /odom_rf2o     lidar odometry (truth + small drift)
  /imu/data_raw  gyro z (truth + bias + noise)
  /scan          LD19-like 0.8 deg raycast of a furnished room, rear sector cropped
  /voltage       11.4 V
  /fake/truth    ground truth in GRID metres (for the fake overhead camera)
Run in an isolated ROS domain (the test sets ROS_DOMAIN_ID).
"""
import math
import os
import random
import sys

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32

# room in GRID metres: grid occupies (0..1, 0..1); walls ~1 m beyond; bed + dresser + bag
SEGS = [((-1.0, -0.7), (2.1, -0.7)), ((2.1, -0.7), (2.1, 1.9)), ((2.1, 1.9), (-1.0, 1.9)),
        ((-1.0, 1.9), (-1.0, -0.7)),
        ((-1.0, 0.2), (-0.45, 0.2)), ((-0.45, 0.2), (-0.45, 1.5)), ((-0.45, 1.5), (-1.0, 1.5)),
        ((1.5, 1.4), (2.1, 1.4)), ((1.5, 1.4), (1.5, 1.9)),
        ((1.45, 0.3), (1.65, 0.3)), ((1.65, 0.3), (1.65, 0.6)), ((1.65, 0.6), (1.45, 0.6)),
        ((1.45, 0.6), (1.45, 0.3))]
FLIP = os.environ.get("FAKE_LIDAR_FLIP") == "1"
A = np.array([s[0] for s in SEGS], float)
B = np.array([s[1] for s in SEGS], float)


def quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class FakeRover(Node):
    def __init__(self, x0, y0, yaw0):
        super().__init__("fake_rover")
        self.x, self.y, self.yaw = x0, y0, yaw0
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.wo = [0.0, 0.0, 0.0]        # wheel odom
        self.lo = [0.0, 0.0, 0.0]        # rf2o
        self.rnd = random.Random(3)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.p_odom = self.create_publisher(Odometry, "/odom", 10)
        self.p_rf2o = self.create_publisher(Odometry, "/odom_rf2o", 10)
        self.p_imu = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.p_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.p_volt = self.create_publisher(Float32, "/voltage", 10)
        self.p_truth = self.create_publisher(Odometry, "/fake/truth", 10)
        self.dt = 0.02
        self.k = 0
        self.create_timer(self.dt, self.step)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def step(self):
        lin, ang = self.cmd
        tv = 0.92 * lin if abs(lin) >= 0.035 else 0.0
        if lin == 0:
            tw = 0.45 * ang if abs(ang) >= 0.3 else 0.0
        else:
            tw = 0.8 * ang
        a = self.dt / 0.12
        self.v += a * (tv - self.v)
        self.w += a * (tw - self.w)
        self.yaw = (self.yaw + self.w * self.dt + math.pi) % (2 * math.pi) - math.pi
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        # wheel odom integrates the COMMAND (wheels turn, rover slips)
        self.wo[2] += ang * self.dt if abs(ang) > 0 else 0.0
        self.wo[0] += lin * math.cos(self.wo[2]) * self.dt
        self.wo[1] += lin * math.sin(self.wo[2]) * self.dt
        # rf2o: true body motion + 1 % scale + noise
        self.lo[2] += self.w * self.dt * 1.01
        self.lo[0] += self.v * 1.01 * math.cos(self.lo[2]) * self.dt
        self.lo[1] += self.v * 1.01 * math.sin(self.lo[2]) * self.dt
        self.k += 1
        now = self.get_clock().now().to_msg()
        if self.k % 5 == 0:                                   # 10 Hz odoms + truth
            for pub, (x, y, yaw), frame in ((self.p_odom, self.wo, "odom"),
                                            (self.p_rf2o, self.lo, "odom"),
                                            (self.p_truth, (self.x, self.y, self.yaw), "grid")):
                o = Odometry()
                o.header.stamp, o.header.frame_id = now, frame
                o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
                o.pose.pose.orientation.z, o.pose.pose.orientation.w = quat(yaw)
                pub.publish(o)
            self.p_volt.publish(Float32(data=11.4))
        if self.k % 3 == 0:                                   # ~17 Hz gyro
            m = Imu()
            m.header.stamp = now
            m.angular_velocity.z = self.w + 0.012 + self.rnd.gauss(0, 0.004)
            self.p_imu.publish(m)
        if self.k % 5 == 0:                                   # 10 Hz scan
            self.p_scan.publish(self.scan(now))

    def scan(self, stamp):
        inc = math.radians(0.8)
        angs = np.arange(0, 2 * math.pi, inc)
        d = np.column_stack([np.cos(angs + self.yaw), np.sin(angs + self.yaw)])
        p = np.array([self.x, self.y])
        e = B - A
        r = np.full(len(angs), np.inf)
        for i in range(len(A)):
            w = A[i] - p
            den = d[:, 0] * (-e[i, 1]) - d[:, 1] * (-e[i, 0])
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (w[0] * (-e[i, 1]) - w[1] * (-e[i, 0])) / den
                u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / den
            hit = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
            r = np.where(hit & (t < r), t, r)
        r = r + np.array([self.rnd.gauss(0, 0.006) for _ in r])
        deg = np.degrees(angs)
        r[(deg >= 225) & (deg <= 315)] = 0.0                  # LD19 angle crop (mast)
        r[np.array([self.rnd.random() < 0.04 for _ in r])] = 0.0
        m = LaserScan()
        m.header.stamp, m.header.frame_id = stamp, "base_lidar_link"
        m.angle_min, m.angle_max, m.angle_increment = 0.0, float(angs[-1]), inc
        m.range_min, m.range_max = 0.03, 12.0
        if FLIP:                                              # mirrored (clockwise) LD19
            r = np.concatenate([r[:1], r[1:][::-1]])
        m.ranges = [float(v) if np.isfinite(v) else 0.0 for v in r]
        return m


def main():
    x0, y0, yaw0 = (float(v) for v in (sys.argv[1:4] if len(sys.argv) >= 4 else (0.5, 0.5, 1.57)))
    rclpy.init()
    try:
        rclpy.spin(FakeRover(x0, y0, yaw0))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exc:                 # SIGTERM at test teardown invalidates the context
        if "context is not valid" not in str(exc) and "context is invalid" not in str(exc):
            raise


if __name__ == "__main__":
    main()
