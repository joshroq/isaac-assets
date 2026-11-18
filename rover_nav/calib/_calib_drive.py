#!/usr/bin/env python3
"""Drive the rover a fixed (vx, wz) for T seconds, capture odom start/end + battery
voltage, and print a machine-readable RESULT line. Zeros /cmd_vel on exit (the base
has NO cmd_vel deadman, so we must stop it explicitly). Called by calib_run.sh.

  python3 _calib_drive.py --vx 0.15 --wz 0 --time 3
"""
import argparse, math, time, sys
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--vx", type=float, default=0.0)
    p.add_argument("--wz", type=float, default=0.0)
    p.add_argument("--time", type=float, default=3.0)
    p.add_argument("--topic", default="/cmd_vel")
    a = p.parse_args()

    rclpy.init()
    n = rclpy.create_node("calib_drive")
    st = {"odom": None, "v": None}
    cur = {"odom": None, "v": None}

    def od(m):
        cur["odom"] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
        if st["odom"] is None:
            st["odom"] = cur["odom"]

    def vv(m):
        cur["v"] = m.data
        if st["v"] is None:
            st["v"] = m.data

    n.create_subscription(Odometry, "/odom", od, 10)
    n.create_subscription(Float32, "/voltage", vv, 10)
    pub = n.create_publisher(Twist, a.topic, 10)

    # wait for first odom
    t0 = time.time()
    while cur["odom"] is None and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if cur["odom"] is None:
        print("RESULT error=no_odom (is start_bringup.sh running?)")
        n.destroy_node(); rclpy.shutdown(); return 1

    # drive, then always zero
    m = Twist(); m.linear.x = a.vx; m.angular.z = a.wz
    t0 = time.time()
    try:
        while time.time() - t0 < a.time:
            pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.02)
    finally:
        z = Twist()
        for _ in range(15):
            pub.publish(z); time.sleep(0.02)

    # settle and read final odom
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    s = st["odom"]; e = cur["odom"]
    dist = math.hypot(e[0] - s[0], e[1] - s[1])
    dyaw = (e[2] - s[2] + math.pi) % (2 * math.pi) - math.pi
    print("RESULT odom_dist=%.4f odom_dyaw_deg=%.2f dx=%.4f dy=%.4f volt_start=%.2f volt_end=%.2f"
          % (dist, math.degrees(dyaw), e[0] - s[0], e[1] - s[1],
             st["v"] if st["v"] is not None else -1.0,
             cur["v"] if cur["v"] is not None else -1.0))
    n.destroy_node(); rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
