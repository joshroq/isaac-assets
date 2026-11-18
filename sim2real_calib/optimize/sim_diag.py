#!/usr/bin/env python3
"""Diagnose the Isaac sim ROS2 side: list topics, find odom, and test which QoS
actually receives messages (QoS mismatch is a common '0 samples' cause). Run with
Isaac's own python while run_ground_plane.py is already up:

    isaacpy sim2real_calib/optimize/sim_diag.py
"""
import time
import argparse
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom-topic", default="/ugv_sim/odom")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    print(f"DDS: domain={os.environ.get('ROS_DOMAIN_ID', '0')} "
          f"localhost_only={os.environ.get('ROS_LOCALHOST_ONLY', '0')}", flush=True)
    rclpy.init()
    node = Node("sim_diag")
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_publishers(args.odom_topic):
            break

    print("=== topics the sim exposes ===")
    topics = node.get_topic_names_and_types()
    for name, types in sorted(topics):
        print(f"  {name:28s} {types}")

    odom_topic = args.odom_topic
    print(f"\nusing odom topic: {odom_topic}")

    counts = {"reliable": 0, "best_effort": 0}
    qos_r = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
    qos_b = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
    last = {"pose": None}

    def cb_r(m):
        counts["reliable"] += 1
        p = m.pose.pose.position
        last["pose"] = (p.x, p.y)

    def cb_b(m):
        counts["best_effort"] += 1
        p = m.pose.pose.position
        last["pose"] = (p.x, p.y)

    node.create_subscription(Odometry, odom_topic, cb_r, qos_r)
    node.create_subscription(Odometry, odom_topic, cb_b, qos_b)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    print("publishing /cmd_vel linear.x=0.3 for 6 s, counting odom ...")
    t0 = time.time()
    while time.time() - t0 < 6.0:
        tw = Twist(); tw.linear.x = 0.3
        pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=0.05)
    # stop
    pub.publish(Twist())

    print(f"\nodom msgs received -> RELIABLE={counts['reliable']}  "
          f"BEST_EFFORT={counts['best_effort']}")
    print(f"last odom pose: {last['pose']}")
    node.destroy_node()
    rclpy.shutdown()
    if not any(counts.values()):
        print("No odometry received. Source ros2env.sh and check /ugv_sim/odom.",
              flush=True)
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
