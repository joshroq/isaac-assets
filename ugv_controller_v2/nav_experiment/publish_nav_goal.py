#!/usr/bin/env python3
"""Publish a Nav2 goal pose, similar to RViz's 2D Nav Goal tool.

Usage:
    source /opt/ros/humble/setup.bash
    source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
    source ~/.local/bin/ros2ddsenv
    python3 ugv_controller_v2/nav_experiment/publish_nav_goal.py --x 2.0 --y 1.0 --yaw 0.0

By default this publishes ``geometry_msgs/PoseStamped`` on ``/goal_pose`` for a
few seconds so Nav2 has time to match the publisher. The frame defaults to
``map``, which is what the RViz navigation goal uses once SLAM/localization is
active.
"""

from __future__ import annotations

import argparse
import math
import time


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a one-shot Nav2 goal pose.")
    parser.add_argument("--x", type=float, required=True, help="Goal X in the target frame.")
    parser.add_argument("--y", type=float, required=True, help="Goal Y in the target frame.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw in radians.")
    parser.add_argument("--frame", default="map", help="Goal frame, usually map.")
    parser.add_argument("--topic", default="/goal_pose", help="Goal topic.")
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Seconds to keep publishing so late subscribers can receive the goal.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Publish rate while duration is active.",
    )
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        default=True,
        help="Stamp the goal with ROS sim time from /clock.",
    )
    parser.add_argument(
        "--no-use-sim-time",
        dest="use_sim_time",
        action="store_false",
        help="Stamp the goal with wall time instead of /clock.",
    )
    return parser.parse_args()


def _goal_msg(node, args: argparse.Namespace):
    from geometry_msgs.msg import PoseStamped

    half_yaw = 0.5 * args.yaw
    msg = PoseStamped()
    msg.header.frame_id = args.frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(half_yaw)
    msg.pose.orientation.w = math.cos(half_yaw)
    return msg


def main() -> int:
    args = _parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("ugv_v2_publish_nav_goal")
    node.set_parameters([
        Parameter("use_sim_time", Parameter.Type.BOOL, args.use_sim_time)
    ])

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(PoseStamped, args.topic, qos)

    period = 1.0 / max(args.rate, 0.1)
    deadline = time.monotonic() + max(args.duration, 0.1)
    count = 0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(_goal_msg(node, args))
            count += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        print(
            f"[publish_nav_goal] Published {count} goal messages to {args.topic}: "
            f"frame={args.frame} x={args.x:.3f} y={args.y:.3f} yaw={args.yaw:.3f}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
