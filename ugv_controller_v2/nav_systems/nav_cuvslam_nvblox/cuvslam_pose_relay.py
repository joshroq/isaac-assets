#!/usr/bin/env python3
"""Stamp a covariance onto cuVSLAM's absolute map-frame pose for the fusion EKF.

WHY
    In the co-equal lidar+visual localization fusion (see this stack's launch,
    use_localization_fusion), a robot_localization EKF owns map->odom by fusing:
        odom0 = /ugv_sim/odom          wheel odometry   (velocities)
        odom1 = /cuvslam/lidar_odom    rf2o laser odom  (velocities, rotation-robust)
        odom2 = /cuvslam/pose          cuVSLAM absolute (x, y, yaw pose)  <-- this node
    cuVSLAM already publishes its loop-closure-corrected absolute pose on
    `visual_slam/vis/slam_odometry` (nav_msgs/Odometry, map frame), the exact analog
    of ORB-SLAM3's /orbslam3/pose in nav_orbslam3. But the Isaac ROS node fills only
    `pose.pose` there ("only populating slam_pose for viz") and leaves the covariance
    ALL ZERO. robot_localization treats a zero pose covariance as an infinitely certain
    measurement, which collapses the filter's state covariance and makes it reject the
    wheel/rf2o backbone — so the EKF needs a sane covariance stamped on.

WHAT
    Subscribe the cuVSLAM map-frame odometry, copy its pose, stamp a diagonal
    x/y/yaw covariance, retag the child frame to the EKF's base_link_frame (the pose
    is planar; the base_link vs base_footprint offset is purely vertical and ignored in
    the EKF's two_d_mode), and republish as nav_msgs/Odometry for odom2. When cuVSLAM
    loses tracking it stops publishing, so this relay goes quiet and the EKF coasts on
    the wheel + rf2o velocities until cuVSLAM recovers — exactly the ORB-SLAM3 behavior.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CuvslamPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("cuvslam_pose_relay")
        self.declare_parameter("input_topic", "/visual_slam/vis/slam_odometry")
        self.declare_parameter("output_topic", "/cuvslam/pose")
        # Base frame the EKF fuses in (its base_link_frame). cuVSLAM tags the pose with
        # its own base_frame (base_link); retag to base_footprint so no TF hop is needed.
        self.declare_parameter("base_frame", "base_footprint")
        # 1-sigma pose uncertainty: ~7 cm in x/y, ~0.1 rad (~6 deg) in yaw. Loose enough
        # that the wheel/rf2o backbone still contributes, tight enough to correct drift.
        self.declare_parameter("std_xy", 0.07)
        self.declare_parameter("std_yaw", 0.10)

        self.output_frame = self.get_parameter("base_frame").value
        var_xy = float(self.get_parameter("std_xy").value) ** 2
        var_yaw = float(self.get_parameter("std_yaw").value) ** 2
        # Row-major 6x6 (x, y, z, roll, pitch, yaw). Only x, y, yaw are fused; the other
        # diagonals get a large-but-finite value so nothing else is ever trusted here.
        self.covariance = [0.0] * 36
        self.covariance[0] = var_xy      # x
        self.covariance[7] = var_xy      # y
        self.covariance[14] = 1e6        # z (unused)
        self.covariance[21] = 1e6        # roll (unused)
        self.covariance[28] = 1e6        # pitch (unused)
        self.covariance[35] = var_yaw    # yaw

        out_topic = self.get_parameter("output_topic").value
        in_topic = self.get_parameter("input_topic").value
        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self._on_odom, 10)
        self.get_logger().info(
            f"cuVSLAM pose relay: {in_topic} -> {out_topic} "
            f"(child_frame={self.output_frame}, std_xy={self.get_parameter('std_xy').value}, "
            f"std_yaw={self.get_parameter('std_yaw').value})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header               # keep stamp + map frame_id
        out.child_frame_id = self.output_frame
        out.pose.pose = msg.pose.pose
        out.pose.covariance = self.covariance
        # twist is intentionally left zero/unused: only the absolute pose is fused here.
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = CuvslamPoseRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
