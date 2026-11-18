#!/usr/bin/env python3
"""Bridge the OAK depth image from BEST_EFFORT to RELIABLE QoS.

depth_image_proc's point_cloud_xyz_node subscribes to the depth image RELIABLY, but the
depthai driver publishes /oak/stereo/image_raw BEST_EFFORT (sensor data). A reliable
reader is QoS-incompatible with a best-effort writer, so the cloud node receives ~0
images and /oak/points trickles out at <1 Hz. image_transport in Humble ignores
qos_overrides, so we can't just flip the node's QoS.

This node subscribes BEST_EFFORT (matches the camera) and republishes the SAME image,
header untouched, on a RELIABLE topic that point_cloud_xyz_node can consume. Result:
/oak/points runs at the stereo rate instead of starving.

    in : /oak/stereo/image_raw        (best_effort, from depthai)
    out: /oak/stereo/image_reliable   (reliable, for depth_image_proc)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class DepthQosRelay(Node):
    def __init__(self):
        super().__init__("depth_qos_relay")
        src = self.declare_parameter("in_topic", "/oak/stereo/image_raw").value
        dst = self.declare_parameter("out_topic", "/oak/stereo/image_reliable").value
        best_effort = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Image, dst, reliable)
        self._sub = self.create_subscription(Image, src, self._pub.publish, best_effort)
        self.get_logger().info(f"depth QoS relay: {src} (best_effort) -> {dst} (reliable)")


def main():
    rclpy.init()
    node = DepthQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
