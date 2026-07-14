#!/usr/bin/env python3
"""
DPVO (Deep Patch Visual Odometry) -> ROS 2 map->odom bridge — UNIQUE to nav_dpvo.

Wraps the DPVO source in projects/slam_source/DPVO-main DIRECTLY (imports the
`dpvo` package, loads the trained `dpvo.pth` network, and runs the same
DPVO(cfg, net).__call__(t, image, intrinsics) loop demo.py uses) and turns its
camera trajectory into the map->odom transform the Nav2 stack consumes — the exact
role slam_toolbox plays in nav_lidar and RTAB-Map plays in nav_rgb. So this is a
"true" DPVO navigation system: DPVO itself is the localizer.

Pipeline (per RGB frame on /oak/rgb/image_rect):
  1. feed the frame + camera intrinsics (from /oak/rgb/camera_info) to DPVO
  2. read DPVO's latest camera pose  T_world_cam  (cam-optical convention)
  3. DPVO is MONOCULAR -> its world is up-to-scale. Estimate the metric scale
     online by matching DPVO's travelled distance to Isaac's /odom, and apply it.
  4. compose  map->odom = C * (T_map_base * inv(T_odom_base))  where T_map_base is
     the scaled DPVO pose moved from the camera optical frame onto base_footprint
     (static TF), T_odom_base is Isaac's odometry, and C anchors map==odom at start.
  5. broadcast map->odom (TF) and publish the scaled patch points as
     /dpvo/map_points (frame=map) for common/visual_map_builder.py -> /map.

DPVO's camera frame uses the ROS optical convention (x right, y down, z forward),
so it maps straight onto oak_rgb_camera_optical_frame. If the trajectory comes out
mirrored/rotated on the first run, flip `~invert_pose` / the axis handling here
(flagged in NAV_README) — not the URDF.

REQUIRES (in the Python env that runs this node): a working DPVO install
(`pip install .` in DPVO-main, with torch + CUDA) and the trained weights
(DPVO-main/dpvo.pth from download_models_and_data.sh). See NAV_README prerequisites.
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import tf2_ros
from tf2_ros import TransformBroadcaster


def _quat_to_mat(qx, qy, qz, qw):
    """Unit quaternion -> 3x3 rotation matrix."""
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _mat_to_quat(R):
    """3x3 rotation matrix -> (qx, qy, qz, qw)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / (tr + 1.0) ** 0.5
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def _pose_to_mat(tx, ty, tz, qx, qy, qz, qw):
    T = np.eye(4)
    T[:3, :3] = _quat_to_mat(qx, qy, qz, qw)
    T[:3, 3] = (tx, ty, tz)
    return T


def _tf_to_mat(tf: TransformStamped):
    t = tf.transform.translation
    q = tf.transform.rotation
    return _pose_to_mat(t.x, t.y, t.z, q.x, q.y, q.z, q.w)


class DpvoBridge(Node):
    def __init__(self):
        super().__init__('dpvo_bridge')

        self.declare_parameter('dpvo_path', '/home/joshroq/projects/slam_source/DPVO-main')
        self.declare_parameter('network', 'dpvo.pth')          # relative to dpvo_path if not absolute
        self.declare_parameter('config', 'config/default.yaml')  # relative to dpvo_path
        self.declare_parameter('rgb_topic', '/oak/rgb/image_rect')
        self.declare_parameter('info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('cloud_topic', '/dpvo/map_points')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_optical_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('stride', 2)                    # process every Nth frame (demo default)
        self.declare_parameter('warmup_frames', 8)            # DPVO needs a few frames to initialise
        self.declare_parameter('scale', 0.0)                  # >0 = fixed metric scale; 0 = estimate online
        self.declare_parameter('min_scale_travel', 0.5)       # m of odom motion before trusting the scale
        self.declare_parameter('invert_pose', False)          # flip if the trajectory comes out mirrored

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.cam_frame = gp('camera_optical_frame').value
        self.stride = int(gp('stride').value)
        self.warmup = int(gp('warmup_frames').value)
        self.fixed_scale = float(gp('scale').value)
        self.min_scale_travel = float(gp('min_scale_travel').value)
        self.invert_pose = bool(gp('invert_pose').value)

        # ── DPVO import + network (from the slam_source tree, directly) ───────────
        dpvo_path = gp('dpvo_path').value
        if dpvo_path not in sys.path:
            sys.path.insert(0, dpvo_path)
        try:
            import torch  # noqa: F401
            from dpvo.config import cfg
            from dpvo.dpvo import DPVO
        except Exception as e:  # pragma: no cover - env dependent
            self.get_logger().fatal(
                f"Could not import DPVO from '{dpvo_path}': {e}\n"
                "Install it (`pip install .` in DPVO-main, needs torch+CUDA) and set "
                "the 'dpvo_path' parameter.")
            raise

        self._torch = torch
        cfg_path = gp('config').value
        if not os.path.isabs(cfg_path):
            cfg_path = os.path.join(dpvo_path, cfg_path)
        cfg.merge_from_file(cfg_path)
        self._cfg = cfg
        self._DPVO = DPVO

        net = gp('network').value
        self._network = net if os.path.isabs(net) else os.path.join(dpvo_path, net)
        if not os.path.exists(self._network):
            self.get_logger().warn(
                f"DPVO weights not found at {self._network} — run "
                "DPVO-main/download_models_and_data.sh or set 'network'.")
        self.slam = None            # created lazily once the first frame gives H, W
        self.frame_idx = 0          # raw frame counter (for stride)
        self.dpvo_t = 0             # timestamp fed to DPVO (monotonic)

        # ── Online monocular-scale + start-anchor state ──────────────────────────
        self.scale = self.fixed_scale if self.fixed_scale > 0 else 1.0
        self._dpvo_prev_xyz = None      # last DPVO camera position (world, unscaled)
        self._odom_prev_xyz = None      # last odom->base translation
        self._dpvo_travel = 0.0
        self._odom_travel = 0.0
        self._anchor = None             # C: left-multiply that makes map==odom at start

        # ── Intrinsics (filled from the first CameraInfo) ────────────────────────
        self.K = None                   # (fx, fy, cx, cy)

        # ── TF ────────────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── I/O ──────────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(CameraInfo, gp('info_topic').value, self._on_info, sensor_qos)
        self.create_subscription(Image, gp('rgb_topic').value, self._on_image, sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, gp('cloud_topic').value, 2)

        self.get_logger().info(
            f"dpvo_bridge up: {gp('rgb_topic').value} -> DPVO -> "
            f"{self.map_frame}->{self.odom_frame} TF + {gp('cloud_topic').value}")

    # ── Callbacks ─────────────────────────────────────────────────────────────────
    def _on_info(self, msg: CameraInfo):
        if self.K is None and msg.k[0] > 0.0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
            self.get_logger().info(f"DPVO intrinsics fx,fy,cx,cy = {self.K}")

    def _image_to_np(self, msg: Image) -> np.ndarray:
        """ROS Image -> HxWx3 uint8 BGR (what DPVO's demo feeds via cv2.imread)."""
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        enc = msg.encoding.lower()
        if enc in ('rgb8', 'rgb'):
            buf = buf[:, :, ::-1]        # -> BGR to match cv2.imread ordering
        elif enc in ('bgr8', 'bgr'):
            pass
        elif enc in ('mono8', '8uc1'):
            buf = np.repeat(buf, 3, axis=2)
        return np.ascontiguousarray(buf[:, :, :3])

    def _on_image(self, msg: Image):
        if self.K is None:
            return  # wait for intrinsics
        self.frame_idx += 1
        if self.frame_idx % self.stride != 0:
            return

        image = self._image_to_np(msg)
        h, w = image.shape[:2]
        h, w = h - h % 16, w - w % 16       # DPVO requires dims divisible by 16
        image = image[:h, :w]
        intr = np.array(self.K, dtype=np.float32)

        torch = self._torch
        img_t = torch.from_numpy(image).permute(2, 0, 1).cuda()
        intr_t = torch.from_numpy(intr).cuda()

        if self.slam is None:
            self.slam = self._DPVO(self._cfg, self._network, ht=h, wd=w, viz=False)

        with torch.no_grad():
            self.slam(self.dpvo_t, img_t, intr_t)
        self.dpvo_t += 1

        if self.slam.n <= self.warmup:
            return  # DPVO not yet initialised — no reliable pose
        self._update_localization(msg.header)

    # ── DPVO pose -> map->odom ─────────────────────────────────────────────────────
    def _latest_pose_world_cam(self):
        """DPVO's most recent camera pose as 4x4 T_world_cam (unscaled)."""
        torch = self._torch
        n = int(self.slam.n)
        # pg.poses_ store world->camera (SE3); invert for camera->world (like terminate()).
        from dpvo.lietorch import SE3
        p = SE3(self.slam.poses[0, n - 1]).inv().data  # [tx,ty,tz,qx,qy,qz,qw]
        p = p.detach().cpu().numpy().reshape(-1)
        T = _pose_to_mat(*p[:3], *p[3:7])
        if self.invert_pose:
            T = np.linalg.inv(T)
        return T, p[:3]

    def _update_scale(self, dpvo_xyz, header):
        """Match DPVO travelled distance to Isaac /odom to recover metric scale."""
        if self.fixed_scale > 0:
            return
        try:
            tf_ob = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        odom_xyz = np.array([tf_ob.transform.translation.x,
                             tf_ob.transform.translation.y,
                             tf_ob.transform.translation.z])
        if self._dpvo_prev_xyz is not None:
            self._dpvo_travel += float(np.linalg.norm(dpvo_xyz - self._dpvo_prev_xyz))
            self._odom_travel += float(np.linalg.norm(odom_xyz - self._odom_prev_xyz))
            if self._odom_travel > self.min_scale_travel and self._dpvo_travel > 1e-3:
                self.scale = self._odom_travel / self._dpvo_travel
        self._dpvo_prev_xyz = dpvo_xyz
        self._odom_prev_xyz = odom_xyz

    def _update_localization(self, header):
        T_world_cam, dpvo_xyz = self._latest_pose_world_cam()
        self._update_scale(dpvo_xyz, header)

        # Apply the metric scale to the (similarity) trajectory: scale translation only.
        T_map_cam = T_world_cam.copy()
        T_map_cam[:3, 3] *= self.scale

        # Camera optical -> base_footprint (static, from the URDF TF tree).
        try:
            tf_bc = self.tf_buffer.lookup_transform(
                self.base_frame, self.cam_frame, rclpy.time.Time())
            tf_ob = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        T_base_cam = _tf_to_mat(tf_bc)
        T_odom_base = _tf_to_mat(tf_ob)

        T_map_base = T_map_cam @ np.linalg.inv(T_base_cam)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        # Anchor: force map==odom at the first valid frame so the rover starts sane.
        if self._anchor is None:
            self._anchor = np.linalg.inv(T_map_odom)
        T_map_odom = self._anchor @ T_map_odom

        self._broadcast(T_map_odom, header.stamp)
        self._publish_points(header.stamp)

    def _broadcast(self, T, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = _mat_to_quat(T[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y = float(qx), float(qy)
        t.transform.rotation.z, t.transform.rotation.w = float(qz), float(qw)
        self.tf_broadcaster.sendTransform(t)

    def _publish_points(self, stamp):
        """Scaled + anchored DPVO patch points -> /dpvo/map_points (frame=map)."""
        if self.cloud_pub.get_subscription_count() == 0:
            return
        pts = self.slam.pg.points_.detach().cpu().numpy()[:int(self.slam.m)]
        if pts.size == 0:
            return
        pts = pts.astype(np.float32) * self.scale
        # Move the world points into the anchored map frame (same C * scale similarity).
        R = self._anchor[:3, :3]
        tvec = self._anchor[:3, 3]
        pts = (pts @ R.T) + tvec

        header = Header(stamp=stamp, frame_id=self.map_frame)
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.is_dense = True
        msg.data = np.ascontiguousarray(pts[:, :3], dtype=np.float32).tobytes()
        self.cloud_pub.publish(msg)


def main():
    rclpy.init()
    node = DpvoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
