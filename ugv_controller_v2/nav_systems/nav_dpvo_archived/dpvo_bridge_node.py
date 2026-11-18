#!/usr/bin/env python3
"""
DPVO (Deep Patch Visual Odometry) -> ROS 2 map->odom bridge — the localizer of nav_dpvo.

Wraps the DPVO source in projects/slam_source/DPVO-main DIRECTLY: it imports the `dpvo`
package, loads the trained `dpvo.pth` network, and runs the same
`DPVO(cfg, net, ht, wd, viz=...)` / `slam(t, image, intrinsics)` loop that DPVO's own
demo.py runs. Its camera trajectory becomes the map->odom transform Nav2 consumes — the
exact role slam_toolbox plays in nav_lidar and RTAB-Map plays in nav_rgb.

Pipeline (per RGB frame on /ugv_sim/oak/rgb/image_rect):
  1. feed the frame + intrinsics (from /ugv_sim/oak/rgb/camera_info) to DPVO
  2. read DPVO's latest camera pose  T_world_cam  (camera-optical convention)
  3. DPVO is MONOCULAR, so its world is up-to-SCALE. Recover the metric scale online by
     matching DPVO's travelled distance against Isaac's /odom, and apply it.
  4. map->odom = C * (T_map_cam * inv(T_base_cam) * inv(T_odom_base)), where C anchors
     map==odom at the first good frame, T_base_cam is the camera-optical TF the baked USD
     publishes, and T_odom_base is Isaac's odometry.
  5. broadcast map->odom (TF) and publish the scaled patch points as /dpvo/map_points
     (frame=map) for common/visual_map_builder.py -> /map.

This is the deep/learned counterpart of nav_orbslam3: DPVO tracks dense *patches* with a
trained recurrent network instead of matching hand-crafted ORB features, and it has no
loop closure by default — so it is a visual ODOMETRY, and it drifts. The monocular scale
step is what nav_orbslam3 gets for free from RGB-D.

NATIVE VISUALIZATION: with viz:=true (the default) DPVO opens its OWN Pangolin window —
DPViewer, from DPVO-main/DPViewer — showing the live image, the tracked patches and the
3D trajectory + patch cloud, exactly as `demo.py --viz` does. It is created inside DPVO
itself (DPVO.start_viewer() -> `from dpviewer import Viewer`) on the live CUDA tensors,
and runs alongside RViz.

RUNS IN THE `dpvo` CONDA ENV, not system python: DPVO needs torch + CUDA + its compiled
extensions. That env is python 3.10 — the same minor as ROS Humble — so /opt/ros's rclpy,
tf2_ros and message packages import straight into it once ROS is sourced. So this stays
one ordinary ROS node (conda torch + system rclpy in a single process); the launch file
just points the process at that interpreter.
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

try:  # the concrete error raised when publishing into a torn-down context
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # pragma: no cover - older rclpy layouts
    class RCLError(Exception):
        pass

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
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
        self.declare_parameter('network', 'dpvo.pth')            # relative to dpvo_path if not absolute
        self.declare_parameter('config', 'config/default.yaml')  # relative to dpvo_path
        # v2 baked-USD topics/frames (see ugv_controller_v2/USD_CONTRACT.md).
        self.declare_parameter('rgb_topic', '/ugv_sim/oak/rgb/image_rect')
        self.declare_parameter('info_topic', '/ugv_sim/oak/rgb/camera_info')
        self.declare_parameter('cloud_topic', '/dpvo/map_points')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_optical_frame', 'Camera_OmniVision_OV9782_Color')
        self.declare_parameter('stride', 1)              # process every Nth frame (Isaac's camera is slow)
        self.declare_parameter('warmup_frames', 8)       # DPVO needs a few frames to initialise
        self.declare_parameter('scale', 0.0)             # >0 = fixed metric scale; 0 = estimate online
        self.declare_parameter('min_scale_travel', 0.5)  # m of odom motion before trusting the scale
        self.declare_parameter('invert_pose', False)     # flip if the trajectory comes out mirrored
        # DPVO's OWN Pangolin viewer (DPViewer): live image + patches + 3D trajectory.
        self.declare_parameter('viz', True)

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.cam_frame = gp('camera_optical_frame').value
        self.stride = max(1, int(gp('stride').value))
        self.warmup = int(gp('warmup_frames').value)
        self.fixed_scale = float(gp('scale').value)
        self.min_scale_travel = float(gp('min_scale_travel').value)
        self.invert_pose = bool(gp('invert_pose').value)
        self.viz = bool(gp('viz').value)

        # ── DPVO import + network (from the slam_source tree, directly) ───────────
        dpvo_path = gp('dpvo_path').value
        if dpvo_path not in sys.path:
            sys.path.insert(0, dpvo_path)
        try:
            import torch
            from dpvo.config import cfg
            from dpvo.dpvo import DPVO
        except Exception as e:
            self.get_logger().fatal(
                f"Could not import DPVO/torch from '{dpvo_path}': {e}\n"
                "nav_dpvo must run in the `dpvo` conda env (torch + CUDA + the compiled "
                "DPVO extensions). Check it with:  nav_systems/nav_dpvo/setup.sh\n"
                "The launch file runs this node with that env's interpreter.")
            raise

        if not torch.cuda.is_available():
            self.get_logger().fatal(
                "torch reports CUDA is NOT available — DPVO is CUDA-only (its cuda_corr / "
                "cuda_ba / lietorch_backends extensions run on the GPU). Check the driver "
                "and that the venv's torch is a CUDA build.")
            raise RuntimeError("DPVO requires CUDA")

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
            self.get_logger().fatal(
                f"DPVO weights not found at {self._network} — run "
                "DPVO-main/download_models_and_data.sh (setup.sh does it) or set 'network'.")
            raise FileNotFoundError(self._network)

        self.slam = None       # created lazily: DPVO needs the image size up front
        self.frame_idx = 0     # raw frame counter (for stride)
        self.dpvo_t = 0        # monotonic timestamp index fed to DPVO

        # ── Online monocular-scale + start-anchor state ──────────────────────────
        self.scale = self.fixed_scale if self.fixed_scale > 0 else 1.0
        self._dpvo_prev_xyz = None
        self._odom_prev_xyz = None
        self._dpvo_travel = 0.0
        self._odom_travel = 0.0
        self._scale_locked = False
        self._anchor = None    # C: left-multiply that makes map==odom at the first good frame

        self.K = None          # (fx, fy, cx, cy) from the first CameraInfo

        # ── TF ────────────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # map->odom is published on a fixed-rate timer (NOT only from the image callback),
        # seeded to IDENTITY so the `map` frame exists from startup — exactly like
        # nav_orbslam3. DPVO is monocular visual odometry: it only initialises once the
        # camera has actually MOVED (dpvo.py drops frames with <2px motion until 8 frames
        # accumulate). Until then it yields no pose, so without this seed nothing would
        # publish map->odom, `map` would never exist, and RViz (fixed frame = map) plus the
        # Nav2 global costmap would fail with "frame [map] does not exist" — you could not
        # even see the robot or set a goal to drive it. With the seed, map==odom at start;
        # once DPVO anchors (it forces map==odom at its init frame too) the handoff to the
        # real transform is jump-free, and _update_localization just updates the matrix the
        # timer already publishes.
        self._map_odom = np.eye(4)   # latest map->odom; identity until DPVO takes over
        self.create_timer(1.0 / 30.0, self._publish_tf)

        # ── I/O (best-effort sensor QoS to match Isaac's publishers) ─────────────
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(CameraInfo, gp('info_topic').value, self._on_info, sensor_qos)
        self.create_subscription(Image, gp('rgb_topic').value, self._on_image, sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, gp('cloud_topic').value, 2)

        self.get_logger().info(
            f"dpvo_bridge up: {gp('rgb_topic').value} -> DPVO -> "
            f"{self.map_frame}->{self.odom_frame} TF + {gp('cloud_topic').value} "
            f"(native DPViewer {'ON' if self.viz else 'off'})")

    # ── Callbacks ─────────────────────────────────────────────────────────────────
    def _on_info(self, msg: CameraInfo):
        if self.K is None and msg.k[0] > 0.0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
            self.get_logger().info(
                f"DPVO intrinsics from camera_info: fx={self.K[0]:.2f} fy={self.K[1]:.2f} "
                f"cx={self.K[2]:.2f} cy={self.K[3]:.2f}")

    def _image_to_np(self, msg: Image) -> np.ndarray:
        """ROS Image -> HxWx3 uint8 BGR (what DPVO's demo feeds it via cv2.imread)."""
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        enc = msg.encoding.lower()
        if enc in ('rgb8', 'rgb', 'rgba8'):
            buf = buf[:, :, :3][:, :, ::-1]   # -> BGR to match cv2.imread ordering
        elif enc in ('bgra8',):
            buf = buf[:, :, :3]
        elif enc in ('mono8', '8uc1'):
            buf = np.repeat(buf, 3, axis=2)
        return np.ascontiguousarray(buf[:, :, :3])

    def _on_image(self, msg: Image):
        if self.K is None:
            self.get_logger().warn(
                'Waiting for camera_info before starting DPVO...', throttle_duration_sec=5.0)
            return
        self.frame_idx += 1
        if self.frame_idx % self.stride != 0:
            return

        image = self._image_to_np(msg)
        h, w = image.shape[:2]
        h, w = h - h % 16, w - w % 16       # DPVO requires dims divisible by 16
        image = image[:h, :w]

        # Intrinsics must be cropped consistently with the image (cx/cy unchanged here,
        # since we crop from the bottom/right only).
        intr = np.array(self.K, dtype=np.float32)

        torch = self._torch
        img_t = torch.from_numpy(image).permute(2, 0, 1).cuda()
        intr_t = torch.from_numpy(intr).cuda()

        if self.slam is None:
            # viz=True makes DPVO open its OWN Pangolin viewer (DPViewer) on these tensors.
            self.slam = self._DPVO(self._cfg, self._network, ht=h, wd=w, viz=self.viz)
            self.get_logger().info(
                f"DPVO initialised ({w}x{h}). Drive the rover to build the trajectory."
                + ("  DPViewer window is live." if self.viz else ""))

        with torch.no_grad():
            self.slam(self.dpvo_t, img_t, intr_t)
        self.dpvo_t += 1

        if self.slam.n <= self.warmup:
            return  # DPVO not yet initialised — no reliable pose
        self._update_localization(msg.header)

    # ── DPVO pose -> map->odom ─────────────────────────────────────────────────────
    def _latest_pose_world_cam(self):
        """DPVO's most recent camera pose as 4x4 T_world_cam (unscaled)."""
        from dpvo.lietorch import SE3
        n = int(self.slam.n)
        # pg.poses_ store world->camera (SE3); invert for camera->world (same as terminate()).
        p = SE3(self.slam.poses[0, n - 1]).inv().data
        p = p.detach().cpu().numpy().reshape(-1)   # [tx,ty,tz, qx,qy,qz,qw]
        T = _pose_to_mat(*p[:3], *p[3:7])
        if self.invert_pose:
            T = np.linalg.inv(T)
        return T, p[:3]

    def _update_scale(self, dpvo_xyz):
        """Match DPVO's travelled distance to Isaac's /odom to recover the metric scale.

        DPVO is monocular: its translations are only defined up to a global scale factor,
        so without this the rover's map->odom would be the right shape but the wrong size.
        """
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
                if not self._scale_locked:
                    self._scale_locked = True
                    self.get_logger().info(
                        f"Monocular scale estimated: {self.scale:.4f} "
                        f"(odom travel {self._odom_travel:.2f} m / DPVO travel "
                        f"{self._dpvo_travel:.2f}). It keeps refining as you drive.")
        self._dpvo_prev_xyz = dpvo_xyz
        self._odom_prev_xyz = odom_xyz

    def _update_localization(self, header):
        T_world_cam, dpvo_xyz = self._latest_pose_world_cam()
        self._update_scale(dpvo_xyz)

        # Apply the metric scale to the (similarity) trajectory: translation only.
        T_map_cam = T_world_cam.copy()
        T_map_cam[:3, 3] *= self.scale

        # Camera optical -> base_footprint, and Isaac's odometry (both from the baked TF).
        try:
            tf_bc = self.tf_buffer.lookup_transform(
                self.base_frame, self.cam_frame, rclpy.time.Time())
            tf_ob = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            self.get_logger().warn('TF not ready (map->odom held back)',
                                   throttle_duration_sec=2.0)
            return
        T_base_cam = _tf_to_mat(tf_bc)
        T_odom_base = _tf_to_mat(tf_ob)

        T_map_base = T_map_cam @ np.linalg.inv(T_base_cam)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        # Anchor: force map==odom at the first valid frame so the rover starts sane. This
        # also cancels the arbitrary orientation of DPVO's world frame.
        if self._anchor is None:
            self._anchor = np.linalg.inv(T_map_odom)
            self.get_logger().info(
                f"DPVO tracking — anchored map==odom, publishing {self.map_frame}->"
                f"{self.odom_frame}.")
        T_map_odom = self._anchor @ T_map_odom

        # Hand the transform to the timer, which owns the map->odom broadcast. It has been
        # publishing identity since startup; from here on it publishes DPVO's estimate.
        self._map_odom = T_map_odom
        self._publish_points(header.stamp)

    def _publish_tf(self):
        """Broadcast the latest map->odom on a fixed-rate timer (identity until DPVO locks).

        Runs even before DPVO has initialised so the `map` frame exists from startup — the
        thing that lets RViz and the Nav2 costmaps come up and lets you drive the rover to
        bootstrap DPVO in the first place. Stamped with the (sim) clock, not an image stamp.
        """
        if not rclpy.ok():
            return
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return   # sim clock (/clock) not received yet — a stamp of 0 is useless to Nav2
        self._broadcast(self._map_odom, now.to_msg())

    def _broadcast(self, T, stamp):
        # On Ctrl-C rclpy tears the context down while this callback may still be in
        # flight; publishing then raises RCLError, which escapes the executor and aborts
        # the process (DPVO/DPViewer C++ threads are still running). Just stop publishing.
        if not rclpy.ok():
            return
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
        if not rclpy.ok() or self.cloud_pub.get_subscription_count() == 0 or self._anchor is None:
            return
        pts = self.slam.pg.points_.detach().cpu().numpy()[:int(self.slam.m)]
        if pts.size == 0:
            return
        pts = pts.astype(np.float32) * self.scale
        # Move the world points into the anchored map frame (the same C * scale similarity).
        R = self._anchor[:3, :3]
        tvec = self._anchor[:3, 3]
        pts = (pts @ R.T) + tvec
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return

        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        # Context torn down under a callback that was already publishing (Ctrl-C race).
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        # Hard-exit past DPViewer's GL/C++ static destructors. DPVO's own teardown
        # (slam.terminate()) is NOT usable here: it calls viewer.join(), which blocks
        # until the human closes the Pangolin window — fine for demo.py, a hang for a
        # Ctrl-C'd ROS node. Without this, the exiting interpreter tears down the live
        # Pangolin/CUDA threads and the process dies with "terminate called without an
        # active exception" (a core dump on every shutdown). We are exiting anyway.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
