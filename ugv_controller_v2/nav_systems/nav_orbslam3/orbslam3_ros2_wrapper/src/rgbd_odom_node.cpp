/**
 * ORB-SLAM3 (RGB-D) -> ROS 2 map->odom bridge — the localizer of nav_orbslam3 (v2).
 *
 * Wraps the ORB_SLAM3 source in projects/slam_source/ORB_SLAM3 DIRECTLY (links its
 * libORB_SLAM3.so and calls ORB_SLAM3::System::TrackRGBD, exactly like the upstream
 * ROS example Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc) and turns its camera
 * trajectory into the map->odom transform Nav2 consumes — the same role slam_toolbox
 * plays in nav_lidar and RTAB-Map plays in nav_rgb. ORB-SLAM3 itself is the localizer:
 * ORB feature tracking, local bundle adjustment, DBoW2 loop closure. RGB-D makes it
 * metric (no monocular scale ambiguity).
 *
 * Per synchronized (RGB, depth) pair on the /ugv_sim/oak topics:
 *   1. TrackRGBD(bgr, depth_metres, t)  ->  Sophus::SE3f  Tcw  (world->camera)
 *   2. Twc = Tcw.inverse()  (camera pose in the ORB-SLAM3 world frame P)
 *   3. map->odom = C * (Twc * inv(T_base_cam) * inv(T_odom_base))   where C anchors
 *      map==odom at the first tracked frame, T_base_cam is the camera-optical TF that
 *      the baked USD publishes, and T_odom_base is Isaac's odometry.
 *   4. broadcast map->odom (TF) and publish the tracked map points as
 *      /orbslam3/map_points (frame=map) for visual_map_builder.py -> /map.
 *
 * The anchor C makes step 3 convention-safe: T_map_odom = inv(T_P_odom(0)) * T_P_odom(t)
 * = T_odom(0)_odom(t), so the arbitrary orientation of ORB-SLAM3's world frame P cancels
 * out. Only T_base_cam has to be right, and that comes from Isaac's live /tf.
 *
 * DIFFERENCES FROM THE v1 NODE (ugv_controller/common/nav_systems/nav_orbslam3):
 *   * Intrinsics are read from the live camera_info topic and the ORB-SLAM3 settings
 *     file is GENERATED from them at startup (settings:="" , the default). The v2 rover's
 *     camera is an RSD455 reference pulled from the Omniverse asset server, so its fx/fy
 *     are not knowable offline — and a wrong fx is the #1 cause of ORB-SLAM3 garbage.
 *     nav_rgb (rtabmap) and nav_depth (depth_image_proc) already take their intrinsics
 *     from this same camera_info; this node now does too. Pass settings:=/path/to.yaml
 *     to override with a static file instead (the v1 behavior).
 *   * RGB is converted to bgr8 by cv_bridge (so rgb8/rgba8/mono8 all work) and the
 *     generated settings therefore declare Camera.RGB: 0.
 *   * Depth is sanitized: Isaac emits inf for "no return", which would otherwise become
 *     map points at infinity. Non-finite and out-of-band values are set to 0 (= invalid).
 *   * Frames default to the v2 baked-USD names (see ugv_controller_v2/USD_CONTRACT.md).
 *
 * REQUIRES a one-time build of libORB_SLAM3.so + this node — see ../build.sh.
 */

#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "System.h"
#include "MapPoint.h"

using std::placeholders::_1;
using std::placeholders::_2;
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using ApproxSync =
    message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

class OrbSlam3RgbdNode : public rclcpp::Node
{
public:
  OrbSlam3RgbdNode() : Node("orbslam3_rgbd_bridge")
  {
    // ── Parameters ────────────────────────────────────────────────────────────
    vocab_ = declare_parameter<std::string>(
        "vocabulary",
        "/home/joshroq/projects/slam_source/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    // Empty (the default) => generate the settings from the live camera_info.
    settings_ = declare_parameter<std::string>("settings", "");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/ugv_sim/oak/rgb/image_rect");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/ugv_sim/oak/stereo/image_raw");
    info_topic_ = declare_parameter<std::string>("camera_info_topic",
                                                 "/ugv_sim/oak/rgb/camera_info");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/orbslam3/map_points");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    // The baked USD's RGB optical frame (USD_CONTRACT.md); it is on /tf, so — unlike
    // v1 — no static_transform_publisher is needed: Isaac owns the whole TF tree.
    cam_frame_ = declare_parameter<std::string>("camera_optical_frame",
                                                "Camera_OmniVision_OV9782_Color");
    use_viewer_ = declare_parameter<bool>("use_viewer", false);
    // Localization outputs. Two modes:
    //   * standalone (default): broadcast map->odom TF directly (publish_tf=true), as
    //     before — ORB-SLAM3 is the sole map->odom source.
    //   * fused: publish_tf=false + publish_pose=true. ORB-SLAM3 instead emits an
    //     ABSOLUTE map-frame pose on pose_topic (nav_msgs/Odometry, child=base_frame),
    //     and a robot_localization EKF fuses it with lidar/wheel odometry to OWN
    //     map->odom. This keeps the pose alive when visual tracking drops (e.g. in-place
    //     rotation), where a TF-only ORB-SLAM3 would freeze map->odom. See the launch.
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_pose_ = declare_parameter<bool>("publish_pose", false);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/orbslam3/pose");
    // Diagonal covariance stamped on the published pose (variance: m^2 / rad^2). Modest,
    // so the EKF corrects visual drift without yanking the estimate on every update.
    pose_cov_xy_ = declare_parameter<double>("pose_cov_xy", 0.02);
    pose_cov_yaw_ = declare_parameter<double>("pose_cov_yaw", 0.02);
    // Wide sync queue: Isaac's camera_info/RGB run much faster than the depth frames,
    // so the matching stamp for a depth frame can sit well back in the RGB buffer
    // (same reason nav_depth uses queue_size 30).
    sync_queue_ = declare_parameter<int>("sync_queue", 30);
    // Depth validity band (m). Isaac publishes inf where nothing was hit.
    depth_min_ = declare_parameter<double>("depth_min", 0.1);
    depth_max_ = declare_parameter<double>("depth_max", 10.0);

    // Settings-generation knobs (used only when `settings` is empty).
    gen_.fps = declare_parameter<int>("camera_fps", 10);
    gen_.baseline = declare_parameter<double>("stereo_baseline", 0.075);
    gen_.th_depth = declare_parameter<double>("stereo_th_depth", 40.0);
    gen_.n_features = declare_parameter<int>("orb_features", 1200);
    gen_.scale_factor = declare_parameter<double>("orb_scale_factor", 1.2);
    gen_.n_levels = declare_parameter<int>("orb_levels", 8);
    gen_.ini_th_fast = declare_parameter<int>("orb_ini_th_fast", 20);
    gen_.min_th_fast = declare_parameter<int>("orb_min_th_fast", 7);
    gen_.out_path = declare_parameter<std::string>("generated_settings_path", "");

    // ── Atlas map persistence (thesis Experiment A: known-map localization) ──────
    // ORB-SLAM3 loads/saves its multi-map "atlas" via the settings-file keys
    // System.LoadAtlasFromFile / System.SaveAtlasToFile (base path, no extension; it
    // reads/writes "<path>.osa"). We inject those into the GENERATED settings so the
    // default camera_info-driven flow works. localization_mode calls
    // System::ActivateLocalizationMode() so ORB-SLAM3 stops adding keyframes and only
    // relocalizes+tracks against the loaded atlas. All default empty/false = the
    // original live-mapping behavior, so nothing changes unless you opt in.
    //   mapping run       : save_atlas:=<base>            (writes <base>.osa on exit)
    //   localization run  : load_atlas:=<base> localization_mode:=true
    load_atlas_ = declare_parameter<std::string>("load_atlas", "");
    save_atlas_ = declare_parameter<std::string>("save_atlas", "");
    localization_mode_ = declare_parameter<bool>("localization_mode", false);

    // ── TF ────────────────────────────────────────────────────────────────────
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // ── I/O (best-effort sensor QoS to match Isaac's publishers) ──────────────
    auto qos = rclcpp::SensorDataQoS();
    rgb_sub_.subscribe(this, rgb_topic_, qos.get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic_, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(
        ApproxSync(sync_queue_), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&OrbSlam3RgbdNode::grabRGBD, this, _1, _2));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 2);
    if (publish_pose_) {
      pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(pose_topic_, 10);
    }

    if (settings_.empty()) {
      // Defer ORB-SLAM3 construction until the intrinsics arrive. Callbacks are
      // serialized on the single-threaded executor, so no locking is needed here.
      RCLCPP_INFO(get_logger(),
                  "Waiting for camera_info on %s to generate the ORB-SLAM3 settings "
                  "(pass settings:=<file.yaml> to use a static one instead)...",
                  info_topic_.c_str());
      info_sub_ = create_subscription<CameraInfoMsg>(
          info_topic_, qos, std::bind(&OrbSlam3RgbdNode::onCameraInfo, this, _1));
    } else {
      startSlam(settings_);
    }

    RCLCPP_INFO(get_logger(), "orbslam3_rgbd_bridge up: %s + %s -> %s->%s TF + %s",
                rgb_topic_.c_str(), depth_topic_.c_str(),
                map_frame_.c_str(), odom_frame_.c_str(), cloud_topic_.c_str());
  }

  ~OrbSlam3RgbdNode() override
  {
    if (slam_) slam_->Shutdown();
  }

private:
  struct GenSettings
  {
    int fps;
    double baseline;
    double th_depth;
    int n_features;
    double scale_factor;
    int n_levels;
    int ini_th_fast;
    int min_th_fast;
    std::string out_path;
  };

  // ── Settings generation from the live camera_info ───────────────────────────
  void onCameraInfo(const CameraInfoMsg::ConstSharedPtr & info)
  {
    if (slam_) return;  // already built (one-shot)

    const double fx = info->k[0], fy = info->k[4];
    const double cx = info->k[2], cy = info->k[5];
    if (fx <= 0.0 || fy <= 0.0 || info->width == 0 || info->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "camera_info on %s is not usable yet (fx=%.1f, %ux%u); waiting.",
                           info_topic_.c_str(), fx, info->width, info->height);
      return;
    }

    // plumb_bob: d = [k1, k2, t1, t2, k3]; Isaac's pinhole is distortion-free.
    auto d = [&info](size_t i) { return info->d.size() > i ? info->d[i] : 0.0; };

    const std::string path = gen_.out_path.empty()
        ? "/tmp/orbslam3_isaac_settings_" + std::to_string(getpid()) + ".yaml"
        : gen_.out_path;

    std::ofstream f(path);
    if (!f) {
      RCLCPP_FATAL(get_logger(), "Cannot write generated ORB-SLAM3 settings: %s", path.c_str());
      throw std::runtime_error("cannot write generated settings");
    }
    f << std::fixed << std::setprecision(6);
    f << "%YAML:1.0\n"
      << "# GENERATED by orbslam3_rgbd_bridge from " << info_topic_ << " — do not edit.\n"
      << "File.version: \"1.0\"\n\n"
      << "Camera.type: \"PinHole\"\n"
      << "Camera1.fx: " << fx << "\n"
      << "Camera1.fy: " << fy << "\n"
      << "Camera1.cx: " << cx << "\n"
      << "Camera1.cy: " << cy << "\n"
      << "Camera1.k1: " << d(0) << "\n"
      << "Camera1.k2: " << d(1) << "\n"
      << "Camera1.p1: " << d(2) << "\n"
      << "Camera1.p2: " << d(3) << "\n"
      << "Camera1.k3: " << d(4) << "\n"
      << "Camera.width: " << info->width << "\n"
      << "Camera.height: " << info->height << "\n"
      << "Camera.fps: " << gen_.fps << "\n"
      // We hand ORB-SLAM3 a BGR image (cv_bridge converts whatever Isaac sends).
      << "Camera.RGB: 0\n\n"
      << "Stereo.ThDepth: " << gen_.th_depth << "\n"
      << "Stereo.b: " << gen_.baseline << "\n"
      // Isaac publishes depth as 32FC1 in METRES -> no rescaling.
      << "RGBD.DepthMapFactor: 1.0\n\n"
      << "ORBextractor.nFeatures: " << gen_.n_features << "\n"
      << "ORBextractor.scaleFactor: " << gen_.scale_factor << "\n"
      << "ORBextractor.nLevels: " << gen_.n_levels << "\n"
      << "ORBextractor.iniThFAST: " << gen_.ini_th_fast << "\n"
      << "ORBextractor.minThFAST: " << gen_.min_th_fast << "\n\n"
      // ORB-SLAM3's Settings parser reads the Viewer block unconditionally.
      << "Viewer.KeyFrameSize: 0.05\n"
      << "Viewer.KeyFrameLineWidth: 1.0\n"
      << "Viewer.GraphLineWidth: 0.9\n"
      << "Viewer.PointSize: 2.0\n"
      << "Viewer.CameraSize: 0.08\n"
      << "Viewer.CameraLineWidth: 3.0\n"
      << "Viewer.ViewpointX: 0.0\n"
      << "Viewer.ViewpointY: -0.7\n"
      << "Viewer.ViewpointZ: -1.8\n"
      << "Viewer.ViewpointF: 500.0\n";
    // Atlas persistence keys (base path, no extension -> ORB-SLAM3 uses "<path>.osa").
    // Only emitted when opted in; absent by default so the map is built live as before.
    if (!load_atlas_.empty())
      f << "\nSystem.LoadAtlasFromFile: \"" << load_atlas_ << "\"\n";
    if (!save_atlas_.empty())
      f << "System.SaveAtlasToFile: \"" << save_atlas_ << "\"\n";
    f.close();

    RCLCPP_INFO(get_logger(),
                "Generated ORB-SLAM3 settings from %s: %ux%u fx=%.2f fy=%.2f cx=%.2f cy=%.2f -> %s",
                info_topic_.c_str(), info->width, info->height, fx, fy, cx, cy, path.c_str());

    startSlam(path);
    info_sub_.reset();  // one-shot: intrinsics are static
  }

  void startSlam(const std::string & settings_file)
  {
    // Atlas keys live in the settings file. We inject them into the GENERATED settings;
    // with a hand-passed settings:=<file> you must add System.LoadAtlasFromFile /
    // SaveAtlasToFile yourself, so warn rather than silently ignore the request.
    if (!settings_.empty() && (!load_atlas_.empty() || !save_atlas_.empty())) {
      RCLCPP_WARN(get_logger(),
                  "load_atlas/save_atlas are only injected into the GENERATED settings "
                  "(settings:=''). With an explicit settings file, add "
                  "System.LoadAtlasFromFile / System.SaveAtlasToFile to it yourself.");
    }
    RCLCPP_INFO(get_logger(), "Loading ORB-SLAM3 vocabulary + settings (RGB-D)%s...",
                localization_mode_ ? " [LOCALIZATION MODE]" : "");
    slam_ = std::make_unique<ORB_SLAM3::System>(
        vocab_, settings_file, ORB_SLAM3::System::RGBD, use_viewer_);
    if (localization_mode_) {
      // Stop local mapping / loop-closing map growth: pure relocalization + tracking
      // against the loaded atlas (Experiment A). Takes effect on the next TrackRGBD.
      slam_->ActivateLocalizationMode();
      RCLCPP_INFO(get_logger(), "ORB-SLAM3 localization mode ON: relocalizing in the "
                                "loaded atlas (no new keyframes).");
    }
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 running. Drive the rover to initialize tracking.");
  }

  // ── Tracking ────────────────────────────────────────────────────────────────
  void grabRGBD(const ImageMsg::ConstSharedPtr & rgb, const ImageMsg::ConstSharedPtr & depth)
  {
    if (!slam_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Images arriving but ORB-SLAM3 is not started yet "
                           "(still waiting on camera_info from %s).", info_topic_.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr cv_rgb;
    cv_bridge::CvImageConstPtr cv_depth;
    try {
      // Force BGR: Isaac may publish rgb8 or rgba8; the generated settings say RGB: 0.
      cv_rgb = cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::BGR8);
      cv_depth = cv_bridge::toCvShare(depth);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat depth_m = toMetres(cv_depth->image);
    if (depth_m.empty()) return;

    const double t = rclcpp::Time(rgb->header.stamp).seconds();
    Sophus::SE3f Tcw = slam_->TrackRGBD(cv_rgb->image, depth_m, t);

    if (slam_->GetTrackingState() != 2 /* OK */) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "ORB-SLAM3 not tracking (state=%d): initializing or lost. "
                           "Drive slowly through textured scenery to (re)initialize.",
                           slam_->GetTrackingState());
      return;  // not tracking -> don't publish a bad pose
    }

    // Camera pose in the ORB-SLAM3 world frame P.
    Eigen::Matrix4f Twc = Tcw.inverse().matrix();
    publishLocalization(Twc, rgb->header.stamp);
    publishMapPoints(rgb->header.stamp);
  }

  /// Depth -> CV_32FC1 metres with invalid pixels zeroed. Isaac sends 32FC1 metres and
  /// uses inf for "no return"; ORB-SLAM3 treats <=0 as "no depth", but would happily
  /// turn an inf into a map point at infinity, so the band filter is not optional.
  cv::Mat toMetres(const cv::Mat & src)
  {
    cv::Mat d;
    if (src.type() == CV_32FC1) {
      d = src.clone();
    } else if (src.type() == CV_16UC1) {
      src.convertTo(d, CV_32FC1, 1.0 / 1000.0);  // mm -> m, if Isaac is ever reconfigured
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Unsupported depth image type %d (want 32FC1 or 16UC1)", src.type());
      return cv::Mat();
    }
    // NaN/inf compare false, so they fall outside `valid` and get zeroed with the
    // out-of-band pixels.
    cv::Mat valid = (d > static_cast<float>(depth_min_)) & (d < static_cast<float>(depth_max_));
    d.setTo(0.0f, ~valid);
    return d;
  }

  // ── map->odom ───────────────────────────────────────────────────────────────
  // T_map_odom = C * (Twc * inv(T_base_cam) * inv(T_odom_base))
  void publishLocalization(const Eigen::Matrix4f & Twc, const rclcpp::Time & stamp)
  {
    Eigen::Matrix4f T_base_cam, T_odom_base;
    try {
      auto tf_bc = tf_buffer_->lookupTransform(base_frame_, cam_frame_, tf2::TimePointZero);
      auto tf_ob = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      T_base_cam = tf2::transformToEigen(tf_bc).matrix().cast<float>();
      T_odom_base = tf2::transformToEigen(tf_ob).matrix().cast<float>();
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", e.what());
      return;
    }

    Eigen::Matrix4f T_P_base = Twc * T_base_cam.inverse();
    Eigen::Matrix4f T_P_odom = T_P_base * T_odom_base.inverse();

    if (!anchored_) {
      anchor_ = T_P_odom.inverse();  // makes map==odom at the first tracked frame
      anchored_ = true;
      RCLCPP_INFO(get_logger(), "ORB-SLAM3 tracking OK — anchored map==odom, publishing %s->%s.",
                  map_frame_.c_str(), odom_frame_.c_str());
    }
    Eigen::Matrix4f T_map_odom = anchor_ * T_P_odom;

    // Standalone mode: ORB-SLAM3 is the sole map->odom source and broadcasts it.
    // Fused mode (publish_tf=false): the EKF owns map->odom, so we stay off TF and only
    // emit the absolute pose below — otherwise two nodes would fight over map->odom.
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = T_map_odom(0, 3);
      tf.transform.translation.y = T_map_odom(1, 3);
      tf.transform.translation.z = T_map_odom(2, 3);
      Eigen::Quaternionf q(T_map_odom.block<3, 3>(0, 0));
      q.normalize();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }

    // Absolute base pose in the map frame, for the EKF to fuse (differential=false).
    // T_map_base = T_map_odom * T_odom_base = anchor * T_P_base.
    if (publish_pose_ && pose_pub_) {
      Eigen::Matrix4f T_map_base = anchor_ * T_P_base;
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = map_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = T_map_base(0, 3);
      odom.pose.pose.position.y = T_map_base(1, 3);
      odom.pose.pose.position.z = T_map_base(2, 3);
      Eigen::Quaternionf qb(T_map_base.block<3, 3>(0, 0));
      qb.normalize();
      odom.pose.pose.orientation.x = qb.x();
      odom.pose.pose.orientation.y = qb.y();
      odom.pose.pose.orientation.z = qb.z();
      odom.pose.pose.orientation.w = qb.w();
      // Row-major 6x6 pose covariance (x,y,z,roll,pitch,yaw). Only x,y,yaw are used by
      // the two_d_mode EKF; the unused axes get a large variance so they never pull it.
      odom.pose.covariance[0]  = pose_cov_xy_;   // x
      odom.pose.covariance[7]  = pose_cov_xy_;   // y
      odom.pose.covariance[14] = 1e6;            // z
      odom.pose.covariance[21] = 1e6;            // roll
      odom.pose.covariance[28] = 1e6;            // pitch
      odom.pose.covariance[35] = pose_cov_yaw_;  // yaw
      pose_pub_->publish(odom);
    }
  }

  void publishMapPoints(const rclcpp::Time & stamp)
  {
    if (cloud_pub_->get_subscription_count() == 0 || !anchored_) return;

    std::vector<ORB_SLAM3::MapPoint *> mps = slam_->GetTrackedMapPoints();
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(mps.size());
    for (auto * mp : mps) {
      if (mp == nullptr || mp->isBad()) continue;
      Eigen::Vector3f p = anchor_.block<3, 3>(0, 0) * mp->GetWorldPos() +
                          anchor_.block<3, 1>(0, 3);
      pts.push_back(p);
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(pts.size());
    msg.is_dense = true;
    msg.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    for (const auto & p : pts) {
      *ix = p.x(); *iy = p.y(); *iz = p.z();
      ++ix; ++iy; ++iz;
    }
    cloud_pub_->publish(msg);
  }

  // ── Members ─────────────────────────────────────────────────────────────────
  std::string vocab_, settings_, rgb_topic_, depth_topic_, info_topic_, cloud_topic_;
  std::string map_frame_, odom_frame_, base_frame_, cam_frame_, pose_topic_;
  bool use_viewer_{false};
  bool publish_tf_{true}, publish_pose_{false};
  int sync_queue_{30};
  double depth_min_{0.1}, depth_max_{10.0};
  double pose_cov_xy_{0.02}, pose_cov_yaw_{0.02};
  GenSettings gen_{};
  std::string load_atlas_, save_atlas_;
  bool localization_mode_{false};

  std::unique_ptr<ORB_SLAM3::System> slam_;

  message_filters::Subscriber<ImageMsg> rgb_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync>> sync_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Eigen::Matrix4f anchor_{Eigen::Matrix4f::Identity()};
  bool anchored_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbSlam3RgbdNode>());
  rclcpp::shutdown();
  return 0;
}
