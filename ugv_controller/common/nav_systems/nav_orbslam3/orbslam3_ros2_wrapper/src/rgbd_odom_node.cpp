/**
 * ORB-SLAM3 (RGB-D) -> ROS 2 map->odom bridge — UNIQUE to nav_orbslam3.
 *
 * Wraps the ORB_SLAM3 source in projects/slam_source/ORB_SLAM3 DIRECTLY (links its
 * libORB_SLAM3.so and calls ORB_SLAM3::System::TrackRGBD, exactly like the upstream
 * ROS example Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc) and turns its camera
 * trajectory into the map->odom transform the Nav2 stack consumes — the same role
 * slam_toolbox plays in nav_lidar and RTAB-Map plays in nav_rgb. So this is a "true"
 * ORB-SLAM3 navigation system: ORB-SLAM3 itself is the localizer, running RGB-D with
 * ORB feature tracking, local BA, and DBoW2 loop closure.
 *
 * Per synchronized (RGB, depth) pair on the /oak topics:
 *   1. TrackRGBD(rgb, depth_metres, t)  ->  Sophus::SE3f  Tcw  (world->camera)
 *   2. Twc = Tcw.inverse()  (camera pose in the ORB-SLAM3 world frame P)
 *   3. map->odom = C * (Twc * inv(T_base_cam) * inv(T_odom_base))   where C anchors
 *      map==odom at the first tracked frame, T_base_cam is the static camera-optical
 *      TF, and T_odom_base is Isaac's odometry.
 *   4. broadcast map->odom (TF) and publish the tracked map points as
 *      /orbslam3/map_points (frame=map) for common/visual_map_builder.py -> /map.
 *
 * ORB-SLAM3's world frame uses the camera-optical convention (z forward), matching
 * oak_rgb_camera_optical_frame, so Twc drops straight onto the robot via the static
 * TF. If the first run comes out mirrored/rotated, that static camera TF (in the
 * launch) is the thing to adjust — never the URDF.
 *
 * REQUIRES a one-time build of libORB_SLAM3.so + this node — see ../build.sh.
 */

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
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
    settings_ = declare_parameter<std::string>("settings", "");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/oak/rgb/image_rect");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/oak/stereo/image_raw");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/orbslam3/map_points");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    cam_frame_ = declare_parameter<std::string>("camera_optical_frame",
                                                "oak_rgb_camera_optical_frame");
    use_viewer_ = declare_parameter<bool>("use_viewer", false);

    if (settings_.empty()) {
      RCLCPP_FATAL(get_logger(), "The 'settings' parameter (ORB-SLAM3 .yaml) is required.");
      throw std::runtime_error("missing settings");
    }

    // ── ORB-SLAM3 system (RGB-D) ──────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Loading ORB-SLAM3 vocabulary + settings (RGB-D)...");
    slam_ = std::make_unique<ORB_SLAM3::System>(
        vocab_, settings_, ORB_SLAM3::System::RGBD, use_viewer_);

    // ── TF ────────────────────────────────────────────────────────────────────
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // ── I/O (best-effort sensor QoS to match Isaac's publishers) ──────────────
    auto qos = rclcpp::SensorDataQoS();
    rgb_sub_.subscribe(this, rgb_topic_, qos.get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic_, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(
        ApproxSync(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&OrbSlam3RgbdNode::grabRGBD, this, _1, _2));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 2);

    RCLCPP_INFO(get_logger(), "orbslam3_rgbd_bridge up: %s + %s -> %s->%s TF + %s",
                rgb_topic_.c_str(), depth_topic_.c_str(),
                map_frame_.c_str(), odom_frame_.c_str(), cloud_topic_.c_str());
  }

  ~OrbSlam3RgbdNode() override
  {
    if (slam_) slam_->Shutdown();
  }

private:
  void grabRGBD(const ImageMsg::ConstSharedPtr & rgb, const ImageMsg::ConstSharedPtr & depth)
  {
    cv_bridge::CvImageConstPtr cv_rgb, cv_depth;
    try {
      cv_rgb = cv_bridge::toCvShare(rgb);
      cv_depth = cv_bridge::toCvShare(depth);  // Isaac depth is 32FC1 in metres
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const double t = rclcpp::Time(rgb->header.stamp).seconds();
    Sophus::SE3f Tcw = slam_->TrackRGBD(cv_rgb->image, cv_depth->image, t);

    if (slam_->GetTrackingState() != 2 /* OK */) {
      return;  // not tracking (initialising / lost) -> don't publish a bad pose
    }

    // Camera pose in the ORB-SLAM3 world frame P.
    Eigen::Matrix4f Twc = Tcw.inverse().matrix();
    publishLocalization(Twc, rgb->header.stamp);
    publishMapPoints(rgb->header.stamp);
  }

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
    }
    Eigen::Matrix4f T_map_odom = anchor_ * T_P_odom;

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
  std::string vocab_, settings_, rgb_topic_, depth_topic_, cloud_topic_;
  std::string map_frame_, odom_frame_, base_frame_, cam_frame_;
  bool use_viewer_{false};

  std::unique_ptr<ORB_SLAM3::System> slam_;

  message_filters::Subscriber<ImageMsg> rgb_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

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
