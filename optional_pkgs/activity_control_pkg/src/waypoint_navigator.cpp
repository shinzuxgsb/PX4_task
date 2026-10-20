#include "activity_control_pkg/waypoint_navigator.hpp"

#include <angles/angles.h>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

WaypointNavigator::WaypointNavigator(
  rclcpp::Node & node,
  const std::string & output_topic,
  const std::string & fastlio_world_frame,
  const std::string & fastlio_body_frame,
  double pos_tol_cm,
  double yaw_tol_deg,
  double height_tol_cm,
  int reach_hold_cycles,
  double target_republish_period_sec)
: node_(node),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0),
  pos_tol_cm_(pos_tol_cm),
  yaw_tol_deg_(yaw_tol_deg),
  height_tol_cm_(height_tol_cm),
  reach_hold_cycles_(std::max(1, reach_hold_cycles)),
  reach_stable_count_(0),
  mission_completed_(false),
  land_sentinel_published_(false),
  target_republish_period_sec_(std::max(0.0, target_republish_period_sec)),
  last_target_publish_time_(0, 0, RCL_ROS_TIME),
  fastlio_world_frame_(fastlio_world_frame),
  fastlio_body_frame_(fastlio_body_frame)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = node_.create_publisher<std_msgs::msg::Float32MultiArray>(output_topic, qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void WaypointNavigator::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    mission_completed_ = false;
    land_sentinel_published_ = false;
    reach_stable_count_ = 0;
    current_idx_ = 0;
    publishCurrent();
  }
}

void WaypointNavigator::addTarget(double x_cm, double y_cm, double z_cm, double yaw_deg)
{
  Target target{};
  target.x_cm = x_cm;
  target.y_cm = y_cm;
  target.z_cm = z_cm;
  target.yaw_deg = yaw_deg;
  addTarget(target);
}

std::size_t WaypointNavigator::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t WaypointNavigator::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void WaypointNavigator::updateHeight(double height_cm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_height_cm_ = height_cm;
  has_height_ = true;
}

void WaypointNavigator::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (mission_completed_) {
    if (!land_sentinel_published_) {
      publishLandingSentinel();
      return;
    }

    if (target_republish_period_sec_ > 0.0) {
      const double elapsed = (node_.now() - last_target_publish_time_).seconds();
      if (elapsed >= target_republish_period_sec_) {
        publishLandingSentinel();
      }
    }
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  if (current_idx_ >= targets_.size()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "当前目标索引越界，等待新的航点输入");
    return;
  }

  if (target_republish_period_sec_ > 0.0) {
    const double elapsed = (node_.now() - last_target_publish_time_).seconds();
    if (elapsed >= target_republish_period_sec_) {
      publishCurrent();
    }
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  RCLCPP_INFO_THROTTLE(
    node_.get_logger(), *node_.get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg);

  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    reach_stable_count_++;
    if (reach_stable_count_ >= reach_hold_cycles_) {
      RCLCPP_INFO(node_.get_logger(), "目标 %zu 已完成", current_idx_);
      reach_stable_count_ = 0;
      current_idx_++;
      if (current_idx_ < targets_.size()) {
        publishCurrent();
      } else {
        mission_completed_ = true;
        current_idx_ = targets_.size();
        land_sentinel_published_ = false;
        RCLCPP_INFO(node_.get_logger(), "所有目标已完成");
      }
    }
  } else {
    reach_stable_count_ = 0;
  }
}

void WaypointNavigator::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

void WaypointNavigator::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);
  last_target_publish_time_ = node_.now();

  RCLCPP_INFO(node_.get_logger(),
    "发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    init_flag ? " (首个)" : "");
}

void WaypointNavigator::publishLandingSentinel()
{
  Target landing{};
  landing.x_cm = -1.0;
  landing.y_cm = -1.0;
  landing.z_cm = -1.0;
  landing.yaw_deg = -1.0;
  publishTarget(landing, false);
  land_sentinel_published_ = true;
  RCLCPP_INFO(node_.get_logger(), "航点完成，发布降落哨兵目标 [-1,-1,-1,-1]");
}

bool WaypointNavigator::getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      fastlio_world_frame_, fastlio_body_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "TF 查询失败 (%s->%s): %s",
      fastlio_world_frame_.c_str(), fastlio_body_frame_.c_str(), ex.what());
    return false;
  }
}

bool WaypointNavigator::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  const bool z_ok = (std::fabs(dz) <= height_tol_cm_);
  const bool xy_ok = (dxy <= pos_tol_cm_);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  return z_ok && xy_ok && yaw_ok;
}

double WaypointNavigator::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double WaypointNavigator::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double WaypointNavigator::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

}  // namespace activity_control_pkg
