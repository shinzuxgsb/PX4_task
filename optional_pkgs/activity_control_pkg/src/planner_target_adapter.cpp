#include "activity_control_pkg/planner_target_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

PlannerTargetAdapterNode::PlannerTargetAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("planner_target_adapter", options),
  prefer_position_cmd_input_(true),
  prefer_path_input_(false),
  path_sample_index_(5),
  use_input_z_(true),
  default_height_cm_(120.0),
  use_fixed_yaw_(true),
  fixed_yaw_deg_(0.0),
  republish_period_sec_(0.2),
  enable_sparse_sampling_(true),
  min_delta_xy_cm_(5.0),
  min_delta_z_cm_(3.0),
  min_delta_yaw_deg_(5.0),
  enable_adaptive_sampling_(true),
  enable_trajectory_switch_reset_(true),
  adaptive_scale_min_(0.45),
  adaptive_scale_max_(1.6),
  adaptive_turn_low_deg_(3.0),
  adaptive_turn_high_deg_(20.0),
  enable_takeoff_height_guard_(true),
  use_external_height_guard_(true),
  external_height_timeout_ms_(500),
  min_valid_target_z_cm_(30.0),
  guarded_takeoff_z_cm_(120.0),
  enable_bootstrap_takeoff_target_(true),
  bootstrap_once_(true),
  bootstrap_takeoff_height_cm_(120.0),
  gate_control_enabled_(false),
  enable_nearest_waypoint_priority_(false),
  nearest_waypoint_forward_only_(true),
  nearest_waypoint_switch_hysteresis_cm_(0.0),
  default_gate_count_(3),
  max_queue_size_(200),
  gate_pos_tol_cm_(8.0),
  gate_z_tol_cm_(6.0),
  gate_yaw_tol_deg_(10.0),
  gate_reach_hold_cycles_(2),
  gate_waiting_trigger_(false),
  gate_remaining_in_segment_(0),
  segment_total_count_(0),
  segment_completed_count_(0),
  gate_reach_stable_count_(0),
  accepted_count_(0),
  skipped_count_(0),
  adaptive_sample_count_(0),
  queued_count_(0),
  reached_count_(0),
  trigger_count_(0),
  trajectory_switch_count_(0),
  has_last_target_(false),
  has_last_velocity_(false),
  has_active_trajectory_id_(false),
  active_trajectory_id_(0),
  has_last_gate_sample_(false),
  has_current_target_(false),
  next_target_index_(0),
  last_reached_index_(-1),
  has_last_reached_target_(false),
  last_reached_time_(0, 0, RCL_ROS_TIME),
  has_pose_(false),
  has_external_height_(false),
  bootstrap_done_(false),
  external_height_cm_(0.0),
  last_external_height_time_(0, 0, RCL_ROS_TIME),
  has_last_raw_target_(false),
  has_prev_raw_target_(false),
  last_target_guard_applied_(false),
  last_height_guard_source_("none"),
  has_last_incoming_target_(false),
  has_last_accepted_target_(false)
{
  input_pose_topic_ = declare_parameter("input_pose_topic", "/ego_planner/next_target");
  input_path_topic_ = declare_parameter("input_path_topic", "/ego_planner/reference_path");
  input_position_cmd_topic_ = declare_parameter(
    "input_position_cmd_topic", "/drone_0_planning/pos_cmd");
  odom_topic_ = declare_parameter("odom_topic", "/fastlio2/lio_odom");
  external_height_topic_ = declare_parameter("external_height_topic", "/height");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  output_mode_ = declare_parameter("output_mode", std::string("position"));
  gate_trigger_topic_ = declare_parameter("gate_trigger_topic", "/waypoint_gate_trigger");
  status_topic_ = declare_parameter("status_topic", "/waypoint_status");
  std::transform(
    output_mode_.begin(), output_mode_.end(), output_mode_.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (output_mode_ != "position" && output_mode_ != "velocity") {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported output_mode=%s, fallback to position",
      output_mode_.c_str());
    output_mode_ = "position";
  }
  prefer_position_cmd_input_ = declare_parameter("prefer_position_cmd_input", true);
  prefer_path_input_ = declare_parameter("prefer_path_input", false);
  const int64_t configured_sample_index = declare_parameter<int64_t>("path_sample_index", 5);
  path_sample_index_ = static_cast<int>(std::max<int64_t>(0, configured_sample_index));
  use_input_z_ = declare_parameter("use_input_z", true);
  default_height_cm_ = declare_parameter("default_height_cm", 120.0);
  use_fixed_yaw_ = declare_parameter("use_fixed_yaw", true);
  fixed_yaw_deg_ = declare_parameter("fixed_yaw_deg", 0.0);
  republish_period_sec_ = std::max(0.0, declare_parameter("republish_period_sec", 0.2));
  enable_sparse_sampling_ = declare_parameter("enable_sparse_sampling", true);
  min_delta_xy_cm_ = std::max(0.0, declare_parameter("min_delta_xy_cm", 5.0));
  min_delta_z_cm_ = std::max(0.0, declare_parameter("min_delta_z_cm", 3.0));
  min_delta_yaw_deg_ = std::max(0.0, declare_parameter("min_delta_yaw_deg", 5.0));
  enable_adaptive_sampling_ = declare_parameter("enable_adaptive_sampling", true);
  enable_trajectory_switch_reset_ = declare_parameter("enable_trajectory_switch_reset", true);
  adaptive_scale_min_ = std::max(0.05, declare_parameter("adaptive_scale_min", 0.45));
  adaptive_scale_max_ = std::max(adaptive_scale_min_, declare_parameter("adaptive_scale_max", 1.6));
  adaptive_turn_low_deg_ = std::max(0.0, declare_parameter("adaptive_turn_low_deg", 3.0));
  adaptive_turn_high_deg_ = std::max(
    adaptive_turn_low_deg_ + 1e-3, declare_parameter("adaptive_turn_high_deg", 20.0));
  enable_takeoff_height_guard_ = declare_parameter("enable_takeoff_height_guard", true);
  use_external_height_guard_ = declare_parameter("use_external_height_guard", true);
  external_height_timeout_ms_ = std::max<int64_t>(
    0, declare_parameter<int64_t>("external_height_timeout_ms", 500));
  min_valid_target_z_cm_ = declare_parameter("min_valid_target_z_cm", 30.0);
  guarded_takeoff_z_cm_ = declare_parameter("guarded_takeoff_z_cm", 120.0);
  min_valid_target_z_cm_ = std::max(-1000.0, min_valid_target_z_cm_);
  guarded_takeoff_z_cm_ = std::max(min_valid_target_z_cm_, guarded_takeoff_z_cm_);
  enable_bootstrap_takeoff_target_ = declare_parameter("enable_bootstrap_takeoff_target", true);
  bootstrap_once_ = declare_parameter("bootstrap_once", true);
  bootstrap_takeoff_height_cm_ = declare_parameter(
    "bootstrap_takeoff_height_cm", guarded_takeoff_z_cm_);
  bootstrap_takeoff_height_cm_ = std::max(min_valid_target_z_cm_, bootstrap_takeoff_height_cm_);
  gate_control_enabled_ = declare_parameter("gate_control_enabled", false);
  enable_nearest_waypoint_priority_ = declare_parameter("enable_nearest_waypoint_priority", false);
  nearest_waypoint_forward_only_ = declare_parameter("nearest_waypoint_forward_only", true);
  nearest_waypoint_switch_hysteresis_cm_ = std::max(
    0.0, declare_parameter("nearest_waypoint_switch_hysteresis_cm", 0.0));
  default_gate_count_ = sanitizeGateCount(
    static_cast<uint32_t>(declare_parameter<int64_t>("default_gate_count", 3)));
  max_queue_size_ = std::max<uint32_t>(20,
    static_cast<uint32_t>(declare_parameter<int64_t>("max_queue_size", 200)));
  gate_pos_tol_cm_ = std::max(1.0, declare_parameter("gate_position_tolerance_cm", 8.0));
  gate_z_tol_cm_ = std::max(1.0, declare_parameter("gate_height_tolerance_cm", 6.0));
  gate_yaw_tol_deg_ = std::max(1.0, declare_parameter("gate_yaw_tolerance_deg", 10.0));
  gate_reach_hold_cycles_ = std::max(
    1, static_cast<int>(declare_parameter<int64_t>("gate_reach_hold_cycles", 2)));
  gate_waiting_trigger_ = gate_control_enabled_;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    input_pose_topic_, rclcpp::QoS(10),
    std::bind(&PlannerTargetAdapterNode::poseCallback, this, std::placeholders::_1));

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    input_path_topic_, rclcpp::QoS(10),
    std::bind(&PlannerTargetAdapterNode::pathCallback, this, std::placeholders::_1));

  position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
    input_position_cmd_topic_, rclcpp::QoS(20),
    std::bind(&PlannerTargetAdapterNode::positionCommandCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(20),
    std::bind(&PlannerTargetAdapterNode::odomCallback, this, std::placeholders::_1));

  external_height_sub_ = create_subscription<std_msgs::msg::Int16>(
    external_height_topic_, rclcpp::QoS(20),
    std::bind(&PlannerTargetAdapterNode::externalHeightCallback, this, std::placeholders::_1));

  gate_trigger_sub_ = create_subscription<std_msgs::msg::UInt32>(
    gate_trigger_topic_, rclcpp::QoS(10),
    std::bind(&PlannerTargetAdapterNode::gateTriggerCallback, this, std::placeholders::_1));

  if (republish_period_sec_ > 0.0) {
    republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(republish_period_sec_)),
      std::bind(&PlannerTargetAdapterNode::republishTimerCallback, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "planner_target_adapter started. output=%s mode=%s pose_in=%s path_in=%s pos_cmd_in=%s gate=%s nearest_priority=%s",
    output_topic_.c_str(), output_mode_.c_str(), input_pose_topic_.c_str(),
    input_path_topic_.c_str(), input_position_cmd_topic_.c_str(), gate_control_enabled_ ? "on" : "off",
    enable_nearest_waypoint_priority_ ? "on" : "off");
  RCLCPP_INFO(
    get_logger(),
    "Gate topics: trigger=%s status=%s odom=%s ext_height=%s default_count=%u",
    gate_trigger_topic_.c_str(), status_topic_.c_str(), odom_topic_.c_str(),
    external_height_topic_.c_str(), default_gate_count_);
  RCLCPP_INFO(
    get_logger(),
    "Takeoff guard: enabled=%s source=%s timeout_ms=%ld min_valid_z=%.2f guarded_z=%.2f",
    enable_takeoff_height_guard_ ? "true" : "false",
    use_external_height_guard_ ? "external_height" : "odom",
    static_cast<long>(external_height_timeout_ms_),
    min_valid_target_z_cm_, guarded_takeoff_z_cm_);
  RCLCPP_INFO(
    get_logger(),
    "Bootstrap target: enabled=%s once=%s height=%.2f",
    enable_bootstrap_takeoff_target_ ? "true" : "false",
    bootstrap_once_ ? "true" : "false",
    bootstrap_takeoff_height_cm_);

  publishStatus("init");
}

void PlannerTargetAdapterNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (prefer_position_cmd_input_ || prefer_path_input_) {
    return;
  }

  TargetCmd target{};
  if (!poseToTarget(msg->pose, target)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignore pose target due to invalid orientation quaternion.");
    return;
  }
  applyTakeoffHeightGuard(target);
  if (gate_control_enabled_ || enable_nearest_waypoint_priority_) {
    enqueueTarget(target);
  } else {
    publishTarget(target);
  }
}

void PlannerTargetAdapterNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (prefer_position_cmd_input_) {
    return;
  }

  if (msg->poses.empty()) {
    return;
  }

  if (!prefer_path_input_) {
    return;
  }

  const std::size_t idx = std::min(
    static_cast<std::size_t>(path_sample_index_), msg->poses.size() - 1);

  TargetCmd target{};
  if (!poseToTarget(msg->poses[idx].pose, target)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignore path target due to invalid orientation quaternion.");
    return;
  }
  applyTakeoffHeightGuard(target);
  if (gate_control_enabled_ || enable_nearest_waypoint_priority_) {
    enqueueTarget(target);
  } else {
    publishTarget(target);
  }
}

void PlannerTargetAdapterNode::positionCommandCallback(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  if (!prefer_position_cmd_input_) {
    return;
  }

  handleTrajectorySwitch(msg->trajectory_id, msg->trajectory_flag);

  TargetCmd target{};
  target.x_cm = meterToCm(msg->position.x);
  target.y_cm = meterToCm(msg->position.y);
  target.z_cm = use_input_z_ ? meterToCm(msg->position.z) : default_height_cm_;
  target.yaw_deg = resolveYawDeg(radToDeg(msg->yaw));
  last_incoming_target_ = target;
  has_last_incoming_target_ = true;

  if (isVelocityOutputMode()) {
    if (!has_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignore PositionCommand velocity output because odom pose is not available yet.");
      return;
    }

    publishVelocity(positionCommandToVelocity(*msg));
    return;
  }

  applyTakeoffHeightGuard(target);

  const double adaptive_scale = computeAdaptiveScale(target);
  const bool should_publish = shouldPublishTarget(target, adaptive_scale);

  if (!should_publish) {
    return;
  }

  // Keep adaptive-sampling history aligned with accepted targets only.
  updateRawTargetHistory(target);

  if (gate_control_enabled_ || enable_nearest_waypoint_priority_) {
    enqueueTarget(target);
  } else {
    publishTarget(target);
  }
}

void PlannerTargetAdapterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_.x_cm = meterToCm(msg->pose.pose.position.x);
  current_pose_.y_cm = meterToCm(msg->pose.pose.position.y);
  current_pose_.z_cm = meterToCm(msg->pose.pose.position.z);
  current_pose_.yaw_deg = radToDeg(tf2::getYaw(msg->pose.pose.orientation));
  has_pose_ = true;
}

void PlannerTargetAdapterNode::externalHeightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  external_height_cm_ = static_cast<double>(msg->data);
  last_external_height_time_ = now();
  has_external_height_ = true;
}

void PlannerTargetAdapterNode::gateTriggerCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
  if (!gate_control_enabled_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Ignore gate trigger because gate_control_enabled=false");
    return;
  }

  const uint32_t requested = sanitizeGateCount(msg->data == 0 ? default_gate_count_ : msg->data);
  ++trigger_count_;
  gate_remaining_in_segment_ = requested;
  segment_total_count_ = requested;
  segment_completed_count_ = 0;
  gate_waiting_trigger_ = false;
  gate_reach_stable_count_ = 0;

  (void)injectBootstrapTargetIfNeeded();

  const int64_t next_index = has_current_target_
    ? static_cast<int64_t>(current_target_.index + 1)
    : (pending_targets_.empty() ? -1 : static_cast<int64_t>(pending_targets_.front().index));
  const int64_t end_index = previewSegmentEndIndex();

  RCLCPP_INFO(
    get_logger(),
    "Gate triggered: count=%u, planned range=[%ld..%ld], queue=%zu",
    requested, static_cast<long>(next_index), static_cast<long>(end_index), pending_targets_.size());

  (void)dispatchNextTargetIfNeeded();
  publishStatus("trigger");
}

void PlannerTargetAdapterNode::republishTimerCallback()
{
  if (isVelocityOutputMode()) {
    if (has_last_velocity_) {
      publishVelocity(last_velocity_);
    }
    publishStatus("timer");
    return;
  }

  if (gate_control_enabled_ || enable_nearest_waypoint_priority_) {
    if (has_current_target_) {
      if (isCurrentTargetReached()) {
        onCurrentTargetReached();
      } else {
        gate_reach_stable_count_ = 0;
      }
      if (has_current_target_) {
        publishTarget(current_target_.cmd);
      }
    } else {
      (void)injectBootstrapTargetIfNeeded();
      (void)dispatchNextTargetIfNeeded();
    }
    publishStatus("timer");
    return;
  }

  if (has_last_target_) {
    publishTarget(last_target_);
  }
  publishStatus("timer");
}

void PlannerTargetAdapterNode::publishTarget(const TargetCmd & cmd)
{
  std_msgs::msg::Float32MultiArray out;
  out.data.resize(4);
  out.data[0] = static_cast<float>(cmd.x_cm);
  out.data[1] = static_cast<float>(cmd.y_cm);
  out.data[2] = static_cast<float>(cmd.z_cm);
  out.data[3] = static_cast<float>(cmd.yaw_deg);

  target_pub_->publish(out);
  last_target_ = cmd;
  has_last_target_ = true;
  last_accepted_target_ = cmd;
  has_last_accepted_target_ = true;
  ++accepted_count_;
}

void PlannerTargetAdapterNode::publishVelocity(const VelocityCmd & cmd)
{
  std_msgs::msg::Float32MultiArray out;
  out.data.resize(4);
  out.data[0] = static_cast<float>(cmd.vx_cm_s);
  out.data[1] = static_cast<float>(cmd.vy_cm_s);
  out.data[2] = static_cast<float>(cmd.vz_cm_s);
  out.data[3] = static_cast<float>(cmd.vyaw_deg_s);

  target_pub_->publish(out);
  last_velocity_ = cmd;
  has_last_velocity_ = true;
  ++accepted_count_;
}

void PlannerTargetAdapterNode::enqueueTarget(const TargetCmd & cmd)
{
  // In gate mode we track sparse-sampling anchor separately from published target.
  last_gate_sample_ = cmd;
  has_last_gate_sample_ = true;

  auto almost_same = [](const TargetCmd & a, const TargetCmd & b) {
    const double dxy = std::hypot(a.x_cm - b.x_cm, a.y_cm - b.y_cm);
    const double dz = std::fabs(a.z_cm - b.z_cm);
    const double dyaw = std::fabs(
      PlannerTargetAdapterNode::normalizeAngleDeg(a.yaw_deg - b.yaw_deg));
    return dxy < 1.0 && dz < 1.0 && dyaw < 1.0;
  };

  if (has_current_target_ && almost_same(current_target_.cmd, cmd)) {
    return;
  }
  if (!pending_targets_.empty() && almost_same(pending_targets_.back().cmd, cmd)) {
    return;
  }

  QueuedTarget entry{};
  entry.cmd = cmd;
  entry.index = next_target_index_++;
  pending_targets_.push_back(entry);
  pruneManagedQueue(cmd);
  ++queued_count_;

  while (pending_targets_.size() > max_queue_size_) {
    // Keep earlier unreleased points to preserve path order.
    pending_targets_.pop_back();
  }

  if (!gate_waiting_trigger_) {
    (void)dispatchNextTargetIfNeeded();
  }
}

void PlannerTargetAdapterNode::pruneManagedQueue(const TargetCmd & latest_cmd)
{
  if (!(gate_control_enabled_ || enable_nearest_waypoint_priority_) || !has_pose_) {
    return;
  }

  const double stale_radius_cm = std::max(gate_pos_tol_cm_, min_delta_xy_cm_);
  for (auto it = pending_targets_.begin(); it != pending_targets_.end();) {
    const bool is_latest =
      std::fabs(it->cmd.x_cm - latest_cmd.x_cm) < 1e-6 &&
      std::fabs(it->cmd.y_cm - latest_cmd.y_cm) < 1e-6 &&
      std::fabs(it->cmd.z_cm - latest_cmd.z_cm) < 1e-6 &&
      std::fabs(it->cmd.yaw_deg - latest_cmd.yaw_deg) < 1e-6;
    if (is_latest) {
      ++it;
      continue;
    }

    const double dist_to_pose = std::hypot(
      it->cmd.x_cm - current_pose_.x_cm,
      it->cmd.y_cm - current_pose_.y_cm);
    if (dist_to_pose <= stale_radius_cm) {
      it = pending_targets_.erase(it);
      continue;
    }

    if (nearest_waypoint_forward_only_ && static_cast<int64_t>(it->index) <= last_reached_index_) {
      it = pending_targets_.erase(it);
      continue;
    }

    ++it;
  }
}

bool PlannerTargetAdapterNode::injectBootstrapTargetIfNeeded()
{
  const bool managed_mode = gate_control_enabled_ || enable_nearest_waypoint_priority_;
  if (!managed_mode || !enable_bootstrap_takeoff_target_) {
    return false;
  }
  if (gate_control_enabled_ && gate_waiting_trigger_) {
    return false;
  }
  if (!has_pose_) {
    return false;
  }
  if (bootstrap_once_ && bootstrap_done_) {
    return false;
  }
  if (has_current_target_ || !pending_targets_.empty()) {
    return false;
  }

  TargetCmd bootstrap_target{};
  bootstrap_target.x_cm = current_pose_.x_cm;
  bootstrap_target.y_cm = current_pose_.y_cm;
  bootstrap_target.z_cm = bootstrap_takeoff_height_cm_;
  bootstrap_target.yaw_deg = use_fixed_yaw_ ? fixed_yaw_deg_ : current_pose_.yaw_deg;
  applyTakeoffHeightGuard(bootstrap_target);

  bootstrap_done_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Injected bootstrap target at current XY: x=%.2f y=%.2f z=%.2f yaw=%.2f",
    bootstrap_target.x_cm, bootstrap_target.y_cm,
    bootstrap_target.z_cm, bootstrap_target.yaw_deg);

  enqueueTarget(bootstrap_target);
  publishStatus("bootstrap_target_injected");
  return true;
}

bool PlannerTargetAdapterNode::dispatchNextTargetIfNeeded()
{
  const bool managed_mode = gate_control_enabled_ || enable_nearest_waypoint_priority_;
  if (!managed_mode || has_current_target_ || (gate_control_enabled_ && gate_waiting_trigger_)) {
    return false;
  }

  if (gate_control_enabled_ && gate_remaining_in_segment_ == 0) {
    gate_waiting_trigger_ = true;
    publishStatus("segment_waiting_trigger");
    return false;
  }

  if (pending_targets_.empty()) {
    publishStatus("segment_waiting_target");
    return false;
  }

  const auto next_it = selectNextQueuedTargetIterator();
  if (next_it == pending_targets_.end()) {
    publishStatus("segment_waiting_target");
    return false;
  }

  current_target_ = *next_it;
  pending_targets_.erase(next_it);
  has_current_target_ = true;
  gate_reach_stable_count_ = 0;

  publishTarget(current_target_.cmd);
  return true;
}

std::deque<PlannerTargetAdapterNode::QueuedTarget>::iterator
PlannerTargetAdapterNode::selectNextQueuedTargetIterator()
{
  if (pending_targets_.empty()) {
    return pending_targets_.end();
  }

  if (!enable_nearest_waypoint_priority_) {
    return pending_targets_.begin();
  }

  if (!has_pose_) {
    for (auto it = pending_targets_.begin(); it != pending_targets_.end(); ++it) {
      if (!nearest_waypoint_forward_only_ || static_cast<int64_t>(it->index) > last_reached_index_) {
        return it;
      }
    }
    return pending_targets_.end();
  }

  auto best_it = pending_targets_.end();
  double best_dist = std::numeric_limits<double>::infinity();
  for (auto it = pending_targets_.begin(); it != pending_targets_.end();) {
    if (nearest_waypoint_forward_only_ && static_cast<int64_t>(it->index) <= last_reached_index_) {
      ++it;
      continue;
    }

    const double dist = std::hypot(it->cmd.x_cm - current_pose_.x_cm, it->cmd.y_cm - current_pose_.y_cm);
    if (dist <= gate_pos_tol_cm_) {
      // Drop stale nearby samples that are already within reach radius of current pose.
      it = pending_targets_.erase(it);
      continue;
    }

    if (dist < best_dist) {
      best_dist = dist;
      best_it = it;
    }
    ++it;
  }

  return best_it;
}

std::deque<PlannerTargetAdapterNode::QueuedTarget>::const_iterator
PlannerTargetAdapterNode::selectNextQueuedTargetIterator() const
{
  if (pending_targets_.empty()) {
    return pending_targets_.end();
  }

  if (!enable_nearest_waypoint_priority_) {
    return pending_targets_.begin();
  }

  if (!has_pose_) {
    for (auto it = pending_targets_.cbegin(); it != pending_targets_.cend(); ++it) {
      if (!nearest_waypoint_forward_only_ || static_cast<int64_t>(it->index) > last_reached_index_) {
        return it;
      }
    }
    return pending_targets_.cend();
  }

  auto best_it = pending_targets_.cend();
  double best_dist = std::numeric_limits<double>::infinity();
  for (auto it = pending_targets_.cbegin(); it != pending_targets_.cend(); ++it) {
    if (nearest_waypoint_forward_only_ && static_cast<int64_t>(it->index) <= last_reached_index_) {
      continue;
    }

    const double dist = std::hypot(it->cmd.x_cm - current_pose_.x_cm, it->cmd.y_cm - current_pose_.y_cm);
    if (dist < best_dist) {
      best_dist = dist;
      best_it = it;
    }
  }

  return best_it;
}

bool PlannerTargetAdapterNode::isCurrentTargetReached() const
{
  if (!has_current_target_ || !has_pose_) {
    return false;
  }

  const double dx = current_target_.cmd.x_cm - current_pose_.x_cm;
  const double dy = current_target_.cmd.y_cm - current_pose_.y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = std::fabs(current_target_.cmd.z_cm - current_pose_.z_cm);
  const double dyaw = std::fabs(
    normalizeAngleDeg(current_target_.cmd.yaw_deg - current_pose_.yaw_deg));

  return dxy <= gate_pos_tol_cm_ && dz <= gate_z_tol_cm_ && dyaw <= gate_yaw_tol_deg_;
}

void PlannerTargetAdapterNode::onCurrentTargetReached()
{
  ++gate_reach_stable_count_;
  if (gate_reach_stable_count_ < gate_reach_hold_cycles_) {
    return;
  }

  gate_reach_stable_count_ = 0;
  last_reached_index_ = static_cast<int64_t>(current_target_.index);
  last_reached_target_ = current_target_.cmd;
  has_last_reached_target_ = true;
  last_reached_time_ = now();
  ++reached_count_;
  has_current_target_ = false;

  if (segment_completed_count_ < segment_total_count_) {
    ++segment_completed_count_;
  }
  if (gate_remaining_in_segment_ > 0) {
    --gate_remaining_in_segment_;
  }

  if (gate_remaining_in_segment_ == 0) {
    gate_waiting_trigger_ = true;
  }

  (void)dispatchNextTargetIfNeeded();
  publishStatus("target_reached");
}

void PlannerTargetAdapterNode::publishStatus(const std::string & reason)
{
  std_msgs::msg::String msg;
  RCLCPP_INFO(get_logger(), "Status update: %s", reason.c_str());
  msg.data = buildStatusJson(reason);
  status_pub_->publish(msg);
}

std::string PlannerTargetAdapterNode::buildStatusJson(const std::string & reason) const
{
  const int64_t current_index = has_current_target_
    ? static_cast<int64_t>(current_target_.index)
    : -1;
  const auto next_it = selectNextQueuedTargetIterator();
  const int64_t next_index = (next_it == pending_targets_.cend())
    ? -1
    : static_cast<int64_t>(next_it->index);
  const int64_t segment_end = previewSegmentEndIndex();
  const bool has_next_target = next_it != pending_targets_.cend();

  TargetCmd next_target{};
  if (has_next_target) {
    next_target = next_it->cmd;
  }

  TargetCmd preview_start_target{};
  TargetCmd preview_end_target{};
  bool has_preview_start = false;
  bool has_preview_end = false;
  if (!pending_targets_.empty()) {
    has_preview_start = true;
    preview_start_target = pending_targets_.front().cmd;

    const uint32_t preview_count = gate_waiting_trigger_
      ? default_gate_count_
      : std::max<uint32_t>(1, gate_remaining_in_segment_);
    const std::size_t preview_offset = std::min<std::size_t>(
      pending_targets_.size() - 1,
      static_cast<std::size_t>(std::max<uint32_t>(1, preview_count) - 1));
    preview_end_target = pending_targets_[preview_offset].cmd;
    has_preview_end = true;
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "{";
  oss << "\"reason\":\"" << reason << "\",";
  oss << "\"output_mode\":\"" << output_mode_ << "\",";
  oss << "\"gate_enabled\":" << (gate_control_enabled_ ? "true" : "false") << ",";
  oss << "\"nearest_priority_enabled\":" << (enable_nearest_waypoint_priority_ ? "true" : "false") << ",";
  oss << "\"nearest_forward_only\":" << (nearest_waypoint_forward_only_ ? "true" : "false") << ",";
  oss << "\"nearest_switch_hysteresis_cm\":" << nearest_waypoint_switch_hysteresis_cm_ << ",";
  oss << "\"state\":\"";
  if (!gate_control_enabled_ && !enable_nearest_waypoint_priority_) {
    oss << "PASSTHROUGH";
  } else if (gate_control_enabled_ && gate_waiting_trigger_) {
    oss << "WAIT_TRIGGER";
  } else if (has_current_target_) {
    oss << "EXECUTING";
  } else {
    oss << "WAIT_TARGET";
  }
  oss << "\",";
  oss << "\"current_index\":" << current_index << ",";
  oss << "\"completed_index\":" << last_reached_index_ << ",";
  oss << "\"next_index\":" << next_index << ",";
  oss << "\"segment_total\":" << segment_total_count_ << ",";
  oss << "\"segment_completed\":" << segment_completed_count_ << ",";
  oss << "\"segment_remaining\":" << gate_remaining_in_segment_ << ",";
  oss << "\"segment_preview_end\":" << segment_end << ",";
  oss << "\"queued\":" << pending_targets_.size() << ",";
  oss << "\"active_trajectory_id\":";
  if (has_active_trajectory_id_) {
    oss << active_trajectory_id_;
  } else {
    oss << "null";
  }
  oss << ",";
  oss << "\"trajectory_switch_count\":" << trajectory_switch_count_ << ",";
  oss << "\"bootstrap_done\":" << (bootstrap_done_ ? "true" : "false") << ",";
  oss << "\"has_last_velocity\":" << (has_last_velocity_ ? "true" : "false") << ",";
  oss << "\"external_height_valid\":" << (has_external_height_ ? "true" : "false") << ",";
  oss << "\"external_height_cm\":" << external_height_cm_ << ",";
  oss << "\"trigger_count\":" << trigger_count_ << ",";
  oss << "\"accepted_count\":" << accepted_count_ << ",";
  oss << "\"skipped_count\":" << skipped_count_ << ",";
  oss << "\"reached_count\":" << reached_count_ << ",";
  oss << "\"last_height_guard_applied\":" << (last_target_guard_applied_ ? "true" : "false") << ",";
  oss << "\"last_height_guard_source\":\"" << last_height_guard_source_ << "\",";
  oss << "\"current_pose\":{";
  oss << "\"x_cm\":" << current_pose_.x_cm << ",";
  oss << "\"y_cm\":" << current_pose_.y_cm << ",";
  oss << "\"z_cm\":" << current_pose_.z_cm << ",";
  oss << "\"yaw_deg\":" << current_pose_.yaw_deg << "},";
  oss << "\"last_incoming_target\":{";
  if (has_last_incoming_target_) {
    oss << "\"x_cm\":" << last_incoming_target_.x_cm << ",";
    oss << "\"y_cm\":" << last_incoming_target_.y_cm << ",";
    oss << "\"z_cm\":" << last_incoming_target_.z_cm << ",";
    oss << "\"yaw_deg\":" << last_incoming_target_.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"last_accepted_target\":{";
  if (has_last_accepted_target_) {
    oss << "\"x_cm\":" << last_accepted_target_.x_cm << ",";
    oss << "\"y_cm\":" << last_accepted_target_.y_cm << ",";
    oss << "\"z_cm\":" << last_accepted_target_.z_cm << ",";
    oss << "\"yaw_deg\":" << last_accepted_target_.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"current_target\":{";
  if (has_current_target_) {
    oss << "\"x_cm\":" << current_target_.cmd.x_cm << ",";
    oss << "\"y_cm\":" << current_target_.cmd.y_cm << ",";
    oss << "\"z_cm\":" << current_target_.cmd.z_cm << ",";
    oss << "\"yaw_deg\":" << current_target_.cmd.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"next_target\":{";
  if (has_next_target) {
    oss << "\"x_cm\":" << next_target.x_cm << ",";
    oss << "\"y_cm\":" << next_target.y_cm << ",";
    oss << "\"z_cm\":" << next_target.z_cm << ",";
    oss << "\"yaw_deg\":" << next_target.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"last_reached_target\":{";
  if (has_last_reached_target_) {
    oss << "\"x_cm\":" << last_reached_target_.x_cm << ",";
    oss << "\"y_cm\":" << last_reached_target_.y_cm << ",";
    oss << "\"z_cm\":" << last_reached_target_.z_cm << ",";
    oss << "\"yaw_deg\":" << last_reached_target_.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"segment_preview_start_target\":{";
  if (has_preview_start) {
    oss << "\"x_cm\":" << preview_start_target.x_cm << ",";
    oss << "\"y_cm\":" << preview_start_target.y_cm << ",";
    oss << "\"z_cm\":" << preview_start_target.z_cm << ",";
    oss << "\"yaw_deg\":" << preview_start_target.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"segment_preview_end_target\":{";
  if (has_preview_end) {
    oss << "\"x_cm\":" << preview_end_target.x_cm << ",";
    oss << "\"y_cm\":" << preview_end_target.y_cm << ",";
    oss << "\"z_cm\":" << preview_end_target.z_cm << ",";
    oss << "\"yaw_deg\":" << preview_end_target.yaw_deg;
  } else {
    oss << "\"x_cm\":null,\"y_cm\":null,\"z_cm\":null,\"yaw_deg\":null";
  }
  oss << "},";
  oss << "\"next_hint\":\"press s to release next segment\"";
  oss << "}";
  return oss.str();
}

uint32_t PlannerTargetAdapterNode::sanitizeGateCount(uint32_t requested) const
{
  if (requested == 0) {
    return 1;
  }
  return std::min<uint32_t>(requested, 100);
}

int64_t PlannerTargetAdapterNode::previewSegmentEndIndex() const
{
  if (!gate_control_enabled_) {
    return -1;
  }
  if (pending_targets_.empty()) {
    return has_current_target_ ? static_cast<int64_t>(current_target_.index) : -1;
  }

  const uint32_t preview_count = gate_waiting_trigger_
    ? default_gate_count_
    : std::max<uint32_t>(1, gate_remaining_in_segment_);
  const int64_t start = static_cast<int64_t>(pending_targets_.front().index);
  return start + static_cast<int64_t>(preview_count) - 1;
}

bool PlannerTargetAdapterNode::shouldPublishTarget(const TargetCmd & cmd, double adaptive_scale)
{
  const bool has_reference = gate_control_enabled_ ? has_last_gate_sample_ : has_last_target_;
  if (!enable_sparse_sampling_ || !has_reference) {
    return true;
  }

  if (enable_adaptive_sampling_) {
    ++adaptive_sample_count_;
  }

  const double effective_xy = min_delta_xy_cm_ * adaptive_scale;
  const double effective_z = min_delta_z_cm_ * adaptive_scale;
  const double effective_yaw = min_delta_yaw_deg_ * adaptive_scale;

  const TargetCmd & reference = gate_control_enabled_ ? last_gate_sample_ : last_target_;

  const double delta_x = cmd.x_cm - reference.x_cm;
  const double delta_y = cmd.y_cm - reference.y_cm;
  const double delta_xy = std::hypot(delta_x, delta_y);
  const double delta_z = std::fabs(cmd.z_cm - reference.z_cm);
  const double delta_yaw = std::fabs(normalizeAngleDeg(cmd.yaw_deg - reference.yaw_deg));

  if (delta_xy < effective_xy &&
    delta_z < effective_z &&
    delta_yaw < effective_yaw)
  {
    ++skipped_count_;
    if (skipped_count_ % 50 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Sparse sampling active: skipped=%lu accepted=%lu (effective thresholds xy=%.2fcm z=%.2fcm yaw=%.2fdeg, scale=%.2f)",
        static_cast<unsigned long>(skipped_count_),
        static_cast<unsigned long>(accepted_count_),
        effective_xy, effective_z, effective_yaw, adaptive_scale);
    }
    return false;
  }

  return true;
}

void PlannerTargetAdapterNode::applyTakeoffHeightGuard(TargetCmd & cmd)
{
  last_target_guard_applied_ = false;
  last_height_guard_source_ = "disabled";
  if (!enable_takeoff_height_guard_) {
    return;
  }

  double check_height_cm = has_pose_ ? current_pose_.z_cm : -1e9;
  bool using_external_height = false;
  last_height_guard_source_ = "odom";
  if (use_external_height_guard_ && has_external_height_) {
    bool external_valid = true;
    if (external_height_timeout_ms_ > 0) {
      const auto elapsed = now() - last_external_height_time_;
      const int64_t elapsed_ms = elapsed.nanoseconds() / 1000000;
      external_valid = elapsed_ms <= external_height_timeout_ms_;
      if (!external_valid) {
        RCLCPP_WARN(
          get_logger(),
          "External height timeout (%ld ms > %ld ms), fallback to odom z",
          static_cast<long>(elapsed_ms), static_cast<long>(external_height_timeout_ms_));
      }
    }

    if (external_valid) {
      check_height_cm = external_height_cm_;
      using_external_height = true;
      last_height_guard_source_ = "external_height";
    }
  }

  // Only guard during takeoff phase; once airborne above threshold, follow planner z directly.
  if (check_height_cm >= min_valid_target_z_cm_) {
    last_height_guard_source_ = using_external_height ? "external_height_pass" : "odom_pass";
    return;
  }

  if (cmd.z_cm < min_valid_target_z_cm_) {
    RCLCPP_WARN(
      get_logger(),
      "Target z=%.2fcm below min_valid_target_z_cm=%.2fcm (height_source=%s height=%.2fcm), guarding to %.2fcm",
      cmd.z_cm, min_valid_target_z_cm_,
      using_external_height ? "external" : "odom",
      check_height_cm, guarded_takeoff_z_cm_);
    cmd.z_cm = guarded_takeoff_z_cm_;
    last_target_guard_applied_ = true;
    last_height_guard_source_ = using_external_height ? "external_height_guarded" : "odom_guarded";
  }
}

double PlannerTargetAdapterNode::computeAdaptiveScale(const TargetCmd & cmd) const
{
  if (!enable_adaptive_sampling_ || !has_last_raw_target_ || !has_prev_raw_target_) {
    return 1.0;
  }

  const double v1_x = last_raw_target_.x_cm - prev_raw_target_.x_cm;
  const double v1_y = last_raw_target_.y_cm - prev_raw_target_.y_cm;
  const double v2_x = cmd.x_cm - last_raw_target_.x_cm;
  const double v2_y = cmd.y_cm - last_raw_target_.y_cm;

  const double norm1 = std::hypot(v1_x, v1_y);
  const double norm2 = std::hypot(v2_x, v2_y);
  if (norm1 < 1e-6 || norm2 < 1e-6) {
    return 1.0;
  }

  const double dot = v1_x * v2_x + v1_y * v2_y;
  const double cross = v1_x * v2_y - v1_y * v2_x;
  const double turn_deg = std::fabs(radToDeg(std::atan2(cross, dot)));

  if (turn_deg <= adaptive_turn_low_deg_) {
    return adaptive_scale_max_;
  }
  if (turn_deg >= adaptive_turn_high_deg_) {
    return adaptive_scale_min_;
  }

  const double ratio = (turn_deg - adaptive_turn_low_deg_) /
    (adaptive_turn_high_deg_ - adaptive_turn_low_deg_);
  return adaptive_scale_max_ - ratio * (adaptive_scale_max_ - adaptive_scale_min_);
}

void PlannerTargetAdapterNode::updateRawTargetHistory(const TargetCmd & cmd)
{
  if (has_last_raw_target_) {
    prev_raw_target_ = last_raw_target_;
    has_prev_raw_target_ = true;
  }
  last_raw_target_ = cmd;
  has_last_raw_target_ = true;
}

void PlannerTargetAdapterNode::handleTrajectorySwitch(uint32_t trajectory_id, uint8_t trajectory_flag)
{
  if (!enable_trajectory_switch_reset_) {
    return;
  }

  if (!has_active_trajectory_id_) {
    active_trajectory_id_ = trajectory_id;
    has_active_trajectory_id_ = true;
    return;
  }

  if (trajectory_id == active_trajectory_id_) {
    return;
  }

  const uint32_t previous_id = active_trajectory_id_;
  active_trajectory_id_ = trajectory_id;
  ++trajectory_switch_count_;

  const bool keep_active_target = enable_nearest_waypoint_priority_ && has_current_target_;
  std::size_t dropped_targets = pending_targets_.size();
  if (!keep_active_target && has_current_target_) {
    ++dropped_targets;
  }
  pending_targets_.clear();
  if (!keep_active_target) {
    has_current_target_ = false;
  }
  gate_reach_stable_count_ = 0;
  if (gate_control_enabled_) {
    gate_remaining_in_segment_ = 0;
    segment_total_count_ = 0;
    segment_completed_count_ = 0;
    gate_waiting_trigger_ = true;
  }

  has_last_target_ = false;
  has_last_velocity_ = false;
  has_last_accepted_target_ = false;
  has_last_gate_sample_ = false;
  has_last_raw_target_ = false;
  has_prev_raw_target_ = false;

  RCLCPP_INFO(
    get_logger(),
    "Trajectory switched: id %u -> %u flag=%u, dropped %zu stale target(s), keep_active=%s",
    previous_id, active_trajectory_id_, static_cast<unsigned int>(trajectory_flag), dropped_targets,
    keep_active_target ? "true" : "false");
  publishStatus("trajectory_switched");
}

PlannerTargetAdapterNode::VelocityCmd PlannerTargetAdapterNode::positionCommandToVelocity(
  const quadrotor_msgs::msg::PositionCommand & msg) const
{
  VelocityCmd velocity{};
  const double vx_world_cm_s = meterToCm(msg.velocity.x);
  const double vy_world_cm_s = meterToCm(msg.velocity.y);
  const double yaw_rad = current_pose_.yaw_deg * M_PI / 180.0;

  velocity.vx_cm_s = vx_world_cm_s * std::cos(yaw_rad) + vy_world_cm_s * std::sin(yaw_rad);
  velocity.vy_cm_s = -vx_world_cm_s * std::sin(yaw_rad) + vy_world_cm_s * std::cos(yaw_rad);
  velocity.vz_cm_s = meterToCm(msg.velocity.z);
  velocity.vyaw_deg_s = radToDeg(msg.yaw_dot);
  return velocity;
}

bool PlannerTargetAdapterNode::isVelocityOutputMode() const
{
  return output_mode_ == "velocity";
}

bool PlannerTargetAdapterNode::poseToTarget(
  const geometry_msgs::msg::Pose & pose,
  TargetCmd & target) const
{
  const double q_norm2 =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  if (q_norm2 < 1e-8) {
    return false;
  }
  const double yaw = tf2::getYaw(pose.orientation);

  target.x_cm = meterToCm(pose.position.x);
  target.y_cm = meterToCm(pose.position.y);
  target.z_cm = use_input_z_ ? meterToCm(pose.position.z) : default_height_cm_;
  target.yaw_deg = resolveYawDeg(radToDeg(yaw));
  return true;
}

double PlannerTargetAdapterNode::resolveYawDeg(double raw_yaw_deg) const
{
  if (use_fixed_yaw_) {
    return fixed_yaw_deg_;
  }
  return raw_yaw_deg;
}

double PlannerTargetAdapterNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double PlannerTargetAdapterNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double PlannerTargetAdapterNode::normalizeAngleDeg(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

}  // namespace activity_control_pkg
