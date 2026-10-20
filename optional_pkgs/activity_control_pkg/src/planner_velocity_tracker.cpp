#include "activity_control_pkg/planner_velocity_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/u_int8.hpp>

namespace activity_control_pkg
{

namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kTakeoffGuardReleaseMarginCm = 10.0;
constexpr double kTakeoffGuardMaxHorizontalVelocityCm = 8.0;

}  // namespace

PlannerVelocityTrackerNode::PlannerVelocityTrackerNode(const rclcpp::NodeOptions & options)
: Node("planner_velocity_tracker", options),
  input_position_cmd_topic_(declare_parameter(
      "input_position_cmd_topic", std::string("/drone_0_planning/pos_cmd"))),
  odom_topic_(declare_parameter("odom_topic", std::string("/fastlio2/lio_odom_imu_high"))),
  ready_topic_(declare_parameter("ready_topic", std::string("/is_st_ready"))),
  output_topic_(declare_parameter("output_topic", std::string("/target_velocity"))),
  status_topic_(declare_parameter(
      "status_topic", std::string("/planner_velocity_tracker/status"))),
    control_mode_topic_(declare_parameter(
      "control_mode_topic", std::string("/precision_landing/control_mode"))),
  world_frame_id_(declare_parameter("world_frame_id", std::string("world"))),
  body_frame_id_(declare_parameter("body_frame_id", std::string("body"))),
  control_frequency_(std::max(1.0, declare_parameter("control_frequency", 60.0))),
  odom_timeout_sec_(std::max(0.01, declare_parameter("odom_timeout_sec", 0.2))),
  cmd_timeout_sec_(std::max(0.01, declare_parameter("cmd_timeout_sec", 0.25))),
  kp_position_x_(declare_parameter("kp_position_x", 0.8)),
  kp_position_y_(declare_parameter("kp_position_y", 0.8)),
  kp_position_z_(declare_parameter("kp_position_z", 1.0)),
  kv_velocity_x_(declare_parameter("kv_velocity_x", 0.35)),
  kv_velocity_y_(declare_parameter("kv_velocity_y", 0.35)),
  kv_velocity_z_(declare_parameter("kv_velocity_z", 0.45)),
  ff_velocity_gain_(declare_parameter("ff_velocity_gain", 1.0)),
  ff_accel_time_sec_(std::max(0.0, declare_parameter("ff_accel_time_sec", 0.0))),
  use_fixed_yaw_(declare_parameter("use_fixed_yaw", true)),
  fixed_yaw_deg_(declare_parameter("fixed_yaw_deg", 0.0)),
  yaw_kp_(declare_parameter("yaw_kp", 1.1)),
  yaw_ff_gain_(declare_parameter("yaw_ff_gain", 1.0)),
  large_yaw_error_deg_threshold_(std::max(0.0, declare_parameter("large_yaw_error_deg_threshold", 60.0))),
  large_yaw_kp_scale_(std::max(1.0, declare_parameter("large_yaw_kp_scale", 1.0))),
  large_yaw_ff_scale_(std::max(1.0, declare_parameter("large_yaw_ff_scale", 1.0))),
  max_linear_velocity_cm_(std::max(1.0, declare_parameter("max_linear_velocity_cm", 35.0))),
  max_vertical_velocity_cm_(std::max(1.0, declare_parameter("max_vertical_velocity_cm", 28.0))),
  max_yaw_velocity_deg_(std::max(1.0, declare_parameter("max_yaw_velocity_deg", 30.0))),
  large_yaw_velocity_deg_(std::max(
      max_yaw_velocity_deg_, declare_parameter("large_yaw_velocity_deg", max_yaw_velocity_deg_))),
  min_publish_speed_cm_(std::max(0.0, declare_parameter("min_publish_speed_cm", 0.5))),
  enable_takeoff_height_guard_(declare_parameter("enable_takeoff_height_guard", true)),
  guarded_takeoff_z_cm_(std::max(0.0, declare_parameter("guarded_takeoff_z_cm", 60.0))),
  enable_bootstrap_takeoff_(declare_parameter("enable_bootstrap_takeoff", true)),
  bootstrap_takeoff_z_cm_(std::max(
      0.0, declare_parameter("bootstrap_takeoff_z_cm", guarded_takeoff_z_cm_))),
  enable_trajectory_switch_reset_(declare_parameter("enable_trajectory_switch_reset", true)),
  publish_zero_on_timeout_(declare_parameter("publish_zero_on_timeout", true)),
  publish_debug_topics_(declare_parameter("publish_debug_topics", true)),
  enable_debug_logs_(declare_parameter("enable_debug_logs", true)),
  odom_velocity_in_body_frame_(declare_parameter("odom_velocity_in_body_frame", true)),
  debug_log_throttle_ms_(
      std::max<int64_t>(0, declare_parameter<int64_t>("debug_log_throttle_ms", 1000))),
    active_control_mode_(static_cast<uint8_t>(std::max<int64_t>(
      0, declare_parameter<int64_t>("active_control_mode", 0)))),
    current_control_mode_(active_control_mode_)
{
  auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, command_qos);
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));

  position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
    input_position_cmd_topic_, rclcpp::QoS(20),
    std::bind(&PlannerVelocityTrackerNode::positionCommandCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(50),
    std::bind(&PlannerVelocityTrackerNode::odomCallback, this, std::placeholders::_1));

  ready_sub_ = create_subscription<std_msgs::msg::UInt8>(
    ready_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PlannerVelocityTrackerNode::readyCallback, this, std::placeholders::_1));

  control_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    control_mode_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PlannerVelocityTrackerNode::controlModeCallback, this, std::placeholders::_1));

  if (publish_debug_topics_) {
    reference_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "planner_velocity_tracker/reference_pose", rclcpp::QoS(10));
    pos_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "planner_velocity_tracker/position_error_world", rclcpp::QoS(10));
    vel_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "planner_velocity_tracker/velocity_error_world", rclcpp::QoS(10));
    cmd_world_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "planner_velocity_tracker/command_world_cm", rclcpp::QoS(10));
    cmd_body_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "planner_velocity_tracker/command_body_cm", rclcpp::QoS(10));
  }

  const double period_sec = 1.0 / control_frequency_;
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
    std::bind(&PlannerVelocityTrackerNode::controlTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "planner_velocity_tracker started. input=%s odom=%s ready=%s output=%s mode_topic=%s active_mode=%u freq=%.1fHz fixed_yaw=%s yaw_limit=%.1f/%.1fdegps bootstrap_takeoff=%s bootstrap_z=%.1fcm",
    input_position_cmd_topic_.c_str(), odom_topic_.c_str(), ready_topic_.c_str(), output_topic_.c_str(),
    control_mode_topic_.c_str(), static_cast<unsigned>(active_control_mode_), control_frequency_, use_fixed_yaw_ ? "true" : "false",
    max_yaw_velocity_deg_, large_yaw_velocity_deg_,
    enable_bootstrap_takeoff_ ? "true" : "false", bootstrap_takeoff_z_cm_);

  publishStatus("init");
}

void PlannerVelocityTrackerNode::positionCommandCallback(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  TrackerVector3 position{
    msg->position.x,
    msg->position.y,
    msg->position.z,
  };

  if (!isFinite(position)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignore PositionCommand with invalid position values.");
    return;
  }

  handleTrajectorySwitch(msg->trajectory_id);

  ref_state_.valid = true;
  ref_state_.stamp =
    (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? now() : rclcpp::Time(msg->header.stamp);
  ref_state_.trajectory_id = msg->trajectory_id;
  ref_state_.trajectory_flag = msg->trajectory_flag;
  ref_state_.pos_world = position;
  ref_state_.vel_world = {
    finiteOrZero(msg->velocity.x),
    finiteOrZero(msg->velocity.y),
    finiteOrZero(msg->velocity.z),
  };
  ref_state_.acc_world = {
    finiteOrZero(msg->acceleration.x),
    finiteOrZero(msg->acceleration.y),
    finiteOrZero(msg->acceleration.z),
  };
  ref_state_.yaw_rad = std::isfinite(msg->yaw) ? msg->yaw : 0.0;
  ref_state_.yaw_rate_rad_s = std::isfinite(msg->yaw_dot) ? msg->yaw_dot : 0.0;
  last_timeout_zero_sent_ = false;
}

void PlannerVelocityTrackerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  const rclcpp::Time stamp =
    (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? now() : rclcpp::Time(msg->header.stamp);
  double yaw_rate_rad_s = 0.0;
  if (odom_state_.valid) {
    const double dt = (stamp - odom_state_.stamp).seconds();
    if (dt > kEpsilon) {
      yaw_rate_rad_s = normalizeAngleRad(yaw - odom_state_.yaw_rad) / dt;
    }
  }

  odom_state_.stamp = stamp;
  odom_state_.pos_world = {
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z,
  };
  odom_state_.yaw_rad = yaw;
  odom_state_.yaw_rate_rad_s = yaw_rate_rad_s;

  const TrackerVector3 raw_velocity{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z,
  };

  odom_state_.vel_world =
    odom_velocity_in_body_frame_ ? rotateBodyToWorld(raw_velocity, odom_state_.yaw_rad) : raw_velocity;
  odom_state_.valid = true;
}

void PlannerVelocityTrackerNode::readyCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 1 && !ready_received_) {
    ready_received_ = true;
    RCLCPP_INFO(get_logger(), "Received bootstrap ready signal on %s.", ready_topic_.c_str());
  }
}

void PlannerVelocityTrackerNode::controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  current_control_mode_ = msg->data;
  const bool next_publish_active = isPublishActive();
  if (next_publish_active == publish_active_) {
    return;
  }

  publish_active_ = next_publish_active;
  resetControlState();
  if (publish_active_) {
    RCLCPP_INFO(
      get_logger(), "planner_velocity_tracker activated on control mode %u.",
      static_cast<unsigned>(current_control_mode_));
  } else {
    RCLCPP_INFO(
      get_logger(), "planner_velocity_tracker muted on control mode %u.",
      static_cast<unsigned>(current_control_mode_));
  }
}

void PlannerVelocityTrackerNode::controlTimerCallback()
{
  if (!publish_active_) {
    publishStatus("inactive_mode");
    return;
  }

  const rclcpp::Time current_time = now();

  if (!odom_state_.valid) {
    tracker_mode_ = TrackerMode::WAIT_ODOM;
    publishZeroCommand();
    publishStatus("wait_odom");
    return;
  }

  if (!ref_state_.valid) {
    if (shouldRunBootstrapTakeoff()) {
      runBootstrapTakeoff();
      return;
    }

    tracker_mode_ = TrackerMode::WAIT_REF;
    publishZeroCommand();
    publishStatus(ready_received_ ? "wait_ref_after_ready" : "wait_ref");
    return;
  }

  if (currentOdomAgeSec(current_time) > odom_timeout_sec_) {
    tracker_mode_ = TrackerMode::TIMEOUT_BRAKE;
    publishZeroCommand();
    publishStatus("odom_timeout");
    return;
  }

  if (currentRefAgeSec(current_time) > cmd_timeout_sec_) {
    tracker_mode_ = TrackerMode::TIMEOUT_BRAKE;
    publishZeroCommand();
    publishStatus("cmd_timeout");
    return;
  }

  tracker_mode_ = TrackerMode::TRACKING;
  updateTakeoffGuardState();

  TrackerVector3 effective_ref_pos_world = ref_state_.pos_world;
  TrackerVector3 effective_ref_vel_world = ref_state_.vel_world;
  TrackerVector3 effective_ref_acc_world = ref_state_.acc_world;
  double effective_ref_yaw_rate_rad_s = ref_state_.yaw_rate_rad_s;
  if (takeoff_guard_active_) {
    effective_ref_pos_world.x = 0.0;
    effective_ref_pos_world.y = 0.0;
    effective_ref_pos_world.z = std::max(effective_ref_pos_world.z, guarded_takeoff_z_cm_ / 100.0);

    effective_ref_vel_world.x = 0.0;
    effective_ref_vel_world.y = 0.0;
    effective_ref_acc_world.x = 0.0;
    effective_ref_acc_world.y = 0.0;
    effective_ref_yaw_rate_rad_s = 0.0;
  }

  last_pos_error_world_ = subtract(effective_ref_pos_world, odom_state_.pos_world);
  last_vel_error_world_ = subtract(effective_ref_vel_world, odom_state_.vel_world);

  TrackerVector3 command_world_m{
    ff_velocity_gain_ * effective_ref_vel_world.x +
      kp_position_x_ * last_pos_error_world_.x +
      kv_velocity_x_ * last_vel_error_world_.x +
      ff_accel_time_sec_ * effective_ref_acc_world.x,
    ff_velocity_gain_ * effective_ref_vel_world.y +
      kp_position_y_ * last_pos_error_world_.y +
      kv_velocity_y_ * last_vel_error_world_.y +
      ff_accel_time_sec_ * effective_ref_acc_world.y,
    ff_velocity_gain_ * effective_ref_vel_world.z +
      kp_position_z_ * last_pos_error_world_.z +
      kv_velocity_z_ * last_vel_error_world_.z +
      ff_accel_time_sec_ * effective_ref_acc_world.z,
  };

  last_command_world_cm_ = scale(command_world_m, 100.0);
  limitWorldVelocity(last_command_world_cm_);

  const double desired_yaw_rad = takeoff_guard_active_
    ? takeoff_guard_hold_yaw_rad_
    : (use_fixed_yaw_ ? degToRad(fixed_yaw_deg_) : ref_state_.yaw_rad);
  last_desired_yaw_rad_ = desired_yaw_rad;
  last_yaw_error_rad_ = normalizeAngleRad(desired_yaw_rad - odom_state_.yaw_rad);
  const double abs_yaw_error_deg = radToDeg(std::fabs(last_yaw_error_rad_));
  last_large_yaw_error_active_ =
    large_yaw_error_deg_threshold_ > 0.0 && abs_yaw_error_deg >= large_yaw_error_deg_threshold_;
  const double yaw_kp_scale = last_large_yaw_error_active_ ? large_yaw_kp_scale_ : 1.0;
  const double yaw_ff_scale = last_large_yaw_error_active_ ? large_yaw_ff_scale_ : 1.0;
  last_yaw_rate_feedback_deg_s_ = radToDeg(yaw_kp_ * yaw_kp_scale * last_yaw_error_rad_);
  last_yaw_rate_feedforward_deg_s_ = radToDeg(yaw_ff_gain_ * yaw_ff_scale * effective_ref_yaw_rate_rad_s);
  last_yaw_rate_unclamped_deg_s_ =
    last_yaw_rate_feedforward_deg_s_ + last_yaw_rate_feedback_deg_s_;
  last_yaw_rate_limit_deg_s_ =
    last_large_yaw_error_active_ ? large_yaw_velocity_deg_ : max_yaw_velocity_deg_;

  TrackerVector3 command_body_cm_vec = rotateWorldToBody(
    last_command_world_cm_, odom_state_.yaw_rad);

  last_command_ = {
    command_body_cm_vec.x,
    command_body_cm_vec.y,
    command_body_cm_vec.z,
    last_yaw_rate_unclamped_deg_s_,
  };

  if (takeoff_guard_active_) {
    const double guard_xy_norm = std::hypot(last_command_.vx_cm_s, last_command_.vy_cm_s);
    if (guard_xy_norm > kTakeoffGuardMaxHorizontalVelocityCm && guard_xy_norm > kEpsilon) {
      const double scale_factor = kTakeoffGuardMaxHorizontalVelocityCm / guard_xy_norm;
      last_command_.vx_cm_s *= scale_factor;
      last_command_.vy_cm_s *= scale_factor;
    }
    last_command_.vz_cm_s = std::max(0.0, last_command_.vz_cm_s);
  }

  limitBodyVelocity(last_command_, last_yaw_rate_limit_deg_s_);
  applyDeadband(last_command_);
  publishVelocityCommand(last_command_);
  publishDebug(desired_yaw_rad);
  publishStatus("tracking");
  last_timeout_zero_sent_ = false;

  if (enable_debug_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_log_throttle_ms_,
      "traj=%u pos_err=(%.2f, %.2f, %.2f)m vel_err=(%.2f, %.2f, %.2f)m cmd_body=(%.1f, %.1f, %.1f)cm/s yaw_err=%.1fdeg",
      ref_state_.trajectory_id,
      last_pos_error_world_.x, last_pos_error_world_.y, last_pos_error_world_.z,
      last_vel_error_world_.x, last_vel_error_world_.y, last_vel_error_world_.z,
      last_command_.vx_cm_s, last_command_.vy_cm_s, last_command_.vz_cm_s,
      radToDeg(last_yaw_error_rad_));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_log_throttle_ms_,
      "yaw ref=%.1fdeg ref_rate=%.1fdegps odom=%.1fdeg odom_rate=%.1fdegps ff=%.1f fb=%.1f raw=%.1f cmd=%.1f limit=%.1f large_turn=%s",
      radToDeg(ref_state_.yaw_rad), radToDeg(ref_state_.yaw_rate_rad_s),
      radToDeg(odom_state_.yaw_rad), radToDeg(odom_state_.yaw_rate_rad_s),
      last_yaw_rate_feedforward_deg_s_, last_yaw_rate_feedback_deg_s_,
      last_yaw_rate_unclamped_deg_s_, last_command_.yaw_rate_deg_s,
      last_yaw_rate_limit_deg_s_, last_large_yaw_error_active_ ? "true" : "false");
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_log_throttle_ms_,
      "takeoff_guard=%s completed=%s odom_z=%.1fcm release_z=%.1fcm guarded_z=%.1fcm takeoff_xy=(%.2f, %.2f)m",
      takeoff_guard_active_ ? "on" : "off",
      takeoff_guard_completed_ ? "true" : "false",
      currentOdomHeightCm(), takeoffGuardReleaseHeightCm(), guarded_takeoff_z_cm_,
      takeoff_guard_origin_pos_world_.x, takeoff_guard_origin_pos_world_.y);
  }
}

bool PlannerVelocityTrackerNode::shouldRunBootstrapTakeoff() const
{
  if (!enable_bootstrap_takeoff_ || !ready_received_ || !odom_state_.valid) {
    return false;
  }

  if (takeoff_guard_completed_) {
    return false;
  }

  return currentOdomHeightCm() < bootstrap_takeoff_z_cm_;
}

void PlannerVelocityTrackerNode::runBootstrapTakeoff()
{
  tracker_mode_ = TrackerMode::TRACKING;
  updateTakeoffGuardState();

  const double desired_height_cm = std::max(bootstrap_takeoff_z_cm_, guarded_takeoff_z_cm_);
  const double desired_height_m = desired_height_cm / 100.0;
  const double desired_yaw_rad = use_fixed_yaw_ ? degToRad(fixed_yaw_deg_) : odom_state_.yaw_rad;
  last_desired_yaw_rad_ = desired_yaw_rad;

  last_pos_error_world_ = {
    -odom_state_.pos_world.x,
    -odom_state_.pos_world.y,
    desired_height_m - odom_state_.pos_world.z,
  };
  last_vel_error_world_ = scale(odom_state_.vel_world, -1.0);

  TrackerVector3 command_world_m{
    kp_position_x_ * last_pos_error_world_.x + kv_velocity_x_ * last_vel_error_world_.x,
    kp_position_y_ * last_pos_error_world_.y + kv_velocity_y_ * last_vel_error_world_.y,
    kp_position_z_ * last_pos_error_world_.z + kv_velocity_z_ * last_vel_error_world_.z,
  };

  last_command_world_cm_ = scale(command_world_m, 100.0);
  limitWorldVelocity(last_command_world_cm_);

  TrackerVector3 command_body_cm_vec = rotateWorldToBody(last_command_world_cm_, odom_state_.yaw_rad);
  last_yaw_error_rad_ = normalizeAngleRad(desired_yaw_rad - odom_state_.yaw_rad);
  const double abs_yaw_error_deg = radToDeg(std::fabs(last_yaw_error_rad_));
  last_large_yaw_error_active_ =
    large_yaw_error_deg_threshold_ > 0.0 && abs_yaw_error_deg >= large_yaw_error_deg_threshold_;
  const double yaw_kp_scale = last_large_yaw_error_active_ ? large_yaw_kp_scale_ : 1.0;
  last_yaw_rate_feedforward_deg_s_ = 0.0;
  last_yaw_rate_feedback_deg_s_ = radToDeg(yaw_kp_ * yaw_kp_scale * last_yaw_error_rad_);
  last_yaw_rate_unclamped_deg_s_ = last_yaw_rate_feedback_deg_s_;
  last_yaw_rate_limit_deg_s_ =
    last_large_yaw_error_active_ ? large_yaw_velocity_deg_ : max_yaw_velocity_deg_;
  last_command_ = {
    command_body_cm_vec.x,
    command_body_cm_vec.y,
    command_body_cm_vec.z,
    last_yaw_rate_unclamped_deg_s_,
  };

  const double bootstrap_xy_norm = std::hypot(last_command_.vx_cm_s, last_command_.vy_cm_s);
  if (bootstrap_xy_norm > kTakeoffGuardMaxHorizontalVelocityCm && bootstrap_xy_norm > kEpsilon) {
    const double scale_factor = kTakeoffGuardMaxHorizontalVelocityCm / bootstrap_xy_norm;
    last_command_.vx_cm_s *= scale_factor;
    last_command_.vy_cm_s *= scale_factor;
  }
  last_command_.vz_cm_s = std::max(0.0, last_command_.vz_cm_s);

  limitBodyVelocity(last_command_, last_yaw_rate_limit_deg_s_);
  applyDeadband(last_command_);
  publishVelocityCommand(last_command_);
  publishDebug(desired_yaw_rad);
  publishStatus("bootstrap_takeoff");
  last_timeout_zero_sent_ = false;

  if (enable_debug_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_log_throttle_ms_,
      "bootstrap_takeoff ready=%s odom_z=%.1fcm target_z=%.1fcm pos_err=(%.2f, %.2f, %.2f)m cmd_body=(%.1f, %.1f, %.1f)cm/s",
      ready_received_ ? "true" : "false",
      currentOdomHeightCm(), desired_height_cm,
      last_pos_error_world_.x, last_pos_error_world_.y, last_pos_error_world_.z,
      last_command_.vx_cm_s, last_command_.vy_cm_s, last_command_.vz_cm_s);
  }
}

void PlannerVelocityTrackerNode::handleTrajectorySwitch(uint32_t trajectory_id)
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

  RCLCPP_INFO(
    get_logger(), "Trajectory switched: id %u -> %u, reset controller state.",
    active_trajectory_id_, trajectory_id);

  active_trajectory_id_ = trajectory_id;
  resetControlState();
}

void PlannerVelocityTrackerNode::resetControlState()
{
  last_command_ = {};
  last_pos_error_world_ = {};
  last_vel_error_world_ = {};
  last_command_world_cm_ = {};
  last_yaw_error_rad_ = 0.0;
  last_desired_yaw_rad_ = 0.0;
  last_yaw_rate_feedforward_deg_s_ = 0.0;
  last_yaw_rate_feedback_deg_s_ = 0.0;
  last_yaw_rate_unclamped_deg_s_ = 0.0;
  last_yaw_rate_limit_deg_s_ = 0.0;
  last_large_yaw_error_active_ = false;
  last_timeout_zero_sent_ = false;
}

bool PlannerVelocityTrackerNode::isPublishActive() const
{
  return current_control_mode_ == active_control_mode_;
}

void PlannerVelocityTrackerNode::publishVelocityCommand(const TrackerCommandState & command)
{
  std_msgs::msg::Float32MultiArray message;
  message.data = {
    static_cast<float>(command.vx_cm_s),
    static_cast<float>(command.vy_cm_s),
    static_cast<float>(command.vz_cm_s),
    static_cast<float>(command.yaw_rate_deg_s),
  };
  target_velocity_pub_->publish(message);
}

void PlannerVelocityTrackerNode::publishZeroCommand()
{
  if (tracker_mode_ == TrackerMode::TIMEOUT_BRAKE && !publish_zero_on_timeout_) {
    return;
  }

  if (tracker_mode_ == TrackerMode::TIMEOUT_BRAKE && last_timeout_zero_sent_) {
    return;
  }

  last_command_ = {};
  last_command_world_cm_ = {};
  last_yaw_rate_feedforward_deg_s_ = 0.0;
  last_yaw_rate_feedback_deg_s_ = 0.0;
  last_yaw_rate_unclamped_deg_s_ = 0.0;
  last_yaw_rate_limit_deg_s_ = 0.0;
  last_large_yaw_error_active_ = false;
  publishVelocityCommand(last_command_);
  last_timeout_zero_sent_ = (tracker_mode_ == TrackerMode::TIMEOUT_BRAKE);
}

void PlannerVelocityTrackerNode::publishDebug(double desired_yaw_rad)
{
  if (!publish_debug_topics_) {
    return;
  }

  publishReferencePose(desired_yaw_rad);

  auto publish_vector = [this](
      const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & publisher,
      const std::string & frame_id,
      const TrackerVector3 & value) {
      geometry_msgs::msg::Vector3Stamped message;
      message.header.stamp = now();
      message.header.frame_id = frame_id;
      message.vector.x = value.x;
      message.vector.y = value.y;
      message.vector.z = value.z;
      publisher->publish(message);
    };

  publish_vector(pos_error_pub_, world_frame_id_, last_pos_error_world_);
  publish_vector(vel_error_pub_, world_frame_id_, last_vel_error_world_);
  publish_vector(cmd_world_pub_, world_frame_id_, last_command_world_cm_);
  publish_vector(
    cmd_body_pub_, body_frame_id_,
    TrackerVector3{last_command_.vx_cm_s, last_command_.vy_cm_s, last_command_.vz_cm_s});
}

void PlannerVelocityTrackerNode::publishReferencePose(double desired_yaw_rad)
{
  if (!reference_pose_pub_) {
    return;
  }

  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = now();
  message.header.frame_id = world_frame_id_;
  message.pose.position.x = ref_state_.pos_world.x;
  message.pose.position.y = ref_state_.pos_world.y;
  message.pose.position.z = ref_state_.pos_world.z;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, desired_yaw_rad);
  message.pose.orientation = tf2::toMsg(quat);
  reference_pose_pub_->publish(message);
}

void PlannerVelocityTrackerNode::publishStatus(const std::string & reason)
{
  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3);
  stream << "{";
  stream << "\"reason\":\"" << reason << "\",";
  stream << "\"mode\":\"" << trackerModeToString() << "\",";
  stream << "\"publish_active\":" << (publish_active_ ? "true" : "false") << ",";
  stream << "\"control_mode\":" << static_cast<unsigned>(current_control_mode_) << ",";
  stream << "\"active_control_mode\":" << static_cast<unsigned>(active_control_mode_) << ",";
  stream << "\"traj_id\":" << ref_state_.trajectory_id << ",";
  stream << "\"ref_valid\":" << (ref_state_.valid ? "true" : "false") << ",";
  stream << "\"odom_valid\":" << (odom_state_.valid ? "true" : "false") << ",";
  stream << "\"ref_age_sec\":" << currentRefAgeSec(now()) << ",";
  stream << "\"odom_age_sec\":" << currentOdomAgeSec(now()) << ",";
  stream << "\"takeoff_guard_active\":" << (takeoff_guard_active_ ? "true" : "false") << ",";
  stream << "\"takeoff_guard_completed\":" << (takeoff_guard_completed_ ? "true" : "false") << ",";
  stream << "\"odom_height_cm\":" << currentOdomHeightCm() << ",";
  stream << "\"takeoff_guard_release_height_cm\":" << takeoffGuardReleaseHeightCm() << ",";
  stream << "\"guarded_takeoff_z_cm\":" << guarded_takeoff_z_cm_ << ",";
  stream << "\"takeoff_origin_xy_m\":[" << takeoff_guard_origin_pos_world_.x << "," << takeoff_guard_origin_pos_world_.y << "],";
  stream << "\"ref_yaw_deg\":" << radToDeg(ref_state_.yaw_rad) << ",";
  stream << "\"ref_yaw_rate_degps\":" << radToDeg(ref_state_.yaw_rate_rad_s) << ",";
  stream << "\"desired_yaw_deg\":" << radToDeg(last_desired_yaw_rad_) << ",";
  stream << "\"odom_yaw_deg\":" << radToDeg(odom_state_.yaw_rad) << ",";
  stream << "\"odom_yaw_rate_degps\":" << radToDeg(odom_state_.yaw_rate_rad_s) << ",";
  stream << "\"pos_error_m\":[" << last_pos_error_world_.x << "," << last_pos_error_world_.y << "," << last_pos_error_world_.z << "],";
  stream << "\"vel_error_mps\":[" << last_vel_error_world_.x << "," << last_vel_error_world_.y << "," << last_vel_error_world_.z << "],";
  stream << "\"cmd_body_cmps\":[" << last_command_.vx_cm_s << "," << last_command_.vy_cm_s << "," << last_command_.vz_cm_s << "],";
  stream << "\"yaw_error_deg\":" << radToDeg(last_yaw_error_rad_) << ",";
  stream << "\"yaw_rate_ff_degps\":" << last_yaw_rate_feedforward_deg_s_ << ",";
  stream << "\"yaw_rate_fb_degps\":" << last_yaw_rate_feedback_deg_s_ << ",";
  stream << "\"yaw_rate_unclamped_degps\":" << last_yaw_rate_unclamped_deg_s_ << ",";
  stream << "\"yaw_rate_limit_degps\":" << last_yaw_rate_limit_deg_s_ << ",";
  stream << "\"large_yaw_error_active\":" << (last_large_yaw_error_active_ ? "true" : "false") << ",";
  stream << "\"yaw_rate_cmd_degps\":" << last_command_.yaw_rate_deg_s << ",";
  stream << "\"yaw_rate_degps\":" << last_command_.yaw_rate_deg_s;
  stream << "}";
  message.data = stream.str();
  status_pub_->publish(message);
}

TrackerVector3 PlannerVelocityTrackerNode::rotateWorldToBody(
  const TrackerVector3 & value,
  double yaw_rad)
{
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);
  return {
    cos_yaw * value.x + sin_yaw * value.y,
    -sin_yaw * value.x + cos_yaw * value.y,
    value.z,
  };
}

TrackerVector3 PlannerVelocityTrackerNode::rotateBodyToWorld(
  const TrackerVector3 & value,
  double yaw_rad)
{
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);
  return {
    cos_yaw * value.x - sin_yaw * value.y,
    sin_yaw * value.x + cos_yaw * value.y,
    value.z,
  };
}

TrackerVector3 PlannerVelocityTrackerNode::subtract(
  const TrackerVector3 & lhs,
  const TrackerVector3 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

TrackerVector3 PlannerVelocityTrackerNode::scale(const TrackerVector3 & value, double scalar)
{
  return {value.x * scalar, value.y * scalar, value.z * scalar};
}

double PlannerVelocityTrackerNode::clampAbs(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

double PlannerVelocityTrackerNode::normalizeAngleRad(double angle_rad)
{
  return angles::normalize_angle(angle_rad);
}

double PlannerVelocityTrackerNode::radToDeg(double angle_rad)
{
  return angle_rad * 180.0 / M_PI;
}

double PlannerVelocityTrackerNode::degToRad(double angle_deg)
{
  return angle_deg * M_PI / 180.0;
}

bool PlannerVelocityTrackerNode::isFinite(const TrackerVector3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double PlannerVelocityTrackerNode::finiteOrZero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

std::string PlannerVelocityTrackerNode::trackerModeToString() const
{
  switch (tracker_mode_) {
    case TrackerMode::WAIT_ODOM:
      return "WAIT_ODOM";
    case TrackerMode::WAIT_REF:
      return "WAIT_REF";
    case TrackerMode::TRACKING:
      return "TRACKING";
    case TrackerMode::TIMEOUT_BRAKE:
      return "TIMEOUT_BRAKE";
  }
  return "UNKNOWN";
}

void PlannerVelocityTrackerNode::limitWorldVelocity(TrackerVector3 & command_world_cm) const
{
  command_world_cm.z = clampAbs(command_world_cm.z, max_vertical_velocity_cm_);

  const double xy_norm = std::hypot(command_world_cm.x, command_world_cm.y);
  if (xy_norm > max_linear_velocity_cm_ && xy_norm > kEpsilon) {
    const double scale_factor = max_linear_velocity_cm_ / xy_norm;
    command_world_cm.x *= scale_factor;
    command_world_cm.y *= scale_factor;
  }
}

void PlannerVelocityTrackerNode::limitBodyVelocity(
  TrackerCommandState & command_body,
  double max_yaw_velocity_deg) const
{
  command_body.vz_cm_s = clampAbs(command_body.vz_cm_s, max_vertical_velocity_cm_);
  command_body.yaw_rate_deg_s = clampAbs(command_body.yaw_rate_deg_s, max_yaw_velocity_deg);

  const double xy_norm = std::hypot(command_body.vx_cm_s, command_body.vy_cm_s);
  if (xy_norm > max_linear_velocity_cm_ && xy_norm > kEpsilon) {
    const double scale_factor = max_linear_velocity_cm_ / xy_norm;
    command_body.vx_cm_s *= scale_factor;
    command_body.vy_cm_s *= scale_factor;
  }
}

void PlannerVelocityTrackerNode::applyDeadband(TrackerCommandState & command_body) const
{
  if (std::fabs(command_body.vx_cm_s) < min_publish_speed_cm_) {
    command_body.vx_cm_s = 0.0;
  }
  if (std::fabs(command_body.vy_cm_s) < min_publish_speed_cm_) {
    command_body.vy_cm_s = 0.0;
  }
  if (std::fabs(command_body.vz_cm_s) < min_publish_speed_cm_) {
    command_body.vz_cm_s = 0.0;
  }
}

double PlannerVelocityTrackerNode::currentOdomAgeSec(const rclcpp::Time & current_time) const
{
  if (!odom_state_.valid) {
    return -1.0;
  }
  return std::max(0.0, (current_time - odom_state_.stamp).seconds());
}

double PlannerVelocityTrackerNode::currentRefAgeSec(const rclcpp::Time & current_time) const
{
  if (!ref_state_.valid) {
    return -1.0;
  }
  return std::max(0.0, (current_time - ref_state_.stamp).seconds());
}

void PlannerVelocityTrackerNode::updateTakeoffGuardState()
{
  if (!enable_takeoff_height_guard_ || !odom_state_.valid) {
    takeoff_guard_active_ = false;
    return;
  }

  if (takeoff_guard_active_) {
    if (currentOdomHeightCm() >= takeoffGuardReleaseHeightCm()) {
      takeoff_guard_active_ = false;
      takeoff_guard_completed_ = true;
    }
    return;
  }

  if (takeoff_guard_completed_) {
    return;
  }

  if (currentOdomHeightCm() < takeoffGuardReleaseHeightCm()) {
    takeoff_guard_active_ = true;
    takeoff_guard_hold_yaw_rad_ = odom_state_.yaw_rad;
    takeoff_guard_origin_pos_world_ = TrackerVector3{};
  }
}

double PlannerVelocityTrackerNode::currentOdomHeightCm() const
{
  return odom_state_.pos_world.z * 100.0;
}

double PlannerVelocityTrackerNode::takeoffGuardReleaseHeightCm() const
{
  return std::max(0.0, guarded_takeoff_z_cm_ - kTakeoffGuardReleaseMarginCm);
}

}  // namespace activity_control_pkg