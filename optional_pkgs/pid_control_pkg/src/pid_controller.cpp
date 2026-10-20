#include "pid_control_pkg/pid_controller.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pid_control_pkg
{

PIDController::PIDController(
  double kp,
  double ki,
  double kd,
  double max_output,
  double min_output,
  double integral_limit,
  double deadzone)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  max_output_(max_output),
  min_output_(min_output),
  integral_limit_(integral_limit),
  deadzone_(deadzone),
  prev_error_(0.0),
  current_error_(0.0),
  integral_(0.0),
  prev_derivative_(0.0),
  first_call_(true),
  derivative_filter_alpha_(0.8)
{
}

double PIDController::calculate(double setpoint, double measured_value, double dt)
{
  current_error_ = setpoint - measured_value;

  if (std::fabs(current_error_) < deadzone_) {
    current_error_ = 0.0;
  }

  if (first_call_) {
    prev_error_ = current_error_;
    first_call_ = false;
  }

  const double proportional = kp_ * current_error_;

  double integral_term = 0.0;
  if (std::fabs(ki_) > 1e-9) {
    integral_ += current_error_ * dt;
    if (integral_ > integral_limit_) {
      integral_ = integral_limit_;
    } else if (integral_ < -integral_limit_) {
      integral_ = -integral_limit_;
    }
    integral_term = ki_ * integral_;
  } else {
    // I-term disabled: avoid hidden accumulation that can trigger late-route oscillation.
    integral_ = 0.0;
  }

  const double derivative_raw = (dt > 0.0) ? (current_error_ - prev_error_) / dt : 0.0;
  const double derivative_filtered = derivative_filter_alpha_ * prev_derivative_ +
    (1.0 - derivative_filter_alpha_) * derivative_raw;
  const double derivative_term = kd_ * derivative_filtered;

  double output = proportional + integral_term + derivative_term;

  if (output > max_output_) {
    output = max_output_;
  } else if (output < min_output_) {
    output = min_output_;
  }

  if (std::fabs(ki_) > 1e-9 && ((output >= max_output_ && current_error_ > 0.0) ||
    (output <= min_output_ && current_error_ < 0.0)))
  {
    integral_ -= current_error_ * dt;
  }

  prev_error_ = current_error_;
  prev_derivative_ = derivative_filtered;

  return output;
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  current_error_ = 0.0;
  integral_ = 0.0;
  prev_derivative_ = 0.0;
  first_call_ = true;
}

void PIDController::setPID(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double max_output, double min_output)
{
  max_output_ = max_output;
  min_output_ = min_output;
}

void PIDController::setIntegralLimit(double integral_limit)
{
  integral_limit_ = integral_limit;
}

void PIDController::setDeadzone(double deadzone)
{
  deadzone_ = deadzone;
}

PositionPIDController::PositionPIDController()
: rclcpp::Node("position_pid_controller"),
  enable_visual_fine_tune_(true),
  enable_pid_debug_publish_(true),
  pojiang_flag_(false),
  pid_x_(0.0, 0.0, 0.0),
  pid_y_(0.0, 0.0, 0.0),
  pid_yaw_(0.0, 0.0, 0.0),
  pid_z_(0.0, 0.0, 0.0),
  pid_xy_speed_(0.0, 0.0, 0.0),
  target_x_cm_(0.0),
  target_y_cm_(0.0),
  target_z_cm_(0.0),
  target_yaw_deg_(0.0),
  has_target_position_(false),
  has_target_height_(false),
  has_external_height_(false),
  current_x_cm_(0.0),
  current_y_cm_(0.0),
  current_yaw_deg_(0.0),
  current_z_cm_(0.0),
  has_current_pose_(false),
  has_qr_offset_(false),
  control_frequency_(100.0),
  fastlio_world_frame_("world"),
  fastlio_body_frame_("body"),
  use_height_topic_for_z_(false),
  yaw_first_threshold_deg_(15.0),
  near_target_distance_cm_(25.0),
  near_yaw_lock_threshold_deg_(5.0),
  enable_target_switch_filter_(false),
  min_target_switch_xy_cm_(8.0),
  min_target_switch_z_cm_(4.0),
  min_target_switch_yaw_deg_(8.0),
  switch_release_distance_cm_(25.0),
  target_switch_accept_count_(0),
  target_switch_skip_count_(0),
  control_mode_(ControlMode::NORMAL),
  Xpos_tolerance_(6.0),
  Ypos_tolerance_(6.0),
  yaw_tolerance_(0.3),
  height_tolerance_(6.0),
  max_linear_vel_(36.0),
  max_angular_vel_(30.0),
  max_vertical_vel_(40.0),
  max_slow_vel_(20.0),
  distance_xy_cm_(0.0),
  error_x_cm_(0.0),
  error_y_cm_(0.0),
  error_yaw_deg_(0.0),
  error_z_cm_(0.0),
  current_pid_mode_(0),
  qr_offset_x_(0.0),
  qr_offset_y_(0.0),
  qr_target_x_(380.0),
  qr_target_y_(170.0),
  qr_data_timeout_(0.5),
  last_qr_update_time_(now()),
  last_update_time_(now()),
  control_mode_topic_("/precision_landing/control_mode"),
  active_control_mode_(1),
  current_control_mode_(1),
  publish_active_(false)
{
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto target_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_position", target_qos,
    std::bind(&PositionPIDController::targetPositionCallback, this, std::placeholders::_1));

  if (use_height_topic_for_z_) {
    height_sub_ = create_subscription<std_msgs::msg::Int16>(
      "/height", rclcpp::QoS(10),
      std::bind(&PositionPIDController::heightCallback, this, std::placeholders::_1));
  }

  pid_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/pid_mode", rclcpp::QoS(10),
    std::bind(&PositionPIDController::pidModeCallback, this, std::placeholders::_1));

  pojiang_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/pojiang", rclcpp::QoS(10),
    std::bind(&PositionPIDController::pojiangCallback, this, std::placeholders::_1));

  control_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    control_mode_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PositionPIDController::controlModeCallback, this, std::placeholders::_1));

  qr_offset_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "/qr_code_position", rclcpp::QoS(10),
    std::bind(&PositionPIDController::qrOffsetCallback, this, std::placeholders::_1));

  target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", rclcpp::QoS(10));

  pid_debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/pid_debug", rclcpp::QoS(10));

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PositionPIDController::onParametersSet, this, std::placeholders::_1));

  const double period_sec = 1.0 / std::max(control_frequency_, 1.0);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
    std::bind(&PositionPIDController::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Position PID Controller initialized");
  RCLCPP_INFO(get_logger(), "Control frequency: %.1f Hz", control_frequency_);
  RCLCPP_INFO(
    get_logger(), "Frames: fastlio_world=%s, fastlio_body=%s",
    fastlio_world_frame_.c_str(), fastlio_body_frame_.c_str());
  RCLCPP_INFO(
    get_logger(), "Control mode topic: %s, active_mode=%u, z_source=%s",
    control_mode_topic_.c_str(), static_cast<unsigned>(active_control_mode_),
    use_height_topic_for_z_ ? "height" : "odom_tf");
}

void PositionPIDController::targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(),
      "Target position message should contain 4 float values [x_cm, y_cm, z_cm, yaw_deg]");
    return;
  }

  const double incoming_x = msg->data[0];
  const double incoming_y = msg->data[1];
  const double incoming_z = msg->data[2];
  const double incoming_yaw = msg->data[3];

  if (enable_target_switch_filter_ && has_target_position_) {
    const double delta_xy = std::hypot(incoming_x - target_x_cm_, incoming_y - target_y_cm_);
    const double delta_z = std::fabs(incoming_z - target_z_cm_);
    const double delta_yaw = std::fabs(normalizeAngleDeg(incoming_yaw - target_yaw_deg_));

    if (delta_xy < min_target_switch_xy_cm_ &&
      delta_z < min_target_switch_z_cm_ &&
      delta_yaw < min_target_switch_yaw_deg_)
    {
      bool allow_release = false;
      if (has_current_pose_) {
        const double dist_to_active_target = std::hypot(target_x_cm_ - current_x_cm_, target_y_cm_ - current_y_cm_);
        allow_release = dist_to_active_target <= switch_release_distance_cm_;
      }

      if (!allow_release) {
        ++target_switch_skip_count_;
        if (target_switch_skip_count_ % 100 == 0) {
          RCLCPP_INFO(
            get_logger(),
            "Target switch filter: skipped=%lu accepted=%lu (min delta: xy=%.2fcm z=%.2fcm yaw=%.2fdeg)",
            static_cast<unsigned long>(target_switch_skip_count_),
            static_cast<unsigned long>(target_switch_accept_count_),
            min_target_switch_xy_cm_, min_target_switch_z_cm_, min_target_switch_yaw_deg_);
        }
        return;
      }
    }
  }

  target_x_cm_ = incoming_x;
  target_y_cm_ = incoming_y;
  target_z_cm_ = incoming_z;
  target_yaw_deg_ = incoming_yaw;
  has_target_position_ = true;
  ++target_switch_accept_count_;
}

void PositionPIDController::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_z_cm_ = static_cast<double>(msg->data);
  has_target_height_ = true;
  has_external_height_ = true;
}

void PositionPIDController::pidModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const uint8_t new_mode = msg->data;
  if (new_mode == current_pid_mode_) {
    return;
  }

  current_pid_mode_ = new_mode;
  if (current_pid_mode_ == 0) {
    applyPIDParams(navigate_params_);
    RCLCPP_INFO(get_logger(), "Switch to Navigate Mode");
  } else if (current_pid_mode_ == 1 && enable_visual_fine_tune_) {
    applyPIDParams(fine_tune_params_);
    RCLCPP_INFO(get_logger(), "Switch to Fine Tune Mode");
  }

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();
  pid_yaw_.reset();
  pid_xy_speed_.reset();
}

void PositionPIDController::pojiangCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  pojiang_flag_ = (msg->data == 1);
}

void PositionPIDController::qrOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  qr_offset_x_ = msg->x;
  qr_offset_y_ = msg->y;
  has_qr_offset_ = true;
  last_qr_update_time_ = now();
}

void PositionPIDController::controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  current_control_mode_ = msg->data;
  const bool next_publish_active = current_control_mode_ == active_control_mode_;
  if (next_publish_active == publish_active_) {
    return;
  }

  publish_active_ = next_publish_active;
  last_update_time_ = now();
  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();
  pid_yaw_.reset();
  pid_xy_speed_.reset();
  if (publish_active_) {
    RCLCPP_INFO(
      get_logger(), "Position PID controller activated on control mode %u.",
      static_cast<unsigned>(current_control_mode_));
  } else {
    RCLCPP_INFO(
      get_logger(), "Position PID controller muted on control mode %u.",
      static_cast<unsigned>(current_control_mode_));
  }
}

bool PositionPIDController::getCurrentPose()
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      fastlio_world_frame_, fastlio_body_frame_, tf2::TimePointZero);

    current_x_cm_ = meterToCm(transform.transform.translation.x);
    current_y_cm_ = meterToCm(transform.transform.translation.y);
    if (!(use_height_topic_for_z_ && has_external_height_)) {
      current_z_cm_ = meterToCm(transform.transform.translation.z);
      has_target_height_ = true;
    }

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_deg_ = radToDeg(yaw);

    has_current_pose_ = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to lookup transform %s -> %s: %s",
      fastlio_world_frame_.c_str(), fastlio_body_frame_.c_str(), ex.what());
    return false;
  }
}

double PositionPIDController::normalizeAngleDeg(double angle_deg) const
{
  const double result = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(result);
}

void PositionPIDController::calculateErrors()
{
  if (current_pid_mode_ == 0 || !enable_visual_fine_tune_) {
    error_x_cm_ = target_x_cm_ - current_x_cm_;
    error_y_cm_ = target_y_cm_ - current_y_cm_;
    distance_xy_cm_ = std::hypot(error_x_cm_, error_y_cm_);
    error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);

    has_qr_offset_ = false;

    if (has_target_height_) {
      error_z_cm_ = target_z_cm_ - current_z_cm_;
    } else {
      error_z_cm_ = 0.0;
    }
  } else if (current_pid_mode_ == 1) {
    const double qr_age = (now() - last_qr_update_time_).seconds();
    const bool qr_data_valid = has_qr_offset_ && (qr_age < qr_data_timeout_);

    if (qr_data_valid && (qr_offset_x_ > 0.0 || qr_offset_y_ > 0.0)) {
      error_x_cm_ = qr_target_x_ - qr_offset_x_;
      error_z_cm_ = -(qr_target_y_ - qr_offset_y_);
    } else {
      error_x_cm_ = 0.0;
      error_z_cm_ = 0.0;
      if (qr_age > qr_data_timeout_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[PID] QR data timeout(%.2fs), hold", qr_age);
      }
    }

    error_y_cm_ = target_y_cm_ - current_y_cm_;
    error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);
    distance_xy_cm_ = std::hypot(error_x_cm_, error_y_cm_);
  }
}

bool PositionPIDController::isTargetReached() const
{
  return (std::fabs(error_x_cm_) <= Xpos_tolerance_ &&
    std::fabs(error_y_cm_) <= Ypos_tolerance_ &&
    std::fabs(error_yaw_deg_) <= yaw_tolerance_ &&
    (std::fabs(error_z_cm_) <= height_tolerance_ || !has_target_height_));
}

void PositionPIDController::setControlMode(ControlMode mode)
{
  control_mode_ = mode;
  double limit = max_linear_vel_;

  switch (mode) {
    case ControlMode::NORMAL:
      limit = max_linear_vel_;
      break;
    case ControlMode::SLOW:
    case ControlMode::LOCK_Y:
    case ControlMode::LOCK_X:
      limit = max_slow_vel_;
      break;
    case ControlMode::HOVER:
      limit = 10.0;
      break;
  }

  pid_x_.setOutputLimits(limit, -limit);
  pid_y_.setOutputLimits(limit, -limit);
  pid_xy_speed_.setOutputLimits(limit, -limit);
}

void PositionPIDController::computeXYVelocityByControlMode(double dt, double & vel_x_cm, double & vel_y_cm)
{
  switch (control_mode_) {
    case ControlMode::NORMAL:
    case ControlMode::SLOW:
    {
      if (distance_xy_cm_ > 0.1) {
        double speed_cmd = -pid_xy_speed_.calculate(0.0, distance_xy_cm_, dt);
        if (speed_cmd < 0.0) {
          speed_cmd = 0.0;
        }

        const double cos_theta = error_x_cm_ / distance_xy_cm_;
        const double sin_theta = error_y_cm_ / distance_xy_cm_;
        vel_x_cm = speed_cmd * cos_theta;
        vel_y_cm = speed_cmd * sin_theta;
      } else {
        vel_x_cm = 0.0;
        vel_y_cm = 0.0;
      }
      break;
    }

    case ControlMode::LOCK_Y:
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt) * 0.4;
      break;

    case ControlMode::LOCK_X:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt) * 0.4;
      break;

    case ControlMode::HOVER:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;

    default:
      vel_x_cm = 0.0;
      vel_y_cm = 0.0;
      break;
  }
}

void PositionPIDController::transformToBodyFrame(double & vel_x_cm, double & vel_y_cm)
{
  const double yaw_rad = degToRad(current_yaw_deg_);
  const double vel_x_body = vel_x_cm * std::cos(yaw_rad) + vel_y_cm * std::sin(yaw_rad);
  const double vel_y_body = -vel_x_cm * std::sin(yaw_rad) + vel_y_cm * std::cos(yaw_rad);
  vel_x_cm = vel_x_body;
  vel_y_cm = vel_y_body;
}

double PositionPIDController::computeYawVelocity(double dt)
{
  return pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);
}

double PositionPIDController::computeZVelocity(double dt)
{
  if (!has_target_height_) {
    return 0.0;
  }

  if (current_pid_mode_ == 0 || !enable_visual_fine_tune_) {
    return pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
  }
  return pid_z_.calculate(0.0, -error_z_cm_, dt);
}

PositionPIDController::VelocityOutput PositionPIDController::computeNavigateModeVelocity(double dt)
{
  VelocityOutput vel{0.0, 0.0, 0.0, 0.0};

  computeXYVelocityByControlMode(dt, vel.vel_x_cm, vel.vel_y_cm);
  transformToBodyFrame(vel.vel_x_cm, vel.vel_y_cm);
  vel.vel_yaw_deg = computeYawVelocity(dt);
  vel.vel_z_cm = computeZVelocity(dt);

  return vel;
}

PositionPIDController::VelocityOutput PositionPIDController::computeFineTuneModeVelocity(double dt)
{
  VelocityOutput vel{0.0, 0.0, 0.0, 0.0};

  vel.vel_x_cm = pid_x_.calculate(0.0, -error_x_cm_, dt);
  const double pid_y_out = pid_y_.calculate(0.0, -error_y_cm_, dt);
  vel.vel_y_cm = pid_y_out * std::cos(degToRad(current_yaw_deg_));
  vel.vel_yaw_deg = computeYawVelocity(dt);
  vel.vel_z_cm = computeZVelocity(dt);

  return vel;
}

std_msgs::msg::Float32MultiArray PositionPIDController::fillCmdVelMessage(const VelocityOutput & vel)
{
  std_msgs::msg::Float32MultiArray cmd_vel;
  cmd_vel.data.resize(4);
  cmd_vel.data[0] = static_cast<float>(vel.vel_x_cm);
  cmd_vel.data[1] = static_cast<float>(vel.vel_y_cm);
  cmd_vel.data[2] = static_cast<float>(vel.vel_z_cm);
  cmd_vel.data[3] = static_cast<float>(vel.vel_yaw_deg);
  return cmd_vel;
}

std_msgs::msg::Float32MultiArray PositionPIDController::processPID(double dt)
{
  calculateErrors();

  VelocityOutput vel{0.0, 0.0, 0.0, 0.0};
  if (current_pid_mode_ == 0 || !enable_visual_fine_tune_) {
    vel = computeNavigateModeVelocity(dt);
  } else if (current_pid_mode_ == 1) {
    vel = computeFineTuneModeVelocity(dt);
  }

  // ROS1 strategy: when yaw error is large, stop XY translation and rotate first.
  if (std::fabs(error_yaw_deg_) > yaw_first_threshold_deg_) {
    vel.vel_x_cm = 0.0;
    vel.vel_y_cm = 0.0;
  }

  auto cmd_vel = fillCmdVelMessage(vel);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "PID output: [%.2f, %.2f, %.2f, %.2f]",
    vel.vel_x_cm, vel.vel_y_cm, vel.vel_z_cm, vel.vel_yaw_deg);

  if (enable_pid_debug_publish_) {
    publishPIDDebug(vel);
  }

  return cmd_vel;
}

void PositionPIDController::controlTimerCallback()
{
  if (!publish_active_) {
    return;
  }

  if (!has_target_position_) {
    return;
  }

  if (!getCurrentPose()) {
    return;
  }

  const rclcpp::Time now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  if (dt <= 0.0) {
    dt = 1.0 / std::max(control_frequency_, 1.0);
  }
  last_update_time_ = now_time;

  auto cmd_vel = processPID(dt);
  if ((target_x_cm_ == -1.0 && target_y_cm_ == -1.0 && target_z_cm_ == -1.0 && target_yaw_deg_ == -1.0) ||
    pojiang_flag_)
  {
    cmd_vel.data[0] = 0.0f;
    cmd_vel.data[1] = 0.0f;
    cmd_vel.data[2] = -30.0f;
    cmd_vel.data[3] = 0.0f;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[PID] Landing mode: z_vel=-30cm/s");
  }

  target_velocity_pub_->publish(cmd_vel);

  if (isTargetReached()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Target reached! yaw error: %.1fdeg", error_yaw_deg_);
  }
}

void PositionPIDController::loadParameters()
{
  control_frequency_ = declare_parameter<double>("control_frequency", 100.0);
  control_mode_topic_ = declare_parameter<std::string>(
    "control_mode_topic", "/precision_landing/control_mode");
  active_control_mode_ = static_cast<uint8_t>(std::max<int64_t>(
    0, declare_parameter<int64_t>("active_control_mode", 1)));
  current_control_mode_ = active_control_mode_;
  publish_active_ = declare_parameter<bool>("start_active", true);
  fastlio_world_frame_ = declare_parameter<std::string>("fastlio_world_frame", "world");
  fastlio_body_frame_ = declare_parameter<std::string>("fastlio_body_frame", "body");
  use_height_topic_for_z_ = declare_parameter<bool>("use_height_topic_for_z", false);
  yaw_first_threshold_deg_ = declare_parameter<double>("yaw_first_threshold_deg", 15.0);
  near_target_distance_cm_ = declare_parameter<double>("near_target_distance_cm", 25.0);
  near_yaw_lock_threshold_deg_ = declare_parameter<double>("near_yaw_lock_threshold_deg", 5.0);
  enable_target_switch_filter_ = declare_parameter<bool>("enable_target_switch_filter", false);
  min_target_switch_xy_cm_ = std::max(0.0, declare_parameter<double>("min_target_switch_xy_cm", 8.0));
  min_target_switch_z_cm_ = std::max(0.0, declare_parameter<double>("min_target_switch_z_cm", 4.0));
  min_target_switch_yaw_deg_ = std::max(0.0, declare_parameter<double>("min_target_switch_yaw_deg", 8.0));
  switch_release_distance_cm_ = std::max(
    0.0, declare_parameter<double>("switch_release_distance_cm", near_target_distance_cm_));
  enable_visual_fine_tune_ = declare_parameter<bool>("enable_visual_fine_tune", true);
  enable_pid_debug_publish_ = declare_parameter<bool>("enable_pid_debug_publish", true);

  qr_target_x_ = declare_parameter<double>("qr_target_x", 380.0);
  qr_target_y_ = declare_parameter<double>("qr_target_y", 170.0);
  qr_data_timeout_ = declare_parameter<double>("qr_data_timeout", 0.5);

  navigate_params_.kp_xy = declare_parameter<double>("navigate.kp_xy", 0.8);
  navigate_params_.ki_xy = declare_parameter<double>("navigate.ki_xy", 0.0);
  navigate_params_.kd_xy = declare_parameter<double>("navigate.kd_xy", 0.07);
  navigate_params_.kp_yaw = declare_parameter<double>("navigate.kp_yaw", 0.8);
  navigate_params_.ki_yaw = declare_parameter<double>("navigate.ki_yaw", 0.0);
  navigate_params_.kd_yaw = declare_parameter<double>("navigate.kd_yaw", 0.08);
  navigate_params_.kp_z = declare_parameter<double>("navigate.kp_z", 1.2);
  navigate_params_.ki_z = declare_parameter<double>("navigate.ki_z", 0.0);
  navigate_params_.kd_z = declare_parameter<double>("navigate.kd_z", 0.0);
  navigate_params_.max_linear_vel = declare_parameter<double>("navigate.max_linear_velocity", 36.0);
  navigate_params_.max_angular_vel = declare_parameter<double>("navigate.max_angular_velocity", 30.0);
  navigate_params_.max_vertical_vel = declare_parameter<double>("navigate.max_vertical_velocity", 40.0);
  navigate_params_.min_linear_vel = declare_parameter<double>("navigate.min_linear_velocity", -36.0);
  navigate_params_.min_angular_vel = declare_parameter<double>("navigate.min_angular_velocity", -30.0);
  navigate_params_.min_vertical_vel = declare_parameter<double>("navigate.min_vertical_velocity", -40.0);
  navigate_params_.Xpos_tolerance = declare_parameter<double>("navigate.Xpos_tolerance", 4.0);
  navigate_params_.Ypos_tolerance = declare_parameter<double>("navigate.Ypos_tolerance", 4.0);
  navigate_params_.yaw_tolerance = declare_parameter<double>("navigate.yaw_tolerance", 5.0);
  navigate_params_.height_tolerance = declare_parameter<double>("navigate.height_tolerance", 3.0);

  fine_tune_params_.kp_xy = declare_parameter<double>("fine_tune.kp_xy", 0.3);
  fine_tune_params_.ki_xy = declare_parameter<double>("fine_tune.ki_xy", 0.0);
  fine_tune_params_.kd_xy = declare_parameter<double>("fine_tune.kd_xy", 0.05);
  fine_tune_params_.kp_yaw = declare_parameter<double>("fine_tune.kp_yaw", 0.8);
  fine_tune_params_.ki_yaw = declare_parameter<double>("fine_tune.ki_yaw", 0.0);
  fine_tune_params_.kd_yaw = declare_parameter<double>("fine_tune.kd_yaw", 0.08);
  fine_tune_params_.kp_z = declare_parameter<double>("fine_tune.kp_z", 0.3);
  fine_tune_params_.ki_z = declare_parameter<double>("fine_tune.ki_z", 0.0);
  fine_tune_params_.kd_z = declare_parameter<double>("fine_tune.kd_z", 0.0);
  fine_tune_params_.max_linear_vel = declare_parameter<double>("fine_tune.max_linear_velocity", 6.0);
  fine_tune_params_.max_angular_vel = declare_parameter<double>("fine_tune.max_angular_velocity", 6.0);
  fine_tune_params_.max_vertical_vel = declare_parameter<double>("fine_tune.max_vertical_velocity", 6.0);
  fine_tune_params_.min_linear_vel = declare_parameter<double>("fine_tune.min_linear_velocity", -6.0);
  fine_tune_params_.min_angular_vel = declare_parameter<double>("fine_tune.min_angular_velocity", -6.0);
  fine_tune_params_.min_vertical_vel = declare_parameter<double>("fine_tune.min_vertical_velocity", -6.0);
  fine_tune_params_.Xpos_tolerance = declare_parameter<double>("fine_tune.Xpos_tolerance", 6.0);
  fine_tune_params_.Ypos_tolerance = declare_parameter<double>("fine_tune.Ypos_tolerance", 6.0);
  fine_tune_params_.yaw_tolerance = declare_parameter<double>("fine_tune.yaw_tolerance", 5.0);
  fine_tune_params_.height_tolerance = declare_parameter<double>("fine_tune.height_tolerance", 6.0);

  max_slow_vel_ = declare_parameter<double>("max_slow_velocity", 20.0);

  applyPIDParams(navigate_params_);
  setControlMode(control_mode_);
  RCLCPP_INFO(get_logger(), "Navigate and fine_tune PID parameter sets loaded");
}

void PositionPIDController::applyPIDParams(const PIDParams & params)
{
  pid_x_.setPID(params.kp_xy, params.ki_xy, params.kd_xy);
  pid_y_.setPID(params.kp_xy, params.ki_xy, params.kd_xy);
  pid_z_.setPID(params.kp_z, params.ki_z, params.kd_z);
  pid_yaw_.setPID(params.kp_yaw, params.ki_yaw, params.kd_yaw);
  pid_xy_speed_.setPID(params.kp_xy, params.ki_xy, params.kd_xy);

  pid_x_.setOutputLimits(params.max_linear_vel, params.min_linear_vel);
  pid_y_.setOutputLimits(params.max_linear_vel, params.min_linear_vel);
  pid_z_.setOutputLimits(params.max_vertical_vel, params.min_vertical_vel);
  pid_yaw_.setOutputLimits(params.max_angular_vel, params.min_angular_vel);
  pid_xy_speed_.setOutputLimits(params.max_linear_vel, params.min_linear_vel);

  pid_x_.setIntegralLimit(20.0);
  pid_y_.setIntegralLimit(20.0);
  pid_z_.setIntegralLimit(30.0);
  pid_yaw_.setIntegralLimit(8.0);
  pid_xy_speed_.setIntegralLimit(20.0);

  max_linear_vel_ = params.max_linear_vel;
  max_angular_vel_ = params.max_angular_vel;
  max_vertical_vel_ = params.max_vertical_vel;

  Xpos_tolerance_ = params.Xpos_tolerance;
  Ypos_tolerance_ = params.Ypos_tolerance;
  yaw_tolerance_ = params.yaw_tolerance;
  height_tolerance_ = params.height_tolerance;

  pid_yaw_.setDeadzone(params.yaw_tolerance);
}

const PositionPIDController::PIDParams & PositionPIDController::getActivePIDParams() const
{
  if (current_pid_mode_ == 1 && enable_visual_fine_tune_) {
    return fine_tune_params_;
  }
  return navigate_params_;
}

void PositionPIDController::publishPIDDebug(const VelocityOutput & vel)
{
  const PIDParams & params = getActivePIDParams();

  std_msgs::msg::Float32MultiArray debug_msg;
  debug_msg.data.resize(24);
  debug_msg.data[0] = static_cast<float>(current_pid_mode_);
  debug_msg.data[1] = static_cast<float>(error_x_cm_);
  debug_msg.data[2] = static_cast<float>(error_y_cm_);
  debug_msg.data[3] = static_cast<float>(error_z_cm_);
  debug_msg.data[4] = static_cast<float>(error_yaw_deg_);
  debug_msg.data[5] = static_cast<float>(distance_xy_cm_);
  debug_msg.data[6] = static_cast<float>(vel.vel_x_cm);
  debug_msg.data[7] = static_cast<float>(vel.vel_y_cm);
  debug_msg.data[8] = static_cast<float>(vel.vel_z_cm);
  debug_msg.data[9] = static_cast<float>(vel.vel_yaw_deg);
  debug_msg.data[10] = static_cast<float>(params.kp_xy);
  debug_msg.data[11] = static_cast<float>(params.ki_xy);
  debug_msg.data[12] = static_cast<float>(params.kd_xy);
  debug_msg.data[13] = static_cast<float>(params.kp_yaw);
  debug_msg.data[14] = static_cast<float>(params.ki_yaw);
  debug_msg.data[15] = static_cast<float>(params.kd_yaw);
  debug_msg.data[16] = static_cast<float>(params.kp_z);
  debug_msg.data[17] = static_cast<float>(params.ki_z);
  debug_msg.data[18] = static_cast<float>(params.kd_z);
  debug_msg.data[19] = static_cast<float>(params.max_linear_vel);
  debug_msg.data[20] = static_cast<float>(params.max_angular_vel);
  debug_msg.data[21] = static_cast<float>(params.max_vertical_vel);
  debug_msg.data[22] = static_cast<float>(params.Xpos_tolerance);
  debug_msg.data[23] = static_cast<float>(params.yaw_tolerance);
  pid_debug_pub_->publish(debug_msg);
}

rcl_interfaces::msg::SetParametersResult PositionPIDController::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  PIDParams updated_navigate = navigate_params_;
  PIDParams updated_fine_tune = fine_tune_params_;
  bool updated_any_pid = false;

  double updated_max_slow_vel = max_slow_vel_;
  bool updated_slow_vel = false;
  bool updated_debug_publish = false;
  bool new_enable_pid_debug_publish = enable_pid_debug_publish_;

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();

    if (name == "enable_pid_debug_publish") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enable_pid_debug_publish must be bool";
        return result;
      }
      new_enable_pid_debug_publish = param.as_bool();
      updated_debug_publish = true;
      continue;
    }

    if (name == "max_slow_velocity") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "max_slow_velocity must be double";
        return result;
      }
      updated_max_slow_vel = param.as_double();
      updated_slow_vel = true;
      continue;
    }

    if (name.rfind("navigate.", 0) == 0) {
      if (!updatePIDParamsFromParameter(param, "navigate", updated_navigate)) {
        result.successful = false;
        result.reason = "unsupported or invalid navigate.* parameter: " + name;
        return result;
      }
      updated_any_pid = true;
      continue;
    }

    if (name.rfind("fine_tune.", 0) == 0) {
      if (!updatePIDParamsFromParameter(param, "fine_tune", updated_fine_tune)) {
        result.successful = false;
        result.reason = "unsupported or invalid fine_tune.* parameter: " + name;
        return result;
      }
      updated_any_pid = true;
      continue;
    }
  }

  auto validate_limits = [](const PIDParams & params) {
      const bool linear_ok = params.max_linear_vel >= 0.0 &&
        params.min_linear_vel <= 0.0 &&
        params.max_linear_vel > params.min_linear_vel;
      const bool angular_ok = params.max_angular_vel >= 0.0 &&
        params.min_angular_vel <= 0.0 &&
        params.max_angular_vel > params.min_angular_vel;
      const bool vertical_ok = params.max_vertical_vel >= 0.0 &&
        params.min_vertical_vel <= 0.0 &&
        params.max_vertical_vel > params.min_vertical_vel;
      return linear_ok && angular_ok && vertical_ok;
    };

  if (updated_any_pid) {
    if (!validate_limits(updated_navigate) || !validate_limits(updated_fine_tune)) {
      result.successful = false;
      result.reason = "invalid max/min velocity limits in PID parameters";
      return result;
    }

    navigate_params_ = updated_navigate;
    fine_tune_params_ = updated_fine_tune;

    applyPIDParams(getActivePIDParams());
    setControlMode(control_mode_);

    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset();
    pid_yaw_.reset();
    pid_xy_speed_.reset();

    RCLCPP_INFO(get_logger(), "Runtime PID parameters updated");
  }

  if (updated_slow_vel) {
    max_slow_vel_ = updated_max_slow_vel;
    setControlMode(control_mode_);
    RCLCPP_INFO(get_logger(), "Runtime max_slow_velocity updated: %.2f", max_slow_vel_);
  }

  if (updated_debug_publish) {
    enable_pid_debug_publish_ = new_enable_pid_debug_publish;
    RCLCPP_INFO(
      get_logger(), "Runtime enable_pid_debug_publish updated: %s",
      enable_pid_debug_publish_ ? "true" : "false");
  }

  return result;
}

bool PositionPIDController::updatePIDParamsFromParameter(
  const rclcpp::Parameter & param,
  const std::string & prefix,
  PIDParams & params)
{
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return false;
  }

  const std::string key = param.get_name().substr(prefix.size() + 1);
  const double value = param.as_double();

  if (key == "kp_xy") {
    params.kp_xy = value;
  } else if (key == "ki_xy") {
    params.ki_xy = value;
  } else if (key == "kd_xy") {
    params.kd_xy = value;
  } else if (key == "kp_yaw") {
    params.kp_yaw = value;
  } else if (key == "ki_yaw") {
    params.ki_yaw = value;
  } else if (key == "kd_yaw") {
    params.kd_yaw = value;
  } else if (key == "kp_z") {
    params.kp_z = value;
  } else if (key == "ki_z") {
    params.ki_z = value;
  } else if (key == "kd_z") {
    params.kd_z = value;
  } else if (key == "max_linear_velocity") {
    params.max_linear_vel = value;
  } else if (key == "max_angular_velocity") {
    params.max_angular_vel = value;
  } else if (key == "max_vertical_velocity") {
    params.max_vertical_vel = value;
  } else if (key == "min_linear_velocity") {
    params.min_linear_vel = value;
  } else if (key == "min_angular_velocity") {
    params.min_angular_vel = value;
  } else if (key == "min_vertical_velocity") {
    params.min_vertical_vel = value;
  } else if (key == "Xpos_tolerance") {
    params.Xpos_tolerance = value;
  } else if (key == "Ypos_tolerance") {
    params.Ypos_tolerance = value;
  } else if (key == "yaw_tolerance") {
    params.yaw_tolerance = value;
  } else if (key == "height_tolerance") {
    params.height_tolerance = value;
  } else {
    return false;
  }

  return true;
}

}  // namespace pid_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_pkg::PositionPIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
