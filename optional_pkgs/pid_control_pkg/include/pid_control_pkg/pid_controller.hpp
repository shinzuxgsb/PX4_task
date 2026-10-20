#ifndef PID_CONTROL_PKG__PID_CONTROLLER_HPP_
#define PID_CONTROL_PKG__PID_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pid_control_pkg
{

class PIDController
{
public:
  PIDController(
    double kp,
    double ki,
    double kd,
    double max_output = 1.0,
    double min_output = -1.0,
    double integral_limit = 10.0,
    double deadzone = 0.0);

  double calculate(double setpoint, double measured_value, double dt);
  void reset();
  void setPID(double kp, double ki, double kd);
  void setOutputLimits(double max_output, double min_output);
  void setIntegralLimit(double integral_limit);
  void setDeadzone(double deadzone);

  double getError() const { return current_error_; }
  double getIntegral() const { return integral_; }

private:
  double kp_;
  double ki_;
  double kd_;
  double max_output_;
  double min_output_;
  double integral_limit_;
  double deadzone_;
  double prev_error_;
  double current_error_;
  double integral_;
  double prev_derivative_;
  bool first_call_;
  double derivative_filter_alpha_;
};

enum class ControlMode
{
  NORMAL = 0,
  SLOW = 1,
  LOCK_Y = 2,
  LOCK_X = 3,
  HOVER = 4
};

class PositionPIDController : public rclcpp::Node
{
public:
  PositionPIDController();
  ~PositionPIDController() override = default;

private:
  struct PIDParams;

  void targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void pidModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void pojiangCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void qrOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void controlTimerCallback();

  bool getCurrentPose();
  void loadParameters();
  void calculateErrors();
  double normalizeAngleDeg(double angle_deg) const;
  bool isTargetReached() const;
  void setControlMode(ControlMode mode);
  std_msgs::msg::Float32MultiArray processPID(double dt);
  void applyPIDParams(const struct PIDParams & params);

  struct VelocityOutput
  {
    double vel_x_cm;
    double vel_y_cm;
    double vel_z_cm;
    double vel_yaw_deg;
  };

  void computeXYVelocityByControlMode(double dt, double & vel_x_cm, double & vel_y_cm);
  void transformToBodyFrame(double & vel_x_cm, double & vel_y_cm);
  double computeYawVelocity(double dt);
  double computeZVelocity(double dt);
  VelocityOutput computeNavigateModeVelocity(double dt);
  VelocityOutput computeFineTuneModeVelocity(double dt);
  std_msgs::msg::Float32MultiArray fillCmdVelMessage(const VelocityOutput & vel);
  const PIDParams & getActivePIDParams() const;
  void publishPIDDebug(const VelocityOutput & vel);
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters);
  bool updatePIDParamsFromParameter(
    const rclcpp::Parameter & param,
    const std::string & prefix,
    PIDParams & params);

  inline double meterToCm(double meter) const { return meter * 100.0; }
  inline double degToRad(double deg) const { return deg * M_PI / 180.0; }
  inline double radToDeg(double rad) const { return rad * 180.0 / M_PI; }

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr pid_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr pojiang_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr qr_offset_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool enable_visual_fine_tune_;
  bool enable_pid_debug_publish_;
  bool pojiang_flag_;

  struct PIDParams
  {
    double kp_xy;
    double ki_xy;
    double kd_xy;
    double kp_yaw;
    double ki_yaw;
    double kd_yaw;
    double kp_z;
    double ki_z;
    double kd_z;
    double max_linear_vel;
    double max_angular_vel;
    double max_vertical_vel;
    double min_linear_vel;
    double min_angular_vel;
    double min_vertical_vel;
    double Xpos_tolerance;
    double Ypos_tolerance;
    double yaw_tolerance;
    double height_tolerance;
  };

  PIDParams navigate_params_;
  PIDParams fine_tune_params_;

  // PID controllers
  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_yaw_;
  PIDController pid_z_;
  PIDController pid_xy_speed_;

  // Targets (cm, deg)
  double target_x_cm_;
  double target_y_cm_;
  double target_z_cm_;
  double target_yaw_deg_;
  bool has_target_position_;
  bool has_target_height_;
  bool has_external_height_;

  // Current state
  double current_x_cm_;
  double current_y_cm_;
  double current_yaw_deg_;
  double current_z_cm_;
  bool has_current_pose_;
  bool has_qr_offset_;

  // Control parameters
  double control_frequency_;
  std::string fastlio_world_frame_;
  std::string fastlio_body_frame_;
  bool use_height_topic_for_z_;
  double yaw_first_threshold_deg_;
  double near_target_distance_cm_;
  double near_yaw_lock_threshold_deg_;
  bool enable_target_switch_filter_;
  double min_target_switch_xy_cm_;
  double min_target_switch_z_cm_;
  double min_target_switch_yaw_deg_;
  double switch_release_distance_cm_;

  uint64_t target_switch_accept_count_;
  uint64_t target_switch_skip_count_;

  ControlMode control_mode_;
  double Xpos_tolerance_;
  double Ypos_tolerance_;
  double yaw_tolerance_;
  double height_tolerance_;

  double max_linear_vel_;
  double max_angular_vel_;
  double max_vertical_vel_;
  double max_slow_vel_;

  double distance_xy_cm_;
  double error_x_cm_;
  double error_y_cm_;
  double error_yaw_deg_;
  double error_z_cm_;
  uint8_t current_pid_mode_;

  double qr_offset_x_;
  double qr_offset_y_;
  double qr_target_x_;
  double qr_target_y_;
  double qr_data_timeout_;
  rclcpp::Time last_qr_update_time_;

  rclcpp::Time last_update_time_;
  std::string control_mode_topic_;
  uint8_t active_control_mode_;
  uint8_t current_control_mode_;
  bool publish_active_;
};

}  // namespace pid_control_pkg

#endif  // PID_CONTROL_PKG__PID_CONTROLLER_HPP_
