#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace activity_control_pkg
{

struct TrackerVector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TrackerRefState
{
  bool valid{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  uint32_t trajectory_id{0};
  uint8_t trajectory_flag{0};
  TrackerVector3 pos_world{};
  TrackerVector3 vel_world{};
  TrackerVector3 acc_world{};
  double yaw_rad{0.0};
  double yaw_rate_rad_s{0.0};
};

struct TrackerOdomState
{
  bool valid{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  TrackerVector3 pos_world{};
  TrackerVector3 vel_world{};
  double yaw_rad{0.0};
  double yaw_rate_rad_s{0.0};
};

struct TrackerCommandState
{
  double vx_cm_s{0.0};
  double vy_cm_s{0.0};
  double vz_cm_s{0.0};
  double yaw_rate_deg_s{0.0};
};

class PlannerVelocityTrackerNode : public rclcpp::Node
{
public:
  explicit PlannerVelocityTrackerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class TrackerMode
  {
    WAIT_ODOM,
    WAIT_REF,
    TRACKING,
    TIMEOUT_BRAKE,
  };

  void positionCommandCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void readyCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void controlTimerCallback();

  void handleTrajectorySwitch(uint32_t trajectory_id);
  void resetControlState();
  void publishVelocityCommand(const TrackerCommandState & command);
  void publishZeroCommand();
  void publishDebug(double desired_yaw_rad);
  void publishReferencePose(double desired_yaw_rad);
  void publishStatus(const std::string & reason);

  static TrackerVector3 rotateWorldToBody(const TrackerVector3 & value, double yaw_rad);
  static TrackerVector3 rotateBodyToWorld(const TrackerVector3 & value, double yaw_rad);
  static TrackerVector3 subtract(const TrackerVector3 & lhs, const TrackerVector3 & rhs);
  static TrackerVector3 scale(const TrackerVector3 & value, double scalar);
  static double clampAbs(double value, double limit);
  static double normalizeAngleRad(double angle_rad);
  static double radToDeg(double angle_rad);
  static double degToRad(double angle_deg);
  static bool isFinite(const TrackerVector3 & value);
  static double finiteOrZero(double value);
  std::string trackerModeToString() const;
  void updateTakeoffGuardState();
  double currentOdomHeightCm() const;
  double takeoffGuardReleaseHeightCm() const;
  bool shouldRunBootstrapTakeoff() const;
  void runBootstrapTakeoff();

  void limitWorldVelocity(TrackerVector3 & command_world_cm) const;
  void limitBodyVelocity(TrackerCommandState & command_body, double max_yaw_velocity_deg) const;
  void applyDeadband(TrackerCommandState & command_body) const;
  double currentOdomAgeSec(const rclcpp::Time & now) const;
  double currentRefAgeSec(const rclcpp::Time & now) const;
  bool isPublishActive() const;

  std::string input_position_cmd_topic_;
  std::string odom_topic_;
  std::string ready_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string control_mode_topic_;
  std::string world_frame_id_;
  std::string body_frame_id_;

  double control_frequency_;
  double odom_timeout_sec_;
  double cmd_timeout_sec_;
  double kp_position_x_;
  double kp_position_y_;
  double kp_position_z_;
  double kv_velocity_x_;
  double kv_velocity_y_;
  double kv_velocity_z_;
  double ff_velocity_gain_;
  double ff_accel_time_sec_;
  bool use_fixed_yaw_;
  double fixed_yaw_deg_;
  double yaw_kp_;
  double yaw_ff_gain_;
  double large_yaw_error_deg_threshold_;
  double large_yaw_kp_scale_;
  double large_yaw_ff_scale_;
  double max_linear_velocity_cm_;
  double max_vertical_velocity_cm_;
  double max_yaw_velocity_deg_;
  double large_yaw_velocity_deg_;
  double min_publish_speed_cm_;
  bool enable_takeoff_height_guard_;
  double guarded_takeoff_z_cm_;
  bool enable_bootstrap_takeoff_;
  double bootstrap_takeoff_z_cm_;
  bool enable_trajectory_switch_reset_;
  bool publish_zero_on_timeout_;
  bool publish_debug_topics_;
  bool enable_debug_logs_;
  bool odom_velocity_in_body_frame_;
  int64_t debug_log_throttle_ms_;
  uint8_t active_control_mode_;
  uint8_t current_control_mode_;
  bool publish_active_{true};

  bool has_active_trajectory_id_{false};
  uint32_t active_trajectory_id_{0};
  bool last_timeout_zero_sent_{false};
  bool takeoff_guard_active_{false};
  bool takeoff_guard_completed_{false};
  bool ready_received_{false};
  TrackerMode tracker_mode_{TrackerMode::WAIT_ODOM};

  TrackerRefState ref_state_{};
  TrackerOdomState odom_state_{};
  TrackerCommandState last_command_{};
  TrackerVector3 last_pos_error_world_{};
  TrackerVector3 last_vel_error_world_{};
  TrackerVector3 last_command_world_cm_{};
  double last_yaw_error_rad_{0.0};
  double last_desired_yaw_rad_{0.0};
  double last_yaw_rate_feedforward_deg_s_{0.0};
  double last_yaw_rate_feedback_deg_s_{0.0};
  double last_yaw_rate_unclamped_deg_s_{0.0};
  double last_yaw_rate_limit_deg_s_{0.0};
  bool last_large_yaw_error_active_{false};
  double takeoff_guard_hold_yaw_rad_{0.0};
  TrackerVector3 takeoff_guard_origin_pos_world_{};

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr cmd_world_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr cmd_body_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace activity_control_pkg