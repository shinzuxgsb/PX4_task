#pragma once

#include <string>
#include <utility>
#include <deque>
#include <limits>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace activity_control_pkg
{

class PlannerTargetAdapterNode : public rclcpp::Node
{
public:
  explicit PlannerTargetAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TargetCmd
  {
    double x_cm;
    double y_cm;
    double z_cm;
    double yaw_deg;
  };

  struct VelocityCmd
  {
    double vx_cm_s;
    double vy_cm_s;
    double vz_cm_s;
    double vyaw_deg_s;
  };

  struct QueuedTarget
  {
    TargetCmd cmd;
    uint64_t index;
  };

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void positionCommandCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void externalHeightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void gateTriggerCallback(const std_msgs::msg::UInt32::SharedPtr msg);
  void republishTimerCallback();

  void publishTarget(const TargetCmd & cmd);
  void publishVelocity(const VelocityCmd & cmd);
  void enqueueTarget(const TargetCmd & cmd);
  bool injectBootstrapTargetIfNeeded();
  bool dispatchNextTargetIfNeeded();
  std::deque<QueuedTarget>::iterator selectNextQueuedTargetIterator();
  std::deque<QueuedTarget>::const_iterator selectNextQueuedTargetIterator() const;
  bool isCurrentTargetReached() const;
  void onCurrentTargetReached();
  void publishStatus(const std::string & reason);
  std::string buildStatusJson(const std::string & reason) const;
  uint32_t sanitizeGateCount(uint32_t requested) const;
  int64_t previewSegmentEndIndex() const;
  bool shouldPublishTarget(const TargetCmd & cmd, double adaptive_scale);
  void applyTakeoffHeightGuard(TargetCmd & cmd);
  double computeAdaptiveScale(const TargetCmd & cmd) const;
  void updateRawTargetHistory(const TargetCmd & cmd);
  void handleTrajectorySwitch(uint32_t trajectory_id, uint8_t trajectory_flag);
  void pruneManagedQueue(const TargetCmd & latest_cmd);
  VelocityCmd positionCommandToVelocity(const quadrotor_msgs::msg::PositionCommand & msg) const;
  bool isVelocityOutputMode() const;
  double resolveYawDeg(double raw_yaw_deg) const;
  static double normalizeAngleDeg(double angle_deg);
  bool poseToTarget(const geometry_msgs::msg::Pose & pose, TargetCmd & target) const;
  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr external_height_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr gate_trigger_sub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::string input_pose_topic_;
  std::string input_path_topic_;
  std::string input_position_cmd_topic_;
  std::string odom_topic_;
  std::string external_height_topic_;
  std::string output_topic_;
  std::string output_mode_;
  std::string gate_trigger_topic_;
  std::string status_topic_;
  bool prefer_position_cmd_input_;
  bool prefer_path_input_;
  int path_sample_index_;
  bool use_input_z_;
  double default_height_cm_;
  bool use_fixed_yaw_;
  double fixed_yaw_deg_;
  double republish_period_sec_;
  bool enable_sparse_sampling_;
  double min_delta_xy_cm_;
  double min_delta_z_cm_;
  double min_delta_yaw_deg_;
  bool enable_adaptive_sampling_;
  bool enable_trajectory_switch_reset_;
  double adaptive_scale_min_;
  double adaptive_scale_max_;
  double adaptive_turn_low_deg_;
  double adaptive_turn_high_deg_;
  bool enable_takeoff_height_guard_;
  bool use_external_height_guard_;
  int64_t external_height_timeout_ms_;
  double min_valid_target_z_cm_;
  double guarded_takeoff_z_cm_;
  bool enable_bootstrap_takeoff_target_;
  bool bootstrap_once_;
  double bootstrap_takeoff_height_cm_;
  bool gate_control_enabled_;
  bool enable_nearest_waypoint_priority_;
  bool nearest_waypoint_forward_only_;
  double nearest_waypoint_switch_hysteresis_cm_;
  uint32_t default_gate_count_;
  uint32_t max_queue_size_;
  double gate_pos_tol_cm_;
  double gate_z_tol_cm_;
  double gate_yaw_tol_deg_;
  int gate_reach_hold_cycles_;
  bool gate_waiting_trigger_;
  uint32_t gate_remaining_in_segment_;
  uint32_t segment_total_count_;
  uint32_t segment_completed_count_;
  int gate_reach_stable_count_;

  uint64_t accepted_count_;
  uint64_t skipped_count_;
  uint64_t adaptive_sample_count_;
  uint64_t queued_count_;
  uint64_t reached_count_;
  uint64_t trigger_count_;
  uint64_t trajectory_switch_count_;

  bool has_last_target_;
  TargetCmd last_target_;
  bool has_last_velocity_;
  VelocityCmd last_velocity_;
  bool has_active_trajectory_id_;
  uint32_t active_trajectory_id_;
  bool has_last_gate_sample_;
  TargetCmd last_gate_sample_;
  bool has_current_target_;
  QueuedTarget current_target_;
  std::deque<QueuedTarget> pending_targets_;
  uint64_t next_target_index_;
  int64_t last_reached_index_;
  bool has_last_reached_target_;
  TargetCmd last_reached_target_;
  rclcpp::Time last_reached_time_;
  bool has_pose_;
  bool has_external_height_;
  bool bootstrap_done_;
  double external_height_cm_;
  rclcpp::Time last_external_height_time_;
  TargetCmd current_pose_;
  bool has_last_raw_target_;
  bool has_prev_raw_target_;
  TargetCmd last_raw_target_;
  TargetCmd prev_raw_target_;
  bool last_target_guard_applied_;
  std::string last_height_guard_source_;
  bool has_last_incoming_target_;
  TargetCmd last_incoming_target_;
  bool has_last_accepted_target_;
  TargetCmd last_accepted_target_;
};

}  // namespace activity_control_pkg
