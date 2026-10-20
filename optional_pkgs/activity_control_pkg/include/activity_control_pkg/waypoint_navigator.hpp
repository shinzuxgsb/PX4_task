#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
};

class WaypointNavigator
{
public:
  WaypointNavigator(
    rclcpp::Node & node,
    const std::string & output_topic,
    const std::string & fastlio_world_frame,
    const std::string & fastlio_body_frame,
    double pos_tol_cm,
    double yaw_tol_deg,
    double height_tol_cm,
    int reach_hold_cycles,
    double target_republish_period_sec);

  void addTarget(const Target & target);
  void addTarget(double x_cm, double y_cm, double z_cm, double yaw_deg);

  std::size_t currentIndex() const;
  std::size_t size() const;

  void updateHeight(double height_cm);
  void tick();

private:
  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);
  void publishLandingSentinel();

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Node & node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;
  int reach_hold_cycles_;
  int reach_stable_count_;
  bool mission_completed_;
  bool land_sentinel_published_;
  double target_republish_period_sec_;
  rclcpp::Time last_target_publish_time_;

  std::string fastlio_world_frame_;
  std::string fastlio_body_frame_;
};

}  // namespace activity_control_pkg
