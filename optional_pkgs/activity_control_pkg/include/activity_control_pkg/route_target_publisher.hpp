#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include "activity_control_pkg/waypoint_navigator.hpp"

namespace activity_control_pkg
{

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);
  void addTarget(double x_cm, double y_cm, double z_cm, double yaw_deg);

  std::size_t currentIndex() const;

  std::size_t size() const;

private:
  void loadWaypointsFromParameters();

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::unique_ptr<WaypointNavigator> navigator_;

  std::string fastlio_world_frame_;
  std::string fastlio_body_frame_;
  std::string output_topic_;
  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;
  int reach_hold_cycles_;
  double target_republish_period_sec_;
};

}  // namespace activity_control_pkg