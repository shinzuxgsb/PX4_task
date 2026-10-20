#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "activity_control_pkg/planner_velocity_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<activity_control_pkg::PlannerVelocityTrackerNode>());
  rclcpp::shutdown();
  return 0;
}