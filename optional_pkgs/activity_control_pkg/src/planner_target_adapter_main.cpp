#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "activity_control_pkg/planner_target_adapter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<activity_control_pkg::PlannerTargetAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
