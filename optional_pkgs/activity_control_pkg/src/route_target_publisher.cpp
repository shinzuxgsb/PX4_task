#include "activity_control_pkg/route_target_publisher.hpp"

#include <algorithm>
#include <sstream>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  pos_tol_cm_(4.0),
  yaw_tol_deg_(5.0),
  height_tol_cm_(3.0),
  reach_hold_cycles_(3),
  target_republish_period_sec_(1.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 4.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 3.0);
  reach_hold_cycles_ = declare_parameter("reach_hold_cycles", 3);
  target_republish_period_sec_ = declare_parameter("target_republish_period_sec", 1.0);
  reach_hold_cycles_ = std::max(1, reach_hold_cycles_);
  target_republish_period_sec_ = std::max(0.0, target_republish_period_sec_);

  fastlio_world_frame_ = declare_parameter("fastlio_world_frame", "world");
  fastlio_body_frame_ = declare_parameter("fastlio_body_frame", "body");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  navigator_ = std::make_unique<WaypointNavigator>(
    *this,
    output_topic_,
    fastlio_world_frame_,
    fastlio_body_frame_,
    pos_tol_cm_,
    yaw_tol_deg_,
    height_tol_cm_,
    reach_hold_cycles_,
    target_republish_period_sec_);

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "RouteTargetPublisher initialized: fastlio_world=%s fastlio_body=%s topic=%s",
    fastlio_world_frame_.c_str(), fastlio_body_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
  RCLCPP_INFO(get_logger(), "Target republish period: %.2fs", target_republish_period_sec_);

  loadWaypointsFromParameters();

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));
}

void RouteTargetPublisherNode::loadWaypointsFromParameters()
{
  const bool auto_start_waypoints = declare_parameter("auto_start_waypoints", true);
  const auto waypoint_rows =
    declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>{});

  if (!auto_start_waypoints) {
    RCLCPP_INFO(get_logger(), "auto_start_waypoints=false, waiting for external waypoint input.");
    return;
  }

  if (waypoint_rows.empty()) {
    RCLCPP_WARN(get_logger(),
      "No waypoints configured. Set parameter 'waypoints' as string rows like "
      "['0,0,128,0','0,-138,131,0'].");
    return;
  }

  for (std::size_t i = 0; i < waypoint_rows.size(); ++i) {
    std::string row = waypoint_rows[i];
    for (char & ch : row) {
      if (ch == ',') {
        ch = ' ';
      }
    }

    std::istringstream iss(row);
    Target target{};
    if (!(iss >> target.x_cm >> target.y_cm >> target.z_cm >> target.yaw_deg)) {
      RCLCPP_ERROR(get_logger(),
        "Invalid waypoint row[%zu]='%s'. Expected four numbers: x y z yaw",
        i, waypoint_rows[i].c_str());
      continue;
    }

    double extra = 0.0;
    if (iss >> extra) {
      RCLCPP_ERROR(get_logger(),
        "Invalid waypoint row[%zu]='%s'. Too many values, expected exactly four.",
        i, waypoint_rows[i].c_str());
      continue;
    }

    navigator_->addTarget(target);
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from parameters.", navigator_->size());
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  navigator_->addTarget(target);
}

void RouteTargetPublisherNode::addTarget(double x_cm, double y_cm, double z_cm, double yaw_deg)
{
  navigator_->addTarget(x_cm, y_cm, z_cm, yaw_deg);
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  return navigator_->currentIndex();
}

std::size_t RouteTargetPublisherNode::size() const
{
  return navigator_->size();
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  navigator_->tick();
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  navigator_->updateHeight(static_cast<double>(msg->data));
}

}  // namespace activity_control_pkg
