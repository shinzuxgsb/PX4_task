#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <angles/angles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

class PrecisionLandingManagerNode : public rclcpp::Node
{
public:
  explicit PrecisionLandingManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("precision_landing_manager", options),
    enable_precision_landing_(declare_parameter("enable_precision_landing", false)),
    final_waypoint_event_topic_(declare_parameter(
        "final_waypoint_event_topic", std::string("/planning/final_waypoint_event"))),
    control_mode_topic_(declare_parameter(
        "control_mode_topic", std::string("/precision_landing/control_mode"))),
    target_topic_(declare_parameter("target_topic", std::string("/target_position"))),
    status_topic_(declare_parameter("status_topic", std::string("/precision_landing/status"))),
    height_topic_(declare_parameter("height_topic", std::string("/height"))),
    use_height_topic_for_z_(declare_parameter("use_height_topic_for_z", false)),
    fastlio_world_frame_(declare_parameter("fastlio_world_frame", std::string("world"))),
    fastlio_body_frame_(declare_parameter("fastlio_body_frame", std::string("body"))),
    waypoint_num_(std::max<int64_t>(0, declare_parameter<int64_t>("waypoint_num", 0))),
    target_yaw_deg_(declare_parameter("target_yaw_deg", 0.0)),
    target_bias_x_cm_(declare_parameter("target_bias_x_cm", 0.0)),
    target_bias_y_cm_(declare_parameter("target_bias_y_cm", 0.0)),
    target_bias_z_cm_(declare_parameter("target_bias_z_cm", 0.0)),
    hold_position_tolerance_cm_(std::max(0.5, declare_parameter("hold_position_tolerance_cm", 4.0))),
    hold_height_tolerance_cm_(std::max(0.5, declare_parameter("hold_height_tolerance_cm", 3.0))),
    hold_yaw_tolerance_deg_(std::max(0.5, declare_parameter("hold_yaw_tolerance_deg", 5.0))),
    hold_time_sec_(std::max(0.0, declare_parameter("hold_time_sec", 0.6))),
    mode_switch_delay_sec_(std::max(0.0, declare_parameter("mode_switch_delay_sec", 0.15))),
    target_republish_period_sec_(std::max(0.02, declare_parameter("target_republish_period_sec", 0.1))),
    timer_period_sec_(std::max(0.02, declare_parameter("timer_period_sec", 0.05))),
    cruise_control_mode_(static_cast<uint8_t>(std::max<int64_t>(
        0, declare_parameter<int64_t>("cruise_control_mode", 0)))),
    precision_control_mode_(static_cast<uint8_t>(std::max<int64_t>(
        0, declare_parameter<int64_t>("precision_control_mode", 1)))),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    state_(State::CRUISE),
    has_height_(false),
    current_height_cm_(0.0),
    last_z_source_(use_height_topic_for_z_ ? "height" : "odom_tf"),
    last_pose_valid_(false),
    last_current_x_cm_(0.0),
    last_current_y_cm_(0.0),
    last_current_z_cm_(0.0),
    last_current_yaw_deg_(0.0),
    last_error_x_cm_(0.0),
    last_error_y_cm_(0.0),
    last_error_xy_cm_(0.0),
    last_error_z_cm_(0.0),
    last_error_yaw_deg_(0.0),
    final_target_valid_(false),
    final_target_x_cm_(0.0),
    final_target_y_cm_(0.0),
    final_target_z_cm_(0.0),
    stable_since_(0, 0, RCL_ROS_TIME),
    has_stable_since_(false),
    last_target_publish_time_(0, 0, RCL_ROS_TIME),
    state_entry_time_(now()),
    last_reached_(false)
  {
    loadFinalTarget();

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    control_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(control_mode_topic_, latched_qos);
    target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(target_topic_, latched_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));

    final_waypoint_event_sub_ = create_subscription<std_msgs::msg::UInt8>(
      final_waypoint_event_topic_, latched_qos,
      std::bind(&PrecisionLandingManagerNode::finalWaypointEventCallback, this, std::placeholders::_1));

    if (use_height_topic_for_z_) {
      height_sub_ = create_subscription<std_msgs::msg::Int16>(
        height_topic_, rclcpp::QoS(10),
        std::bind(&PrecisionLandingManagerNode::heightCallback, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_period_sec_)),
      std::bind(&PrecisionLandingManagerNode::timerCallback, this));

    publishControlMode(cruise_control_mode_);
    publishStatus("init");

    RCLCPP_INFO(
      get_logger(),
      "precision_landing_manager started. enabled=%s event_topic=%s control_mode_topic=%s target_topic=%s final_target_valid=%s z_source=%s bias_cm=(%.1f, %.1f, %.1f)",
      enable_precision_landing_ ? "true" : "false",
      final_waypoint_event_topic_.c_str(), control_mode_topic_.c_str(), target_topic_.c_str(),
      final_target_valid_ ? "true" : "false",
      use_height_topic_for_z_ ? "height" : "odom_tf",
      target_bias_x_cm_, target_bias_y_cm_, target_bias_z_cm_);
  }

private:
  enum class State
  {
    CRUISE,
    ARMING,
    PRECISION_HOLD,
    LANDING,
  };

  void loadFinalTarget()
  {
    for (int64_t index = 0; index < 50; ++index) {
      declare_parameter("waypoint" + std::to_string(index) + "_x", 0.0);
      declare_parameter("waypoint" + std::to_string(index) + "_y", 0.0);
      declare_parameter("waypoint" + std::to_string(index) + "_z", 0.0);
    }

    if (waypoint_num_ <= 0) {
      RCLCPP_WARN(get_logger(), "Precision landing disabled: waypoint_num <= 0.");
      final_target_valid_ = false;
      return;
    }

    const int64_t final_index = waypoint_num_ - 1;
    double waypoint_x_m = 0.0;
    double waypoint_y_m = 0.0;
    double waypoint_z_m = 0.0;
    get_parameter("waypoint" + std::to_string(final_index) + "_x", waypoint_x_m);
    get_parameter("waypoint" + std::to_string(final_index) + "_y", waypoint_y_m);
    get_parameter("waypoint" + std::to_string(final_index) + "_z", waypoint_z_m);

    const double raw_target_x_cm = waypoint_x_m * 100.0;
    const double raw_target_y_cm = waypoint_y_m * 100.0;
    const double raw_target_z_cm = waypoint_z_m * 100.0;

    final_target_x_cm_ = raw_target_x_cm + target_bias_x_cm_;
    final_target_y_cm_ = raw_target_y_cm + target_bias_y_cm_;
    final_target_z_cm_ = raw_target_z_cm + target_bias_z_cm_;
    final_target_valid_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Final precision target loaded: raw=(%.1f, %.1f, %.1f) bias=(%.1f, %.1f, %.1f) final=(%.1f, %.1f, %.1f) yaw=%.1f",
      raw_target_x_cm, raw_target_y_cm, raw_target_z_cm,
      target_bias_x_cm_, target_bias_y_cm_, target_bias_z_cm_,
      final_target_x_cm_, final_target_y_cm_, final_target_z_cm_, target_yaw_deg_);
  }

  void finalWaypointEventCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data == 0U) {
      state_ = State::CRUISE;
      state_entry_time_ = now();
      has_stable_since_ = false;
      last_reached_ = false;
      publishControlMode(cruise_control_mode_);
      publishStatus("cruise_reset");
      return;
    }

    if (msg->data != 1U) {
      return;
    }

    if (!enable_precision_landing_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignore final waypoint event because precision landing is disabled.");
      return;
    }

    if (!final_target_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignore final waypoint event because final target is invalid.");
      return;
    }

    if (state_ != State::CRUISE) {
      return;
    }

    state_ = State::ARMING;
    state_entry_time_ = now();
    has_stable_since_ = false;
    last_reached_ = false;
    publishPrecisionTarget();
    publishStatus("arming_precision_hold");
  }

  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    current_height_cm_ = static_cast<double>(msg->data);
    has_height_ = true;
  }

  void timerCallback()
  {
    const rclcpp::Time now_time = now();

    switch (state_) {
      case State::CRUISE:
        return;

      case State::ARMING:
        publishPrecisionTargetIfDue(now_time);
        if ((now_time - state_entry_time_).seconds() >= mode_switch_delay_sec_) {
          publishControlMode(precision_control_mode_);
          state_ = State::PRECISION_HOLD;
          state_entry_time_ = now_time;
          has_stable_since_ = false;
          publishStatus("precision_hold_active");
        }
        return;

      case State::PRECISION_HOLD:
      {
        publishPrecisionTargetIfDue(now_time);

        double current_x_cm = 0.0;
        double current_y_cm = 0.0;
        double current_z_cm = 0.0;
        double current_yaw_deg = 0.0;
        if (!getCurrentPose(current_x_cm, current_y_cm, current_z_cm, current_yaw_deg)) {
          return;
        }

        const bool reached = isWithinTolerance(
          current_x_cm, current_y_cm, current_z_cm, current_yaw_deg);
        logPrecisionHold(now_time, reached);
        if (reached != last_reached_) {
          publishStatus(reached ? "precision_target_reached" : "precision_target_lost");
          last_reached_ = reached;
        }

        if (!reached) {
          has_stable_since_ = false;
          return;
        }

        if (!has_stable_since_) {
          stable_since_ = now_time;
          has_stable_since_ = true;
          return;
        }

        if ((now_time - stable_since_).seconds() >= hold_time_sec_) {
          publishLandingSentinel();
          publishControlMode(precision_control_mode_);
          state_ = State::LANDING;
          state_entry_time_ = now_time;
          publishStatus("landing_sentinel_published");
        }
        return;
      }

      case State::LANDING:
        publishLandingSentinelIfDue(now_time);
        return;
    }
  }

  bool getCurrentPose(
    double & current_x_cm,
    double & current_y_cm,
    double & current_z_cm,
    double & current_yaw_deg)
  {
    try {
      const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        fastlio_world_frame_, fastlio_body_frame_, tf2::TimePointZero);
      current_x_cm = transform.transform.translation.x * 100.0;
      current_y_cm = transform.transform.translation.y * 100.0;
      current_z_cm = transform.transform.translation.z * 100.0;
      last_z_source_ = "odom_tf";
      if (use_height_topic_for_z_ && has_height_) {
        current_z_cm = current_height_cm_;
        last_z_source_ = "height";
      }

      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      current_yaw_deg = yaw * 180.0 / M_PI;
      last_pose_valid_ = true;
      last_current_x_cm_ = current_x_cm;
      last_current_y_cm_ = current_y_cm;
      last_current_z_cm_ = current_z_cm;
      last_current_yaw_deg_ = current_yaw_deg;
      return true;
    } catch (const tf2::TransformException & ex) {
      last_pose_valid_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Precision landing TF lookup failed (%s->%s): %s",
        fastlio_world_frame_.c_str(), fastlio_body_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool isWithinTolerance(
    double current_x_cm,
    double current_y_cm,
    double current_z_cm,
    double current_yaw_deg)
  {
    const double dx = final_target_x_cm_ - current_x_cm;
    const double dy = final_target_y_cm_ - current_y_cm;
    const double dxy = std::hypot(dx, dy);
    const double dz = std::fabs(final_target_z_cm_ - current_z_cm);
    const double yaw_error = std::fabs(normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg));
    last_error_x_cm_ = dx;
    last_error_y_cm_ = dy;
    last_error_xy_cm_ = dxy;
    last_error_z_cm_ = dz;
    last_error_yaw_deg_ = yaw_error;
    return dxy <= hold_position_tolerance_cm_ &&
           dz <= hold_height_tolerance_cm_ &&
           yaw_error <= hold_yaw_tolerance_deg_;
  }

  void logPrecisionHold(const rclcpp::Time & now_time, bool reached)
  {
    if (!last_pose_valid_) {
      return;
    }

    const double stable_sec = has_stable_since_
      ? std::max(0.0, (now_time - stable_since_).seconds())
      : 0.0;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "precision_hold target=(%.1f, %.1f, %.1f, %.1f) current=(%.1f, %.1f, %.1f, %.1f) err=(dx=%.1f, dy=%.1f, dxy=%.1f, dz=%.1f, dyaw=%.1f) bias=(%.1f, %.1f, %.1f) z_source=%s reached=%s stable=%.2fs/%.2fs",
      final_target_x_cm_, final_target_y_cm_, final_target_z_cm_, target_yaw_deg_,
      last_current_x_cm_, last_current_y_cm_, last_current_z_cm_, last_current_yaw_deg_,
      last_error_x_cm_, last_error_y_cm_, last_error_xy_cm_, last_error_z_cm_, last_error_yaw_deg_,
      target_bias_x_cm_, target_bias_y_cm_, target_bias_z_cm_,
      last_z_source_.c_str(), reached ? "true" : "false", stable_sec, hold_time_sec_);
  }

  double normalizeAngleDeg(double angle_deg) const
  {
    const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
    return angles::to_degrees(normalized);
  }

  void publishPrecisionTargetIfDue(const rclcpp::Time & now_time)
  {
    if ((now_time - last_target_publish_time_).seconds() >= target_republish_period_sec_) {
      publishPrecisionTarget();
    }
  }

  void publishLandingSentinelIfDue(const rclcpp::Time & now_time)
  {
    if ((now_time - last_target_publish_time_).seconds() >= target_republish_period_sec_) {
      publishLandingSentinel();
    }
  }

  void publishPrecisionTarget()
  {
    publishTarget(final_target_x_cm_, final_target_y_cm_, final_target_z_cm_, target_yaw_deg_);
  }

  void publishLandingSentinel()
  {
    publishTarget(-1.0, -1.0, -1.0, -1.0);
  }

  void publishTarget(double x_cm, double y_cm, double z_cm, double yaw_deg)
  {
    std_msgs::msg::Float32MultiArray message;
    message.data = {
      static_cast<float>(x_cm),
      static_cast<float>(y_cm),
      static_cast<float>(z_cm),
      static_cast<float>(yaw_deg),
    };
    target_pub_->publish(message);
    last_target_publish_time_ = now();
  }

  void publishControlMode(uint8_t mode)
  {
    std_msgs::msg::UInt8 message;
    message.data = mode;
    control_mode_pub_->publish(message);
  }

  void publishStatus(const std::string & reason)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "{";
    stream << "\"reason\":\"" << reason << "\",";
    stream << "\"state\":\"" << stateToString(state_) << "\",";
    stream << "\"enabled\":" << (enable_precision_landing_ ? "true" : "false") << ",";
    stream << "\"final_target_valid\":" << (final_target_valid_ ? "true" : "false") << ",";
        stream << "\"use_height_topic_for_z\":" << (use_height_topic_for_z_ ? "true" : "false") << ",";
        stream << "\"z_source\":\"" << last_z_source_ << "\",";
    stream << "\"control_mode_topic\":\"" << control_mode_topic_ << "\",";
        stream << "\"target_bias_cm\":[" << target_bias_x_cm_ << "," << target_bias_y_cm_ << "," << target_bias_z_cm_ << "],";
        stream << "\"hold_tolerance_cm\":[" << hold_position_tolerance_cm_ << "," << hold_height_tolerance_cm_ << "],";
        stream << "\"hold_tolerance_yaw_deg\":" << hold_yaw_tolerance_deg_ << ",";
        stream << "\"stable_sec\":"
          << (has_stable_since_ ? std::max(0.0, (now() - stable_since_).seconds()) : 0.0) << ",";
        stream << "\"last_reached\":" << (last_reached_ ? "true" : "false") << ",";
        if (last_pose_valid_) {
          stream << "\"current_pose\":["
            << last_current_x_cm_ << "," << last_current_y_cm_ << ","
            << last_current_z_cm_ << "," << last_current_yaw_deg_ << "],";
          stream << "\"error\":["
            << last_error_x_cm_ << "," << last_error_y_cm_ << ","
            << last_error_xy_cm_ << "," << last_error_z_cm_ << ","
            << last_error_yaw_deg_ << "],";
        } else {
          stream << "\"current_pose\":null,";
          stream << "\"error\":null,";
        }
    stream << "\"target\":[" << final_target_x_cm_ << "," << final_target_y_cm_ << "," << final_target_z_cm_ << "," << target_yaw_deg_ << "]";
    stream << "}";
    message.data = stream.str();
    status_pub_->publish(message);
  }

  static std::string stateToString(State state)
  {
    switch (state) {
      case State::CRUISE:
        return "cruise";
      case State::ARMING:
        return "arming";
      case State::PRECISION_HOLD:
        return "precision_hold";
      case State::LANDING:
        return "landing";
    }
    return "unknown";
  }

  bool enable_precision_landing_;
  std::string final_waypoint_event_topic_;
  std::string control_mode_topic_;
  std::string target_topic_;
  std::string status_topic_;
  std::string height_topic_;
  bool use_height_topic_for_z_;
  std::string fastlio_world_frame_;
  std::string fastlio_body_frame_;
  int64_t waypoint_num_;
  double target_yaw_deg_;
  double target_bias_x_cm_;
  double target_bias_y_cm_;
  double target_bias_z_cm_;
  double hold_position_tolerance_cm_;
  double hold_height_tolerance_cm_;
  double hold_yaw_tolerance_deg_;
  double hold_time_sec_;
  double mode_switch_delay_sec_;
  double target_republish_period_sec_;
  double timer_period_sec_;
  uint8_t cruise_control_mode_;
  uint8_t precision_control_mode_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  State state_;
  bool has_height_;
  double current_height_cm_;
  std::string last_z_source_;
  bool last_pose_valid_;
  double last_current_x_cm_;
  double last_current_y_cm_;
  double last_current_z_cm_;
  double last_current_yaw_deg_;
  double last_error_x_cm_;
  double last_error_y_cm_;
  double last_error_xy_cm_;
  double last_error_z_cm_;
  double last_error_yaw_deg_;
  bool final_target_valid_;
  double final_target_x_cm_;
  double final_target_y_cm_;
  double final_target_z_cm_;
  rclcpp::Time stable_since_;
  bool has_stable_since_;
  rclcpp::Time last_target_publish_time_;
  rclcpp::Time state_entry_time_;
  bool last_reached_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr final_waypoint_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::PrecisionLandingManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}