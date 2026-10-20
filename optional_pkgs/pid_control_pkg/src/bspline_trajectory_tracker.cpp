#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/msg/bspline.hpp"

namespace pid_control_pkg
{

namespace
{

using ego_planner::UniformBspline;

Eigen::Vector3d toEigenVector3d(const Eigen::VectorXd & value)
{
  if (value.size() < 3) {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(value(0), value(1), value(2));
}

double clampAbs(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

double normalizeAngleDeg(double angle_deg)
{
  return angles::to_degrees(angles::normalize_angle(angles::from_degrees(angle_deg)));
}

}  // namespace

class BsplineTrajectoryTracker : public rclcpp::Node
{
public:
  BsplineTrajectoryTracker()
  : rclcpp::Node("bspline_trajectory_tracker"),
    control_frequency_(declare_parameter("control_frequency", 100.0)),
    odom_timeout_sec_(declare_parameter("odom_timeout_sec", 0.2)),
    kp_position_xy_(declare_parameter("kp_position_xy", 0.9)),
    kp_position_z_(declare_parameter("kp_position_z", 1.2)),
    kv_velocity_xy_(declare_parameter("kv_velocity_xy", 0.45)),
    kv_velocity_z_(declare_parameter("kv_velocity_z", 0.6)),
    ff_velocity_gain_(declare_parameter("ff_velocity_gain", 1.0)),
    use_fixed_yaw_(declare_parameter("use_fixed_yaw", true)),
    fixed_yaw_deg_(declare_parameter("fixed_yaw_deg", 0.0)),
    yaw_kp_(declare_parameter("yaw_kp", 1.2)),
    max_linear_velocity_cm_(declare_parameter("max_linear_velocity_cm", 40.0)),
    max_vertical_velocity_cm_(declare_parameter("max_vertical_velocity_cm", 35.0)),
    max_yaw_velocity_deg_(declare_parameter("max_yaw_velocity_deg", 30.0)),
    capture_distance_threshold_m_(declare_parameter("capture_distance_threshold_m", 0.6)),
    capture_search_backward_sec_(declare_parameter("capture_search_backward_sec", 0.5)),
    capture_search_forward_sec_(declare_parameter("capture_search_forward_sec", 2.0)),
    capture_sample_dt_sec_(declare_parameter("capture_sample_dt_sec", 0.02)),
    heading_speed_threshold_mps_(declare_parameter("heading_speed_threshold_mps", 0.1)),
    enable_debug_publish_(declare_parameter("enable_debug_publish", true)),
    enable_debug_logs_(declare_parameter("enable_debug_logs", true)),
    debug_log_throttle_ms_(declare_parameter("debug_log_throttle_ms", 1000)),
    world_frame_id_(declare_parameter("world_frame_id", std::string("world"))),
    body_frame_id_(declare_parameter("body_frame_id", std::string("body"))),
    last_desired_yaw_deg_(fixed_yaw_deg_)
  {
    bspline_sub_ = create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline", rclcpp::QoS(10),
      std::bind(&BsplineTrajectoryTracker::bsplineCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(50),
      std::bind(&BsplineTrajectoryTracker::odomCallback, this, std::placeholders::_1));

    target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "target_velocity", rclcpp::QoS(10));
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "bspline_tracker_debug", rclcpp::QoS(10));
    phase_debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "bspline_tracker_phase", rclcpp::QoS(10));
    reference_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "bspline_tracker_reference_pose", rclcpp::QoS(10));
    position_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "bspline_tracker_position_error_body", rclcpp::QoS(10));
    velocity_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "bspline_tracker_velocity_error_body", rclcpp::QoS(10));
    command_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "bspline_tracker_command_body", rclcpp::QoS(10));

    const double period_sec = 1.0 / std::max(control_frequency_, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
      std::bind(&BsplineTrajectoryTracker::controlTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Bspline trajectory tracker started. freq=%.1fHz fixed_yaw=%s capture=%.2fm",
      control_frequency_, use_fixed_yaw_ ? "true" : "false", capture_distance_threshold_m_);
  }

private:
  struct OdomState
  {
    bool valid{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_body{Eigen::Vector3d::Zero()};
    double yaw_deg{0.0};
  };

  struct TrajectoryState
  {
    bool active{false};
    int64_t traj_id{0};
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time phase_anchor_ros_time{0, 0, RCL_ROS_TIME};
    double phase_anchor_traj_time{0.0};
    double duration{0.0};
    UniformBspline position_traj;
    UniformBspline velocity_traj;
  };

  struct AlignmentResult
  {
    double aligned_t{0.0};
    double theoretical_distance{std::numeric_limits<double>::quiet_NaN()};
    double aligned_distance{std::numeric_limits<double>::quiet_NaN()};
    double search_start{0.0};
    double search_end{0.0};
    bool used_capture{false};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_state_.stamp = msg->header.stamp;
    odom_state_.position_world.x() = msg->pose.pose.position.x;
    odom_state_.position_world.y() = msg->pose.pose.position.y;
    odom_state_.position_world.z() = msg->pose.pose.position.z;
    odom_state_.velocity_body.x() = msg->twist.twist.linear.x;
    odom_state_.velocity_body.y() = msg->twist.twist.linear.y;
    odom_state_.velocity_body.z() = msg->twist.twist.linear.z;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    odom_state_.yaw_deg = yaw * 180.0 / M_PI;
    odom_state_.valid = true;
  }

  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.size() < 2) {
      RCLCPP_WARN(get_logger(), "Ignore invalid Bspline message with empty control points or knots.");
      return;
    }

    Eigen::MatrixXd pos_pts(3, static_cast<int>(msg->pos_pts.size()));
    Eigen::VectorXd knots(static_cast<int>(msg->knots.size()));
    for (std::size_t i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(0, static_cast<int>(i)) = msg->pos_pts[i].x;
      pos_pts(1, static_cast<int>(i)) = msg->pos_pts[i].y;
      pos_pts(2, static_cast<int>(i)) = msg->pos_pts[i].z;
    }
    for (std::size_t i = 0; i < msg->knots.size(); ++i) {
      knots(static_cast<int>(i)) = msg->knots[i];
    }

    const double interval = std::max(1e-3, msg->knots[1] - msg->knots[0]);
    UniformBspline position_traj(pos_pts, msg->order, interval);
    position_traj.setKnot(knots);
    UniformBspline velocity_traj = position_traj.getDerivative();

    TrajectoryState next_state;
    next_state.active = true;
    next_state.traj_id = msg->traj_id;
    next_state.start_time = rclcpp::Time(msg->start_time.sec, msg->start_time.nanosec, RCL_ROS_TIME);
    next_state.duration = position_traj.getTimeSum();
    next_state.position_traj = position_traj;
    next_state.velocity_traj = velocity_traj;

    const rclcpp::Time now = this->now();
    const double theoretical_t = std::clamp((now - next_state.start_time).seconds(), 0.0, next_state.duration);
    const AlignmentResult alignment = alignTrajectoryPhase(next_state, theoretical_t);
    next_state.phase_anchor_ros_time = now;
    next_state.phase_anchor_traj_time = alignment.aligned_t;
    active_traj_ = next_state;

    publishPhaseDebug(
      now, theoretical_t, active_traj_.phase_anchor_traj_time, alignment.theoretical_distance,
      alignment.aligned_distance, alignment.search_start, alignment.search_end,
      alignment.used_capture);

    if (enable_debug_logs_) {
      RCLCPP_INFO(
        get_logger(),
        "Accepted Bspline traj_id=%ld theory_t=%.3f aligned_t=%.3f duration=%.3f capture=%s theory_dist=%.3f aligned_dist=%.3f",
        static_cast<long>(active_traj_.traj_id), theoretical_t, active_traj_.phase_anchor_traj_time,
        active_traj_.duration, alignment.used_capture ? "true" : "false",
        alignment.theoretical_distance, alignment.aligned_distance);
    }
  }

  AlignmentResult alignTrajectoryPhase(TrajectoryState & traj, double theoretical_t) const
  {
    AlignmentResult result;
    result.aligned_t = theoretical_t;
    if (!odom_state_.valid) {
      return result;
    }

    const Eigen::Vector3d theoretical_ref = toEigenVector3d(traj.position_traj.evaluateDeBoorT(theoretical_t));
    const double theory_distance = (theoretical_ref - odom_state_.position_world).norm();
    result.theoretical_distance = theory_distance;
    if (theory_distance <= capture_distance_threshold_m_) {
      result.aligned_distance = theory_distance;
      return result;
    }

    result.search_start = std::max(0.0, theoretical_t - capture_search_backward_sec_);
    result.search_end = std::min(traj.duration, theoretical_t + capture_search_forward_sec_);
    double best_t = theoretical_t;
    double best_distance = theory_distance;

    for (double t = result.search_start; t <= result.search_end; t += capture_sample_dt_sec_) {
      const Eigen::Vector3d ref = toEigenVector3d(traj.position_traj.evaluateDeBoorT(t));
      const double distance = (ref - odom_state_.position_world).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_t = t;
      }
    }

    result.aligned_t = best_t;
    result.aligned_distance = best_distance;
    result.used_capture = std::abs(best_t - theoretical_t) > 1e-4;
    return result;
  }

  bool odomFresh(const rclcpp::Time & now) const
  {
    return odom_state_.valid && (now - odom_state_.stamp).seconds() <= odom_timeout_sec_;
  }

  double activeTrajectoryTime(const rclcpp::Time & now) const
  {
    if (!active_traj_.active) {
      return 0.0;
    }
    const double advanced = (now - active_traj_.phase_anchor_ros_time).seconds();
    return std::clamp(active_traj_.phase_anchor_traj_time + advanced, 0.0, active_traj_.duration);
  }

  std_msgs::msg::Float32MultiArray makeVelocityMessage(
    double vx_cm_s, double vy_cm_s, double vz_cm_s, double yaw_rate_deg_s) const
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
      static_cast<float>(vx_cm_s),
      static_cast<float>(vy_cm_s),
      static_cast<float>(vz_cm_s),
      static_cast<float>(yaw_rate_deg_s),
    };
    return msg;
  }

  void publishZeroCommand()
  {
    target_velocity_pub_->publish(makeVelocityMessage(0.0, 0.0, 0.0, 0.0));
  }

  void publishVectorStamped(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    const Eigen::Vector3d & value,
    const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & publisher)
  {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.vector.x = value.x();
    msg.vector.y = value.y();
    msg.vector.z = value.z();
    publisher->publish(msg);
  }

  void publishReferencePose(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & p_ref_world,
    double desired_yaw_deg)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_id_;
    msg.pose.position.x = p_ref_world.x();
    msg.pose.position.y = p_ref_world.y();
    msg.pose.position.z = p_ref_world.z();
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, angles::from_degrees(desired_yaw_deg));
    msg.pose.orientation = tf2::toMsg(quat);
    reference_pose_pub_->publish(msg);
  }

  void publishPhaseDebug(
    const rclcpp::Time & stamp,
    double theoretical_t,
    double active_t,
    double theoretical_distance,
    double aligned_distance,
    double search_start,
    double search_end,
    bool used_capture)
  {
    if (!enable_debug_publish_) {
      return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
      static_cast<float>(active_traj_.traj_id),
      static_cast<float>(stamp.seconds()),
      static_cast<float>(theoretical_t),
      static_cast<float>(active_t),
      static_cast<float>(active_t - theoretical_t),
      static_cast<float>(theoretical_distance),
      static_cast<float>(aligned_distance),
      static_cast<float>(search_start),
      static_cast<float>(search_end),
      static_cast<float>(used_capture ? 1.0 : 0.0),
    };
    phase_debug_pub_->publish(msg);
  }

  double resolveDesiredYawDeg(const Eigen::Vector3d & v_ref_world)
  {
    if (use_fixed_yaw_) {
      last_desired_yaw_deg_ = fixed_yaw_deg_;
      return fixed_yaw_deg_;
    }

    const double xy_speed = std::hypot(v_ref_world.x(), v_ref_world.y());
    if (xy_speed > heading_speed_threshold_mps_) {
      last_desired_yaw_deg_ = std::atan2(v_ref_world.y(), v_ref_world.x()) * 180.0 / M_PI;
    }
    return last_desired_yaw_deg_;
  }

  void publishDebug(
    double theoretical_t,
    double active_t,
    const Eigen::Vector3d & p_ref_world,
    const Eigen::Vector3d & e_pos_body,
    const Eigen::Vector3d & e_vel_body,
    const Eigen::Vector3d & v_ref_body,
    const Eigen::Vector3d & cmd_body,
    double desired_yaw_deg,
    double yaw_error_deg,
    double yaw_rate_deg_s)
  {
    if (!enable_debug_publish_) {
      return;
    }

    std_msgs::msg::Float32MultiArray debug_msg;
    debug_msg.data = {
      static_cast<float>(active_traj_.traj_id),
      static_cast<float>(theoretical_t),
      static_cast<float>(active_t),
      static_cast<float>(p_ref_world.x()),
      static_cast<float>(p_ref_world.y()),
      static_cast<float>(p_ref_world.z()),
      static_cast<float>(odom_state_.position_world.x()),
      static_cast<float>(odom_state_.position_world.y()),
      static_cast<float>(odom_state_.position_world.z()),
      static_cast<float>(e_pos_body.x()),
      static_cast<float>(e_pos_body.y()),
      static_cast<float>(e_pos_body.z()),
      static_cast<float>(e_vel_body.x()),
      static_cast<float>(e_vel_body.y()),
      static_cast<float>(e_vel_body.z()),
      static_cast<float>(v_ref_body.x()),
      static_cast<float>(v_ref_body.y()),
      static_cast<float>(v_ref_body.z()),
      static_cast<float>(cmd_body.x()),
      static_cast<float>(cmd_body.y()),
      static_cast<float>(cmd_body.z()),
      static_cast<float>(desired_yaw_deg),
      static_cast<float>(yaw_error_deg),
      static_cast<float>(yaw_rate_deg_s),
    };
    debug_pub_->publish(debug_msg);
  }

  void controlTimerCallback()
  {
    const rclcpp::Time now = this->now();

    if (!active_traj_.active) {
      publishZeroCommand();
      return;
    }

    if (!odomFresh(now)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Tracker odom timeout. Publish zero velocity for safety.");
      publishZeroCommand();
      return;
    }

    const double active_t = activeTrajectoryTime(now);
    const double theoretical_t = std::clamp((now - active_traj_.start_time).seconds(), 0.0, active_traj_.duration);
    const Eigen::Vector3d p_ref_world = toEigenVector3d(active_traj_.position_traj.evaluateDeBoorT(active_t));
    Eigen::Vector3d v_ref_world = Eigen::Vector3d::Zero();
    if (active_t < active_traj_.duration) {
      v_ref_world = toEigenVector3d(active_traj_.velocity_traj.evaluateDeBoorT(active_t));
    }

    const double yaw_rad = odom_state_.yaw_deg * M_PI / 180.0;
    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);

    const Eigen::Vector3d e_pos_world = p_ref_world - odom_state_.position_world;
    Eigen::Vector3d e_pos_body;
    e_pos_body.x() = cos_yaw * e_pos_world.x() + sin_yaw * e_pos_world.y();
    e_pos_body.y() = -sin_yaw * e_pos_world.x() + cos_yaw * e_pos_world.y();
    e_pos_body.z() = e_pos_world.z();

    Eigen::Vector3d v_ref_body;
    v_ref_body.x() = cos_yaw * v_ref_world.x() + sin_yaw * v_ref_world.y();
    v_ref_body.y() = -sin_yaw * v_ref_world.x() + cos_yaw * v_ref_world.y();
    v_ref_body.z() = v_ref_world.z();

    const Eigen::Vector3d e_vel_body = v_ref_body - odom_state_.velocity_body;

    Eigen::Vector3d cmd_body = Eigen::Vector3d::Zero();
    cmd_body.x() = ff_velocity_gain_ * v_ref_body.x() + kp_position_xy_ * e_pos_body.x() + kv_velocity_xy_ * e_vel_body.x();
    cmd_body.y() = ff_velocity_gain_ * v_ref_body.y() + kp_position_xy_ * e_pos_body.y() + kv_velocity_xy_ * e_vel_body.y();
    cmd_body.z() = ff_velocity_gain_ * v_ref_body.z() + kp_position_z_ * e_pos_body.z() + kv_velocity_z_ * e_vel_body.z();

    cmd_body.x() = clampAbs(cmd_body.x() * 100.0, max_linear_velocity_cm_);
    cmd_body.y() = clampAbs(cmd_body.y() * 100.0, max_linear_velocity_cm_);
    cmd_body.z() = clampAbs(cmd_body.z() * 100.0, max_vertical_velocity_cm_);

    const double desired_yaw_deg = resolveDesiredYawDeg(v_ref_world);
    const double yaw_error_deg = normalizeAngleDeg(desired_yaw_deg - odom_state_.yaw_deg);
    const double yaw_rate_deg_s = clampAbs(yaw_kp_ * yaw_error_deg, max_yaw_velocity_deg_);

    publishReferencePose(now, p_ref_world, desired_yaw_deg);
    publishVectorStamped(now, body_frame_id_, e_pos_body, position_error_pub_);
    publishVectorStamped(now, body_frame_id_, e_vel_body, velocity_error_pub_);
    publishVectorStamped(now, body_frame_id_, cmd_body, command_pub_);
    publishPhaseDebug(
      now, theoretical_t, active_t,
      (toEigenVector3d(active_traj_.position_traj.evaluateDeBoorT(theoretical_t)) - odom_state_.position_world).norm(),
      (p_ref_world - odom_state_.position_world).norm(),
      std::max(0.0, theoretical_t - capture_search_backward_sec_),
      std::min(active_traj_.duration, theoretical_t + capture_search_forward_sec_),
      std::abs(active_t - theoretical_t) > 1e-4);

    if (enable_debug_logs_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), debug_log_throttle_ms_,
        "traj=%ld theory_t=%.2f active_t=%.2f pos_err_body=(%.2f, %.2f, %.2f)m vel_err_body=(%.2f, %.2f, %.2f)m yaw_err=%.1fdeg cmd=(%.1f, %.1f, %.1f)cm/s",
        static_cast<long>(active_traj_.traj_id), theoretical_t, active_t,
        e_pos_body.x(), e_pos_body.y(), e_pos_body.z(),
        e_vel_body.x(), e_vel_body.y(), e_vel_body.z(),
        yaw_error_deg, cmd_body.x(), cmd_body.y(), cmd_body.z());
    }

    target_velocity_pub_->publish(makeVelocityMessage(
      cmd_body.x(), cmd_body.y(), cmd_body.z(), yaw_rate_deg_s));
    publishDebug(
      theoretical_t, active_t, p_ref_world, e_pos_body, e_vel_body, v_ref_body, cmd_body,
      desired_yaw_deg, yaw_error_deg, yaw_rate_deg_s);
  }

  double control_frequency_;
  double odom_timeout_sec_;
  double kp_position_xy_;
  double kp_position_z_;
  double kv_velocity_xy_;
  double kv_velocity_z_;
  double ff_velocity_gain_;
  bool use_fixed_yaw_;
  double fixed_yaw_deg_;
  double yaw_kp_;
  double max_linear_velocity_cm_;
  double max_vertical_velocity_cm_;
  double max_yaw_velocity_deg_;
  double capture_distance_threshold_m_;
  double capture_search_backward_sec_;
  double capture_search_forward_sec_;
  double capture_sample_dt_sec_;
  double heading_speed_threshold_mps_;
  bool enable_debug_publish_;
  bool enable_debug_logs_;
  int debug_log_throttle_ms_;
  std::string world_frame_id_;
  std::string body_frame_id_;
  double last_desired_yaw_deg_;

  OdomState odom_state_;
  TrajectoryState active_traj_;

  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr phase_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace pid_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pid_control_pkg::BsplineTrajectoryTracker>());
  rclcpp::shutdown();
  return 0;
}