// =====================================================================
//  px4_bridge_node.cpp  ——  上位机 <-> PX4 飞控桥接（uXRCE-DDS）
//
//  上行（给飞控）—— 沿用 24 电赛旧工程的话题与单位约定：
//      订阅 /target_velocity  (std_msgs/Float32MultiArray)
//           data = [vx_cm_s, vy_cm_s, vz_cm_s, vyaw_deg_s]
//      发布 /fmu/in/offboard_control_mode  (velocity = true，持续发)
//      发布 /fmu/in/trajectory_setpoint    (velocity，NED，m/s)
//
//  下行（给上位机）—— 翻译成旧工程那套话题，老节点可原样复用：
//      订阅 /fmu/out/vehicle_local_position -> /height      (Int16, cm)
//      订阅 /fmu/out/vehicle_status
//                                    -> /is_st_ready (UInt8, armed && offboard 时 1)
//                                    -> /fc_armed    (Bool)
//                                    -> /fc_offboard (Bool)
//
//  坐标/单位换算（关键，别改错）：
//      · 旧工程输出是 ROS 地图坐标系(ENU)、cm/s、deg/s
//      · PX4 TrajectorySetpoint 要的是 NED、m/s、rad/s
//      · ENU -> NED:  x_ned = y_enu ; y_ned = x_enu ; z_ned = -z_enu
//      · yaw 在 NED 是顺时针为正，所以偏航角速度取反（yaw_sign 可调）
//
//  安全：
//      · OffboardControlMode 以 cmd_freq 持续发布（PX4 要求 >2Hz，这里默认 50Hz）
//      · /target_velocity 超时（setpoint_timeout 秒）后发**零速度**，不保持上一条
// =====================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kCmdArmDisarm = 400;   // MAV_CMD_COMPONENT_ARM_DISARM
constexpr uint32_t kCmdDoSetMode = 176;   // MAV_CMD_DO_SET_MODE
constexpr float kCustomModeOffboard = 6.0f;  // PX4 custom_mode: OFFBOARD
}  // namespace

class Px4Bridge : public rclcpp::Node
{
public:
  Px4Bridge() : Node("px4_bridge")
  {
    in_topic_         = declare_parameter<std::string>("in_topic", "/target_velocity");
    cmd_freq_         = declare_parameter<double>("cmd_freq", 50.0);
    setpoint_timeout_ = declare_parameter<double>("setpoint_timeout", 0.5);
    enu_to_ned_       = declare_parameter<bool>("enu_to_ned", true);
    yaw_sign_         = declare_parameter<double>("yaw_sign", -1.0);
    max_speed_        = declare_parameter<double>("max_speed", 2.0);       // m/s 水平
    max_up_speed_     = declare_parameter<double>("max_up_speed", 1.0);    // m/s 上升
    max_down_speed_   = declare_parameter<double>("max_down_speed", 1.5);  // m/s 下降
    max_yaw_rate_     = declare_parameter<double>("max_yaw_rate", 1.0);    // rad/s
    auto_arm_         = declare_parameter<bool>("auto_arm", false);
    auto_offboard_    = declare_parameter<bool>("auto_offboard", false);
    auto_delay_sec_   = declare_parameter<double>("auto_delay_sec", 5.0);

    const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
    command_pub_  = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos_pub);

    height_pub_   = create_publisher<std_msgs::msg::Int16>("/height", 10);
    ready_pub_    = create_publisher<std_msgs::msg::UInt8>(
      "/is_st_ready", rclcpp::QoS(10).transient_local());
    armed_pub_    = create_publisher<std_msgs::msg::Bool>(
      "/fc_armed", rclcpp::QoS(10).transient_local());
    offboard_state_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/fc_offboard", rclcpp::QoS(10).transient_local());

    setpoint_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      in_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onTargetVelocity(msg); });

    local_pos_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_sub,
      [this](VehicleLocalPosition::ConstSharedPtr msg) { onLocalPosition(msg); });
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", qos_sub,
      [this](VehicleStatus::ConstSharedPtr msg) { onVehicleStatus(msg); });

    const double period = 1.0 / std::max(2.0, cmd_freq_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      [this]() { tick(); });

    start_time_ = now();
    RCLCPP_INFO(get_logger(), "px4_bridge 已启动：%s -> PX4 Offboard（ENU->NED=%s）",
                in_topic_.c_str(), enu_to_ned_ ? "on" : "off");
    if (auto_arm_ || auto_offboard_) {
      RCLCPP_WARN(get_logger(),
                  "自动解锁/切 Offboard 已开启（%.1f 秒后执行）——真机请先拆桨验证！",
                  auto_delay_sec_);
    }
  }

private:
  // 注意：不能声明为 const —— const 版本的 get_clock() 返回 const Clock，
  // 而 rclcpp::Clock::now() 不是 const 成员函数，会编译不过。
  uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void onTargetVelocity(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s 需要 4 个 float [vx,vy,vz cm/s, vyaw deg/s]，收到 %zu 个",
                           in_topic_.c_str(), msg->data.size());
      return;
    }
    double vx = msg->data[0] / 100.0;                 // cm/s -> m/s
    double vy = msg->data[1] / 100.0;
    double vz = msg->data[2] / 100.0;
    double wyaw = msg->data[3] * kPi / 180.0;         // deg/s -> rad/s

    if (enu_to_ned_) {
      const double ex = vx, ey = vy;                  // ENU
      vx = ey;
      vy = ex;
      vz = -vz;                                       // NED
    }
    wyaw *= yaw_sign_;

    vx = std::clamp(vx, -max_speed_, max_speed_);
    vy = std::clamp(vy, -max_speed_, max_speed_);
    vz = std::clamp(vz, -max_up_speed_, max_down_speed_);  // NED：z 向下为正
    wyaw = std::clamp(wyaw, -max_yaw_rate_, max_yaw_rate_);

    target_[0] = vx;
    target_[1] = vy;
    target_[2] = vz;
    target_[3] = wyaw;
    last_setpoint_ = now();
    have_setpoint_ = true;
  }

  void onLocalPosition(const VehicleLocalPosition::ConstSharedPtr msg)
  {
    // PX4 local position 是 NED，z 向下为正；对外给"高度"（向上为正，cm）
    std_msgs::msg::Int16 h;
    h.data = static_cast<int16_t>(std::lround(-static_cast<double>(msg->z) * 100.0));
    height_pub_->publish(h);
  }

  void onVehicleStatus(const VehicleStatus::ConstSharedPtr msg)
  {
    const bool armed = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
    const bool offboard = (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD);

    std_msgs::msg::Bool b;
    b.data = armed;
    armed_pub_->publish(b);
    b.data = offboard;
    offboard_state_pub_->publish(b);

    std_msgs::msg::UInt8 r;
    r.data = (armed && offboard) ? 1 : 0;
    ready_pub_->publish(r);
  }

  void sendVehicleCommand(uint32_t cmd, float p1, float p2)
  {
    VehicleCommand m{};
    m.timestamp = now_us();
    m.command = cmd;
    m.param1 = p1;
    m.param2 = p2;
    m.target_system = 1;
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    m.confirmation = 0;
    m.from_external = true;
    command_pub_->publish(m);
  }

  void tick()
  {
    const auto t = now();
    const double age = have_setpoint_ ? (t - last_setpoint_).seconds() : 1e9;
    if (age > setpoint_timeout_) {
      if (have_setpoint_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%s 已 %.2fs 无新指令，发送零速度", in_topic_.c_str(), age);
      }
      target_[0] = target_[1] = target_[2] = target_[3] = 0.0;
    }

    // 1) Offboard 控制模式（必须持续发，否则 PX4 会退出 Offboard）
    OffboardControlMode ocm{};
    ocm.timestamp = now_us();
    ocm.position = false;
    ocm.velocity = true;
    ocm.acceleration = false;
    ocm.attitude = false;
    ocm.body_rate = false;
    ocm.thrust_and_torque = false;
    ocm.direct_actuator = false;
    offboard_pub_->publish(ocm);

    // 2) 速度设定值（未控的维度用 NaN）
    TrajectorySetpoint sp{};
    sp.timestamp = now_us();
    const float nan32 = std::nanf("");
    sp.position = {nan32, nan32, nan32};
    sp.velocity = {static_cast<float>(target_[0]),
                   static_cast<float>(target_[1]),
                   static_cast<float>(target_[2])};
    sp.acceleration = {nan32, nan32, nan32};
    sp.jerk = {nan32, nan32, nan32};
    sp.yaw = nan32;
    sp.yawspeed = static_cast<float>(target_[3]);
    setpoint_pub_->publish(sp);

    // 3) 可选的自动解锁 / 切 Offboard
    const double elapsed = (t - start_time_).seconds();
    if (auto_arm_ && !arm_sent_ && elapsed > auto_delay_sec_) {
      sendVehicleCommand(kCmdArmDisarm, 1.0f, 0.0f);
      arm_sent_ = true;
      RCLCPP_WARN(get_logger(), "已发送解锁命令");
    }
    if (auto_offboard_ && !offboard_sent_ && elapsed > auto_delay_sec_ + 1.0) {
      sendVehicleCommand(kCmdDoSetMode, 1.0f, kCustomModeOffboard);
      offboard_sent_ = true;
      RCLCPP_WARN(get_logger(), "已发送切换 Offboard 命令");
    }
  }

  // ---- 参数 ----
  std::string in_topic_;
  double cmd_freq_{50.0};
  double setpoint_timeout_{0.5};
  double yaw_sign_{-1.0};
  double max_speed_{2.0};
  double max_up_speed_{1.0};
  double max_down_speed_{1.5};
  double max_yaw_rate_{1.0};
  double auto_delay_sec_{5.0};
  bool enu_to_ned_{true};
  bool auto_arm_{false};
  bool auto_offboard_{false};

  // ---- 状态 ----
  rclcpp::Time start_time_;
  rclcpp::Time last_setpoint_;
  bool have_setpoint_{false};
  bool arm_sent_{false};
  bool offboard_sent_{false};
  double target_[4]{0.0, 0.0, 0.0, 0.0};

  // ---- ROS 接口 ----
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr offboard_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4Bridge>());
  rclcpp::shutdown();
  return 0;
}
