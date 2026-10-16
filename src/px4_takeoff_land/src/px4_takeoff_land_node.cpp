// =====================================================================
//  px4_takeoff_land_node.cpp
//  原地起飞 -> 悬停 -> 原地降落（PX4 Offboard / uXRCE-DDS）
//
//  这个节点补的是 px4_bridge 缺的那一环：px4_bridge 只做「速度透传」，
//  没有「解锁 / 切 Offboard / 起飞 / 悬停 / 降落」的任务状态机。
//
//  ---------------------------------------------------------------------
//  前提（缺一不可，详见同目录 README.md）
//    1. 飞控已刷 PX4 1.15.4 标准板级固件（**不需要改 PX4 源码**，
//       offboard 起飞是 1.15 的内置能力，见
//       src/modules/mc_pos_control/MulticopterPositionControl.cpp:417）
//    2. 香橙派上跑着 Micro-XRCE-DDS-Agent，且 PX4 的 uxrce_dds_client
//       已连上（`ros2 topic echo /fmu/out/vehicle_status` 有数据）
//    3. EKF2 有有效的本地位置/速度估计：
//         室外 -> GPS 定位即可
//         室内无 GPS -> 必须加光流 + 测距（或 VIO/mocap）喂给 EKF2，
//         否则 Offboard 会被 PX4 直接拒绝
//         （依据：
//          src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp）
//    4. 飞控参数按 README 清单配好（解锁方式 / 失控保护 / 起飞 ramp）
//
//  ---------------------------------------------------------------------
//  2026-10-09 新增：可选的小任务航线（mission_enable:=true）
//      起飞(到 target_alt) -> 沿**起飞时刻机头方向**前进 forward_dist
//      -> 原地掉头 turn_deg -> 保持新航向飞回起飞点正上方
//      -> 悬停 hover_time -> 交还 PX4 自动降落
//      全部用「位置 + 绝对航向」设定值（NED 系），因此要求 control_mode=position
//      （写成 velocity 会被自动改回 position 并打警告）。
//      参数：mission_enable / forward_dist / turn_deg / pos_tol / yaw_tol_deg /
//            settle_time / leg_timeout
//      每段都会把「目标点 + 当前误差」打进日志，方便飞完复盘。
//
//  ---------------------------------------------------------------------
//  2026-10-10（第二次炸机复盘后）四处改动
//    1. **解锁是"按当前模式"判定的**：PX4 的 pre_flight_checks_pass 就是
//       Commander.cpp:1834 的 canArm(nav_state)。上一次任务中止后飞控停在 OFFBOARD，
//       而此刻没有 Offboard 流 → modeCheck.cpp 的 mode_req_offboard_signal 判死 →
//       解锁被拒 → 任务报「解锁超时」。停在 AUTO_LAND / AUTO_MISSION 也同理。
//       → 新增 SetArmMode 阶段：先等飞控说"可以解锁"，不行就先切一次 POSCTL。
//    2. 链路看门狗 link_timeout 0.5s → 1.0s（实测这条话题 ~100Hz、最大间隔 0.021s，
//       1.0s 一定是真断；0.5s 在香橙派忙的时候容易误伤）。
//    3. 订阅 /fmu/out/failsafe_flags，解锁被拒时把原因翻成人话打出来。
//    4. dry_run 不再假报「解锁超时」（干跑本来就不会解锁，等 8 秒报错是误导）。
//
//  ---------------------------------------------------------------------
//  2026-10-10 深夜第二轮（用户要求：先别掉头 + 把之前提的优化加上）
//    A. **任务形态**：`return_home=false`（新默认）→ 起飞到 0.5m → 沿机头方向前进
//       `forward_dist`（现在默认 1.2m）→ 悬停 → 交还 PX4 降落。**不掉头、不返航**。
//       要恢复原来的"掉头+返航"就给 `return_home:=true`（turn_deg 照旧）。
//    B. **掉头分段缓转** `turn_step_deg`（默认 30°，0=一次转到位）：每次只把航向设定值
//       推一小段、等机体大致跟上再推下一段 —— 避免一次性 180° 让电机瞬间吃满差速
//       （10-10 炸机就发生在"开始掉头"那一瞬）。现在不掉头，但参数留着，
//       以后要重新开掉头时它就是安全阀。
//    C. **航向交叉校验** `yaw_xcheck_deg`：把飞控 EKF 的 heading 和 EV 桥发给飞控的那份
//       位姿（`/fmu/in/vehicle_visual_odometry`，NED 系）的 yaw 对着看，持续对不上就中止降落。
//       依据：2D 激光 SLAM 没有 IMU，快速掉头/漂移时航向可能"缓慢但持续地跑偏"，
//       队友在 EV 桥里的 45° 单帧跳变门限挡不住这种；而 EKF 吃到错误航向就会驱动电机猛追。
//       ⚠️ 2026-10-10 实测踩坑：不能用 `/tracked_pose`！那是 SLAM 的 map 系，原点航向任意
//       （当时实测 /tracked_pose yaw≈0° 而飞控 heading≈90.4°，差 90°），直接比会一解锁就误报。
//       这里比较的是"两者差值相对地面基准的变化量"，所以框架常量差（map↔NED 对齐）自动被消掉。
//    D. **高度交叉校验** `alt_xcheck_m`：把飞控的 z 和定高器 TOF(/tofsense/height) 对着看。
//       用**起飞前在地面记下的两者差值**当基准，比的是"变化量"，消掉固定偏差；
//       机体倾斜超过 `tilt_gate_deg` 时不做这项（倾斜会让 TOF 斜射、读数变长 ——
//       10-10 那次"1.5m/2.07m"其实就是机体在翻滚时的斜射伪值，不是真高度）。
//    E. 飞控"已解锁→未解锁"时单独打一条提示（多半是重启/掉电/触发保护），
//       并提醒去看 agent 日志里的 session re-established。
//
//  安全约定
//    · OffboardControlMode 以 control_freq 持续发布（PX4 要求周期小于
//      COM_OF_LOSS_T，默认 1s，否则会退出 Offboard 触发失控保护）
//    · 任何异常/超时 -> 进入 ABORT：交还 PX4 自动降落
//    · 第一次务必拆桨 / 用 SITL 验证；dry_run 参数可先只跑流程不发指令
//    · **不要和 px4_bridge 同时运行**：两者都发
//      /fmu/in/offboard_control_mode，会互相打架
// =====================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::VehicleOdometry;
using std_msgs::msg::Float32;
using px4_msgs::msg::FailsafeFlags;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace {
// 把角度规整到 (-pi, pi]
double wrapPi(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

// MAVLink / PX4 命令 ID（来自 msg/VehicleCommand.msg）
constexpr uint32_t kCmdNavLand   = 21;    // VEHICLE_CMD_NAV_LAND
constexpr uint32_t kCmdDoSetMode = 176;   // VEHICLE_CMD_DO_SET_MODE
constexpr uint32_t kCmdArmDisarm = 400;   // VEHICLE_CMD_COMPONENT_ARM_DISARM
// custom main mode（来自 src/modules/commander/px4_custom_mode.h）
constexpr float kMainModeOffboard = 6.0f;
constexpr float kMainModePosctl   = 3.0f;   // 解锁前的"归位"模式：这一族只要本地位置有效就能解锁
}  // namespace

class TakeoffLand : public rclcpp::Node
{
public:
  TakeoffLand() : Node("px4_takeoff_land")
  {
    // ---- 任务参数 ----
    target_alt_       = declare_parameter<double>("target_alt", 2.0);      // m，相对起飞点
    hover_time_       = declare_parameter<double>("hover_time", 5.0);      // s
    climb_vel_        = declare_parameter<double>("climb_vel", 0.5);       // m/s（velocity 模式用）
    descend_vel_      = declare_parameter<double>("descend_vel", 0.5);     // m/s（velocity 模式用）
    land_switch_alt_  = declare_parameter<double>("land_switch_alt", 0.3); // m，低于此高度交还 PX4 降落
    alt_tol_          = declare_parameter<double>("alt_tol", 0.15);        // m，高度到位判定
    // 2026-10-09 晚 事故后调整：offboard 流从 50 Hz 降到 **10 Hz**。
    // 原因：飞控↔香橙派那条 921600 串口的接收方向实测已跑到 ~74 KB/s（83% 占空）
    // （PX4 按 dds_topics.yaml 全速发 sensor_combined 100 Hz、local_position 95 Hz…），
    // 我们再往里多发一份就更容易把它顶到溢出 → DDS 会话超时重建 → PX4 掉出 Offboard。
    // PX4 只要求 offboard 周期 < COM_OF_LOSS_T（默认 1 s），10 Hz 有 10 倍余量。
    control_freq_     = declare_parameter<double>("control_freq", 10.0);   // Hz
    // 飞控链路看门狗：/fmu/out/vehicle_local_position 超过这么久没更新才判定链路断。
    // ⚠️ 别改成 vehicle_status —— 它只有 1.3Hz，会频繁误判（2026-10-09 踩过）
    // 2026-10-10：0.5 → 1.0s。实测这条话题 ~100Hz、最大间隔 0.021s（连丢 ~100 帧才可能到 1s），
    //   而 0.5s 在香橙派忙（SLAM/建图）时容易误伤。飞控自己的 Offboard 失效保护是
    //   COM_OF_LOSS_T（默认 1s），所以放宽到 1s 不会更危险。
    link_timeout_     = declare_parameter<double>("link_timeout", 1.0);    // s
    prestream_sec_    = declare_parameter<double>("prestream_sec", 2.0);   // s，切 Offboard 前先发流
    start_delay_      = declare_parameter<double>("start_delay", 3.0);     // s，启动后等待
    arm_timeout_      = declare_parameter<double>("arm_timeout", 8.0);     // s
    offboard_timeout_ = declare_parameter<double>("offboard_timeout", 8.0); // s
    takeoff_timeout_  = declare_parameter<double>("takeoff_timeout", 30.0); // s
    descend_timeout_  = declare_parameter<double>("descend_timeout", 45.0); // s
    land_timeout_     = declare_parameter<double>("land_timeout", 60.0);   // s
    max_alt_          = declare_parameter<double>("max_alt", 0.0);         // m，>0 时作硬上限保护
    control_mode_     = declare_parameter<std::string>("control_mode", "position"); // position | velocity
    dry_run_          = declare_parameter<bool>("dry_run", false);         // true = 只跑流程不发指令

    // ---- 2026-10-09 新增：小任务航线参数（mission_enable=true 才生效）----
    mission_enable_   = declare_parameter<bool>("mission_enable", false);
    forward_dist_     = declare_parameter<double>("forward_dist", 0.5);    // m，沿起飞时刻机头方向
    turn_deg_         = declare_parameter<double>("turn_deg", 180.0);      // deg，掉头角（正=顺时针/向右）
    pos_tol_          = declare_parameter<double>("pos_tol", 0.15);        // m，水平到位容差
    yaw_tol_deg_      = declare_parameter<double>("yaw_tol_deg", 10.0);    // deg，航向到位容差
    settle_time_      = declare_parameter<double>("settle_time", 0.5);     // s，连续在容差内多久算到位
    leg_timeout_      = declare_parameter<double>("leg_timeout", 20.0);    // s，每段超时（超时即 ABORT 降落）

    // ---- 2026-10-10 深夜第二轮（见文件头 A~D）----
    // A. 任务形态：false = 前进到位就悬停降落（不掉头、不返航）；true = 恢复"掉头+飞回原点"
    return_home_        = declare_parameter<bool>("return_home", false);
    // B. 掉头分段缓转：>0 时每次最多转这么多度，等机体跟上再推下一段；0 = 一次转到位（老行为）
    turn_step_deg_      = declare_parameter<double>("turn_step_deg", 30.0);
    // C. 航向交叉校验：飞控 heading 与 SLAM yaw 差超过它 → 持续 xcheck_hold 就中止（0=关）
    yaw_xcheck_deg_     = declare_parameter<double>("yaw_xcheck_deg", 20.0);
    // D. 高度交叉校验：飞控 z 与定高器 TOF 的**变化量**差超过它 → 持续 xcheck_hold 就中止（0=关）
    alt_xcheck_m_       = declare_parameter<double>("alt_xcheck_m", 0.5);
    alt_xcheck_abort_m_ = declare_parameter<double>("alt_xcheck_abort_m", 0.9);
    xcheck_hold_        = declare_parameter<double>("xcheck_hold", 1.0);   // s，两道校验都要连续超限这么久才动作
    tilt_gate_deg_      = declare_parameter<double>("tilt_gate_deg", 25.0); // deg，机体倾斜超过它就不做高度校验

    if (control_mode_ != "position" && control_mode_ != "velocity") {
      RCLCPP_WARN(get_logger(), "control_mode='%s' 不合法，回退到 position",
                  control_mode_.c_str());
      control_mode_ = "position";
    }
    if (mission_enable_ && control_mode_ != "position") {
      RCLCPP_WARN(get_logger(),
                  "mission_enable 需要 control_mode=position（任务段用位置+绝对航向设定值），"
                  "已自动改回 position");
      control_mode_ = "position";
    }

    // QoS 与 px4_bridge 保持一致（该组合已在板上联调通过）
    const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
    command_pub_  = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos_pub);
    state_pub_    = create_publisher<std_msgs::msg::String>("/mission_state", 10);

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", qos_sub,
      [this](VehicleStatus::ConstSharedPtr m) {
        status_ = *m;
        have_status_ = true;
        last_status_time_ = now();
        // 2026-10-10：飞控"已解锁→未解锁"单独报一次 —— 空中出现这个基本就是飞控重启/掉电/触发保护
        if (was_armed_ && m->arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
          RCLCPP_ERROR(get_logger(),
                       "⚠️ 飞控由「已解锁」变成「未解锁」（nav_state=%u）—— 多半是飞控重启/掉电/"
                       "触发保护。查 /tmp/microxrce_agent.log 里有没有 session re-established",
                       static_cast<unsigned>(m->nav_state));
        }
        was_armed_ = (m->arming_state == VehicleStatus::ARMING_STATE_ARMED);
      });
    lpos_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_sub,
      [this](VehicleLocalPosition::ConstSharedPtr m) {
        lpos_ = *m;
        have_lpos_ = true;
        last_lpos_time_ = now();
      });
    land_sub_ = create_subscription<VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos_sub,
      [this](VehicleLandDetected::ConstSharedPtr m) { land_ = *m; have_land_ = true; });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos_sub,
      [this](VehicleCommandAck::ConstSharedPtr m) { onAck(m); });
    // 2026-10-10：解锁被拒时用来"翻译原因"（offboard 信号 / 无任务 / 无 home / 无手动输入…）
    failsafe_sub_ = create_subscription<FailsafeFlags>(
      "/fmu/out/failsafe_flags", qos_sub,
      [this](FailsafeFlags::ConstSharedPtr m) { failsafe_ = *m; have_failsafe_ = true; });
    // 2026-10-10：两道交叉校验的来源 —— EV 桥发给飞控的 NED 位姿（/fmu/in/vehicle_visual_odometry）、
    // 定高器高度（/tofsense/height）、机体姿态（/fmu/out/vehicle_attitude）。
    // 注意用 EV 桥的**输出**而不是 /tracked_pose：后者是 SLAM 的 map 系，航向基准是任意的。
    odom_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", qos_sub,
      [this](VehicleOdometry::ConstSharedPtr m) {
        odom_ = *m;
        have_odom_ = true;
        last_odom_time_ = now();
      });
    tof_sub_ = create_subscription<Float32>(
      "/tofsense/height", qos_sub,
      [this](Float32::ConstSharedPtr m) {
        tof_m_ = static_cast<double>(m->data);
        have_tof_ = true;
        last_tof_time_ = now();
      });
    att_sub_ = create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_sub,
      [this](VehicleAttitude::ConstSharedPtr m) { att_ = *m; have_att_ = true; });

    const double period = 1.0 / std::max(2.0, control_freq_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      [this]() { tick(); });

    start_time_ = now();
    last_status_time_ = start_time_;   // 与 node clock 同时基，避免 Time 相减抛异常
    last_lpos_time_   = start_time_;
    last_odom_time_   = start_time_;   // 2026-10-10：交叉校验用到的几个时间戳也要同时基
    last_tof_time_    = start_time_;
    yaw_bad_since_    = start_time_;
    alt_bad_since_    = start_time_;
    phase_start_ = start_time_;

    RCLCPP_INFO(get_logger(),
                "起飞-降落任务节点已就绪：目标高度 %.2fm / 悬停 %.1fs / 控制模式 %s / dry_run=%s",
                target_alt_, hover_time_, control_mode_.c_str(), dry_run_ ? "on" : "off");
    RCLCPP_INFO(get_logger(), "%.1f 秒后开始执行；真机请先拆桨或用 SITL 验证！",
                start_delay_);
    if (mission_enable_) {
      RCLCPP_INFO(get_logger(),
                  "任务航线已启用：前进 %.2fm → 掉头 %.0f° → %s"
                  "（到位容差 %.2fm / %.0f°，每段超时 %.0fs，settle %.1fs）",
                  forward_dist_, turn_deg_,
                  return_home_ ? "飞回原位" : "就地悬停降落（不返航）",
                  pos_tol_, yaw_tol_deg_, leg_timeout_, settle_time_);
    }
    if (dry_run_) {
      RCLCPP_WARN(get_logger(), "dry_run 模式：只打印流程，不会解锁/切换模式/降落");
    }
  }

private:
  enum class Phase { WaitReady, Prestream, SetArmMode, Arming, Offboard, Takeoff,
                     MissionForward, MissionTurn, MissionReturn,
                     Hover, Descend, Handover, Done, Abort };

  // get_clock() 非 const 版本才能调用 now()（与 px4_bridge 的坑一致）
  uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  double phaseElapsed() const
  {
    return (now() - phase_start_).seconds();
  }

  void setPhase(Phase p)
  {
    if (p == phase_) {
      return;
    }
    phase_ = p;
    phase_start_ = now();
    settle_ok_ = false;      // 每进一段都重新开始"连续到位"计时

    const char *name = "?";
    switch (p) {
    case Phase::WaitReady: name = "WAIT_READY"; break;
    case Phase::Prestream: name = "PRESTREAM"; break;
    case Phase::SetArmMode: name = "SET_ARM_MODE"; break;
    case Phase::Arming:    name = "ARMING";    break;
    case Phase::Offboard:  name = "OFFBOARD";  break;
    case Phase::Takeoff:   name = "TAKEOFF";   break;
    case Phase::MissionForward: name = "MISSION_FORWARD"; break;
    case Phase::MissionTurn:    name = "MISSION_TURN";    break;
    case Phase::MissionReturn:  name = "MISSION_RETURN";  break;
    case Phase::Hover:     name = "HOVER";     break;
    case Phase::Descend:   name = "DESCEND";   break;
    case Phase::Handover:  name = "HANDOVER_TO_PX4_LAND"; break;
    case Phase::Done:      name = "DONE";      break;
    case Phase::Abort:     name = "ABORT";     break;
    }
    RCLCPP_INFO(get_logger(), "== 阶段 -> %s", name);

    std_msgs::msg::String s;
    s.data = name;
    state_pub_->publish(s);
  }

  // 相对起飞点的高度（米，向上为正）
  double altitudeAgl() const
  {
    if (!have_lpos_) {
      return 0.0;
    }
    if (lpos_.dist_bottom_valid) {
      return static_cast<double>(lpos_.dist_bottom);
    }
    return z_ref_ - static_cast<double>(lpos_.z);  // NED：z 向下为正
  }

  bool armed() const
  {
    return have_status_ && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  bool inOffboard() const
  {
    return have_status_ && status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool localPosValid() const
  {
    return have_lpos_ && lpos_.xy_valid && lpos_.z_valid &&
           lpos_.v_xy_valid && lpos_.v_z_valid;
  }

  bool landed() const
  {
    return have_land_ && land_.landed;
  }

  /// 飞控此刻是否允许解锁。
  /// ⚠️ 这不是"飞机健康度"——PX4 里它就是 `canArm(当前 nav_state)`（Commander.cpp:1834），
  /// **按模式分组判定**：停在 OFFBOARD 而没有 Offboard 流、停在 AUTO_MISSION 而没有任务…
  /// 都会是 false。2026-10-10 实测：OFFBOARD+无流 → false；切 POSCTL → true；切回 → false。
  bool armableNow() const
  {
    return have_status_ && status_.pre_flight_checks_pass;
  }

  /// 把"飞控为什么不让解锁"翻译成人话（数据来自 /fmu/out/failsafe_flags）。
  std::string armBlockReason() const
  {
    if (!have_failsafe_) {
      return "（还没收到 /fmu/out/failsafe_flags）";
    }
    const auto &f = failsafe_;
    std::vector<std::string> bad;
    if (f.offboard_control_signal_lost) bad.emplace_back("无 Offboard 信号（当前模式属于 Offboard 族）");
    if (f.local_position_invalid)       bad.emplace_back("本地位置无效");
    if (f.local_altitude_invalid)       bad.emplace_back("高度无效");
    if (f.attitude_invalid)             bad.emplace_back("姿态无效");
    if (f.angular_velocity_invalid)     bad.emplace_back("角速度无效");
    if (f.global_position_invalid)      bad.emplace_back("无全局位置（没 GPS）");
    if (f.home_position_invalid)        bad.emplace_back("无 home 点");
    if (f.auto_mission_missing)         bad.emplace_back("没有任务（AUTO_MISSION 模式会因此拒解锁）");
    if (f.manual_control_signal_lost)   bad.emplace_back("无手动控制输入");
    if (f.gcs_connection_lost)          bad.emplace_back("无地面站连接");
    if (f.battery_warning != 0)         bad.emplace_back("电池告警");
    if (f.battery_low_remaining_time)   bad.emplace_back("剩余飞行时间不足");
    if (f.battery_unhealthy)            bad.emplace_back("电池不健康");
    if (f.fd_critical_failure || f.fd_esc_arming_failure ||
        f.fd_motor_failure || f.fd_imbalanced_prop) bad.emplace_back("失效检测器报警");
    if (have_status_ && (f.mode_req_prevent_arming & (1u << status_.nav_state)) != 0) {
      bad.emplace_back("当前飞行模式本身不适合解锁（先切模式）");
    }
    if (bad.empty()) {
      return "（failsafe_flags 里没有明显异常；用 QGC 的 Ready-to-Fly 列表看）";
    }
    std::string s;
    for (size_t i = 0; i < bad.size(); ++i) {
      if (i > 0) { s += "；"; }
      s += bad[i];
    }
    return s;
  }

  // ---------- 2026-10-10 新增：交叉校验用的几个小工具 ----------

  /// 飞控 EKF 的高度（相对起飞参考点，米；用 z 而不是 dist_bottom，
  /// 因为 dist_bottom 在 dist_bottom_sensor_bitfield=NONE 时是"地形估计"）
  double ekfAltAboveRef() const
  {
    return have_lpos_ ? (z_ref_ - static_cast<double>(lpos_.z)) : 0.0;
  }

  /// 机体倾斜角（度）。PX4 四元数 q = [w,x,y,z]（机体系 FRD→NED），
  /// z 轴在 NED 下的 z 分量 = 1-2(x²+y²) → 倾斜 = acos(它)。
  double tiltDeg() const
  {
    if (!have_att_) {
      return 0.0;
    }
    const double x = static_cast<double>(att_.q[1]);
    const double y = static_cast<double>(att_.q[2]);
    const double c = std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0);
    return std::acos(c) * 180.0 / M_PI;
  }

  static double yawOfQuat(double w, double x, double y, double z)
  {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  /// EV 桥发往飞控的位姿是否新鲜（15Hz；这里放宽到 0.5s）
  bool evYawFresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() < 0.5;
  }

  /// EV 位姿的航向（rad，NED）。PX4 VehicleOdometry 的四元数同样是 [w,x,y,z]。
  double evYaw() const
  {
    return yawOfQuat(static_cast<double>(odom_.q[0]), static_cast<double>(odom_.q[1]),
                     static_cast<double>(odom_.q[2]), static_cast<double>(odom_.q[3]));
  }

  /// 两道"交叉校验"：不是控制，只是"两个独立来源对不上就喊停"。
  /// 每帧调用；只在"已解锁且在离地"时起作用。
  void runCrossChecks()
  {
    const bool flying = armed() && have_lpos_ && ekfAltAboveRef() > 0.2;

    // ---------- C. 航向：飞控 EKF heading vs EV 桥位姿 yaw（都换算到 NED 后比"变化量"）----------
    if (!flying || yaw_xcheck_deg_ <= 0.0 || !evYawFresh() || !have_yaw_ev_ref_) {
      yaw_bad_ = false;
    } else {
      const double fh = static_cast<double>(lpos_.heading);
      const double d = std::fabs(wrapPi(wrapPi(evYaw() - fh) - yaw_ev_offset_)) * 180.0 / M_PI;
      if (d > yaw_xcheck_deg_) {
        if (!yaw_bad_) {
          yaw_bad_ = true;
          yaw_bad_since_ = now();
        }
        const double held = (now() - yaw_bad_since_).seconds();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "航向对不上：飞控 %.1f° / EV %.1f°（相对基准偏差了 %.1f°，门限 %.0f°，已持续 %.1fs）",
                             fh * 180.0 / M_PI, evYaw() * 180.0 / M_PI, d, yaw_xcheck_deg_, held);
        if (held > xcheck_hold_) {
          RCLCPP_ERROR(get_logger(),
                       "航向交叉校验失败：EV 航向相对飞控跑偏 %.1f° 持续 %.1fs（门限 %.0f°/%.1fs）"
                       "—— 飞控可能正被错误的航向估计带跑，立即中止降落",
                       d, held, yaw_xcheck_deg_, xcheck_hold_);
          abortMission("航向交叉校验失败（EV 航向与飞控 heading 持续对不上）");
          return;
        }
      } else {
        yaw_bad_ = false;
      }
    }

    // ---------- D. 高度：飞控 z vs 定高器 TOF（比"变化量"，消掉固定偏差）----------
    const bool alt_src_ok = have_tof_ && have_tof_ref_ && have_att_ &&
                            (now() - last_tof_time_).seconds() < 0.5 &&
                            tiltDeg() < tilt_gate_deg_;
    if (!flying || alt_xcheck_m_ <= 0.0 || !alt_src_ok) {
      // 倾斜大 / TOF 不新鲜时不判（倾斜会让 TOF 斜射、读数变长，属已知伪值）
      alt_bad_ = false;
    } else {
      const double d = std::fabs((tof_m_ - ekfAltAboveRef()) - tof_ekf_offset_);
      if (d > alt_xcheck_m_) {
        if (!alt_bad_) {
          alt_bad_ = true;
          alt_bad_since_ = now();
        }
        const double held = (now() - alt_bad_since_).seconds();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "高度对不上：定高器 %.2fm / 飞控 %.2fm（差 %.2fm，地面基准 %.2fm，"
                             "倾斜 %.0f°，已持续 %.1fs）",
                             tof_m_, ekfAltAboveRef(), d, tof_ekf_offset_, tiltDeg(), held);
        if (alt_xcheck_abort_m_ > 0.0 && d > alt_xcheck_abort_m_ && held > xcheck_hold_) {
          RCLCPP_ERROR(get_logger(),
                       "高度交叉校验失败：定高器与飞控高度差 %.2fm 持续 %.1fs（中止门限 %.2fm）"
                       "—— 高度回路里的某个来源跑偏了，立即中止降落", d, held, alt_xcheck_abort_m_);
          abortMission("高度交叉校验失败（定高器与飞控的高度差过大）");
          return;
        }
      } else {
        alt_bad_ = false;
      }
    }
  }

  void sendVehicleCommand(uint32_t cmd, float p1 = 0.f, float p2 = 0.f, float p3 = 0.f,
                          float p4 = 0.f, double p5 = 0.0, double p6 = 0.0, float p7 = 0.f)
  {
    if (dry_run_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[dry_run] 本应发送命令 %u (p1=%.2f p2=%.2f)", cmd, p1, p2);
      return;
    }
    VehicleCommand m{};
    m.timestamp = now_us();
    m.command = cmd;
    m.param1 = p1; m.param2 = p2; m.param3 = p3; m.param4 = p4;
    m.param5 = p5; m.param6 = p6; m.param7 = p7;
    m.target_system = 1;
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    m.confirmation = 0;
    m.from_external = true;
    command_pub_->publish(m);
  }

  void onAck(const VehicleCommandAck::ConstSharedPtr msg)
  {
    if (msg->command == kCmdArmDisarm || msg->command == kCmdDoSetMode ||
        msg->command == kCmdNavLand) {
      RCLCPP_INFO(get_logger(), "命令 %u 回执：result=%u", msg->command, msg->result);
    }
  }

  // 记录参考点（起飞点 + 机头朝向），并按当前机头方向重算任务航点。
  //
  // 2026-10-10：改成可以在"真正起飞前"再抓一次。
  //   原来只在 WaitReady 抓，到真正离地中间还隔着 Prestream(2s) + 解锁 + 切 Offboard，
  //   好几秒。这几秒里 SLAM 只要漂几厘米，飞机就会去"回到几秒前的旧点"，
  //   表现就是**一起飞就朝某个方向蹿一下**。现在进 Offboard 后再抓一次。
  void captureReference(const char *tag)
  {
    x_ref_ = static_cast<double>(lpos_.x);
    y_ref_ = static_cast<double>(lpos_.y);
    z_ref_ = static_cast<double>(lpos_.z);
    yaw_ref_ = static_cast<double>(lpos_.heading);
    if (mission_enable_) {
      wp1_x_ = x_ref_ + forward_dist_ * std::cos(yaw_ref_);
      wp1_y_ = y_ref_ + forward_dist_ * std::sin(yaw_ref_);
      yaw_wp2_ = wrapPi(yaw_ref_ + turn_deg_ * M_PI / 180.0);
      yaw_cmd_ = yaw_ref_;   // 分段缓转从当前航向起步（turn_step_deg=0 时不影响）
      // 2026-10-10：地面记账 —— 记下"定高器 TOF 与飞控 z 的固定差"。高度交叉校验比的是
      // **变化量**，这样就不用纠结两者本来差多少（实测两者常差 0.2~0.3m）。
      if (have_tof_ && altitudeAgl() < 0.2) {
        tof_ekf_offset_ = tof_m_ - ekfAltAboveRef();
        have_tof_ref_ = true;
        RCLCPP_INFO(get_logger(), "[%s] 高度基准：定高器 %.2fm − 飞控 %.2fm = %.2fm",
                    tag, tof_m_, ekfAltAboveRef(), tof_ekf_offset_);
      }
      // 航向基准：记下"EV 位姿 yaw − 飞控 heading"这个常量差（map 系与 NED 的对齐关系），
      // 之后只比它的**变化量** —— 这样坐标系常量差、以及"起飞时航向本身是 90°"都不影响判断。
      if (evYawFresh()) {
        yaw_ev_offset_ = wrapPi(evYaw() - yaw_ref_);
        have_yaw_ev_ref_ = true;
        RCLCPP_INFO(get_logger(), "[%s] 航向基准：EV %.1f° − 飞控 %.1f° = %.1f°",
                    tag, evYaw() * 180.0 / M_PI, yaw_ref_ * 180.0 / M_PI,
                    yaw_ev_offset_ * 180.0 / M_PI);
      }
    }
    RCLCPP_INFO(get_logger(), "[%s] 参考点 x=%.2f y=%.2f z=%.2f 航向=%.1f°",
                tag, x_ref_, y_ref_, z_ref_, yaw_ref_ * 180.0 / M_PI);
  }

  // 发布 Offboard 控制模式 + 设定值（必须持续发）
  // z_override：显式指定 NED 的 z 目标（不给时按 climb/descend 推断）
  void publishOffboard(bool climb, bool descend, double z_override = std::nan(""))
  {
    OffboardControlMode ocm{};
    ocm.timestamp = now_us();
    ocm.position = (control_mode_ == "position");
    ocm.velocity = (control_mode_ == "velocity");
    offboard_pub_->publish(ocm);

    const float nan32 = std::nanf("");
    TrajectorySetpoint sp{};
    sp.timestamp = now_us();

    if (control_mode_ == "position") {
      double z_target = z_ref_;
      if (!std::isnan(z_override)) {
        z_target = z_override;
      } else if (climb) {
        z_target = z_ref_ - target_alt_;
      } else if (descend) {
        z_target = z_ref_ - land_switch_alt_;
      }
      sp.position = {static_cast<float>(x_ref_), static_cast<float>(y_ref_),
                     static_cast<float>(z_target)};
      sp.velocity = {nan32, nan32, nan32};
      sp.acceleration = {nan32, nan32, nan32};
    } else {
      double vz = 0.0;
      if (climb) {
        vz = -climb_vel_;     // NED：负 = 上升
      } else if (descend) {
        vz = descend_vel_;    // NED：正 = 下降
      }
      sp.position = {nan32, nan32, nan32};
      sp.velocity = {0.f, 0.f, static_cast<float>(vz)};   // 原地：水平速度 0
      sp.acceleration = {nan32, nan32, nan32};
    }
    sp.jerk = {nan32, nan32, nan32};
    sp.yaw = nan32;       // 不指定绝对航向
    sp.yawspeed = 0.0f;   // 航向角速度 0 -> 保持当前机头朝向
    setpoint_pub_->publish(sp);
  }

  // ---- 2026-10-09：任务段用的「位置 + 绝对航向」设定值 ----
  // PX4 对 yaw/yawspeed 是"二选一"：给绝对航向时把 yawspeed 置 NaN
  // （FlightTaskOffboard 里先看 yaw 是否有限）。位置单位 m、NED；航向 rad、0=北、正=向东。
  void publishMissionSetpoint(double x, double y, double z, double yaw)
  {
    OffboardControlMode ocm{};
    ocm.timestamp = now_us();
    ocm.position = true;
    ocm.velocity = false;
    offboard_pub_->publish(ocm);

    const float nan32 = std::nanf("");
    TrajectorySetpoint sp{};
    sp.timestamp = now_us();
    sp.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    sp.velocity = {nan32, nan32, nan32};
    sp.acceleration = {nan32, nan32, nan32};
    sp.jerk = {nan32, nan32, nan32};
    sp.yaw = static_cast<float>(yaw);
    sp.yawspeed = nan32;
    setpoint_pub_->publish(sp);
  }

  double posErr(double x, double y) const
  {
    return std::hypot(x - static_cast<double>(lpos_.x), y - static_cast<double>(lpos_.y));
  }

  /// 位置到位判定：误差**持续** < pos_tol_ 达 settle_time_ 才算到位（避免"路过"就算到）
  bool posSettled(double x, double y)
  {
    if (posErr(x, y) < pos_tol_) {
      if (!settle_ok_) { settle_ok_ = true; settle_since_ = now(); }
      return (now() - settle_since_).seconds() >= settle_time_;
    }
    settle_ok_ = false;
    return false;
  }

  /// 航向到位判定（同样要连续 settle_time_）
  bool yawSettled(double yaw_target)
  {
    const double err = std::fabs(wrapPi(yaw_target - static_cast<double>(lpos_.heading)));
    if (err < yaw_tol_deg_ * M_PI / 180.0) {
      if (!settle_ok_) { settle_ok_ = true; settle_since_ = now(); }
      return (now() - settle_since_).seconds() >= settle_time_;
    }
    settle_ok_ = false;
    return false;
  }

  void abortMission(const std::string &why)
  {
    // 2026-10-09 晚：中止时把现场状态一起打出来。上次炸机只看到「离开 Offboard 或掉解锁」，
    // 分不清是掉解锁还是掉模式、链路断没断，只能去翻别的日志。
    const double age = have_status_ ? (now() - last_status_time_).seconds() : -1.0;
    RCLCPP_ERROR(get_logger(),
                 "任务中止：%s（armed=%d nav_state=%u inOffboard=%d，vehicle_status 已 %.2fs 未更新）",
                 why.c_str(), armed() ? 1 : 0,
                 have_status_ ? static_cast<unsigned>(status_.nav_state) : 999u,
                 inOffboard() ? 1 : 0, age);
    setPhase(Phase::Abort);
    // 交还 PX4：自动降落（比在空中直接 disarm 安全）
    sendVehicleCommand(kCmdNavLand);
  }

  void tick()
  {
    const double since_start = (now() - start_time_).seconds();

    // 2026-10-09 晚 事故修复：飞控链路看门狗。
    // 上次炸机：飞行中 uXRCE-DDS 会话断开（Agent 日志 "session re-established"），
    // PX4 因 COM_OF_LOSS_T(1s) 收不到 offboard 流而掉出 Offboard。
    //
    // ⚠️ 2026-10-09 22:0x 修正：看门狗原来盯 /fmu/out/vehicle_status，但那个话题
    //    **只有 1.3 Hz**（实测平均间隔 0.77s）→ 0.5s 超时一多半时间都会误判！
    //    后果：21:49 和 21:50 两次任务都在**返航段中途**被自己的看门狗掐断 →
    //    NAV_LAND 就地落下（用户看到"只返回了 60cm"），而 Agent 日志里根本没有
    //    会话重建、中止那一刻日志还写着 inOffboard=1 / nav_state=14。
    //    → 改成盯 **/fmu/out/vehicle_local_position（实测 95 Hz）**：0.5s 收不到
    //      才真的是链路断了（相当于连丢 47 帧）。
    // 2026-10-10（第二次炸机复盘）：这次"链路中断"的真凶不是 DDS，而是**飞控自己重启了**——
    //   证据：agent 日志 17:52:48 `session re-established`、飞控 hrt 反推开机时刻 = 17:52:50、
    //   latest_arming_reason 被清零、串口 fe/brk 一个都没涨（链路干净）。日志里要把这一层写出来，
    //   别再让人以为只是网络抖动。
    const bool link_watch_active = (phase_ != Phase::WaitReady && phase_ != Phase::Abort &&
                                    phase_ != Phase::Done && have_lpos_);
    const double lpos_age = have_lpos_ ? (now() - last_lpos_time_).seconds() : 0.0;

    if (link_watch_active && lpos_age > link_timeout_) {
      abortMission("飞控链路中断（/fmu/out/vehicle_local_position 超时未更新；"
                   "飞控也可能刚重启过——去 agent 日志找 session re-established）");
      return;
    }
    if (link_watch_active && lpos_age > 0.5 && !link_hickup_logged_) {
      RCLCPP_WARN(get_logger(),
                  "[提示] /fmu/out/vehicle_local_position 停了 %.2fs（未到 %.1fs 阈值，继续观察）",
                  lpos_age, link_timeout_);
      link_hickup_logged_ = true;
    }
    if (have_lpos_ && lpos_age < 0.2) {
      link_hickup_logged_ = false;
    }

    // 2026-10-10：两道交叉校验（航向 / 高度）。放在状态机之前 —— 一旦对不上就立刻中止，
    // 不再往状态机里灌新的设定值。
    if (phase_ != Phase::WaitReady && phase_ != Phase::Abort && phase_ != Phase::Done) {
      runCrossChecks();
      if (phase_ == Phase::Abort) {
        return;
      }
    }

    switch (phase_) {
      // ---------- 0. 等条件就绪 ----------
      case Phase::WaitReady:
        if (since_start < start_delay_) {
          return;
        }
        if (!have_status_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "还没收到 /fmu/out/vehicle_status —— Micro-XRCE-DDS-Agent 连上了吗？");
          return;
        }
        if (armed()) {
          abortMission("启动时飞机已经是解锁状态");
          return;
        }
        if (!localPosValid()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "本地位置/速度估计无效（xy/z/v_xy/v_z）—— Offboard 会被飞控拒绝；"
                               "室内请确认光流+测距（或 VIO），室外等 GPS 定位");
          return;
        }
        // 记录起飞点参考（本地 NED 原点不一定在起飞点，所以按当前值取参考）
        captureReference("起飞前");
        if (!std::isfinite(yaw_ref_)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "飞控还没给出有效航向 heading（NaN）—— 任务段的「前进方向」要靠它，等一下");
          return;
        }

        if (mission_enable_) {
          // 「前进」= **起飞时刻机头方向**。NED 下 x 北、y 东，航向 0=北、正=向东（顺时针），
          // 所以机头单位向量就是 (cos(yaw), sin(yaw))。
          wp1_x_ = x_ref_ + forward_dist_ * std::cos(yaw_ref_);
          wp1_y_ = y_ref_ + forward_dist_ * std::sin(yaw_ref_);
          yaw_wp2_ = wrapPi(yaw_ref_ + turn_deg_ * M_PI / 180.0);
          RCLCPP_INFO(get_logger(),
                      "任务航线（NED）：起飞 → z=%.2f | 前进 %.2fm → (%.2f, %.2f) 航向 %.1f° | "
                      "%s | 悬停 %.1fs 后降落",
                      z_ref_ - target_alt_, forward_dist_, wp1_x_, wp1_y_,
                      yaw_ref_ * 180.0 / M_PI,
                      (turn_deg_ == 0.0 && !return_home_)
                          ? "不掉头、不返航（就地降落）"
                          : (return_home_ ? "掉头后飞回起飞点正上方"
                                          : "掉头后就地悬停降落"),
                      hover_time_);
          if (turn_deg_ != 0.0) {
            RCLCPP_INFO(get_logger(), "掉头规划：目标航向 %.1f°（转 %.0f°），分段缓转步长 %.0f°%s",
                        yaw_wp2_ * 180.0 / M_PI, turn_deg_, turn_step_deg_,
                        return_home_ ? "，之后飞回起飞点" : "，之后就地降落");
          }
        }
        setPhase(Phase::Prestream);
        return;

      // ---------- 1. 先持续发 Offboard 流（切模式前必须有信号） ----------
      case Phase::Prestream:
        publishOffboard(false, false);
        if (phaseElapsed() >= prestream_sec_) {
          if (dry_run_) {
            // 2026-10-10：原来干跑也会进 ARMING，等 8 秒后报"解锁超时"——干跑本来就不解锁，
            // 这条 ERROR 纯属误导（用户就是被它吓到）。
            RCLCPP_WARN(get_logger(),
                        "干跑结束：Offboard 流已连发 %.1fs。真跑时接着会：切模式 → 解锁 → 切 Offboard → "
                        "起飞到 %.2fm → 任务段 → 悬停 %.1fs → 交还 PX4 降落。"
                        "干跑不解锁，所以这里不再报「解锁超时」。",
                        prestream_sec_, target_alt_, hover_time_);
            setPhase(Phase::Done);
            return;
          }
          setPhase(Phase::SetArmMode);
        }
        return;

      // ---------- 2. 解锁前把"当前模式"弄成可解锁的 ----------
      // 2026-10-10 新增。实测确认：PX4 的 pre_flight_checks_pass 就是 canArm(当前 nav_state)
      //   （Commander.cpp:1834），按模式分组判定。上次任务中止后飞控停在 OFFBOARD，
      //   那一刻没有 Offboard 流 → modeCheck.cpp 的 mode_req_offboard_signal 判死 →
      //   解锁被拒（命令 400 result=1）→ 任务报"解锁超时"。停在 AUTO_LAND / AUTO_MISSION 同理。
      //   实测三连：OFFBOARD+无流 = false；切 POSCTL = true；切回 OFFBOARD = false。
      // 对策：先等飞控自己说"可以解锁"（OFFBOARD 族有我们正在发的流就已经满足）；
      //   不行就明确切一次 POSCTL（这一族只要本地位置有效、手动控制不丢就能解锁）再进 ARMING。
      case Phase::SetArmMode: {
        publishOffboard(false, false);   // 兼顾 OFFBOARD 族：让"需要 Offboard 信号"这一条满足

        if (armableNow()) {
          if (!armable_logged_) {
            armable_logged_ = true;
            RCLCPP_INFO(get_logger(), "飞控已允许解锁（当前 nav_state=%u）",
                        static_cast<unsigned>(status_.nav_state));
          }
          setPhase(Phase::Arming);
          return;
        }

        if (phaseElapsed() > 0.3 && setmode_tries_ < 10) {
          if (setmode_tries_ == 0) {
            RCLCPP_WARN(get_logger(), "飞控现在不让解锁（nav_state=%u）：%s → 先切 POSCTL 再解锁",
                        static_cast<unsigned>(status_.nav_state), armBlockReason().c_str());
          }
          sendVehicleCommand(kCmdDoSetMode, 1.0f, kMainModePosctl);
          setmode_tries_++;
        }

        if (phaseElapsed() > 4.0) {
          RCLCPP_WARN(get_logger(),
                      "切 POSCTL 没让它变成可解锁（nav_state=%u，试了 %d 次）→ 照样进 ARMING；"
                      "若还是被拒，原因：%s",
                      static_cast<unsigned>(status_.nav_state), setmode_tries_,
                      armBlockReason().c_str());
          setPhase(Phase::Arming);
        }
        return;
      }

      // ---------- 3. 解锁 ----------
      case Phase::Arming: {
        publishOffboard(false, false);
        if (phaseElapsed() > 0.2 && !arm_sent_) {
          sendVehicleCommand(kCmdArmDisarm, 1.0f, 0.0f);  // param1=1 解锁
          arm_sent_ = true;
          RCLCPP_WARN(get_logger(), "已发送解锁命令");
        }
        if (armed()) {
          RCLCPP_INFO(get_logger(), "已解锁");
          setPhase(Phase::Offboard);
          return;
        }
        if (phaseElapsed() > arm_timeout_) {
          if (!armableNow()) {
            RCLCPP_ERROR(get_logger(),
                         "解锁被飞控拒绝：pre_flight_checks_pass=false（nav_state=%u）。"
                         "飞控给的理由（按模式分组判定）：%s",
                         static_cast<unsigned>(status_.nav_state), armBlockReason().c_str());
          }
          abortMission("解锁超时（飞控 pre-arm 检查没过，原因见上一条 ERROR；"
                       "也看看是不是别人已经解锁/遥控器在手里）");
        }
        return;
      }

      // ---------- 4. 切 Offboard ----------
      case Phase::Offboard:
        publishOffboard(false, false);
        if (!armed()) {
          abortMission("等待 Offboard 期间掉了解锁");
          return;
        }
        if (inOffboard()) {
          RCLCPP_INFO(get_logger(), "已进入 Offboard");
          // 2026-10-10：真正离地前**再抓一次参考点**，消掉"目标点 vs 实际离地点"
          // 之间那几秒的 SLAM 漂移（这是起飞小蹿的主要来源之一）。
          captureReference("进入Offboard后");
          setPhase(Phase::Takeoff);
          return;
        }
        // 反复发几次（PX4 收到一次即可，重复发更稳）
        if (phaseElapsed() > 0.5 && offboard_cmd_tries_ < 10) {
          sendVehicleCommand(kCmdDoSetMode, 1.0f, kMainModeOffboard);
          offboard_cmd_tries_++;
        }
        if (phaseElapsed() > offboard_timeout_) {
          abortMission("切 Offboard 超时（多半是本地位置/速度估计无效被拒）");
        }
        return;

      // ---------- 5. 起飞（Offboard 内置起飞 ramp） ----------
      case Phase::Takeoff: {
        // 2026-10-10：改成按 climb_vel **线性爬升**，而不是一步跳到 target_alt。
        // 一步跳会让 PX4 用接近最大爬升率猛起，配合地面效应就是"一起飞就蹿一下/倾一下"。
        const double climbed = std::min(target_alt_, std::max(0.0, climb_vel_ * phaseElapsed()));
        publishOffboard(false, false, z_ref_ - climbed);
        const double alt = altitudeAgl();
        if (max_alt_ > 0.0 && alt > target_alt_ + 1.0) {
          abortMission("超过高度硬上限保护");
          return;
        }
        if (!armed() || !inOffboard()) {
          abortMission("起飞过程中离开 Offboard 或掉解锁");
          return;
        }
        if (alt >= target_alt_ - alt_tol_) {
          RCLCPP_INFO(get_logger(), "到达目标高度 %.2fm", alt);
          if (mission_enable_) {
            RCLCPP_INFO(get_logger(), "任务开始：沿起飞航向 %.1f° 前进 %.2fm → 目标点 (%.2f, %.2f)",
                        yaw_ref_ * 180.0 / M_PI, forward_dist_, wp1_x_, wp1_y_);
            setPhase(Phase::MissionForward);
          } else {
            setPhase(Phase::Hover);
          }
          return;
        }
        if (phaseElapsed() > takeoff_timeout_) {
          abortMission("起飞超时");
        }
        return;
      }

      // ---------- 6. 任务段 1：前进 forward_dist（沿起飞时刻机头方向） ----------
      case Phase::MissionForward:
        publishMissionSetpoint(wp1_x_, wp1_y_, z_ref_ - target_alt_, yaw_ref_);
        if (!armed() || !inOffboard()) {
          abortMission("任务(前进)过程中离开 Offboard 或掉解锁");
          return;
        }
        if (max_alt_ > 0.0 && altitudeAgl() > target_alt_ + 1.0) {
          abortMission("超过高度硬上限保护");
          return;
        }
        if (posSettled(wp1_x_, wp1_y_)) {
          RCLCPP_INFO(get_logger(), "前进段到位：水平误差 %.3fm（目标 %.2f, %.2f）",
                      posErr(wp1_x_, wp1_y_), wp1_x_, wp1_y_);
          // 2026-10-10：turn_deg=0 且不返航 → 没有掉头这一步，直接进悬停（少一次航向指令）
          if (turn_deg_ == 0.0 && !return_home_) {
            RCLCPP_INFO(get_logger(), "任务段结束（不掉头、不返航）→ 悬停 %.1fs 后降落", hover_time_);
            setPhase(Phase::Hover);
          } else {
            setPhase(Phase::MissionTurn);
          }
          return;
        }
        if (phaseElapsed() > leg_timeout_) {
          abortMission("前进段超时（没走到目标点，先查定位是否漂移）");
        }
        return;

      // ---------- 7. 任务段 2：原地掉头 turn_deg（2026-10-10：改成可分段缓转）----------
      case Phase::MissionTurn:
        // 分段缓转：每次只把"该发的航向设定值"往前推 turn_step_deg_，等机体大致跟上再推下一段。
        // 目的：避免一次性 180° 让四个电机瞬间吃满差速（10-10 炸机就发生在"开始掉头"那一瞬）。
        // turn_step_deg_ = 0 时退化成老行为（一次给到最终航向）。
        if (turn_step_deg_ > 0.0) {
          const double remain = wrapPi(yaw_wp2_ - yaw_cmd_);
          const double remain_deg = std::fabs(remain) * 180.0 / M_PI;
          if (remain_deg > 1e-6 && yawSettled(yaw_cmd_)) {
            const double step = std::min(std::fabs(remain), turn_step_deg_ * M_PI / 180.0);
            yaw_cmd_ = wrapPi(yaw_cmd_ + (remain > 0.0 ? step : -step));
            RCLCPP_INFO(get_logger(), "掉头分段：目标航向推到 %.1f°（还剩 %.1f° 到 %.1f°）",
                        yaw_cmd_ * 180.0 / M_PI,
                        std::fabs(wrapPi(yaw_wp2_ - yaw_cmd_)) * 180.0 / M_PI,
                        yaw_wp2_ * 180.0 / M_PI);
          }
        } else {
          yaw_cmd_ = yaw_wp2_;
        }
        publishMissionSetpoint(wp1_x_, wp1_y_, z_ref_ - target_alt_, yaw_cmd_);
        if (!armed() || !inOffboard()) {
          abortMission("任务(掉头)过程中离开 Offboard 或掉解锁");
          return;
        }
        if (max_alt_ > 0.0 && altitudeAgl() > target_alt_ + 1.0) {
          abortMission("超过高度硬上限保护");
          return;
        }
        if (std::fabs(wrapPi(yaw_wp2_ - yaw_cmd_)) < 1e-6 && yawSettled(yaw_wp2_)) {
          RCLCPP_INFO(get_logger(), "掉头完成：航向 %.1f°（目标 %.1f°）",
                      static_cast<double>(lpos_.heading) * 180.0 / M_PI,
                      yaw_wp2_ * 180.0 / M_PI);
          // 2026-10-10：默认不返航 —— 掉头完就在原地悬停降落（return_home:=true 才飞回起飞点）
          if (return_home_) {
            setPhase(Phase::MissionReturn);
          } else {
            RCLCPP_INFO(get_logger(), "return_home=false：不返航，就地悬停 %.1fs 后降落", hover_time_);
            setPhase(Phase::Hover);
          }
          return;
        }
        if (phaseElapsed() > leg_timeout_) {
          abortMission("掉头段超时");
        }
        return;

      // ---------- 8. 任务段 3：保持新航向飞回起飞点正上方 ----------
      case Phase::MissionReturn:
        publishMissionSetpoint(x_ref_, y_ref_, z_ref_ - target_alt_, yaw_wp2_);
        if (!armed() || !inOffboard()) {
          abortMission("任务(返航)过程中离开 Offboard 或掉解锁");
          return;
        }
        if (max_alt_ > 0.0 && altitudeAgl() > target_alt_ + 1.0) {
          abortMission("超过高度硬上限保护");
          return;
        }
        if (posSettled(x_ref_, y_ref_)) {
          RCLCPP_INFO(get_logger(), "已回到起飞点正上方：水平误差 %.3fm", posErr(x_ref_, y_ref_));
          RCLCPP_INFO(get_logger(), "任务完成 → 悬停 %.1fs 后降落", hover_time_);
          setPhase(Phase::Hover);
          return;
        }
        if (phaseElapsed() > leg_timeout_) {
          abortMission("返航段超时");
        }
        return;

      // ---------- 9. 原地悬停 ----------
      case Phase::Hover:
        // 2026-10-10 修 bug：原来调的 publishOffboard(false,false) 算出来 z_target = z_ref_，
        // 而 z_ref_ 是**起飞前的地面高度** —— 等于命令飞机降回地面，"悬停"根本没悬停。
        publishOffboard(false, false, z_ref_ - target_alt_);
        if (!armed() || !inOffboard()) {
          abortMission("悬停过程中离开 Offboard 或掉解锁");
          return;
        }
        if (phaseElapsed() >= hover_time_) {
          RCLCPP_INFO(get_logger(), "悬停结束，开始下降");
          setPhase(Phase::Descend);
        }
        return;

      // ---------- 10. 下降 ----------
      case Phase::Descend: {
        publishOffboard(false, /*descend=*/true);
        if (!armed() || !inOffboard()) {
          abortMission("下降过程中离开 Offboard 或掉解锁");
          return;
        }
        const double alt = altitudeAgl();
        if (alt <= land_switch_alt_) {
          RCLCPP_INFO(get_logger(), "已降到 %.2fm，交还 PX4 自动降落", alt);
          setPhase(Phase::Handover);
          return;
        }
        if (phaseElapsed() > descend_timeout_) {
          abortMission("下降超时");
        }
        return;
      }

      // ---------- 11. 交还 PX4 降落 ----------
      case Phase::Handover:
        // 切换期间飞控可能还在 Offboard，继续发流直到它真的离开，
        // 免得因 COM_OF_LOSS_T 先触发一次多余的 Offboard 失效保护
        if (inOffboard()) {
          publishOffboard(false, /*descend=*/true);
        }
        if (!land_sent_) {
          sendVehicleCommand(kCmdNavLand);   // VEHICLE_CMD_NAV_LAND
          land_sent_ = true;
          RCLCPP_WARN(get_logger(),
                      "已发送 NAV_LAND，等待落地上锁（COM_DISARM_LAND 默认 2s）");
        }
        if (landed() && !landed_logged_) {
          landed_logged_ = true;
          RCLCPP_INFO(get_logger(), "落地检测：landed");
        }
        if (!armed()) {
          RCLCPP_INFO(get_logger(), "已落地并自动上锁，任务完成");
          setPhase(Phase::Done);
          return;
        }
        if (phaseElapsed() > land_timeout_) {
          RCLCPP_ERROR(get_logger(), "降落超时，请人工接管（可发 disarm 或 QGC 手动降落）");
          setPhase(Phase::Abort);
        }
        return;

      case Phase::Done:
        return;

      case Phase::Abort:
        // 中止后不再控制，交给 PX4 的 NAV_LAND / 失控保护
        return;
    }
  }

  // ---- 参数 ----
  double target_alt_{2.0}, hover_time_{5.0}, climb_vel_{0.5}, descend_vel_{0.5};
  double land_switch_alt_{0.3}, alt_tol_{0.15}, control_freq_{10.0};
  double prestream_sec_{2.0}, start_delay_{3.0};
  double arm_timeout_{8.0}, offboard_timeout_{8.0}, takeoff_timeout_{30.0};
  double descend_timeout_{45.0}, land_timeout_{60.0}, max_alt_{0.0};
  std::string control_mode_{"position"};
  bool dry_run_{false};
  // 任务航线（2026-10-09 新增）
  bool mission_enable_{false};
  double forward_dist_{0.5}, turn_deg_{180.0}, pos_tol_{0.15}, yaw_tol_deg_{10.0};
  double settle_time_{0.5}, leg_timeout_{20.0};
  double link_timeout_{1.0};
  // ---- 2026-10-10 深夜第二轮 ----
  bool   return_home_{false};          // false = 前进到位就悬停降落（不掉头、不返航）
  double turn_step_deg_{30.0};         // 掉头分段步长（度）；0 = 一次转到位
  double yaw_xcheck_deg_{20.0};        // 航向交叉校验门限（度）；0 = 关闭
  double alt_xcheck_m_{0.5};           // 高度交叉校验门限（m，比变化量）；0 = 关闭
  double alt_xcheck_abort_m_{0.9};     // 高度差超过它且持续 xcheck_hold_ 就中止
  double xcheck_hold_{1.0};            // s，两道校验都要连续超限这么久才动作
  double tilt_gate_deg_{25.0};         // deg，倾斜超过它就不做高度校验（TOF 斜射会变长）

  // ---- 状态 ----
  Phase phase_{Phase::WaitReady};
  rclcpp::Time start_time_, phase_start_;
  bool have_status_{false}, have_lpos_{false}, have_land_{false}, have_failsafe_{false};
  rclcpp::Time last_status_time_;
  rclcpp::Time last_lpos_time_;       // 看门狗用这个（vehicle_local_position 是高频话题）
  VehicleStatus status_{};
  VehicleLocalPosition lpos_{};
  VehicleLandDetected land_{};
  FailsafeFlags failsafe_{};          // 解锁被拒时用来解释原因
  double x_ref_{0.0}, y_ref_{0.0}, z_ref_{0.0}, yaw_ref_{0.0};
  double wp1_x_{0.0}, wp1_y_{0.0}, yaw_wp2_{0.0};   // 任务段目标点 / 目标航向（NED）
  double yaw_cmd_{0.0};                            // 分段缓转时"当前该发的航向设定值"
  double tof_ekf_offset_{0.0};                     // 地面记下的 (TOF - 飞控z)，高度校验的基准
  bool   have_tof_ref_{false};
  double tof_m_{0.0};
  bool   have_tof_{false};
  rclcpp::Time last_tof_time_;
  VehicleOdometry odom_{};                         // EV 桥发给飞控的 NED 位姿
  bool   have_odom_{false};
  rclcpp::Time last_odom_time_;
  double yaw_ev_offset_{0.0};                      // 地面记下的 (EV yaw − 飞控 heading)
  bool   have_yaw_ev_ref_{false};
  VehicleAttitude att_{};
  bool   have_att_{false};
  bool   was_armed_{false};
  bool   yaw_bad_{false}, alt_bad_{false};
  rclcpp::Time yaw_bad_since_, alt_bad_since_;
  bool settle_ok_{false};                          // "连续到位"计时状态
  rclcpp::Time settle_since_;
  bool arm_sent_{false}, land_sent_{false}, landed_logged_{false};
  bool armable_logged_{false};        // "飞控已允许解锁"只打一次
  bool link_hickup_logged_{false};    // 链路短暂停顿只提示一次（恢复后自动复位）
  int offboard_cmd_tries_{0};
  int setmode_tries_{0};              // SetArmMode 阶段切 POSCTL 的重试次数

  // ---- ROS 接口 ----
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Float32>::SharedPtr tof_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr att_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffLand>());
  rclcpp::shutdown();
  return 0;
}
