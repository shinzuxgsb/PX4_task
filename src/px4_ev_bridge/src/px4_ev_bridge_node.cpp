// =====================================================================
//  px4_ev_bridge_node.cpp   (v0.2 —— 水平位姿 + 垂直高度 合成 EV)
//  雷达 2D SLAM + TOFSense-M 定高器  ->  PX4 外部视觉里程计
//
//  数据流：
//      LDS-50C-E-R -> bluesea2 -> cartographer -> /tracked_pose
//                                                      |
//      TOFSense-M -> tofsense_m  -> /tofsense/height   |
//                                                      v
//                       本节点 -> /fmu/in/vehicle_visual_odometry -> EKF2
//
//  分工：
//      水平位置 x,y / 水平速度 / 航向  <- 雷达 SLAM（cartographer）
//      垂直位置 z                      <- 定高器 TOFSense-M
//
//  速度的来龙去脉（2026-10-09）
//      —— 白天：一度**取消**上报速度 ——
//      上游 cartographer 只给位姿、不给速度。以前这里用 Δpos/Δt + 一阶低通**造**了一个
//      速度发出去；但位置差分和位置是同一份信息（统计上不独立），还会把量化噪声放大
//      （Δpos/Δt 对高频的增益 ~2/dt），EKF2 却把它当成一次独立的"速度测量"融合 ——
//      有把 EKF 带偏的风险。
//      —— 晚上：一度按用户要求**恢复上报**，但加了三道约束 ——
//      1) 参数 report_velocity 控制；**2026-10-10 起默认 false（不上报）** ——
//         因为随后出现了"朝固定方向偏 + 越飞越倾"，与"假速度拖 EKF"的机理吻合，
//         先按最保守的配置来。想吃速度就显式 report_velocity:=true 再试。
//      2) 速度方差用保守值（launch 里 velocity_sigma 默认 1.0 m/s），EKF 不会过度自信；
//      3) 飞控侧还能用 EKF2_EV_CTRL 的 bit2(=速度) 决定要不要融合 —— 两边都可控。
//      velocity_topic 若配了**真速度源**（VIO/光流），优先走那一路，不再自己差分。
//
//  PX4 行为备注：
//      PX4 的 EKF2 对 EV 速度的判定是 if (ev_odom_vel.isAllFinite())（EKF2.cpp:2167），
//      所以**速度填 NaN 就等于"本帧不提供速度"**，这是官方支持的做法。
//
//  为什么走 EV：
//      EKF2 订阅 vehicle_visual_odometry（见 EKF2.hpp:328）。打开
//      EKF2_EV_CTRL 后，位置/速度/航向才进 EKF，
//      vehicle_local_position 的 xy_valid / v_xy_valid / z_valid 才会变 true，
//      Offboard 位置控制（px4_takeoff_land）才被 PX4 允许
//      （依据 commander/HealthAndArmingChecks/checks/offboardCheck.cpp）。
//
//  坐标系（关键，错了会直接飞走）：
//      SLAM 一般输出 ENU：x 东、y 北、z 上、yaw 从东起逆时针为正
//      PX4 EV 要 NED：x 北、y 东、z 下、yaw 从北起顺时针为正
//      位置： x_ned = y_enu, y_ned = x_enu
//      高度： z_ned = -height_agl   （EKF 原点在地面时）
//      航向： yaw_ned = pi/2 - yaw_enu
//      四元数： body(FRD) -> NED。EKF2 只取里面的 yaw
//      （ev_yaw_control.cpp:48 = getEulerYaw(ev_sample.quat)），
//      所以 roll/pitch 填 0 即可，2D SLAM 够用。
//
//  对应的飞控参数：
//      EKF2_EV_CTRL = 15   （bit0 水平位置 + bit1 垂直位置 + bit2 速度 + bit3 航向）
//      EKF2_HGT_REF = 3    （高度参考 = Vision）
//      EKF2_EV_DELAY / EKF2_EV_POS_X/Y/Z 按实测调
//
//  发布节奏（重要）：EKF2 要求 EV 采样间隔 < EV_MAX_INTERVAL(=200 ms，见
//            ekf2/EKF/common.h)，否则 ev_control.cpp 里的 starting_conditions_passing
//            永远为假，EV 融合根本不会启动；超过 2*200 ms 还会把已开启的 EV 融合关掉。
//            cartographer 的 /tracked_pose 远达不到 5 Hz，所以本节点按**固定频率**
//            ("publish_rate_hz"，默认 30 Hz) 重发最近一帧位姿，而不是"收到新位姿才发"。
//            SLAM 超过 pose_timeout 没更新时**停止发布**，让 EKF2 自己超时退出 EV 融合
//            （绝不能把冻结的旧位置继续喂给 EKF2）。
//            高度用最近一次定高器读数，超时（height_timeout）就退回"不可信"状态。
// =====================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

using px4_msgs::msg::VehicleOdometry;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr float kBadVariance = 100.0f;  // 表示"这个维度不可信"

double wrapPi(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a < -kPi) { a += 2.0 * kPi; }
  return a;
}

// 从四元数取 yaw（ZYX）
double yawFromQuat(double x, double y, double z, double w)
{
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-9) {
    return 0.0;
  }
  x /= n; y /= n; z /= n; w /= n;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class EvBridge : public rclcpp::Node
{
public:
  EvBridge() : Node("px4_ev_bridge")
  {
    pose_topic_      = declare_parameter<std::string>("pose_topic", "/tracked_pose");
    pose_msg_type_   = declare_parameter<std::string>("pose_msg_type", "pose_stamped");
    // 速度默认**不上报**（见文件头 2026-10-09 说明）。
    // 只有这里明确指向一个**真速度源**（VIO / 光流 / 带速度输出的 SLAM）时，才发速度。
    velocity_topic_  = declare_parameter<std::string>("velocity_topic", "");
    // 2026-10-09 晚：恢复"位置差分算速度并上报"（用户要求飞控能收到速度）。
    // 想回到"只发位置、不发速度"：report_velocity:=false。
    // 2026-10-10：**默认改回 false（不上报）**。理由见文件头：位置差分出来的"速度"和位置
    // 不是独立信息，EKF2 却把它当独立观测吃进去，会拖着估计漂 —— 这与"朝固定方向偏 +
    // 越飞越倾"的症状吻合，也是队友当初关掉它的原因。想再试一次：report_velocity:=true。
    report_velocity_ = declare_parameter<bool>("report_velocity", false);
    vel_alpha_       = declare_parameter<double>("vel_filter_alpha", 0.7);
    enu_to_ned_      = declare_parameter<bool>("enu_to_ned", true);
    yaw_offset_deg_  = declare_parameter<double>("yaw_offset_deg", 0.0);
    pose_timeout_    = declare_parameter<double>("pose_timeout", 0.5);
    position_sigma_  = declare_parameter<double>("position_sigma", 0.05);   // m
    velocity_sigma_  = declare_parameter<double>("velocity_sigma", 0.10);   // m/s
    yaw_sigma_deg_   = declare_parameter<double>("yaw_sigma_deg", 2.0);
    quality_         = declare_parameter<int>("quality", 100);
    reset_jump_m_    = declare_parameter<double>("reset_jump_m", 1.0);
    log_period_sec_  = declare_parameter<double>("log_period_sec", 5.0);
    // 2026-10-08 修复：默认发布率要与上游真实位姿率（雷达 15Hz）接近，
    // 否则每个样本会被复读多次，EKF2 会把复读当成独立测量 -> 位置置信度虚增
    // （改 fly.lua 的 motion_filter 前实测：位姿其实只有 ~1.5Hz，被复读 20 次）。
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 15.0);  // Hz，必须 >5
    stamp_timeout_   = declare_parameter<double>("stamp_timeout", 0.3);     // s，位姿时间戳多久不推进就停发
    jump_pause_sec_  = declare_parameter<double>("jump_pause_sec", 1.0);    // s，检测到位姿跳变后暂停发布多久

    if (publish_rate_hz_ < 5.0) {
      RCLCPP_WARN(get_logger(),
                  "publish_rate_hz=%.1f 低于 EKF2 的 5 Hz 下限（EV_MAX_INTERVAL=200ms），"
                  "已强制改为 5 Hz", publish_rate_hz_);
      publish_rate_hz_ = 5.0;
    }
    if (publish_rate_hz_ > 100.0) {
      publish_rate_hz_ = 100.0;
    }

    // ---- 定高器（垂直位置）----
    use_height_      = declare_parameter<bool>("use_height", true);
    height_topic_    = declare_parameter<std::string>("height_topic", "/tofsense/height");
    height_timeout_  = declare_parameter<double>("height_timeout", 0.25);   // s
    height_offset_   = declare_parameter<double>("height_offset", 0.0);     // m，标定用
    // 2026-10-10：2cm 是"谎报精度"。TOF 实测贴地读 0.045/0.43、飞行中出 2.55m 尖峰，
    // 真实精度给 15cm 更诚实 —— 关键是**EKF2 的 innovation gate 只有在方差诚实时才拦得住
    // 1~2m 的离谱值**；σ=2cm 时 2m 误差相当于 100σ，EKF 会当成真值直接吃下去。
    height_sigma_    = declare_parameter<double>("height_sigma", 0.15);     // m
    // 2026-10-10 坠机修复：TOF 的离谱读数**会连续出现多帧**，而它直接就是 EV 的 z。
    //
    // 事故现场（EV 桥日志 10:27:02，飞机实际约 0.5m 高）：
    //    0.69 → 1.38 → 2.20 → 0.69 → 0.22 m   （3 秒内乱跳）
    // 老判据是"连续 5 帧偏离就接受新高度"，正好被这个序列命中 —— 2.20m 灌进 EKF2，
    // 位置环以为飞机在 2.65m、目标是 0.56m，于是**命令下降 2m**，实际把飞机压向地面。
    //
    // 新判据：偏离值只有在**连续多帧彼此一致**（都落在 height_stable_band 内）时才被
    // 当作"真的换了高度"接受 —— 乱跳的序列永远凑不齐这几帧。
    height_jump_max_     = declare_parameter<double>("height_jump_max", 0.2);    // m，单帧允许的台阶
    // 2026-10-10：0.5 m/s 太紧 —— 真起飞可能比这快（PX4 MPC_Z_VEL_MAX_UP 默认 3 m/s），
    // 会被误判成"跳变"而把高度卡在地面值 → EKF 以为没起来 → 一路爬升。放宽到 2 m/s。
    height_rate_max_     = declare_parameter<double>("height_rate_max", 2.0);    // m/s，按时间积分的速率上限
    height_stable_band_  = declare_parameter<double>("height_stable_band", 0.1); // m，候选平台内的一致性带宽
    height_jump_confirm_ = declare_parameter<int>("height_jump_confirm", 25);    // 连续多少帧一致才认账

    if (pose_msg_type_ != "pose_stamped" && pose_msg_type_ != "pose_with_covariance") {
      RCLCPP_WARN(get_logger(), "pose_msg_type='%s' 不识别，按 pose_stamped 处理",
                  pose_msg_type_.c_str());
      pose_msg_type_ = "pose_stamped";
    }

    const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    ev_pub_ = create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos_pub);

    if (pose_msg_type_ == "pose_with_covariance") {
      pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, qos_sub,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
          onPose(m->pose.pose, rclcpp::Time(m->header.stamp));
        });
    } else {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, qos_sub,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          onPose(m->pose, rclcpp::Time(m->header.stamp));
        });
    }

    if (!velocity_topic_.empty()) {
      vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        velocity_topic_, qos_sub,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
          raw_vx_enu_ = m->twist.linear.x;
          raw_vy_enu_ = m->twist.linear.y;
          have_vel_topic_ = true;
        });
    }

    if (use_height_) {
      height_sub_ = create_subscription<std_msgs::msg::Float32>(
        height_topic_, qos_sub,
        [this](std_msgs::msg::Float32::ConstSharedPtr m) {
          if (!std::isfinite(m->data) || m->data <= 0.0f) {
            return;
          }
          const double h = static_cast<double>(m->data);
          const rclcpp::Time t = now();

          // 第一帧：还没有参考值，直接接受
          if (!have_height_) {
            height_ = h;
            last_height_ = t;
            have_height_ = true;
            height_candidate_count_ = 0;
            return;
          }

          // 允许的变化量 = max(固定台阶, 速率 × 距上次接受的时间)
          //   正常飞行高度变化很慢，所以不会误伤；而 3 秒跳 2m 绝对过不去。
          const double dt = std::max(0.0, (t - last_height_).seconds());
          const double allowed = std::max(height_jump_max_, height_rate_max_ * dt);

          if (std::fabs(h - height_) <= allowed) {
            height_ = h;
            last_height_ = t;              // 只有被接受的读数才刷新时间戳
            height_candidate_count_ = 0;
            return;
          }

          // ---- 偏离：必须连续多帧彼此一致，才认为是"真的换了高度" ----
          if (height_candidate_count_ == 0 ||
              std::fabs(h - height_candidate_) > height_stable_band_) {
            height_candidate_ = h;
            height_candidate_count_ = 1;
          } else {
            height_candidate_count_++;
          }

          if (height_candidate_count_ >= height_jump_confirm_) {
            // 2026-10-10 二次加固：实测过"TOF 稳定错值平台"（连续 10 帧稳定在 1.97m，
            // 飞机实际只有 0.24m）—— 光看"稳定"不够，**稳定值也必须满足速率约束**：
            // 按 height_rate_max 算，2m 的假高度至少要 4 秒才可能被追上，而 0.5m 的真起飞
            // 是逐帧缓慢变化的、走的是上面那条"允许台阶"路径，不受影响。
            const double allowed_plateau = std::max(height_jump_max_, height_rate_max_ * dt);
            if (std::fabs(h - height_) > allowed_plateau) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                   "高度稳定在 %.2fm，但与有效值差 %.2fm，超过速率上限（%.2f m/s × %.1fs = %.2fm）—— 继续丢弃",
                                   h, std::fabs(h - height_), height_rate_max_, dt, allowed_plateau);
              return;
            }
            RCLCPP_WARN(get_logger(),
                        "高度连续 %d 帧稳定在 %.2fm（原有效值 %.2fm），接受为新高度（真起飞/降落/换地面）",
                        height_candidate_count_, h, height_);
            height_ = h;
            last_height_ = t;
            height_candidate_count_ = 0;
            return;
          }

          // 既没被接受、也没形成稳定平台 —— 丢弃，并且**不刷新 last_height_**。
          // 超过 height_timeout 后 EV 高度会自己变成"缺失/不可信"（z 方差填 kBadVariance），
          // 让 EKF2 先松手，而不是把错值喂进去把飞机带下去。
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "高度读数 %.2fm 偏离有效值 %.2fm，且未稳定（连续第 %d 帧）—— 丢弃",
                               h, std::fabs(h - height_), height_candidate_count_);
        });
    }

    // 固定频率发布（见文件头说明）
    pub_timer_ = create_wall_timer(
      std::chrono::microseconds(static_cast<int64_t>(1e6 / publish_rate_hz_)),
      [this]() { publishTick(); });

    watchdog_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { checkStale(); });
    last_stamp_change_ = now();

    RCLCPP_INFO(get_logger(),
                "EV 桥已启动：位姿 %s(%s) + 高度 %s -> /fmu/in/vehicle_visual_odometry，"
                "发布频率 %.1f Hz",
                pose_topic_.c_str(), pose_msg_type_.c_str(),
                use_height_ ? height_topic_.c_str() : "(关闭)", publish_rate_hz_);
    RCLCPP_INFO(get_logger(),
                "对应飞控参数：EKF2_EV_CTRL=15、EKF2_HGT_REF=3，"
                "并按实测设置 EKF2_EV_DELAY / EKF2_EV_POS_X/Y/Z");
  }

private:
  // 注意：不能声明成 const —— const 版 get_clock() 返回 const Clock，
  // 而 rclcpp::Clock::now() 非 const，会编译失败（和 px4_bridge 同一个坑）。
  uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void onPose(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp)
  {
    const double x = pose.position.x;
    const double y = pose.position.y;
    const double yaw = yawFromQuat(pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "位姿里有非有限值，丢弃");
      return;
    }

    // ---- 重定位跳变检测（只查位置）----
    // 2026-10-11：按用户要求**去掉航向跳变检测**（回到 2026-10-09 晚能飞的那版行为）。
    //   原因：我们的 SLAM 在对称环境里航向会频繁 180° 跳，加了航向判据之后
    //   每跳一次就 reset_counter++ 并停发 EV 1 秒；而 EV 同时携带高度
    //   （EKF2_HGT_REF=3），于是每次跳变都会让 EKF2 同时丢掉水平位置和高度参考
    //   ——实测 30 秒内触发 223 次，比"航向跳变本身"危险得多。
    //   仍然保留：位置台阶 > reset_jump_m_（1 m）才 reset + 静默 1 s（10-09 晚就是这样）。
    if (have_pose_) {
      const double jump = std::hypot(x - enu_x_, y - enu_y_);
      if (jump > reset_jump_m_) {
        reset_counter_ = static_cast<uint8_t>(reset_counter_ + 1);
        // 跳变这一帧不能让"位置差分速度"参与 —— 否则会算出一个巨大的假速度喂给 EKF。
        have_prev_ = false;
        // —— 2026-10-08 事故修复 ——
        // 跳变通常意味着 SLAM 重定位/重启（新地图原点）。除了用 reset_counter 告知
        // EKF2 之外，再**暂停发布 jump_pause_sec**：这样 EKF2 会先超时关掉 EV 融合，
        // 再以干净的 reset_counter 重新接入，避免把两段不连续的位姿混在一起。
        pause_until_ = now() + rclcpp::Duration(std::chrono::duration<double>(jump_pause_sec_));
        RCLCPP_WARN(get_logger(), "SLAM 位姿跳变 %.2fm，reset_counter -> %u，暂停发 EV %.1fs",
                    jump, static_cast<unsigned>(reset_counter_), jump_pause_sec_);
      }
    }

    // ---- 位置差分求速度（ENU）----
    // 只在"没有真速度源 且 允许上报"时才做；Δpos/Δt 对高频有增益，用一阶低通压一下。
    if (report_velocity_ && !have_vel_topic_ && have_prev_) {
      const double dt = (stamp - prev_stamp_).seconds();
      if (dt > 0.005 && dt < 0.5) {
        const double rx = (x - prev_x_) / dt;
        const double ry = (y - prev_y_) / dt;
        raw_vx_enu_ = vel_alpha_ * raw_vx_enu_ + (1.0 - vel_alpha_) * rx;
        raw_vy_enu_ = vel_alpha_ * raw_vy_enu_ + (1.0 - vel_alpha_) * ry;
      }
    }
    prev_x_ = x;
    prev_y_ = y;
    prev_stamp_ = stamp;
    have_prev_ = true;

    enu_x_ = x;
    enu_y_ = y;
    enu_yaw_ = yaw;
    last_pose_ = now();
    have_pose_ = true;

    // —— 防冻结（2026-10-08 事故修复）——
    // 上游 tf_to_pose 用的是"最新可用 TF"，SLAM 挂掉后仍会以 30Hz 重发**同一帧旧位姿**，
    // 所以"有没有新消息到达"这个判据会被完全绕过。这里改成看**时间戳有没有推进**。
    // 实测位姿 stamp 与本机时钟有固定偏移（约 +1.1s），故不做绝对时间比较。
    const int64_t stamp_us = stamp.nanoseconds() / 1000;  // ns -> us
    if (stamp_us == 0) {
      // 上游没给时间戳：退回"按到达时间"判断
      last_stamp_change_ = now();
    } else if (stamp_us != last_stamp_us_) {
      last_stamp_us_ = stamp_us;
      last_stamp_change_ = now();
    }
  }

  /// 固定频率发布：只在位姿新鲜时发（不发冻结的旧数据）
  void publishTick()
  {
    if (!have_pose_) {
      return;
    }
    if ((now() - last_pose_).seconds() > pose_timeout_) {
      // SLAM 掉线：停发，让 EKF2 在 2*EV_MAX_INTERVAL 后自动关闭 EV 融合
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "位姿已过期，暂停发 EV（等待 SLAM 恢复）");
      return;
    }
    // 位姿跳变（SLAM 重定位/重启）后的静默期
    if (now() < pause_until_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "位姿跳变后的静默期，暂停发 EV（让 EKF2 干净地重新接入）");
      return;
    }
    // 防冻结：消息一直在到、但内容/时间戳冻住了（上游节点挂了、拿旧 TF 反复重发）
    const double stamp_age = (now() - last_stamp_change_).seconds();
    if (stamp_age > stamp_timeout_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "位姿时间戳已 %.1f 秒没有推进（SLAM 冻结或已挂）——停发 EV，"
                            "让 EKF2 超时关闭 EV 融合、飞控进入位置失效保护", stamp_age);
      return;
    }
    publishEv();
  }

  void publishEv()
  {
    const double yaw_ned = wrapPi((enu_to_ned_ ? (kPi / 2.0 - enu_yaw_) : enu_yaw_)
                                  + yaw_offset_deg_ * kPi / 180.0);

    VehicleOdometry m{};
    m.timestamp = now_us();
    m.timestamp_sample = m.timestamp;
    m.pose_frame = VehicleOdometry::POSE_FRAME_NED;

    float x_ned, y_ned, vx_ned, vy_ned;
    if (enu_to_ned_) {
      x_ned = static_cast<float>(enu_y_);
      y_ned = static_cast<float>(enu_x_);
      vx_ned = static_cast<float>(raw_vy_enu_);
      vy_ned = static_cast<float>(raw_vx_enu_);
    } else {
      x_ned = static_cast<float>(enu_x_);
      y_ned = static_cast<float>(enu_y_);
      vx_ned = static_cast<float>(raw_vx_enu_);
      vy_ned = static_cast<float>(raw_vy_enu_);
    }

    // ---- 垂直：来自定高器，NED 下 z 向下为正 ----
    float z_ned = 0.0f;
    float z_var = kBadVariance;
    const bool have_h = use_height_ && have_height_;
    const bool height_ok = have_h && (now() - last_height_).seconds() < height_timeout_;
    // 2026-10-10：高度超时/被拒时**沿用最后一个有效高度**，而不是谎报 z=0。
    // z=0 等于告诉 EKF2"我在原点高度(地面)上" —— 对空中的飞机是个危险的假值；
    // 沿用旧值 + 把方差填成"不可信"，才是"我不知道"的正确表达。
    if (have_h) {
      z_ned = static_cast<float>(-(height_ + height_offset_));
    }
    if (height_ok) {
      z_var = static_cast<float>(height_sigma_ * height_sigma_);
    }

    m.position = {x_ned, y_ned, z_ned};

    // ---- 速度：velocity_topic 优先；否则按 report_velocity 用位置差分上报 ----
    // report_velocity=false 时填 NaN = "本帧不提供速度"，EKF2 只在
    // ev_odom_vel.isAllFinite() 时才把它当速度测量（EKF2.cpp:2167）。
    if (have_vel_topic_ || report_velocity_) {
      m.velocity = {vx_ned, vy_ned, 0.0f};
    } else {
      m.velocity = {std::nanf(""), std::nanf(""), std::nanf("")};
    }

    // body(FRD) -> NED，只保留 yaw（EKF2 只取 yaw）
    const double half = yaw_ned / 2.0;
    m.q = {static_cast<float>(std::cos(half)), 0.0f, 0.0f, static_cast<float>(std::sin(half))};

    m.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;

    const float pv = static_cast<float>(position_sigma_ * position_sigma_);
    const float yaw_sigma = static_cast<float>(yaw_sigma_deg_ * kPi / 180.0);
    const float av = yaw_sigma * yaw_sigma;
    m.position_variance = {pv, pv, z_var};
    if (have_vel_topic_ || report_velocity_) {
      // 保守方差：launch 里 velocity_sigma 默认 1.0 m/s -> 方差 1.0，EKF 不会过度自信
      const float vv = static_cast<float>(velocity_sigma_ * velocity_sigma_);
      m.velocity_variance = {vv, vv, kBadVariance};
    } else {
      // 不上报速度：方差填"不可信"，双保险
      m.velocity_variance = {kBadVariance, kBadVariance, kBadVariance};
    }
    m.orientation_variance = {av, av, av};

    m.reset_counter = reset_counter_;
    m.quality = static_cast<int8_t>(quality_);
    m.angular_velocity = {std::nanf(""), std::nanf(""), std::nanf("")};

    ev_pub_->publish(m);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(),
                         static_cast<int>(log_period_sec_ * 1000.0),
                         "EV: NED pos(%.2f, %.2f, %.2f) yaw %.1f° | 速度%s | 高度%s",
                         m.position[0], m.position[1], m.position[2], yaw_ned * 180.0 / kPi,
                         have_vel_topic_ ? "有(外部源)"
                                         : (report_velocity_ ? "有(位置差分)" : "不上报(交给飞控IMU)"),
                         height_ok ? "有效" : "缺失");
  }

  void checkStale()
  {
    if (!have_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "还没收到 %s 的位姿 —— SLAM 起了吗？话题名对吗？",
                           pose_topic_.c_str());
      return;
    }
    const double age = (now() - last_pose_).seconds();
    if (age > pose_timeout_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "%.2fs 没收到新位姿，EKF2 会停掉 EV 融合（不要起飞）", age);
    }
    if (use_height_) {
      const double h_age = have_height_ ? (now() - last_height_).seconds() : 1e9;
      if (h_age > height_timeout_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "%.2fs 没收到 %s 的高度 —— 垂直接近会失效（不要起飞）",
                              h_age, height_topic_.c_str());
      }
    }
  }

  // ---- 参数 ----
  std::string pose_topic_{"/tracked_pose"};
  std::string pose_msg_type_{"pose_stamped"};
  std::string velocity_topic_;
  bool report_velocity_{false};  // 没有真速度源时，是否用位置差分上报速度（2026-10-10 默认关闭）
  double vel_alpha_{0.7};        // 差分速度的一阶低通系数 (0..1)
  bool enu_to_ned_{true};
  double yaw_offset_deg_{0.0};
  double pose_timeout_{0.5};
  double position_sigma_{0.05};
  double velocity_sigma_{0.10};
  double yaw_sigma_deg_{2.0};
  int quality_{100};
  double reset_jump_m_{1.0};
  double log_period_sec_{5.0};
  double publish_rate_hz_{30.0};
  double stamp_timeout_{0.3};
  double jump_pause_sec_{1.0};
  bool use_height_{true};
  std::string height_topic_{"/tofsense/height"};
  double height_timeout_{0.25};
  double height_offset_{0.0};
  double height_sigma_{0.15};
  double height_jump_max_{0.2};      // m，单帧允许的高度台阶
  double height_rate_max_{2.0};      // m/s，按时间积分的速率上限
  double height_stable_band_{0.1};   // m，候选平台内的一致性带宽
  int    height_jump_confirm_{25};   // 连续多少帧彼此一致才接受新高度

  // ---- 状态 ----
  bool have_pose_{false};
  bool have_vel_topic_{false};
  bool have_prev_{false};
  bool have_height_{false};
  double height_candidate_{0.0};     // 正在观察的候选新高度
  int    height_candidate_count_{0}; // 该候选已连续出现的帧数
  double enu_x_{0.0}, enu_y_{0.0}, enu_yaw_{0.0};
  double prev_x_{0.0}, prev_y_{0.0};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  double height_{0.0};
  rclcpp::Time last_pose_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_height_{0, 0, RCL_ROS_TIME};
  double raw_vx_enu_{0.0}, raw_vy_enu_{0.0};
  uint8_t reset_counter_{0};

  // ---- ROS 接口 ----
  rclcpp::Publisher<VehicleOdometry>::SharedPtr ev_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  int64_t last_stamp_us_{0};
  rclcpp::Time last_stamp_change_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pause_until_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvBridge>());
  rclcpp::shutdown();
  return 0;
}
