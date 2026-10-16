"""雷达 SLAM 水平位姿 + TOFSense-M 垂直高度 -> PX4 外部视觉里程计。

前提：Micro-XRCE-DDS-Agent 已在运行，cartographer 在发 /tracked_pose，
      tofsense_m 在发 /tofsense/height。

    ros2 launch px4_ev_bridge ev_bridge.launch.py
    ros2 launch px4_ev_bridge ev_bridge.launch.py pose_topic:=/pose \
         pose_msg_type:=pose_with_covariance          # slam_toolbox
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pose_topic', default_value='/tracked_pose'),
        DeclareLaunchArgument('pose_msg_type', default_value='pose_stamped'),
        DeclareLaunchArgument('velocity_topic', default_value=''),
        DeclareLaunchArgument('enu_to_ned', default_value='true'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('use_height', default_value='true'),
        DeclareLaunchArgument('height_topic', default_value='/tofsense/height'),
        DeclareLaunchArgument('height_offset', default_value='0.0'),
        # 2026-10-10：TOF 真实精度给 15cm（原来 2cm 是谎报，EKF2 的门限因此拦不住离谱值）
        DeclareLaunchArgument('height_sigma', default_value='0.15'),
        # 2026-10-09 晚 事故修复：高度（TOF）单帧跳变保护。
        # 实测贴地时 TOF 读到过 2.55 m / 0.567 m，而它直接就是 EV 的 z；
        # 一个 2 m 的 z 台阶灌进 EKF2 会让位置估计失效 → PX4 掉出 Offboard。
        # 2026-10-10 坠机修复：TOF 的错值是"连续多帧乱跳"，老判据（连续 N 帧偏离就接受）拦不住。
        # 现在要求偏离值连续多帧**彼此一致**才接受；对不上就丢弃并让它超时变"不可信"。
        DeclareLaunchArgument('height_jump_max', default_value='0.2'),
        DeclareLaunchArgument('height_rate_max', default_value='2.0'),
        DeclareLaunchArgument('height_stable_band', default_value='0.1'),
        DeclareLaunchArgument('height_jump_confirm', default_value='25'),
        # 2026-10-11：航向跳变检测已按用户要求移除（回到 2026-10-09 晚能飞的行为）。
        # 保留位置跳变门限 reset_jump_m（默认 1.0m，见节点内部参数）。
        DeclareLaunchArgument('reset_jump_m', default_value='1.0'),
        DeclareLaunchArgument('jump_pause_sec', default_value='1.0'),
        # ===== 2026-10-09：EV 上报的不确定度（σ）与飞控参数对齐 =====
        #
        # 关键机制（PX4 1.15.4 实测）：
        #   EKF2_EV_NOISE_MD 默认为 0 = "噪声取报文里报的方差，飞控参数只作下限"
        #   （EKF2.cpp 里就是 ev_var = fmaxf(param^2, msg_var)）。
        #   → 有效噪声 = max(EKF2_EVP/EVV/EVA_NOISE, 这里报的 σ)。
        #   也就是说：**只要这里报得比飞控参数大，"信多少"就在香橙派这侧决定**。
        #
        #   position_sigma 0.05 -> 0.25 m
        #     SLAM 位置有台阶/被机身倾斜污染，不该当成 2.5 cm 精度来喂 EKF。
        #   velocity_sigma 1.00 m/s（只在 velocity_topic 配了**真速度源**时才起作用）
        #     2026-10-09：EV 桥已经**不再自己算速度**了。原来那个"速度"是拿 SLAM 位置
        #     差分（Δpos/Δt + α=0.7 低通）造出来的，和位置不独立、还会放大量化噪声，
        #     却被 EKF 当成独立的速度测量融合 —— 那正是"假速度"进飞控的路径。
        #     现在默认发 NaN（= 本帧不提供速度，EKF2.cpp:2167 `isAllFinite()` 判定），
        #     速度交给飞控自己的 IMU 积分。该参数保留给以后真接了 VIO/光流的情形。
        #   yaw_sigma_deg 25.0（2026-10-08 已调，保持）
        #     让 100 Hz 陀螺主导航向；SLAM 航向跳 100° 时 EKF 只被带动约 3%。
        #     注：早先注释说 EKF2_EVA_NOISE "范围只有 0.05~0.1" 是错的 ——
        #     1.15.4 的 module.yaml 里它只有 min=0.05，没有 max。
        DeclareLaunchArgument('position_sigma', default_value='0.25'),
        DeclareLaunchArgument('velocity_sigma', default_value='1.0'),
        # 2026-10-10：默认改成 **false（不上报速度）**。
        # 位置差分出来的"速度"和位置不是独立信息，EKF2 却把它当独立观测吃，会拖着估计漂——
        # 与"朝固定方向偏 + 越飞越倾"吻合。想吃速度就显式 report_velocity:=true 再试。
        DeclareLaunchArgument('report_velocity', default_value='false'),
        DeclareLaunchArgument('vel_filter_alpha', default_value='0.7'),
        DeclareLaunchArgument('yaw_sigma_deg', default_value='25.0'),
        Node(
            package='px4_ev_bridge',
            executable='px4_ev_bridge_node',
            name='px4_ev_bridge',
            output='screen',
            parameters=[{
                'pose_topic': LaunchConfiguration('pose_topic'),
                'pose_msg_type': LaunchConfiguration('pose_msg_type'),
                'velocity_topic': LaunchConfiguration('velocity_topic'),
                # 数值参数一律用 ParameterValue 强制类型：命令行写 `velocity_sigma:=1`（整数）
                # 会被 YAML 解析成 int，而节点声明的是 double → rclcpp 抛异常、节点崩溃。
                'enu_to_ned': ParameterValue(LaunchConfiguration('enu_to_ned'), value_type=bool),
                'yaw_offset_deg': ParameterValue(LaunchConfiguration('yaw_offset_deg'), value_type=float),
                'use_height': ParameterValue(LaunchConfiguration('use_height'), value_type=bool),
                'height_topic': LaunchConfiguration('height_topic'),
                'height_offset': ParameterValue(LaunchConfiguration('height_offset'), value_type=float),
                'height_sigma': ParameterValue(LaunchConfiguration('height_sigma'), value_type=float),
                'height_jump_max': ParameterValue(LaunchConfiguration('height_jump_max'), value_type=float),
                'height_rate_max': ParameterValue(LaunchConfiguration('height_rate_max'), value_type=float),
                'height_stable_band': ParameterValue(LaunchConfiguration('height_stable_band'), value_type=float),
                'height_jump_confirm': ParameterValue(LaunchConfiguration('height_jump_confirm'), value_type=int),
                'reset_jump_m': ParameterValue(LaunchConfiguration('reset_jump_m'), value_type=float),
                'jump_pause_sec': ParameterValue(LaunchConfiguration('jump_pause_sec'), value_type=float),
                'position_sigma': ParameterValue(LaunchConfiguration('position_sigma'), value_type=float),
                'velocity_sigma': ParameterValue(LaunchConfiguration('velocity_sigma'), value_type=float),
                'report_velocity': ParameterValue(LaunchConfiguration('report_velocity'), value_type=bool),
                'vel_filter_alpha': ParameterValue(LaunchConfiguration('vel_filter_alpha'), value_type=float),
                'yaw_sigma_deg': ParameterValue(LaunchConfiguration('yaw_sigma_deg'), value_type=float),
            }],
        ),
    ])
