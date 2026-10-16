"""原地起飞 / 悬停 / 原地降落 任务。

前提：Micro-XRCE-DDS-Agent 已在运行，且 PX4 的 uxrce_dds_client 已连上。
建议先用 dry_run:=true 跑一遍确认话题链路，再真跑。

    ros2 launch px4_takeoff_land takeoff_land.launch.py
    ros2 launch px4_takeoff_land takeoff_land.launch.py dry_run:=true

小任务航线（2026-10-09 新增，mission_enable:=true 才生效）：
    起飞 0.5m → 沿起飞航向前进 0.5m → 原地掉头 180° → 保持新航向回起飞点正上方
    → 悬停 5s → 交还 PX4 自动降落

    ros2 launch px4_takeoff_land takeoff_land.launch.py \\
        target_alt:=0.5 hover_time:=5.0 \\
        mission_enable:=true forward_dist:=0.5 turn_deg:=180
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 2026-10-10：手敲 `ros2 launch` 的那个 shell 里没有 FASTRTPS_DEFAULT_PROFILES_FILE，
    # 会踩 FastDDS 同机共享内存的坑（话题看得见、数据收不到/停住）。
    # 这里强制整条链路走 UDP，与 systemd 单元 px4-ev-chain.service 里的设置保持一致。
    _udp_profile = os.path.expanduser('~/px4_ws/tools/fastdds_udp_only.xml')
    _env_actions = ([SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', _udp_profile)]
                    if os.path.exists(_udp_profile) else [])

    return LaunchDescription(_env_actions + [
        DeclareLaunchArgument('target_alt', default_value='2.0'),
        DeclareLaunchArgument('hover_time', default_value='5.0'),
        DeclareLaunchArgument('climb_vel', default_value='0.5'),
        DeclareLaunchArgument('descend_vel', default_value='0.5'),
        DeclareLaunchArgument('control_mode', default_value='position'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='3.0'),
        # 2026-10-09：小任务航线参数
        DeclareLaunchArgument('mission_enable', default_value='false'),
        DeclareLaunchArgument('forward_dist', default_value='0.5'),
        DeclareLaunchArgument('turn_deg', default_value='180.0'),
        DeclareLaunchArgument('pos_tol', default_value='0.15'),
        DeclareLaunchArgument('yaw_tol_deg', default_value='10.0'),
        DeclareLaunchArgument('settle_time', default_value='0.5'),
        DeclareLaunchArgument('leg_timeout', default_value='20.0'),
        # 2026-10-10 深夜第二轮：任务形态（默认"只前进、不掉头、不返航"）+ 两道交叉校验
        DeclareLaunchArgument('return_home', default_value='false'),
        DeclareLaunchArgument('turn_step_deg', default_value='30.0'),
        DeclareLaunchArgument('yaw_xcheck_deg', default_value='20.0'),
        DeclareLaunchArgument('alt_xcheck_m', default_value='0.5'),
        DeclareLaunchArgument('alt_xcheck_abort_m', default_value='0.9'),
        DeclareLaunchArgument('xcheck_hold', default_value='1.0'),
        DeclareLaunchArgument('tilt_gate_deg', default_value='25.0'),
        # 2026-10-09 事故后调整：offboard 流降到 10 Hz（飞控串口接收方向已占 83%），
        # 并加飞控链路看门狗 link_timeout（vehicle_status 超时即报错中止）
        DeclareLaunchArgument('control_freq', default_value='10.0'),
        DeclareLaunchArgument('link_timeout', default_value='0.5'),
        Node(
            package='px4_takeoff_land',
            executable='px4_takeoff_land_node',
            name='px4_takeoff_land',
            output='screen',
            parameters=[{
                # 注意：这里必须用 ParameterValue(..., value_type=float/bool) 强制类型。
                # 否则命令行写 `turn_deg:=180`（整数）会被 YAML 解析成 int，
                # 而节点声明的是 double → rclcpp 抛 InvalidParameterTypeException 直接崩：
                #   "parameter 'turn_deg' has invalid type: ... is of type {double},
                #    setting it to {integer} is not allowed"
                # 同类坑：hover_time:=5 / leg_timeout:=20 / target_alt:=1 ...
                'target_alt': ParameterValue(LaunchConfiguration('target_alt'), value_type=float),
                'hover_time': ParameterValue(LaunchConfiguration('hover_time'), value_type=float),
                'climb_vel': ParameterValue(LaunchConfiguration('climb_vel'), value_type=float),
                'descend_vel': ParameterValue(LaunchConfiguration('descend_vel'), value_type=float),
                'control_mode': LaunchConfiguration('control_mode'),
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                'start_delay': ParameterValue(LaunchConfiguration('start_delay'), value_type=float),
                'mission_enable': ParameterValue(LaunchConfiguration('mission_enable'), value_type=bool),
                'forward_dist': ParameterValue(LaunchConfiguration('forward_dist'), value_type=float),
                'turn_deg': ParameterValue(LaunchConfiguration('turn_deg'), value_type=float),
                'pos_tol': ParameterValue(LaunchConfiguration('pos_tol'), value_type=float),
                'yaw_tol_deg': ParameterValue(LaunchConfiguration('yaw_tol_deg'), value_type=float),
                'settle_time': ParameterValue(LaunchConfiguration('settle_time'), value_type=float),
                'leg_timeout': ParameterValue(LaunchConfiguration('leg_timeout'), value_type=float),
                'return_home': ParameterValue(LaunchConfiguration('return_home'), value_type=bool),
                'turn_step_deg': ParameterValue(LaunchConfiguration('turn_step_deg'), value_type=float),
                'yaw_xcheck_deg': ParameterValue(LaunchConfiguration('yaw_xcheck_deg'), value_type=float),
                'alt_xcheck_m': ParameterValue(LaunchConfiguration('alt_xcheck_m'), value_type=float),
                'alt_xcheck_abort_m': ParameterValue(LaunchConfiguration('alt_xcheck_abort_m'), value_type=float),
                'xcheck_hold': ParameterValue(LaunchConfiguration('xcheck_hold'), value_type=float),
                'tilt_gate_deg': ParameterValue(LaunchConfiguration('tilt_gate_deg'), value_type=float),
                'control_freq': ParameterValue(LaunchConfiguration('control_freq'), value_type=float),
                'link_timeout': ParameterValue(LaunchConfiguration('link_timeout'), value_type=float),
            }],
        ),
    ])
