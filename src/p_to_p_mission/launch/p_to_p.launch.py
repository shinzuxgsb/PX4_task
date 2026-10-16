"""A→B 点对点任务（PX4 Offboard 版）—— 只起"任务 + 桥"，不起雷达/SLAM/EV。

逻辑来自 onboard_ws/p_to_p_flight_px4（用户 2026-10-12 给的参考件）。
运行前提（由本机 run.sh / px4-ev-chain 负责，别在这里重复拉起）：
  · 雷达 + cartographer（提供 map→base_link / map→laser_link 的 TF）
  · px4_ev_bridge（把 SLAM+定高合成 vehicle_visual_odometry 喂 EKF2）
  · MicroXRCEAgent（ttyS6）

安全：默认 bench_only:=true —— 只做预演，绝不发控制/模式命令。
真跑要显式 bench_only:=false，且遥控器先解锁、再切 Offboard（require_rc_offboard=true）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    mission_share = get_package_share_directory("p_to_p_mission")
    px4_share = get_package_share_directory("p_to_p_px4")
    bench = ParameterValue(LaunchConfiguration("bench_only"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("bench_only", default_value="true",
                              description="true=只预演，绝不向 PX4 发控制/模式命令"),
        DeclareLaunchArgument("b_offset_x_cm", default_value="100.0",
                              description="B 点：相对起飞机头系的前向距离（cm）"),
        DeclareLaunchArgument("b_offset_y_cm", default_value="0.0",
                              description="B 点：相对起飞机头系的左向距离（cm）"),
        DeclareLaunchArgument("cruise_height_cm", default_value="50.0",
                              description="巡航高度（相对起飞点，cm）"),
        DeclareLaunchArgument("terminal_behavior", default_value="hold",
                              description="hold=到 B 悬停；guided_land_handoff=限速下降到地面基线上方约 4cm 再交还 PX4 Land"),
        Node(package="p_to_p_mission", executable="p_to_p_mission_node",
             name="p_to_p_mission", output="screen",
             parameters=[os.path.join(mission_share, "config", "p_to_p.yaml"), {
                 "b_offset_x_cm": ParameterValue(LaunchConfiguration("b_offset_x_cm"), value_type=float),
                 "b_offset_y_cm": ParameterValue(LaunchConfiguration("b_offset_y_cm"), value_type=float),
                 "cruise_height_cm": ParameterValue(LaunchConfiguration("cruise_height_cm"), value_type=float),
                 "terminal_behavior": LaunchConfiguration("terminal_behavior"),
             }]),
        Node(package="p_to_p_px4", executable="px4_offboard_bridge",
             name="px4_offboard_bridge", output="screen",
             parameters=[os.path.join(px4_share, "config", "px4.yaml"),
                         {"bench_only": bench}]),
    ])