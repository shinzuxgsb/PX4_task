"""一条命令拉起整套无人机链路（雷达 + SLAM + PX4 桥接）。

    ros2 launch mavlink_bridge drone_stack.launch.py

启动顺序（用 TimerAction 错开，Cartographer 依赖 laser_link 的 TF）：
  0s  雷达驱动 (bluesea2, UDP)
  3s  robot_state_publisher（从 URDF 提供 laser_link/base_link）
  5s  Cartographer 2D（出 odom->base_link）
  7s  mavlink_bridge（含 tf_to_odom + tofsense）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ↓ 147 适配 1/2（2026-10-16）：原文件写死 /home/orangepi/install/my_carto_pkg/share/my_carto_pkg
#   （那台机器的 ~ 根工作区）。这里改成"从当前 overlay 里找"，布局随便换都能用。
DEFAULT_MY_CARTO_SHARE = get_package_share_directory("my_carto_pkg")
# ↓ 147 适配 2/2：147 上雷达是串口款 LDS-50C-R（/dev/ttyS4 @921600），默认用本包里
#   的 uart 参数；网口款 LDS-50C-E 的参数（lidar_net.yaml）也一并放进包里，
#   用 `lidar_params_file:=.../config/lidar_net.yaml` 覆盖即可。
DEFAULT_LIDAR_PARAMS = os.path.join(
    get_package_share_directory("mavlink_bridge"), "config", "lidar_uart.yaml"
)


def generate_launch_description():
    my_carto_share_arg = DeclareLaunchArgument(
        "my_carto_share", default_value=DEFAULT_MY_CARTO_SHARE
    )
    lidar_params_arg = DeclareLaunchArgument(
        "lidar_params_file", default_value=DEFAULT_LIDAR_PARAMS
    )

    my_carto_share = LaunchConfiguration("my_carto_share")
    bluesea_share = get_package_share_directory("bluesea2")
    bridge_share = get_package_share_directory("mavlink_bridge")

    # URDF 在 launch 时读进来，省掉一个外部参数文件
    with open(os.path.join(DEFAULT_MY_CARTO_SHARE, "urdf", "fly.urdf"), "r") as handle:
        robot_desc = handle.read()

    # 注意：这里直接起节点，不用 bluesea2 自带的 launch。
    # 那个 launch 内部声明了名为 "params_file" 的 launch 参数，而 ROS 2 的
    # launch 参数是**全局**的——一旦在这里设置，后面 include 的
    # mavlink_bridge.launch.py 读到的同名参数就会被顶掉，导致桥接节点
    # 加载到雷达的配置（实测踩过这个坑）。
    lidar = Node(
        package="bluesea2",
        executable="bluesea2_node",
        name="bluesea_node",
        parameters=[LaunchConfiguration("lidar_params_file")],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        output="screen",
    )

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory",
            os.path.join(DEFAULT_MY_CARTO_SHARE, "configuration_files"),
            "-configuration_basename",
            "amphi.lua",
        ],
        remappings=[("scan", "scan")],
        output="screen",
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "mavlink_bridge.launch.py")
        )
    )

    return LaunchDescription(
        [
            my_carto_share_arg,
            lidar_params_arg,
            lidar,
            TimerAction(period=3.0, actions=[robot_state_publisher]),
            TimerAction(period=5.0, actions=[cartographer]),
            TimerAction(period=7.0, actions=[bridge]),
        ]
    )
