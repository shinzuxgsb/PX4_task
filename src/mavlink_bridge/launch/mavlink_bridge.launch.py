import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mavlink_bridge")
    default_params = os.path.join(share, "config", "mavlink_bridge.yaml")
    default_tf_params = os.path.join(share, "config", "tf_to_odom.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="mavlink_bridge 参数文件",
    )
    tf_params_arg = DeclareLaunchArgument(
        "tf_params_file",
        default_value=default_tf_params,
        description="tf_to_odom 参数文件",
    )
    tofsense_params_arg = DeclareLaunchArgument(
        "tofsense_params_file",
        default_value=os.path.join(share, "config", "tofsense.yaml"),
        description="tofsense 参数文件",
    )
    start_tf_arg = DeclareLaunchArgument(
        "start_tf_to_odom",
        default_value="true",
        description="是否同时启动 TF->Odometry 节点（用 FAST-LIO 等自带里程计时可关掉）",
    )
    start_tof_arg = DeclareLaunchArgument(
        "start_tofsense",
        default_value="true",
        description="是否同时启动 TOFSense 测距节点",
    )

    bridge = Node(
        package="mavlink_bridge",
        executable="mavlink_bridge",
        name="mavlink_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    tf_to_odom = Node(
        package="mavlink_bridge",
        executable="tf_to_odom",
        name="tf_to_odom",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("tf_params_file")],
        condition=IfCondition(LaunchConfiguration("start_tf_to_odom")),
    )

    tofsense = Node(
        package="mavlink_bridge",
        executable="tofsense_node",
        name="tofsense",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("tofsense_params_file")],
        condition=IfCondition(LaunchConfiguration("start_tofsense")),
    )

    return LaunchDescription(
        [
            params_arg,
            tf_params_arg,
            tofsense_params_arg,
            start_tf_arg,
            start_tof_arg,
            bridge,
            tf_to_odom,
            tofsense,
        ]
    )
