import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("pid_control_pkg"), "config", "px4_pid.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="PX4-localized PID parameter file",
            ),
            Node(
                package="pid_control_pkg",
                executable="position_pid_controller",
                name="position_pid_controller",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
