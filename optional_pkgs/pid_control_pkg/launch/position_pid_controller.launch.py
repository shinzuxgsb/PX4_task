from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    enable_target_switch_filter = LaunchConfiguration("enable_target_switch_filter")
    control_mode_topic = LaunchConfiguration("control_mode_topic")
    active_control_mode = LaunchConfiguration("active_control_mode")
    start_active = LaunchConfiguration("start_active")
    use_height_topic_for_z = LaunchConfiguration("use_height_topic_for_z")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("pid_control_pkg"),
                "config",
                "position_pid_controller.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "enable_target_switch_filter",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "control_mode_topic",
            default_value="/precision_landing/control_mode",
        ),
        DeclareLaunchArgument(
            "active_control_mode",
            default_value="1",
        ),
        DeclareLaunchArgument(
            "start_active",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_height_topic_for_z",
            default_value="false",
        ),
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[
                params_file,
                {
                    "enable_target_switch_filter": enable_target_switch_filter,
                    "control_mode_topic": control_mode_topic,
                    "active_control_mode": active_control_mode,
                    "start_active": start_active,
                    "use_height_topic_for_z": use_height_topic_for_z,
                },
            ],
        )
    ])
