from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    input_bspline_topic = LaunchConfiguration("input_bspline_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    output_topic = LaunchConfiguration("output_topic")
    use_fixed_yaw = LaunchConfiguration("use_fixed_yaw")
    fixed_yaw_deg = LaunchConfiguration("fixed_yaw_deg")
    enable_debug_publish = LaunchConfiguration("enable_debug_publish")
    enable_debug_logs = LaunchConfiguration("enable_debug_logs")
    debug_log_throttle_ms = LaunchConfiguration("debug_log_throttle_ms")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("pid_control_pkg"),
                "config",
                "bspline_trajectory_tracker.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "input_bspline_topic",
            default_value="/drone_0_planning/bspline",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/fastlio2/lio_odom_imu_high",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/target_velocity",
        ),
        DeclareLaunchArgument(
            "use_fixed_yaw",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "fixed_yaw_deg",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "enable_debug_publish",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_debug_logs",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "debug_log_throttle_ms",
            default_value="1000",
        ),
        Node(
            package="pid_control_pkg",
            executable="bspline_trajectory_tracker",
            name="bspline_trajectory_tracker",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_fixed_yaw": use_fixed_yaw,
                    "fixed_yaw_deg": fixed_yaw_deg,
                    "enable_debug_publish": enable_debug_publish,
                    "enable_debug_logs": enable_debug_logs,
                    "debug_log_throttle_ms": debug_log_throttle_ms,
                },
            ],
            remappings=[
                ("planning/bspline", input_bspline_topic),
                ("odom", odom_topic),
                ("target_velocity", output_topic),
            ],
        ),
    ])