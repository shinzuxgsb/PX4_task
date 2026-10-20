from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    input_position_cmd_topic = LaunchConfiguration("input_position_cmd_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    ready_topic = LaunchConfiguration("ready_topic")
    output_topic = LaunchConfiguration("output_topic")
    status_topic = LaunchConfiguration("status_topic")
    control_mode_topic = LaunchConfiguration("control_mode_topic")
    active_control_mode = LaunchConfiguration("active_control_mode")
    cmd_timeout_sec = LaunchConfiguration("cmd_timeout_sec")
    use_fixed_yaw = LaunchConfiguration("use_fixed_yaw")
    fixed_yaw_deg = LaunchConfiguration("fixed_yaw_deg")
    yaw_kp = LaunchConfiguration("yaw_kp")
    yaw_ff_gain = LaunchConfiguration("yaw_ff_gain")
    large_yaw_error_deg_threshold = LaunchConfiguration("large_yaw_error_deg_threshold")
    large_yaw_kp_scale = LaunchConfiguration("large_yaw_kp_scale")
    large_yaw_ff_scale = LaunchConfiguration("large_yaw_ff_scale")
    max_yaw_velocity_deg = LaunchConfiguration("max_yaw_velocity_deg")
    large_yaw_velocity_deg = LaunchConfiguration("large_yaw_velocity_deg")
    enable_takeoff_height_guard = LaunchConfiguration("enable_takeoff_height_guard")
    guarded_takeoff_z_cm = LaunchConfiguration("guarded_takeoff_z_cm")
    enable_bootstrap_takeoff = LaunchConfiguration("enable_bootstrap_takeoff")
    bootstrap_takeoff_z_cm = LaunchConfiguration("bootstrap_takeoff_z_cm")
    enable_trajectory_switch_reset = LaunchConfiguration("enable_trajectory_switch_reset")
    publish_debug_topics = LaunchConfiguration("publish_debug_topics")
    enable_debug_logs = LaunchConfiguration("enable_debug_logs")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("activity_control_pkg"),
                "config",
                "planner_velocity_tracker.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "input_position_cmd_topic",
            default_value="/drone_0_planning/pos_cmd",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/fastlio2/lio_odom_imu_high",
        ),
        DeclareLaunchArgument(
            "ready_topic",
            default_value="/is_st_ready",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/target_velocity",
        ),
        DeclareLaunchArgument(
            "status_topic",
            default_value="/planner_velocity_tracker/status",
        ),
        DeclareLaunchArgument(
            "control_mode_topic",
            default_value="/precision_landing/control_mode",
        ),
        DeclareLaunchArgument(
            "active_control_mode",
            default_value="0",
        ),
        DeclareLaunchArgument(
            "cmd_timeout_sec",
            default_value="0.6",
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
            "yaw_kp",
            default_value="1.1",
        ),
        DeclareLaunchArgument(
            "yaw_ff_gain",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "large_yaw_error_deg_threshold",
            default_value="60.0",
        ),
        DeclareLaunchArgument(
            "large_yaw_kp_scale",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "large_yaw_ff_scale",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "max_yaw_velocity_deg",
            default_value="30.0",
        ),
        DeclareLaunchArgument(
            "large_yaw_velocity_deg",
            default_value="30.0",
        ),
        DeclareLaunchArgument(
            "enable_takeoff_height_guard",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "guarded_takeoff_z_cm",
            default_value="60.0",
        ),
        DeclareLaunchArgument(
            "enable_bootstrap_takeoff",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "bootstrap_takeoff_z_cm",
            default_value="60.0",
        ),
        DeclareLaunchArgument(
            "enable_trajectory_switch_reset",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "publish_debug_topics",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_debug_logs",
            default_value="true",
        ),
        Node(
            package="activity_control_pkg",
            executable="planner_velocity_tracker_node",
            name="planner_velocity_tracker",
            output="screen",
            parameters=[
                params_file,
                {
                    "input_position_cmd_topic": input_position_cmd_topic,
                    "odom_topic": odom_topic,
                    "ready_topic": ready_topic,
                    "output_topic": output_topic,
                    "status_topic": status_topic,
                    "control_mode_topic": control_mode_topic,
                    "active_control_mode": active_control_mode,
                    "cmd_timeout_sec": cmd_timeout_sec,
                    "use_fixed_yaw": use_fixed_yaw,
                    "fixed_yaw_deg": fixed_yaw_deg,
                    "yaw_kp": yaw_kp,
                    "yaw_ff_gain": yaw_ff_gain,
                    "large_yaw_error_deg_threshold": large_yaw_error_deg_threshold,
                    "large_yaw_kp_scale": large_yaw_kp_scale,
                    "large_yaw_ff_scale": large_yaw_ff_scale,
                    "max_yaw_velocity_deg": max_yaw_velocity_deg,
                    "large_yaw_velocity_deg": large_yaw_velocity_deg,
                    "enable_takeoff_height_guard": enable_takeoff_height_guard,
                    "guarded_takeoff_z_cm": guarded_takeoff_z_cm,
                    "enable_bootstrap_takeoff": enable_bootstrap_takeoff,
                    "bootstrap_takeoff_z_cm": bootstrap_takeoff_z_cm,
                    "enable_trajectory_switch_reset": enable_trajectory_switch_reset,
                    "publish_debug_topics": publish_debug_topics,
                    "enable_debug_logs": enable_debug_logs,
                },
            ],
        ),
    ])