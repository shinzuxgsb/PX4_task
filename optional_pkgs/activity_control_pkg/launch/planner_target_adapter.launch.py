from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    input_position_cmd_topic = LaunchConfiguration("input_position_cmd_topic")
    input_pose_topic = LaunchConfiguration("input_pose_topic")
    input_path_topic = LaunchConfiguration("input_path_topic")
    output_topic = LaunchConfiguration("output_topic")
    output_mode = LaunchConfiguration("output_mode")
    prefer_position_cmd_input = LaunchConfiguration("prefer_position_cmd_input")
    prefer_path_input = LaunchConfiguration("prefer_path_input")
    use_input_z = LaunchConfiguration("use_input_z")
    default_height_cm = LaunchConfiguration("default_height_cm")
    enable_sparse_sampling = LaunchConfiguration("enable_sparse_sampling")
    min_delta_xy_cm = LaunchConfiguration("min_delta_xy_cm")
    min_delta_z_cm = LaunchConfiguration("min_delta_z_cm")
    min_delta_yaw_deg = LaunchConfiguration("min_delta_yaw_deg")
    use_fixed_yaw = LaunchConfiguration("use_fixed_yaw")
    fixed_yaw_deg = LaunchConfiguration("fixed_yaw_deg")
    enable_adaptive_sampling = LaunchConfiguration("enable_adaptive_sampling")
    enable_trajectory_switch_reset = LaunchConfiguration("enable_trajectory_switch_reset")
    adaptive_scale_min = LaunchConfiguration("adaptive_scale_min")
    adaptive_scale_max = LaunchConfiguration("adaptive_scale_max")
    adaptive_turn_low_deg = LaunchConfiguration("adaptive_turn_low_deg")
    adaptive_turn_high_deg = LaunchConfiguration("adaptive_turn_high_deg")
    enable_takeoff_height_guard = LaunchConfiguration("enable_takeoff_height_guard")
    use_external_height_guard = LaunchConfiguration("use_external_height_guard")
    external_height_topic = LaunchConfiguration("external_height_topic")
    external_height_timeout_ms = LaunchConfiguration("external_height_timeout_ms")
    min_valid_target_z_cm = LaunchConfiguration("min_valid_target_z_cm")
    guarded_takeoff_z_cm = LaunchConfiguration("guarded_takeoff_z_cm")
    enable_bootstrap_takeoff_target = LaunchConfiguration("enable_bootstrap_takeoff_target")
    bootstrap_once = LaunchConfiguration("bootstrap_once")
    bootstrap_takeoff_height_cm = LaunchConfiguration("bootstrap_takeoff_height_cm")
    gate_control_enabled = LaunchConfiguration("gate_control_enabled")
    enable_nearest_waypoint_priority = LaunchConfiguration("enable_nearest_waypoint_priority")
    nearest_waypoint_forward_only = LaunchConfiguration("nearest_waypoint_forward_only")
    nearest_waypoint_switch_hysteresis_cm = LaunchConfiguration("nearest_waypoint_switch_hysteresis_cm")
    default_gate_count = LaunchConfiguration("default_gate_count")
    gate_trigger_topic = LaunchConfiguration("gate_trigger_topic")
    status_topic = LaunchConfiguration("status_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    max_queue_size = LaunchConfiguration("max_queue_size")
    gate_position_tolerance_cm = LaunchConfiguration("gate_position_tolerance_cm")
    gate_height_tolerance_cm = LaunchConfiguration("gate_height_tolerance_cm")
    gate_yaw_tolerance_deg = LaunchConfiguration("gate_yaw_tolerance_deg")
    gate_reach_hold_cycles = LaunchConfiguration("gate_reach_hold_cycles")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("activity_control_pkg"),
                "config",
                "planner_target_adapter.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "input_position_cmd_topic",
            default_value="/drone_0_planning/pos_cmd",
        ),
        DeclareLaunchArgument(
            "input_pose_topic",
            default_value="/ego_planner/next_target",
        ),
        DeclareLaunchArgument(
            "input_path_topic",
            default_value="/ego_planner/reference_path",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/target_position",
        ),
        DeclareLaunchArgument(
            "output_mode",
            default_value="position",
        ),
        DeclareLaunchArgument(
            "prefer_position_cmd_input",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "prefer_path_input",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_input_z",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "default_height_cm",
            default_value="120.0",
        ),
        DeclareLaunchArgument(
            "enable_sparse_sampling",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "min_delta_xy_cm",
            default_value="3.0",
        ),
        DeclareLaunchArgument(
            "min_delta_z_cm",
            default_value="2.0",
        ),
        DeclareLaunchArgument(
            "min_delta_yaw_deg",
            default_value="4.0",
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
            "enable_adaptive_sampling",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_trajectory_switch_reset",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "adaptive_scale_min",
            default_value="0.45",
        ),
        DeclareLaunchArgument(
            "adaptive_scale_max",
            default_value="1.6",
        ),
        DeclareLaunchArgument(
            "adaptive_turn_low_deg",
            default_value="3.0",
        ),
        DeclareLaunchArgument(
            "adaptive_turn_high_deg",
            default_value="20.0",
        ),
        DeclareLaunchArgument(
            "enable_takeoff_height_guard",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_external_height_guard",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "external_height_topic",
            default_value="/height",
        ),
        DeclareLaunchArgument(
            "external_height_timeout_ms",
            default_value="500",
        ),
        DeclareLaunchArgument(
            "min_valid_target_z_cm",
            default_value="30.0",
        ),
        DeclareLaunchArgument(
            "guarded_takeoff_z_cm",
            default_value="120.0",
        ),
        DeclareLaunchArgument(
            "enable_bootstrap_takeoff_target",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "bootstrap_once",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "bootstrap_takeoff_height_cm",
            default_value="120.0",
        ),
        DeclareLaunchArgument(
            "gate_control_enabled",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_nearest_waypoint_priority",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "nearest_waypoint_forward_only",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "nearest_waypoint_switch_hysteresis_cm",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "default_gate_count",
            default_value="3",
        ),
        DeclareLaunchArgument(
            "gate_trigger_topic",
            default_value="/waypoint_gate_trigger",
        ),
        DeclareLaunchArgument(
            "status_topic",
            default_value="/waypoint_status",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/fastlio2/lio_odom",
        ),
        DeclareLaunchArgument(
            "max_queue_size",
            default_value="200",
        ),
        DeclareLaunchArgument(
            "gate_position_tolerance_cm",
            default_value="8.0",
        ),
        DeclareLaunchArgument(
            "gate_height_tolerance_cm",
            default_value="6.0",
        ),
        DeclareLaunchArgument(
            "gate_yaw_tolerance_deg",
            default_value="10.0",
        ),
        DeclareLaunchArgument(
            "gate_reach_hold_cycles",
            default_value="2",
        ),
        Node(
            package="activity_control_pkg",
            executable="planner_target_adapter_node",
            name="planner_target_adapter",
            output="screen",
            parameters=[
                params_file,
                {
                    "input_position_cmd_topic": input_position_cmd_topic,
                    "input_pose_topic": input_pose_topic,
                    "input_path_topic": input_path_topic,
                    "output_topic": output_topic,
                    "output_mode": output_mode,
                    "prefer_position_cmd_input": prefer_position_cmd_input,
                    "prefer_path_input": prefer_path_input,
                    "use_input_z": use_input_z,
                    "default_height_cm": default_height_cm,
                    "enable_sparse_sampling": enable_sparse_sampling,
                    "min_delta_xy_cm": min_delta_xy_cm,
                    "min_delta_z_cm": min_delta_z_cm,
                    "min_delta_yaw_deg": min_delta_yaw_deg,
                    "use_fixed_yaw": use_fixed_yaw,
                    "fixed_yaw_deg": fixed_yaw_deg,
                    "enable_adaptive_sampling": enable_adaptive_sampling,
                    "enable_trajectory_switch_reset": enable_trajectory_switch_reset,
                    "adaptive_scale_min": adaptive_scale_min,
                    "adaptive_scale_max": adaptive_scale_max,
                    "adaptive_turn_low_deg": adaptive_turn_low_deg,
                    "adaptive_turn_high_deg": adaptive_turn_high_deg,
                    "enable_takeoff_height_guard": enable_takeoff_height_guard,
                    "use_external_height_guard": use_external_height_guard,
                    "external_height_topic": external_height_topic,
                    "external_height_timeout_ms": external_height_timeout_ms,
                    "min_valid_target_z_cm": min_valid_target_z_cm,
                    "guarded_takeoff_z_cm": guarded_takeoff_z_cm,
                    "enable_bootstrap_takeoff_target": enable_bootstrap_takeoff_target,
                    "bootstrap_once": bootstrap_once,
                    "bootstrap_takeoff_height_cm": bootstrap_takeoff_height_cm,
                    "gate_control_enabled": gate_control_enabled,
                    "enable_nearest_waypoint_priority": enable_nearest_waypoint_priority,
                    "nearest_waypoint_forward_only": nearest_waypoint_forward_only,
                    "nearest_waypoint_switch_hysteresis_cm": nearest_waypoint_switch_hysteresis_cm,
                    "default_gate_count": default_gate_count,
                    "gate_trigger_topic": gate_trigger_topic,
                    "status_topic": status_topic,
                    "odom_topic": odom_topic,
                    "max_queue_size": max_queue_size,
                    "gate_position_tolerance_cm": gate_position_tolerance_cm,
                    "gate_height_tolerance_cm": gate_height_tolerance_cm,
                    "gate_yaw_tolerance_deg": gate_yaw_tolerance_deg,
                    "gate_reach_hold_cycles": gate_reach_hold_cycles,
                },
            ],
        )
    ])
