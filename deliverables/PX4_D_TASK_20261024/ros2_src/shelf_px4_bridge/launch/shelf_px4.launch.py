"""Start the D-theme payload nodes plus the PX4 Offboard lifecycle adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    shelf_share = get_package_share_directory("shelf_scan_pkg")
    shelf_launch = os.path.join(shelf_share, "launch", "shelf_scan.launch.py")

    args = [
        DeclareLaunchArgument(
            "flight_enabled",
            default_value="false",
            description="allow automatic arm after the two-command GO sequence",
        ),
        DeclareLaunchArgument("wait_for_go", default_value="true"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("code_mode", default_value="qr"),
        DeclareLaunchArgument("capture_fps", default_value="120.0"),
        DeclareLaunchArgument("detect_fps", default_value="30.0"),
        DeclareLaunchArgument("preview_fps", default_value="30.0"),
        DeclareLaunchArgument("camera_flip_ud", default_value="true"),
        DeclareLaunchArgument("camera_flip_lr", default_value="true"),
        DeclareLaunchArgument("preview_enabled", default_value="true"),
        DeclareLaunchArgument("laser_driver", default_value="gpio"),
        DeclareLaunchArgument("gpio_mode", default_value="gpiod"),
        DeclareLaunchArgument("gpio_chip", default_value="gpiochip1"),
        DeclareLaunchArgument("gpio_line", default_value="4"),
        DeclareLaunchArgument("laser_active_high", default_value="false"),
        DeclareLaunchArgument("start_gs_link", default_value="false"),
        DeclareLaunchArgument("ws_url", default_value="ws://127.0.0.1:8890/ws"),
        DeclareLaunchArgument("cruise_alt_cm", default_value="150.0"),
        DeclareLaunchArgument("require_battery", default_value="false"),
        DeclareLaunchArgument("minimum_battery_pct", default_value="25"),
        DeclareLaunchArgument("require_tof_ground", default_value="true"),
        DeclareLaunchArgument("tof_topic", default_value="/tofsense/range"),
        DeclareLaunchArgument(
            "ready_file", default_value="/tmp/shelf_preflight_ready"
        ),
        DeclareLaunchArgument(
            "go_file", default_value="/tmp/shelf_takeoff_go"
        ),
        DeclareLaunchArgument(
            "abort_file", default_value="/tmp/shelf_abort"
        ),
        DeclareLaunchArgument(
            "heartbeat_file", default_value="/tmp/shelf_px4_adapter.alive"
        ),
    ]

    mission_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(shelf_launch),
        launch_arguments={
            "camera_device": LaunchConfiguration("camera_device"),
            "code_mode": LaunchConfiguration("code_mode"),
            "capture_fps": LaunchConfiguration("capture_fps"),
            "detect_fps": LaunchConfiguration("detect_fps"),
            "preview_fps": LaunchConfiguration("preview_fps"),
            "camera_flip_ud": LaunchConfiguration("camera_flip_ud"),
            "camera_flip_lr": LaunchConfiguration("camera_flip_lr"),
            "preview_enabled": LaunchConfiguration("preview_enabled"),
            "laser_driver": LaunchConfiguration("laser_driver"),
            "gpio_mode": LaunchConfiguration("gpio_mode"),
            "gpio_chip": LaunchConfiguration("gpio_chip"),
            "gpio_line": LaunchConfiguration("gpio_line"),
            "laser_active_high": LaunchConfiguration("laser_active_high"),
            "start_gs_link": LaunchConfiguration("start_gs_link"),
            "ws_url": LaunchConfiguration("ws_url"),
            "cruise_alt_cm": LaunchConfiguration("cruise_alt_cm"),
        }.items(),
    )
    adapter = Node(
        package="shelf_px4_bridge",
        executable="shelf_px4_adapter",
        name="shelf_px4_adapter",
        output="screen",
        emulate_tty=True,
        on_exit=Shutdown(reason="shelf PX4 adapter finished"),
        parameters=[
            {
                "flight_enabled": ParameterValue(
                    LaunchConfiguration("flight_enabled"), value_type=bool
                ),
                "wait_for_go": ParameterValue(
                    LaunchConfiguration("wait_for_go"), value_type=bool
                ),
                "require_battery": ParameterValue(
                    LaunchConfiguration("require_battery"), value_type=bool
                ),
                "minimum_battery_pct": ParameterValue(
                    LaunchConfiguration("minimum_battery_pct"), value_type=int
                ),
                "require_tof_ground": ParameterValue(
                    LaunchConfiguration("require_tof_ground"), value_type=bool
                ),
                "tof_topic": LaunchConfiguration("tof_topic"),
                "ready_file": LaunchConfiguration("ready_file"),
                "go_file": LaunchConfiguration("go_file"),
                "abort_file": LaunchConfiguration("abort_file"),
                "heartbeat_file": LaunchConfiguration("heartbeat_file"),
            }
        ],
    )
    return LaunchDescription(args + [mission_nodes, adapter])
