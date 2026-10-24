"""D-theme shelf mission nodes localized for the PX4 MAVLink flight stack.

This launch intentionally does not start the lidar or Cartographer stack.
The already validated ``mavlink_ws`` service owns those processes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    shelf_share = get_package_share_directory("shelf_scan_pkg")
    pid_share = get_package_share_directory("pid_control_pkg")
    default_params = os.path.join(shelf_share, "config", "shelf_scan.yaml")
    default_field = os.path.join(shelf_share, "config", "field.json")
    default_pid = os.path.join(pid_share, "config", "px4_pid.yaml")

    arguments = [
        DeclareLaunchArgument("shelf_params_file", default_value=default_params),
        DeclareLaunchArgument("field_config_path", default_value=default_field),
        DeclareLaunchArgument("pid_config_file", default_value=default_pid),
        DeclareLaunchArgument("device_id", default_value="px4-d-01"),
        DeclareLaunchArgument("start_pid", default_value="true"),
        DeclareLaunchArgument("start_gs_link", default_value="false"),
        DeclareLaunchArgument("start_code_reader", default_value="true"),
        DeclareLaunchArgument("start_laser", default_value="true"),
        DeclareLaunchArgument("start_mission", default_value="true"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("code_mode", default_value="qr"),
        DeclareLaunchArgument("camera_flip_ud", default_value="true"),
        DeclareLaunchArgument("camera_flip_lr", default_value="true"),
        DeclareLaunchArgument("preview_enabled", default_value="true"),
        DeclareLaunchArgument("preview_port", default_value="8080"),
        DeclareLaunchArgument("laser_driver", default_value="gpio"),
        DeclareLaunchArgument("gpio_mode", default_value="gpiod"),
        DeclareLaunchArgument("gpio_chip", default_value="gpiochip1"),
        DeclareLaunchArgument("gpio_line", default_value="4"),
        DeclareLaunchArgument("laser_active_high", default_value="false"),
        DeclareLaunchArgument("laser_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("ws_url", default_value="ws://127.0.0.1:8890/ws"),
        DeclareLaunchArgument("ws_token", default_value=""),
        DeclareLaunchArgument("cruise_alt_cm", default_value="150.0"),
    ]

    params = LaunchConfiguration("shelf_params_file")
    field_params = {
        "field_config_path": LaunchConfiguration("field_config_path"),
        "device_id": LaunchConfiguration("device_id"),
        "cruise_alt_cm": ParameterValue(
            LaunchConfiguration("cruise_alt_cm"), value_type=float
        ),
        "auto_start_mode": "none",
        "start_on_takeoff": True,
        "takeoff_start_mode": "sweep",
        "publish_mission_complete": True,
    }

    pid = Node(
        package="pid_control_pkg",
        executable="position_pid_controller",
        name="position_pid_controller",
        output="screen",
        parameters=[LaunchConfiguration("pid_config_file")],
        condition=IfCondition(LaunchConfiguration("start_pid")),
    )
    laser = Node(
        package="shelf_scan_pkg",
        executable="shelf_laser_node",
        name="shelf_laser_node",
        output="screen",
        parameters=[
            params,
            {
                "driver": LaunchConfiguration("laser_driver"),
                "gpio_mode": LaunchConfiguration("gpio_mode"),
                "gpio_chip": LaunchConfiguration("gpio_chip"),
                "gpio_line": ParameterValue(
                    LaunchConfiguration("gpio_line"), value_type=int
                ),
                "gpiod_hold_sec": 0.10,
                "active_high": ParameterValue(
                    LaunchConfiguration("laser_active_high"), value_type=bool
                ),
                "serial_port": LaunchConfiguration("laser_serial_port"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_laser")),
    )
    code_reader = Node(
        package="shelf_scan_pkg",
        executable="shelf_code_reader_node",
        name="shelf_code_reader_node",
        output="screen",
        parameters=[
            params,
            {
                "camera_device": LaunchConfiguration("camera_device"),
                "code_mode": LaunchConfiguration("code_mode"),
                "camera_flip_ud": ParameterValue(
                    LaunchConfiguration("camera_flip_ud"), value_type=bool
                ),
                "camera_flip_lr": ParameterValue(
                    LaunchConfiguration("camera_flip_lr"), value_type=bool
                ),
                "preview_enabled": ParameterValue(
                    LaunchConfiguration("preview_enabled"), value_type=bool
                ),
                "preview_port": ParameterValue(
                    LaunchConfiguration("preview_port"), value_type=int
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_code_reader")),
    )
    gs_link = Node(
        package="shelf_scan_pkg",
        executable="shelf_gs_link_node",
        name="shelf_gs_link_node",
        output="screen",
        parameters=[
            params,
            {
                "device_id": LaunchConfiguration("device_id"),
                "ws_enabled": True,
                "ws_url": LaunchConfiguration("ws_url"),
                "ws_token": LaunchConfiguration("ws_token"),
                "serial_enabled": False,
                "udp_enabled": False,
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_gs_link")),
    )
    mission = Node(
        package="shelf_scan_pkg",
        executable="shelf_mission_node",
        name="shelf_mission_node",
        output="screen",
        parameters=[params, field_params],
        condition=IfCondition(LaunchConfiguration("start_mission")),
    )

    return LaunchDescription(
        arguments + [pid, laser, code_reader, gs_link, mission]
    )
