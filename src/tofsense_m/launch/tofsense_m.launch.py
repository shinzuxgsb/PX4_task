"""TOFSense-M 激光定高器驱动。

    ros2 launch tofsense_m tofsense_m.launch.py
    ros2 launch tofsense_m tofsense_m.launch.py port:=/dev/ttyS3 range_mode:=min
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('baud', default_value='921600'),
        DeclareLaunchArgument('frame_id', default_value='tof_link'),
        DeclareLaunchArgument('range_mode', default_value='min'),
        # 2026-10-10 新增：抗抽风闸门（有效区数下限 / 信号下限 / 中值滤波窗口）
        DeclareLaunchArgument('min_valid_zones', default_value='4'),
        DeclareLaunchArgument('min_range', default_value='0.02'),
        DeclareLaunchArgument('min_signal', default_value='0'),
        DeclareLaunchArgument('median_window', default_value='5'),
        DeclareLaunchArgument('distance_scale', default_value='1000000.0'),
        Node(
            package='tofsense_m',
            executable='tofsense_m_node',
            name='tofsense_m',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'frame_id': LaunchConfiguration('frame_id'),
                'range_mode': LaunchConfiguration('range_mode'),
                'min_valid_zones': LaunchConfiguration('min_valid_zones'),
                'min_range': LaunchConfiguration('min_range'),
                'min_signal': LaunchConfiguration('min_signal'),
                'median_window': LaunchConfiguration('median_window'),
                'distance_scale': LaunchConfiguration('distance_scale'),
            }],
        ),
    ])
