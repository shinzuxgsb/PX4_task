"""启动 PX4 桥接节点。

前提：Micro-XRCE-DDS-Agent 已在运行，且 PX4 的 uxrce_dds_client 已连上。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('in_topic', default_value='/target_velocity'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('auto_offboard', default_value='false'),
        DeclareLaunchArgument('enu_to_ned', default_value='true'),
        DeclareLaunchArgument('yaw_sign', default_value='-1.0'),
        Node(
            package='px4_bridge',
            executable='px4_bridge_node',
            name='px4_bridge',
            output='screen',
            parameters=[{
                'in_topic': LaunchConfiguration('in_topic'),
                'auto_arm': LaunchConfiguration('auto_arm'),
                'auto_offboard': LaunchConfiguration('auto_offboard'),
                'enu_to_ned': LaunchConfiguration('enu_to_ned'),
                'yaw_sign': LaunchConfiguration('yaw_sign'),
            }],
        ),
    ])
