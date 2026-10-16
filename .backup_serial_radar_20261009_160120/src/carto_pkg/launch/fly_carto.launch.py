"""cartographer 2D 建图（配合 bluesea2 网口雷达）。

雷达驱动由 run.sh 单独拉起：
    ros2 run bluesea2 bluesea2_node --ros-args --params-file .../udp_lidar.yaml
本 launch 只负责：robot_state_publisher(TF) + cartographer_node + occupancy_grid。

用法：
    ros2 launch carto_pkg fly_carto.launch.py
    ros2 launch carto_pkg fly_carto.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('carto_pkg')
    urdf_file = os.path.join(share, 'urdf', 'fly.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    rviz_config = os.path.join(share, 'rviz', 'cartographer.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_basename = LaunchConfiguration('config_basename')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        # 2026-10-08 事故修复：cartographer 曾在起飞瞬间抛 std::length_error 崩掉，
        # 而 launch 默认不会重启死掉的节点 -> 崩了就永久没位姿（还会被下游当成"新数据"）。
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            # 把跟踪位姿单独发布到 /tracked_pose（px4_ev_bridge 默认订阅它）。
            # 不设这个参数时只能靠 TF(map->odom->base_link) 拿位姿。
            'publish_tracked_pose': True,
        }],
        arguments=[
            '-configuration_directory', os.path.join(share, 'configuration_files'),
            '-configuration_basename', config_basename,
        ],
        remappings=[('scan', '/scan')],
    )

    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 rviz2（无显示器时留 false）'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_basename', default_value='fly.lua',
                              description='configuration_files 下的 lua 文件名'),
        TimerAction(period=0.5, actions=[robot_state_publisher]),
        TimerAction(period=2.0, actions=[cartographer, occupancy]),
        TimerAction(period=3.0, actions=[rviz]),
    ])
