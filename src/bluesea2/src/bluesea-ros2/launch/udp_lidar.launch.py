"""bluesea2 网口雷达驱动（UDP）启动文件。

自带的 `udp_lidar.launch` 是 Python 内容但扩展名不对，ros2 launch 认不出来，
所以这里补一份标准命名的 `.launch.py`（内容等价，并把 LifecycleNode 改成 Node）。

    ros2 launch bluesea2 udp_lidar.launch.py
    ros2 launch bluesea2 udp_lidar.launch.py params_file:=/path/to/udp_lidar.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('bluesea2')

    params_file = LaunchConfiguration('params_file')
    declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'udp_lidar.yaml'),
        description='雷达参数文件路径',
    )

    driver = Node(
        package='bluesea2',
        executable='bluesea2_node',
        name='bluesea_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([declare, driver])
