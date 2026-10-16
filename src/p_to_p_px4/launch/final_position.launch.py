"""照搬参考件 onboard_ws/p_to_p_px4/launch/final_position.launch.py 的入口（2026-10-12 补记 6）。

差别只有三处硬件适配（用户要求"引脚配置按我们原先的来"）：
  · serial_device = /dev/ttyS6、baud_rate = 1500000（参考件默认 2000000）
  · 雷达参数文件 = bluesea2/params/uart_lidar.yaml（串口款 LDS-50C-R，/dev/ttyS4）
  · 建图部分用我们的 carto_pkg（参考件是 my_carto_pkg）；SLAM 配置用 amphi.lua
    —— 那份文件两边逐字节相同（md5 c3a83c4169…），也正是参考件飞行时用的配置。

⚠️ 本 launch 会**自己拉起整条链**（agent + 雷达 + cartographer + 任务 + 两个桥），
   所以启之前必须先停掉已有链路，否则会和 px4-ev-chain / run.sh 抢资源、并出现
   **两个 /fmu/in/vehicle_visual_odometry 发布者**（项目红线）：

       sudo systemctl stop px4-ev-chain          # 先停现有链路
       bash ~/px4_ws/run.sh stop
       ros2 launch p_to_p_px4 final_position.launch.py            # 默认就是真跑（bench_only=false）
       ros2 launch p_to_p_px4 final_position.launch.py bench_only:=true   # 只预演

   还要先把飞控参数按参考件改好（见 docs/CHANGELOG.md 补记 6 的"飞控侧"）：
   EKF2_EV_CTRL 关掉垂直位置位、EKF2_HGT_REF 指向飞控测距/气压计。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    px4_share = get_package_share_directory("p_to_p_px4")
    mission_share = get_package_share_directory("p_to_p_mission")
    mapping_share = get_package_share_directory("carto_pkg")
    lidar_share = get_package_share_directory("bluesea2")

    px4_params = os.path.join(px4_share, "config", "px4.yaml")
    mission_params = os.path.join(mission_share, "config", "p_to_p.yaml")

    start_agent = LaunchConfiguration("start_agent")
    use_rviz = LaunchConfiguration("use_rviz")
    serial_device = LaunchConfiguration("serial_device")
    baud_rate = LaunchConfiguration("baud_rate")
    agent_executable = LaunchConfiguration("agent_executable")
    lidar_params = LaunchConfiguration("lidar_params_file")
    bench = ParameterValue(LaunchConfiguration("bench_only"), value_type=bool)

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_share, "launch", "fly_carto.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "config_basename": LaunchConfiguration("slam_config_basename"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_agent", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyS6"),
        DeclareLaunchArgument("baud_rate", default_value="1500000"),
        DeclareLaunchArgument("agent_executable", default_value="MicroXRCEAgent"),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=os.path.join(lidar_share, "params", "uart_lidar.yaml"),
        ),
        # 2026-10-14：SLAM 配置从 amphi.lua（纯激光）改成 fly.lua（use_imu_data=true）
        # —— 2D 单线激光 + 纯扫描匹配在起降（垂直运动）时会把"切面变化"当成水平运动，
        #    表现为起飞漂移（CHANGELOG 续6 记的 ~24 cm 漂就是这一类）。fly.lua 用飞控 IMU
        #    给扫描匹配一个旋转先验；飞控没上电时 /imu 无数据，cartographer 退回纯激光。
        DeclareLaunchArgument("slam_config_basename", default_value="fly.lua"),
        DeclareLaunchArgument("start_imu_bridge", default_value="true",
                              description="是否起 px4_imu_bridge（/fmu/out/sensor_combined → /imu，"
                                          "配合 fly.lua 的 TRAJECTORY_BUILDER_2D.use_imu_data）"),
        DeclareLaunchArgument("bench_only", default_value="false",
                              description="true=只预演（不发任何 /fmu/in 控制消息）"),
        DeclareLaunchArgument("start_tof_bridge", default_value="false",
                              description="参考件的定高链路（TOF ttyS3 → MAVLink → ttyS1）。"
                                          "本机不接那根线 ⇒ 默认 false；高度改由 tof/"
                                          "odometry_bridge 的 use_height 走 EV（见 px4.yaml）"),
        DeclareLaunchArgument("start_tofsense", default_value="true",
                              description="是否起本机定高器 tofsense_m（给 EV 提供高度，本机方案需要）"),
        DeclareLaunchArgument("start_mission", default_value="true",
                              description="是否起参考件任务状态机 p_to_p_mission。"
                                          "只打算用遥控器定点飞时设 false（避免误碰 Offboard 被任务接管）"),
        DeclareLaunchArgument("start_offboard", default_value="true",
                              description="是否起 px4_offboard_bridge。设 false 则完全不会发 /fmu/in/offboard_*"),

        # 0) 定高链路（照参考件：TOF 在派上 /dev/ttyS3 @921600 → MAVLink DISTANCE_SENSOR → /dev/ttyS1 @57600）
        #    ⚠️ /dev/ttyS3 只能被一个进程读：起它之前必须停掉 px4_ws 里的 tofsense_m 节点
        #    （run_p2p.sh 的 start 分支会连同旧链一起清掉）。
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("start_tof_bridge")),
            cmd=["ros2", "run", "p_to_p_px4", "tof_mavlink_bridge"],
            output="screen",
        ),

        # 0b) 本机定高器（2026-10-13：保留我们的 EV 高度 —— 与上面的 tof_mavlink_bridge 二选一）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("tofsense_m"),
                             "launch", "tofsense_m.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("start_tofsense")),
        ),

        # 1) Micro-XRCE-DDS Agent（ttyS6 @ 1500000 —— 我们的飞控就是这么配的）
        ExecuteProcess(
            condition=IfCondition(start_agent),
            cmd=[agent_executable, "serial", "--dev", serial_device, "-b", baud_rate],
            output="screen",
        ),

        # 2) 雷达驱动（照参考件由 launch 负责；我们 carto_pkg 的 launch 不含雷达）
        Node(
            # 2026-10-13：加 respawn —— 雷达驱动万一挂了不至于把整链带走
            respawn=True,
            respawn_delay=2.0,
            package="bluesea2",
            executable="bluesea2_node",
            name="bluesea_node",
            namespace="/",
            output="screen",
            emulate_tty=True,
            parameters=[lidar_params],
        ),

        # 2b) 飞控 IMU → /imu（喂给 cartographer 的 fly.lua 做旋转预测）
        #     2026-10-14：起降漂移复盘后启用（2D 单线激光纯扫描匹配在垂直运动时会漂）。
        #     飞控没上电时 /imu 无数据，cartographer 会退回纯激光（见 fly.lua 注释），
        #     所以这里无条件拉起、不等待；外面套一层 while 让桥挂了能自己起来。
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("start_imu_bridge")),
            cmd=["bash", "-c",
                 "while true; do python3 $HOME/px4_ws/px4_imu_bridge.py; "
                 "echo '[imu_bridge] exited, respawn in 2s'; sleep 2; done"],
            output="screen",
        ),

        # 3) 建图：robot_state_publisher + cartographer + occupancy（carto_pkg）
        mapping,

        # 4) 任务状态机（遥控器定点飞时可用 start_mission:=false 关掉）
        Node(
            condition=IfCondition(LaunchConfiguration("start_mission")),
            package="p_to_p_mission",
            executable="p_to_p_mission_node",
            name="p_to_p_mission",
            output="screen",
            parameters=[mission_params],
        ),

        # 5) 感知层 EV：odom→base_link → /fmu/in/vehicle_visual_odometry（参考件原样）
        Node(
            # 2026-10-13：两处本机改动（冷启动鲁棒性，见 docs/CHANGELOG.md 补记 12）
            #   · startup_timeout_sec 45 → 120：雷达握手要 25~40 s，45 s 会踩线自杀
            #     （实测 RuntimeError: Lidar odometry unhealthy for 45.0s: SCAN_STALE）
            #   · respawn=True：万一还是死了，launch 会把它拉回来（否则 EV 断了没人管）
            respawn=True,
            respawn_delay=3.0,
            package="p_to_p_px4",
            executable="cartographer_odometry_bridge",
            name="cartographer_odometry_bridge",
            output="screen",
            parameters=[px4_params, {"enabled": True,
                                     "startup_timeout_sec": 120.0}],
        ),

        # 6) Offboard 桥（start_offboard:=false 时完全不起，/fmu/in/offboard_* 一条都不会发）
        Node(
            condition=IfCondition(LaunchConfiguration("start_offboard")),
            package="p_to_p_px4",
            executable="px4_offboard_bridge",
            name="px4_offboard_bridge",
            output="screen",
            parameters=[
                px4_params,
                {
                    "bench_only": bench,
                    "control_mode": "position",
                    "allow_land_command": True,
                },
            ],
        ),
    ])
