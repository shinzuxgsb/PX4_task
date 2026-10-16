from glob import glob
from setuptools import find_packages, setup

package_name = "p_to_p_px4"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # 2026-10-12 补记 6：照搬参考件的 final_position 入口
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yeyulun16-dot",
    maintainer_email="maintainer@example.com",
    description="PX4 uXRCE-DDS offboard bridge (DDS only) for this aircraft.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "px4_offboard_bridge = p_to_p_px4.offboard_bridge:main",
            # 2026-10-12 补记 6：照搬参考件的感知层（EV 由 cartographer_odometry_bridge 发）
            "cartographer_odometry_bridge = p_to_p_px4.odometry_bridge:main",
            # 2026-10-12 补记 7：照搬参考件的"定高链路"（TOF 在派上 -> MAVLink DISTANCE_SENSOR -> 飞控）
            "tof_mavlink_bridge = p_to_p_px4.tof_mavlink_bridge:main",
        ],
    },
)
