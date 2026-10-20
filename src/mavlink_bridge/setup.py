from setuptools import setup

package_name = "mavlink_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/mavlink_bridge.launch.py", "launch/drone_stack.launch.py"],
        ),
        ("share/" + package_name + "/config", ["config/mavlink_bridge.yaml"]),
        ("share/" + package_name + "/config", ["config/tf_to_odom.yaml"]),
        ("share/" + package_name + "/config", ["config/tofsense.yaml"]),
        ("share/" + package_name + "/config", ["config/lidar_uart.yaml"]),
        ("share/" + package_name + "/config", ["config/lidar_net.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="PX4 MAVLink bridge replacing uart_to_stm32 (topic-compatible)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mavlink_bridge = mavlink_bridge.bridge_node:main",
            "tf_to_odom = mavlink_bridge.tf_to_odom:main",
            "tofsense_node = mavlink_bridge.tofsense_node:main",
        ],
    },
)
