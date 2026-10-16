from glob import glob
from setuptools import find_packages, setup


package_name = "p_to_p_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yeyulun16-dot",
    maintainer_email="maintainer@example.com",
    description="Configurable A-to-B UAV mission coordinator for ROS 2 Humble.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "p_to_p_mission_node = p_to_p_mission.mission_node:main",
        ],
    },
)
