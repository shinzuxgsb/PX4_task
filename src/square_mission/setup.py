from glob import glob

from setuptools import setup

package_name = "square_mission"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("test/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="PX4 offboard clockwise square mission",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "square_mission = square_mission.square_mission:main",
        ],
    },
)
