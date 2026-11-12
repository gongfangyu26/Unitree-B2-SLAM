from setuptools import find_packages, setup
from glob import glob
import os

package_name = "b2_slam_3d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@todo.todo",
    description="宇树 B2 的 RTAB-Map 六自由度三维激光建图与定位功能包",
    license="BSD-3-Clause",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "sensor_time_sync = b2_slam_3d.sensor_time_sync:main",
            "slam_preflight = b2_slam_3d.slam_preflight:main",
            "occupancy_grid_preflight = "
            "b2_slam_3d.occupancy_grid_preflight:main",
            "pointcloud_snapshot = b2_slam_3d.pointcloud_snapshot:main",
        ],
    },
)
