from setuptools import find_packages, setup
from glob import glob
import os

package_name = "b2_navigation"

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
    description="宇树 B2 的 Nav2 定位、全局规划和局部避障基线",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "navigation_gate = b2_navigation.navigation_gate:main",
            "navigation_safety_monitor = "
            "b2_navigation.navigation_safety_monitor:main",
            "waypoint_recorder = b2_navigation.waypoint_recorder:main",
        ],
    },
)
