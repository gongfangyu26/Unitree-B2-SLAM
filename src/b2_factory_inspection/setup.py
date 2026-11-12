from setuptools import find_packages, setup

package_name = "b2_factory_inspection"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md", "OPERATION_GUIDE.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/mapping_3d.launch.py",
                "launch/autonomous_inspection.launch.py",
                "launch/corridor_patrol.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/patrol_waypoints.yaml",
                "config/inspection_targets.yaml",
                "config/photo_quality.yaml",
                "config/corridor_no_recovery.xml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@todo.todo",
    description="宇树 B2 自主巡检、图像质量复拍与安全联锁功能包",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "photo_quality_node = b2_factory_inspection.photo_quality_node:main",
            "mission_manager = b2_factory_inspection.mission_manager:main",
            "corridor_patrol = b2_factory_inspection.corridor_patrol:main",
            "aruco_target_locator = b2_factory_inspection.aruco_target_locator:main",
            "safety_monitor = b2_factory_inspection.safety_monitor:main",
            "jetson_board_test = b2_factory_inspection.jetson_board_test:main",
            "front_lidar_distance = b2_factory_inspection.front_lidar_distance:main",
        ],
    },
)
