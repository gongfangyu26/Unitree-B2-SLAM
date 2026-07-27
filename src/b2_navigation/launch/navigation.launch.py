"""
实机 Nav2 2D 基线:
驱动 + 点云转 LaserScan + AMCL + Nav2 + RViz

参考:
  /home/nvidia/b2_code/navigation.launch.py (Go2 参考)
  https://ztl3106742440-hub.github.io/go2-tutorial/04-perception/12-nav2-baseline/
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")

    nav_share = Path(get_package_share_directory("b2_navigation"))
    slam_share = Path(get_package_share_directory("b2_slam"))
    driver_share = Path(get_package_share_directory("b2_driver"))

    nav2_params = str(nav_share / "config" / "nav2_params.yaml")
    scan_params = str(slam_share / "config" / "pointcloud_to_laserscan_params.yaml")
    driver_launch = str(driver_share / "launch" / "driver.launch.py")
    rviz_config = str(nav_share / "rviz" / "navigation.rviz")

    # 1) 底层驱动:提供 /odom、TF、/joint_states
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    # 2) 点云时间戳修复 (B2 雷达话题是 /rslidar_points)
    timestamp_fix = Node(
        package="b2_slam",
        executable="pointcloud_timestamp_fix",
        name="pointcloud_timestamp_fix",
        parameters=[
            {
                "input_topic": "/rslidar_points",
                "output_topic": "/utlidar/cloud_fixed",
                # /odom 只有 20 Hz,scan 时间戳回退 0.10s,AMCL 查 odom→base 才追得上
                "backdate_sec": 0.10,
            }
        ],
        output="screen",
    )

    # 3) 3D 点云 → 2D LaserScan
    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[scan_params],
        remappings=[
            ("cloud_in", "/utlidar/cloud_fixed"),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    # 4) 定位相关: map_server + amcl + 专属 lifecycle
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[nav2_params, {"yaml_filename": map_yaml}],
        output="screen",
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[nav2_params],
        output="screen",
    )

    localization_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
        output="screen",
    )

    # 5) 导航相关: planner + controller + behavior + bt + 专属 lifecycle
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[nav2_params],
        output="screen",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "/cmd_vel")],
        output="screen",
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[nav2_params],
        output="screen",
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[nav2_params],
        output="screen",
    )

    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
        output="screen",
    )

    # ⚠ 关键: 导航节点延迟 3 秒启动，确保 map_server + amcl 先进入 ACTIVE
    delayed_navigation = TimerAction(
        period=3.0,
        actions=[
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            navigation_lifecycle,
        ],
    )

    # 6) RViz: Fixed Frame=map
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Path to the chapter-11 saved map yaml file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch Nav2 RViz.",
            ),
            rviz,
            driver,
            timestamp_fix,
            pointcloud_to_scan,
            map_server,
            amcl,
            localization_lifecycle,
            delayed_navigation,
        ]
    )