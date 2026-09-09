"""
实机 Nav2 二维导航基线.

驱动、点云转激光、定位、规划和局部避障。

参考:
  /home/nvidia/b2_code/navigation.launch.py (Go2 参考)
  https://ztl3106742440-hub.github.io/go2-tutorial/04-perception/12-nav2-baseline/
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    start_driver = LaunchConfiguration("start_driver")
    use_amcl = LaunchConfiguration("use_amcl")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cloud_backdate = LaunchConfiguration("cloud_backdate_sec")

    nav_share = Path(get_package_share_directory("b2_navigation"))
    slam_share = Path(get_package_share_directory("b2_slam"))
    driver_share = Path(get_package_share_directory("b2_driver_py"))

    nav2_params = str(nav_share / "config" / "nav2_params.yaml")
    scan_params = str(slam_share / "config" / "pointcloud_to_laserscan_params.yaml")
    driver_launch = str(driver_share / "launch" / "driver.launch.py")
    rviz_config = str(nav_share / "rviz" / "navigation.rviz")

    # 1) 底层驱动:提供 /odom、TF、/joint_states
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={"use_rviz": "false"}.items(),
        condition=IfCondition(start_driver),
    )

    # 2) 点云接收时间兼容；默认话题来自 2026-09-08 实机清单。
    timestamp_fix = Node(
        package="b2_slam",
        executable="pointcloud_timestamp_fix",
        name="pointcloud_timestamp_fix",
        parameters=[
            {
                "input_topic": cloud_topic,
                "output_topic": "/utlidar/cloud_fixed",
                # 仅用于当前无硬件同步阶段；取得 rosbag 后应按实测调整。
                "backdate_sec": ParameterValue(
                    cloud_backdate, value_type=float
                ),
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
        condition=IfCondition(use_amcl),
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
        condition=IfCondition(use_amcl),
    )

    # 使用 3D RTAB-Map 定位时，只管理二维静态地图，不启动 AMCL。
    map_only_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_only",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
        output="screen",
        condition=UnlessCondition(use_amcl),
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
                description="Nav2 使用的二维占据地图 YAML 文件。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 Nav2 的 RViz 界面。",
            ),
            DeclareLaunchArgument(
                "start_driver",
                default_value="true",
                description="是否同时启动 B2 Python 驱动和安全速度桥。",
            ),
            DeclareLaunchArgument(
                "use_amcl",
                default_value="true",
                description="true 使用二维 AMCL；false 时由外部 3D 定位发布 map 到 odom。",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/rslidar_points",
                description="用于二维障碍投影的 PointCloud2 输入话题。",
            ),
            DeclareLaunchArgument(
                "cloud_backdate_sec",
                default_value="0.05",
                description="点云接收时间回退量，仅用于临时 TF 兼容。",
            ),
            rviz,
            driver,
            timestamp_fix,
            pointcloud_to_scan,
            map_server,
            amcl,
            localization_lifecycle,
            map_only_lifecycle,
            delayed_navigation,
        ]
    )
