"""启动默认锁定的 B2 二维自主导航基础链."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_paths(context):
    """启动任何节点前检查地图和定位数据库路径."""
    map_path = Path(
        LaunchConfiguration("map").perform(context)
    ).expanduser()
    if not map_path.is_file():
        raise RuntimeError(f"二维地图 YAML 不存在：{map_path}")

    mode = LaunchConfiguration("localization_mode").perform(context)
    if mode == "rtabmap":
        database_path = Path(
            LaunchConfiguration("database_path").perform(context)
        ).expanduser()
        if not database_path.is_file():
            raise RuntimeError(
                f"RTAB-Map 定位数据库不存在：{database_path}"
            )
    return []


def generate_launch_description() -> LaunchDescription:
    """生成地图、定位、规划、避障和双重安全门启动描述."""
    map_yaml = LaunchConfiguration("map")
    database_path = LaunchConfiguration("database_path")
    initial_pose_topic = LaunchConfiguration("initial_pose_topic")
    use_rviz = LaunchConfiguration("use_rviz")
    start_driver = LaunchConfiguration("start_driver")
    start_safety_stack = LaunchConfiguration("start_safety_stack")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cloud_backdate = LaunchConfiguration("cloud_backdate_sec")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_linear_y = LaunchConfiguration("max_linear_y")
    max_angular_z = LaunchConfiguration("max_angular_z")
    min_effective_linear_x = LaunchConfiguration(
        "min_effective_linear_x"
    )
    min_effective_linear_y = LaunchConfiguration(
        "min_effective_linear_y"
    )
    min_effective_angular_z = LaunchConfiguration(
        "min_effective_angular_z"
    )
    max_armed_duration = LaunchConfiguration("max_armed_duration_sec")

    nav_share = Path(get_package_share_directory("b2_navigation"))
    slam_share = Path(get_package_share_directory("b2_slam"))
    slam3d_share = Path(get_package_share_directory("b2_slam_3d"))
    driver_share = Path(get_package_share_directory("b2_driver_py"))

    nav2_params = str(nav_share / "config" / "nav2_params.yaml")
    scan_params = str(
        slam_share / "config" / "pointcloud_to_laserscan_params.yaml"
    )
    rtabmap_params = str(slam3d_share / "config" / "rtabmap_3d.yaml")
    driver_launch = str(driver_share / "launch" / "driver.launch.py")
    rviz_config = LaunchConfiguration("rviz_config")
    start_waypoint_recorder = LaunchConfiguration(
        "start_waypoint_recorder"
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            "use_rviz": "false",
            "lidar_frame": lidar_frame,
            "publish_lidar_alias": publish_lidar_alias,
            "max_linear_x": max_linear_x,
            "max_linear_y": max_linear_y,
            "max_angular_z": max_angular_z,
            "min_effective_linear_x": min_effective_linear_x,
            "min_effective_linear_y": min_effective_linear_y,
            "min_effective_angular_z": min_effective_angular_z,
            "allow_reverse": "true",
            "allow_lateral": "true",
            "allow_combined_translation": "false",
        }.items(),
        condition=IfCondition(start_driver),
    )

    timestamp_fix = Node(
        package="b2_slam",
        executable="pointcloud_timestamp_fix",
        name="navigation_pointcloud_timestamp_fix",
        parameters=[{
            "input_topic": cloud_topic,
            "output_topic": "/b2_navigation/cloud_fixed",
            # RoboSense 实机发布端为 RELIABLE；导航链显式匹配，
            # 避免长时运行后仅保留 DDS endpoint 却不再收帧。
            "input_reliability": "reliable",
            "backdate_sec": ParameterValue(
                cloud_backdate, value_type=float
            ),
        }],
        output="screen",
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="navigation_pointcloud_to_laserscan",
        parameters=[scan_params],
        remappings=[
            ("cloud_in", "/b2_navigation/cloud_fixed"),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[nav2_params, {"yaml_filename": map_yaml}],
        output="screen",
    )

    map_preflight = Node(
        package="b2_slam_3d",
        executable="occupancy_grid_preflight",
        name="navigation_map_preflight",
        parameters=[{
            "map_topic": "/map",
            "expected_frame": "map",
            "stream_timeout_sec": 3.0,
        }],
        condition=IfCondition(start_safety_stack),
        output="screen",
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[nav2_params],
        # B2 厂商 QtServer 占用公共 /initialpose 并可发布零位姿。
        # 为导航定位使用独立话题，避免真实初始位姿被覆盖。
        remappings=[
            ("/initialpose", initial_pose_topic),
        ],
        condition=LaunchConfigurationEquals("localization_mode", "amcl"),
        output="screen",
    )

    amcl_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
        condition=LaunchConfigurationEquals("localization_mode", "amcl"),
        output="screen",
    )

    rtabmap_map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_only",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server"],
        }],
        condition=LaunchConfigurationEquals(
            "localization_mode", "rtabmap"
        ),
        output="screen",
    )

    rtabmap_sync = Node(
        package="b2_slam_3d",
        executable="sensor_time_sync",
        name="navigation_3d_sensor_time_sync",
        parameters=[{
            "cloud_in": cloud_topic,
            "cloud_out": "/b2_navigation/cloud_sync",
            "odom_in": "/odom",
            "odom_out": "/b2_navigation/odom_sync",
            "restamp_cloud": True,
            "restamp_odom": False,
            "cloud_backdate_sec": ParameterValue(
                cloud_backdate, value_type=float
            ),
        }],
        condition=LaunchConfigurationEquals(
            "localization_mode", "rtabmap"
        ),
        output="screen",
    )

    rtabmap_preflight = Node(
        package="b2_slam_3d",
        executable="slam_preflight",
        name="navigation_3d_preflight",
        parameters=[{
            "cloud_topic": "/b2_navigation/cloud_sync",
            "odom_topic": "/b2_navigation/odom_sync",
            "odom_frame": "odom",
            "base_frame": "base_link",
        }],
        condition=LaunchConfigurationEquals(
            "localization_mode", "rtabmap"
        ),
        output="screen",
    )

    rtabmap_localization = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        parameters=[
            rtabmap_params,
            {
                "database_path": ParameterValue(
                    database_path, value_type=str
                ),
                "Mem/IncrementalMemory": "false",
                "Mem/InitWMWithAllNodes": "true",
            },
        ],
        remappings=[
            ("scan_cloud", "/b2_navigation/cloud_sync"),
            ("odom", "/b2_navigation/odom_sync"),
        ],
        condition=LaunchConfigurationEquals(
            "localization_mode", "rtabmap"
        ),
        output="screen",
    )

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
        parameters=[nav2_params, {
            # Keep the planner's forward ceiling consistent with the bridge.
            # A higher requested ceiling still needs measured braking tests.
            "FollowPath.max_vel_x": ParameterValue(max_linear_x, value_type=float),
            "FollowPath.max_speed_xy": ParameterValue(max_linear_x, value_type=float),
        }],
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        output="screen",
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "/cmd_vel_nav")],
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
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "planner_server",
                "controller_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
        output="screen",
    )
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

    safety_monitor_amcl = Node(
        package="b2_navigation",
        executable="navigation_safety_monitor",
        name="b2_navigation_safety_monitor",
        parameters=[{
            "require_slam_preflight": False,
        }],
        condition=LaunchConfigurationEquals("localization_mode", "amcl"),
        output="screen",
    )
    safety_monitor_rtabmap = Node(
        package="b2_navigation",
        executable="navigation_safety_monitor",
        name="b2_navigation_safety_monitor",
        parameters=[{
            "require_slam_preflight": True,
        }],
        condition=LaunchConfigurationEquals(
            "localization_mode", "rtabmap"
        ),
        output="screen",
    )
    navigation_gate = Node(
        package="b2_navigation",
        executable="navigation_gate",
        name="b2_navigation_command_gate",
        parameters=[{
            "input_topic": "/cmd_vel_nav",
            "output_topic": "/cmd_vel",
            "max_armed_duration_sec": ParameterValue(
                max_armed_duration, value_type=float
            ),
            "supervisor_heartbeat_topic": ParameterValue(
                LaunchConfiguration("supervisor_heartbeat_topic"), value_type=str
            ),
        }],
        condition=IfCondition(start_safety_stack),
        output="screen",
    )
    waypoint_recorder = Node(
        package="b2_navigation",
        executable="waypoint_recorder",
        name="b2_waypoint_recorder",
        parameters=[{
            "output_file": str(
                Path.home()
                / "b2_maps"
                / "b2_patrol_waypoints.yaml"
            ),
            "loop_route": True,
        }],
        condition=IfCondition(start_waypoint_recorder),
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Nav2 使用的二维 OccupancyGrid 地图 YAML 文件。",
        ),
        DeclareLaunchArgument(
            "localization_mode",
            default_value="amcl",
            choices=["amcl", "rtabmap"],
            description="AMCL 平面定位或 RTAB-Map 三维数据库定位。",
        ),
        DeclareLaunchArgument(
            "initial_pose_topic",
            default_value="/b2_navigation/initialpose",
            description=(
                "AMCL 专用初始位姿话题，"
                "避免与 B2 QtServer 的 /initialpose 冲突。"
            ),
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value=str(Path.home() / ".ros" / "b2_rtabmap_3d.db"),
            description="localization_mode=rtabmap 时使用的既有数据库。",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(
                nav_share / "rviz" / "navigation_clean.rviz"
            ),
            description=(
                "RViz 配置；默认使用纯地图巡航视图，"
                "navigation.rviz 保留为完整调试视图。"
            ),
        ),
        DeclareLaunchArgument(
            "start_driver",
            default_value="true",
            description="已有驱动运行时必须设为 false，避免重复节点。",
        ),
        DeclareLaunchArgument(
            "start_safety_stack",
            default_value="true",
            description=(
                "启动地图检查和默认锁定的速度门；"
                "只读自主安全监视器始终启动。"
            ),
        ),
        DeclareLaunchArgument(
            "start_waypoint_recorder",
            default_value="true",
            description="启动只记录点击且不发送导航目标的巡航点工具。",
        ),
        DeclareLaunchArgument(
            "cloud_topic", default_value="/rslidar_points"
        ),
        DeclareLaunchArgument(
            "cloud_backdate_sec", default_value="0.05"
        ),
        DeclareLaunchArgument("lidar_frame", default_value="rslidar"),
        DeclareLaunchArgument(
            "publish_lidar_alias", default_value="true"
        ),
        DeclareLaunchArgument("max_linear_x", default_value="0.30"),
        DeclareLaunchArgument("max_linear_y", default_value="0.30"),
        DeclareLaunchArgument("max_angular_z", default_value="0.35"),
        DeclareLaunchArgument(
            "min_effective_linear_x",
            default_value="0.30",
            description="保留实测起步值；连续 Nav2 小速度响应需实机复测。",
        ),
        DeclareLaunchArgument(
            "min_effective_linear_y", default_value="0.30"
        ),
        DeclareLaunchArgument(
            "min_effective_angular_z",
            default_value="0.35",
            description="将 Nav2 非零小角速度提升到实测有效转向值。",
        ),
        DeclareLaunchArgument(
            "max_armed_duration_sec",
            default_value="60.0",
            description="人工点击导航安全门单次最长解锁时间。",
        ),
        DeclareLaunchArgument(
            "supervisor_heartbeat_topic",
            default_value="",
            description="可选任务存活心跳；设置后丢失心跳会锁定导航门。",
        ),
        OpaqueFunction(function=_validate_paths),
        rviz,
        driver,
        timestamp_fix,
        pointcloud_to_scan,
        map_server,
        map_preflight,
        amcl,
        amcl_lifecycle,
        rtabmap_map_lifecycle,
        rtabmap_sync,
        rtabmap_preflight,
        rtabmap_localization,
        safety_monitor_amcl,
        safety_monitor_rtabmap,
        navigation_gate,
        waypoint_recorder,
        delayed_navigation,
    ])
