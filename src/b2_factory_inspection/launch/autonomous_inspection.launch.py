"""启动 3D 定位、Nav2 局部避障、目标定位和自主巡检任务."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    factory_share = Path(get_package_share_directory("b2_factory_inspection"))
    driver_share = Path(get_package_share_directory("b2_driver_py"))
    navigation_share = Path(get_package_share_directory("b2_navigation"))
    slam_share = Path(get_package_share_directory("b2_slam_3d"))

    map_file = LaunchConfiguration("map")
    database_path = LaunchConfiguration("database_path")
    waypoints_file = LaunchConfiguration("waypoints_file")
    targets_file = LaunchConfiguration("targets_file")
    image_topic = LaunchConfiguration("image_topic")
    compressed_topic = LaunchConfiguration("compressed_image_topic")
    use_compressed = LaunchConfiguration("use_compressed")
    use_rviz = LaunchConfiguration("use_rviz")
    use_aruco = LaunchConfiguration("use_aruco")
    photo_save_dir = LaunchConfiguration("photo_save_dir")
    settle_time = LaunchConfiguration("settle_time_sec")
    continue_after_failure = LaunchConfiguration("continue_after_failure")
    mission_autostart = LaunchConfiguration("mission_autostart")
    mission_autostart_delay = LaunchConfiguration("mission_autostart_delay_sec")
    cloud_backdate = LaunchConfiguration("cloud_backdate_sec")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    max_linear_y = LaunchConfiguration("max_linear_y")
    min_effective_linear_y = LaunchConfiguration("min_effective_linear_y")
    allow_reverse = LaunchConfiguration("allow_reverse")
    allow_lateral = LaunchConfiguration("allow_lateral")
    allow_combined_translation = LaunchConfiguration(
        "allow_combined_translation"
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver_share / "launch" / "driver.launch.py")),
        launch_arguments={
            "use_rviz": "false",
            "lidar_frame": lidar_frame,
            "publish_lidar_alias": publish_lidar_alias,
            "max_linear_y": max_linear_y,
            "min_effective_linear_y": min_effective_linear_y,
            "allow_reverse": allow_reverse,
            "allow_lateral": allow_lateral,
            "allow_combined_translation": allow_combined_translation,
        }.items(),
    )

    time_sync = Node(
        package="b2_slam_3d",
        executable="sensor_time_sync",
        name="sensor_time_sync",
        parameters=[
            {
                "cloud_in": cloud_topic,
                "cloud_out": "/b2_slam_3d/cloud_sync",
                "odom_in": odom_topic,
                "odom_out": "/b2_slam_3d/odom_sync",
                "restamp_cloud": True,
                "restamp_odom": False,
                "cloud_backdate_sec": ParameterValue(
                    cloud_backdate, value_type=float
                ),
            }
        ],
        output="screen",
    )

    slam_preflight = Node(
        package="b2_slam_3d",
        executable="slam_preflight",
        name="slam_preflight",
        parameters=[
            {
                "cloud_topic": "/b2_slam_3d/cloud_sync",
                "odom_topic": "/b2_slam_3d/odom_sync",
                "odom_frame": "odom",
                "base_frame": "base_link",
            }
        ],
        output="screen",
    )

    rtabmap_localization = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        parameters=[
            str(slam_share / "config" / "rtabmap_3d.yaml"),
            {
                "database_path": ParameterValue(database_path, value_type=str),
                "Mem/IncrementalMemory": "false",
                "Mem/InitWMWithAllNodes": "true",
            },
        ],
        remappings=[
            ("scan_cloud", "/b2_slam_3d/cloud_sync"),
            ("odom", "/b2_slam_3d/odom_sync"),
        ],
        output="screen",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "navigation.launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_rviz": use_rviz,
            "start_driver": "false",
            "use_amcl": "false",
            "cloud_topic": cloud_topic,
            "cloud_backdate_sec": cloud_backdate,
        }.items(),
    )

    target_locator = Node(
        package="b2_factory_inspection",
        executable="aruco_target_locator",
        parameters=[
            {
                "targets_file": targets_file,
                "image_topic": image_topic,
                "compressed_image_topic": compressed_topic,
                "use_compressed": ParameterValue(use_compressed, value_type=bool),
            }
        ],
        condition=IfCondition(use_aruco),
        output="screen",
    )

    photo_quality = Node(
        package="b2_factory_inspection",
        executable="photo_quality_node",
        parameters=[
            str(factory_share / "config" / "photo_quality.yaml"),
            {
                "image_topic": image_topic,
                "compressed_image_topic": compressed_topic,
                "use_compressed": ParameterValue(use_compressed, value_type=bool),
                "save_dir": ParameterValue(photo_save_dir, value_type=str),
            }
        ],
        output="screen",
    )

    safety_monitor = Node(
        package="b2_factory_inspection",
        executable="safety_monitor",
        parameters=[
            {
                "robot_state_topic": "/b2/state_ok",
                "scan_topic": "/scan",
                "map_frame": "map",
                "base_frame": "base_link",
                "require_scan": True,
                "require_localization": True,
                "require_slam_preflight": True,
            }
        ],
        output="screen",
    )

    mission = Node(
        package="b2_factory_inspection",
        executable="mission_manager",
        parameters=[
            {
                "waypoints_file": waypoints_file,
                "settle_time_sec": ParameterValue(settle_time, value_type=float),
                "continue_after_inspection_failure": ParameterValue(
                    continue_after_failure, value_type=bool
                ),
                "autostart": ParameterValue(mission_autostart, value_type=bool),
                "autostart_delay_sec": ParameterValue(
                    mission_autostart_delay, value_type=float
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Nav2 使用的二维投影地图 YAML。"),
            DeclareLaunchArgument("database_path", description="RTAB-Map 三维数据库路径。"),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=str(factory_share / "config" / "patrol_waypoints.yaml"),
                description="巡检路线和候选观测位姿 YAML 文件。",
            ),
            DeclareLaunchArgument(
                "targets_file",
                default_value=str(factory_share / "config" / "inspection_targets.yaml"),
                description="户外验证用 ArUco 目标区域配置。",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="未压缩相机图像话题。",
            ),
            DeclareLaunchArgument(
                "compressed_image_topic",
                default_value="/camera/image_raw/compressed",
                description="压缩相机图像话题。",
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="false",
                description="是否使用压缩图像输入。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="是否在本机启动 RViz；Jetson 无显示部署时保持 false。",
            ),
            DeclareLaunchArgument(
                "use_aruco",
                default_value="true",
                description="是否启动户外功能验证用 ArUco 目标定位器。",
            ),
            DeclareLaunchArgument(
                "photo_save_dir",
                default_value="~/.ros/b2_inspection_photos",
                description="照片和同名质量 JSON 报告保存目录。",
            ),
            DeclareLaunchArgument(
                "settle_time_sec",
                default_value="1.5",
                description="到达观测位姿后静止等待再拍照的秒数。",
            ),
            DeclareLaunchArgument(
                "continue_after_failure",
                default_value="false",
                description="所有候选均失败后是否继续；实机默认 false 并安全暂停。",
            ),
            DeclareLaunchArgument(
                "mission_autostart",
                default_value="false",
                description="安全条件齐备后是否由 Jetson 自动开始巡检。",
            ),
            DeclareLaunchArgument(
                "mission_autostart_delay_sec",
                default_value="10.0",
                description="Jetson 自动开始前的最少等待时间。",
            ),
            DeclareLaunchArgument(
                "cloud_backdate_sec",
                default_value="0.05",
                description="点云接收时间回退量，仅用于临时 TF 兼容调试。",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/rslidar_points",
                description="三维建图和二维避障共用的 PointCloud2 输入。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="RTAB-Map 使用的里程计输入。",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="rslidar",
                description="点云消息中的实际 frame_id。",
            ),
            DeclareLaunchArgument(
                "publish_lidar_alias",
                default_value="true",
                description="是否发布 lidar_link 到点云 frame_id 的别名 TF。",
            ),
            DeclareLaunchArgument(
                "allow_reverse",
                default_value="true",
                description="后退已完成单轴实机验收；控制锁和安全联锁仍默认生效。",
            ),
            DeclareLaunchArgument(
                "max_linear_y",
                default_value="0.30",
                description="2026-09-09 实测有效横移速度上限。",
            ),
            DeclareLaunchArgument(
                "min_effective_linear_y",
                default_value="0.30",
                description="2026-09-09 实测最低明确横移速度。",
            ),
            DeclareLaunchArgument(
                "allow_lateral",
                default_value="true",
                description="左右横移已完成单轴实机验收；任务仍默认不自动启动。",
            ),
            DeclareLaunchArgument(
                "allow_combined_translation",
                default_value="false",
                description="组合平移尚未实机验收，默认阻止 x/y 同时非零。",
            ),
            driver,
            time_sync,
            slam_preflight,
            rtabmap_localization,
            navigation,
            target_locator,
            photo_quality,
            safety_monitor,
            mission,
        ]
    )
