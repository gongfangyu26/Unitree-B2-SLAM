"""
启动宇树 B2 的 RTAB-Map 六自由度三维激光建图.

数据依次经过 Python 驱动、时间兼容节点、RTAB-Map，最终发布三维地图和地图坐标变换。
本启动文件不使用点云转二维激光或 slam_toolbox。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("b2_slam_3d"))
    driver_share = Path(get_package_share_directory("b2_driver_py"))

    params_file = str(pkg_share / "config" / "rtabmap_3d.yaml")
    default_rviz_config = str(pkg_share / "rviz" / "slam_3d.rviz")
    driver_launch = str(driver_share / "launch" / "driver.launch.py")

    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    delete_db = LaunchConfiguration("delete_db")
    cloud_backdate = LaunchConfiguration("cloud_backdate_sec")
    database_path = LaunchConfiguration("database_path")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    start_driver = LaunchConfiguration("start_driver")
    grid_topic = LaunchConfiguration("grid_topic")
    start_grid_preflight = LaunchConfiguration("start_grid_preflight")

    # Keep these as the final ROS parameter source. The historical Nav2
    # projection remains the default; cloud export can explicitly preserve Z.
    grid_parameters = {
        "Grid/3D": ParameterValue(
            LaunchConfiguration("grid_3d"), value_type=str
        ),
        "Grid/CellSize": "0.05",
        "Grid/MaxGroundHeight": "0.08",
        "Grid/MaxObstacleHeight": ParameterValue(
            LaunchConfiguration("grid_max_obstacle_height"), value_type=str
        ),
        "Grid/RayTracing": ParameterValue(
            LaunchConfiguration("grid_ray_tracing"), value_type=str
        ),
        "Grid/NoiseFilteringRadius": "0.12",
        "Grid/NoiseFilteringMinNeighbors": "5",
    }

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            "use_rviz": "false",
            "lidar_frame": lidar_frame,
            "publish_lidar_alias": publish_lidar_alias,
        }.items(),
        condition=IfCondition(start_driver),
    )

    sync = Node(
        package="b2_slam_3d",
        executable="sensor_time_sync",
        name="sensor_time_sync",
        output="screen",
        parameters=[{
            "cloud_in": cloud_topic,
            "cloud_out": "/b2_slam_3d/cloud_sync",
            "odom_in": odom_topic,
            "odom_out": "/b2_slam_3d/odom_sync",
            "restamp_cloud": True,
            "restamp_odom": False,
            "cloud_backdate_sec": ParameterValue(
                cloud_backdate, value_type=float
            ),
        }],
    )

    preflight = Node(
        package="b2_slam_3d",
        executable="slam_preflight",
        name="slam_preflight",
        output="screen",
        parameters=[{
            "cloud_topic": "/b2_slam_3d/cloud_sync",
            "odom_topic": "/b2_slam_3d/odom_sync",
            "odom_frame": "odom",
            "base_frame": "base_link",
        }],
    )

    grid_preflight = Node(
        package="b2_slam_3d",
        executable="occupancy_grid_preflight",
        name="mapping_grid_preflight",
        output="screen",
        parameters=[{
            "map_topic": grid_topic,
            "expected_frame": "map",
            # 2026-09-29 实机记录中地图更新约 0.2 Hz；10 秒可覆盖正常间隔，
            # 同时仍能在数据链真正停止时给出失败状态。
            "stream_timeout_sec": 10.0,
            "require_updates": True,
        }],
        condition=IfCondition(start_grid_preflight),
    )

    # 新建地图时删除旧数据库，继续建图时保留数据库。
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[
            params_file,
            grid_parameters,
            {"database_path": ParameterValue(database_path, value_type=str)},
        ],
        remappings=[
            ("scan_cloud", "/b2_slam_3d/cloud_sync"),
            ("odom", "/b2_slam_3d/odom_sync"),
        ],
        arguments=["--delete_db_on_start"],
        condition=IfCondition(delete_db),
    )

    # 不删除数据库时使用第二个条件节点，避免启动表达式的引号问题。
    rtabmap_keep_db = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[
            params_file,
            grid_parameters,
            {"database_path": ParameterValue(database_path, value_type=str)},
        ],
        remappings=[
            ("scan_cloud", "/b2_slam_3d/cloud_sync"),
            ("odom", "/b2_slam_3d/odom_sync"),
        ],
        condition=UnlessCondition(delete_db),
    )

    viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace="rtabmap",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            # 同步点云发布端为 Best Effort，和建图节点保持一致。
            "qos_scan": 2,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "approx_sync": True,
        }],
        remappings=[
            ("scan_cloud", "/b2_slam_3d/cloud_sync"),
            ("odom", "/b2_slam_3d/odom_sync"),
        ],
        condition=IfCondition(use_rtabmap_viz),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="b2_slam_3d_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rtabmap_viz",
            default_value="true",
            description="是否启动 RTAB-Map 可视化界面。",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="是否使用项目 RViz 配置显示三维点云。",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz 三维点云配置文件。",
        ),
        DeclareLaunchArgument(
            "delete_db",
            default_value="false",
            description="启动时是否删除 database_path 指向的旧数据库。",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value=str(Path.home() / ".ros" / "b2_rtabmap_3d.db"),
            description="保存或继续使用的 RTAB-Map 三维数据库绝对路径。",
        ),
        DeclareLaunchArgument(
            "cloud_backdate_sec",
            default_value="0.05",
            description="点云接收时间向前回退的秒数，仅用于 TF 兼容调试。",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/rslidar_points",
            description="PointCloud2 点云输入；现场也发现 /livox/lidar，须先确认类型。",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom",
            description="里程计输入；更换为板载 LIO-SAM 前须先核对坐标系和 TF。",
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
            "start_driver",
            default_value="true",
            description="是否启动 B2 状态转换和安全速度桥。",
        ),
        DeclareLaunchArgument(
            "grid_topic",
            default_value="/rtabmap/grid_prob_map",
            description="待检查和保存的 RTAB-Map 二维 OccupancyGrid 话题。",
        ),
        DeclareLaunchArgument(
            "grid_3d", default_value="false", choices=["true", "false"],
            description="true 保留地图点云高度；false 沿用 Nav2 二维投影配置。",
        ),
        DeclareLaunchArgument(
            "grid_max_obstacle_height", default_value="1.0",
            description="地图点云障碍高度上限（米），0 关闭此高度裁剪。",
        ),
        DeclareLaunchArgument(
            "grid_ray_tracing", default_value="true", choices=["true", "false"],
            description="体素射线追踪；导出三维表面点云时可设 false 减少计算与内存。",
        ),
        DeclareLaunchArgument(
            "start_grid_preflight",
            default_value="true",
            description="是否检查二维地图结构和已知/障碍区域比例。",
        ),
        driver,
        sync,
        preflight,
        grid_preflight,
        rtabmap,
        rtabmap_keep_db,
        viz,
        rviz,
    ])
