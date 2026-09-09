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
    driver_launch = str(driver_share / "launch" / "driver.launch.py")

    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    delete_db = LaunchConfiguration("delete_db")
    cloud_backdate = LaunchConfiguration("cloud_backdate_sec")
    database_path = LaunchConfiguration("database_path")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    start_driver = LaunchConfiguration("start_driver")

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
            "cloud_backdate_sec": ParameterValue(cloud_backdate, value_type=float),
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

    # 新建地图时删除旧数据库，继续建图时保留数据库。
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[
            params_file,
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rtabmap_viz",
            default_value="true",
            description="是否启动 RTAB-Map 可视化界面。",
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
        driver,
        sync,
        preflight,
        rtabmap,
        rtabmap_keep_db,
        viz,
    ])
