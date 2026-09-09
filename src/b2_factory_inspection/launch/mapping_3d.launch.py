"""启动适用于工厂坡道场景的三维建图流程."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """生成三维建图启动描述."""
    slam3d_share = Path(get_package_share_directory("b2_slam_3d"))
    slam3d_launch = str(slam3d_share / "launch" / "slam_3d.launch.py")

    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    delete_db = LaunchConfiguration("delete_db")
    cloud_backdate_sec = LaunchConfiguration("cloud_backdate_sec")
    database_path = LaunchConfiguration("database_path")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    start_driver = LaunchConfiguration("start_driver")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam3d_launch),
        launch_arguments={
            "use_rtabmap_viz": use_rtabmap_viz,
            "delete_db": delete_db,
            "cloud_backdate_sec": cloud_backdate_sec,
            "database_path": database_path,
            "cloud_topic": cloud_topic,
            "odom_topic": odom_topic,
            "lidar_frame": lidar_frame,
            "publish_lidar_alias": publish_lidar_alias,
            "start_driver": start_driver,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rtabmap_viz",
                default_value="true",
                description="是否显示 RTAB-Map 可视化窗口。",
            ),
            DeclareLaunchArgument(
                "delete_db",
                default_value="false",
                description="启动时是否删除指定的旧三维数据库。",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=str(Path.home() / ".ros" / "b2_rtabmap_3d.db"),
                description="RTAB-Map 三维数据库绝对路径。",
            ),
            DeclareLaunchArgument(
                "cloud_backdate_sec",
                default_value="0.05",
                description="点云接收时间回退量，仅用于临时 TF 兼容。",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/rslidar_points",
                description="三维 PointCloud2 输入话题。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="三维建图使用的里程计话题。",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="rslidar",
                description="点云消息中的实际 frame_id。",
            ),
            DeclareLaunchArgument(
                "publish_lidar_alias",
                default_value="true",
                description="是否发布雷达 frame_id 别名 TF。",
            ),
            DeclareLaunchArgument(
                "start_driver",
                default_value="true",
                description="是否同时启动 B2 Python 状态转换。",
            ),
            slam,
        ]
    )
