"""启动二维 SLAM Toolbox 备用建图流程；论文主流程使用三维 RTAB-Map."""

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
    """生成可配置点云输入的二维建图启动描述."""
    slam_share = Path(get_package_share_directory("b2_slam"))
    driver_share = Path(get_package_share_directory("b2_driver_py"))

    cloud_topic = LaunchConfiguration("cloud_topic")
    cloud_backdate = LaunchConfiguration("cloud_backdate_sec")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    use_rviz = LaunchConfiguration("use_rviz")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(driver_share / "launch" / "driver.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "lidar_frame": lidar_frame,
            "publish_lidar_alias": publish_lidar_alias,
        }.items(),
    )

    timestamp_fix = Node(
        package="b2_slam",
        executable="pointcloud_timestamp_fix",
        name="pointcloud_timestamp_fix",
        parameters=[{
            "input_topic": cloud_topic,
            "output_topic": "/utlidar/cloud_fixed",
            "backdate_sec": ParameterValue(cloud_backdate, value_type=float),
        }],
        output="screen",
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[
            str(slam_share / "config" / "pointcloud_to_laserscan_params.yaml")
        ],
        remappings=[
            ("cloud_in", "/utlidar/cloud_fixed"),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[str(slam_share / "config" / "slam_toolbox_params.yaml")],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(slam_share / "config" / "slam.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/rslidar_points",
            description="二维备用建图使用的 PointCloud2 输入。",
        ),
        DeclareLaunchArgument(
            "cloud_backdate_sec",
            default_value="0.05",
            description="点云接收时间回退量，仅用于临时 TF 兼容。",
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
            "use_rviz",
            default_value="true",
            description="是否启动二维建图 RViz。",
        ),
        driver,
        timestamp_fix,
        pointcloud_to_scan,
        slam,
        rviz,
    ])
