"""Launch 3D-SLAM fixed-route patrol with short-range obstacle avoidance."""

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
    slam3d_share = Path(get_package_share_directory("b2_slam_3d"))

    waypoints_file = LaunchConfiguration("waypoints_file")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    loop = LaunchConfiguration("loop")
    database_path = LaunchConfiguration("database_path")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver_share / "launch" / "driver.launch.py")),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    sync = Node(
        package="b2_slam_3d",
        executable="sensor_time_sync",
        name="factory_3d_sensor_time_sync",
        parameters=[
            {
                "cloud_in": "/rslidar_points",
                "cloud_out": "/b2_slam_3d/cloud_sync",
                "odom_in": "/odom",
                "odom_out": "/b2_slam_3d/odom_sync",
                "cloud_backdate_sec": 0.05,
                "odom_backdate_sec": 0.05,
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
            str(slam3d_share / "config" / "rtabmap_3d.yaml"),
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

    timestamp_fix = Node(
        package="b2_factory_inspection",
        executable="factory_pointcloud_timestamp_fix",
        name="patrol_pointcloud_timestamp_fix",
        parameters=[
            {
                "input_topic": "/rslidar_points",
                "output_topic": "/factory_patrol/cloud_fixed",
                "backdate_sec": 0.10,
            }
        ],
        output="screen",
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="factory_patrol_pointcloud_to_laserscan",
        parameters=[str(factory_share / "config" / "factory_pointcloud_to_laserscan.yaml")],
        remappings=[
            ("cloud_in", "/factory_patrol/cloud_fixed"),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    patrol = Node(
        package="b2_factory_inspection",
        executable="fixed_route_3d_patrol",
        name="fixed_route_3d_patrol",
        parameters=[
            {
                "waypoints_file": waypoints_file,
                "global_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "loop": ParameterValue(loop, value_type=bool),
                "front_stop_distance": 0.85,
                "front_clear_distance": 1.25,
                "side_clear_distance": 0.75,
                "max_linear_speed": 0.28,
                "max_lateral_speed": 0.22,
                "max_angular_speed": 0.55,
            }
        ],
        output="screen",
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace="rtabmap",
        parameters=[
            {
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "subscribe_scan_cloud": True,
                "subscribe_scan": False,
                "qos_scan": 2,
                "subscribe_rgb": False,
                "subscribe_depth": False,
                "approx_sync": True,
            }
        ],
        remappings=[
            ("scan_cloud", "/b2_slam_3d/cloud_sync"),
            ("odom", "/b2_slam_3d/odom_sync"),
        ],
        output="screen",
        condition=IfCondition(use_rtabmap_viz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=str(factory_share / "config" / "patrol_waypoints.yaml"),
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/home/unitree/b2_ws/src/maps_3d/outdoor_test_rtabmap.db",
                description="RTAB-Map 3D database built during mapping.",
            ),
            DeclareLaunchArgument("use_rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("loop", default_value="true"),
            driver,
            sync,
            rtabmap_localization,
            timestamp_fix,
            pointcloud_to_scan,
            patrol,
            rtabmap_viz,
        ]
    )
