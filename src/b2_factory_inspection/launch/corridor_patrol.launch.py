"""走廊地图定位、Nav2 和默认待命的有限圈数巡逻。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """复用唯一导航与安全链，不启动历史巡检任务管理器。"""
    nav_share = Path(get_package_share_directory("b2_navigation"))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav_share / "launch" / "navigation.launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "localization_mode": "amcl",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_driver": LaunchConfiguration("start_driver"),
            "start_safety_stack": "true",
            "start_waypoint_recorder": "false",
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "lidar_frame": LaunchConfiguration("lidar_frame"),
            "max_linear_x": LaunchConfiguration("max_linear_x"),
            "max_armed_duration_sec": LaunchConfiguration("mission_timeout_sec"),
            "supervisor_heartbeat_topic": "/patrol/supervisor_alive",
        }.items(),
    )
    recorder = Node(
        package="b2_navigation", executable="waypoint_recorder",
        parameters=[{
            "output_file": ParameterValue(LaunchConfiguration("waypoints_file"), value_type=str),
            "loop_route": True,
        }], output="screen",
    )
    patrol = Node(
        package="b2_factory_inspection", executable="corridor_patrol",
        parameters=[{
            "waypoints_file": ParameterValue(
                LaunchConfiguration("waypoints_file"), value_type=str
            ),
            "route_mode": LaunchConfiguration("route_mode"),
            "laps": ParameterValue(LaunchConfiguration("laps"), value_type=int),
            "goal_timeout_sec": ParameterValue(
                LaunchConfiguration("goal_timeout_sec"), value_type=float
            ),
            "mission_timeout_sec": ParameterValue(
                LaunchConfiguration("mission_timeout_sec"), value_type=float
            ),
            "code_revision": ParameterValue(LaunchConfiguration("code_revision"), value_type=str),
        }], output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("map", description="本次走廊的二维地图 YAML 绝对路径。"),
        DeclareLaunchArgument("waypoints_file", description="本次走廊路线 YAML，采点后由 recorder 保存。"),
        DeclareLaunchArgument("laps", default_value="1", description="闭环圈数 1~10，首轮 1 圈。"),
        DeclareLaunchArgument("route_mode", default_value="loop", choices=["loop", "once"]),
        DeclareLaunchArgument("goal_timeout_sec", default_value="60.0",
                              description="单个目标的最长时间；到点立即进入下一目标。"),
        DeclareLaunchArgument("mission_timeout_sec", default_value="600.0",
                              description="全部圈数总任务与导航门限时，默认/上限 600 秒。"),
        DeclareLaunchArgument("max_linear_x", default_value="0.60",
                              description="巡逻前进速度上限 m/s；0.60 尚待实机制动与导航验证。"),
        DeclareLaunchArgument("code_revision", default_value="unrecorded"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/rslidar_points"),
        DeclareLaunchArgument("lidar_frame", default_value="rslidar"),
        navigation, recorder, patrol,
    ])
