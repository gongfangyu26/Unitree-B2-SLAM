"""启动 B2 Python 状态转换、安全速度桥和机器人模型."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    b2_description_share = get_package_share_directory("b2_description")
    b2_driver_share = get_package_share_directory("b2_driver_py")

    use_rviz = LaunchConfiguration("use_rviz")
    lidar_frame = LaunchConfiguration("lidar_frame")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_linear_y = LaunchConfiguration("max_linear_y")
    max_angular_z = LaunchConfiguration("max_angular_z")
    min_effective_linear_x = LaunchConfiguration("min_effective_linear_x")
    min_effective_linear_y = LaunchConfiguration("min_effective_linear_y")
    min_effective_angular_z = LaunchConfiguration(
        "min_effective_angular_z"
    )
    allow_reverse = LaunchConfiguration("allow_reverse")
    allow_lateral = LaunchConfiguration("allow_lateral")
    allow_combined_translation = LaunchConfiguration(
        "allow_combined_translation"
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                b2_description_share,
                "launch",
                "display.launch.py",
            )
        ),
        launch_arguments={"use_joint_state_publisher": "false"}.items(),
    )

    # 点云实际 frame_id 为 rslidar 时，先把它映射到 URDF 的雷达坐标系。
    # 这里的单位变换只表示两个名字对应同一传感器坐标系，不代表雷达外参为零。
    lidar_frame_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_frame_alias",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "lidar_link", lidar_frame,
        ],
        condition=IfCondition(publish_lidar_alias),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(b2_driver_share, "rviz", "display.rviz"),
        ],
        condition=IfCondition(use_rviz),
    )

    safe_twist_bridge = Node(
        package="b2_twist_bridge_py",
        executable="twist_bridge",
        parameters=[{
            "start_enabled": False,
            "max_linear_x": ParameterValue(max_linear_x, value_type=float),
            "max_linear_y": ParameterValue(max_linear_y, value_type=float),
            "max_angular_z": ParameterValue(max_angular_z, value_type=float),
            "min_effective_linear_x": ParameterValue(
                min_effective_linear_x, value_type=float
            ),
            "min_effective_linear_y": ParameterValue(
                min_effective_linear_y, value_type=float
            ),
            "min_effective_angular_z": ParameterValue(
                min_effective_angular_z, value_type=float
            ),
            "allow_reverse": ParameterValue(allow_reverse, value_type=bool),
            "allow_lateral": ParameterValue(allow_lateral, value_type=bool),
            "allow_combined_translation": ParameterValue(
                allow_combined_translation, value_type=bool
            ),
        }],
        output="screen",
    )

    driver = Node(
        package="b2_driver_py",
        executable="driver",
        parameters=[
            os.path.join(b2_driver_share, "params", "driver.yaml")
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz。",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="rslidar",
                description="2026-09-09 实机确认的 RoboSense 点云 frame_id。",
            ),
            DeclareLaunchArgument(
                "publish_lidar_alias",
                default_value="true",
                description="是否发布 lidar_link 到实际点云 frame_id 的别名 TF。",
            ),
            DeclareLaunchArgument(
                "max_linear_x",
                default_value="0.30",
                description="B2 实测前进速度上限。",
            ),
            DeclareLaunchArgument(
                "max_linear_y",
                default_value="0.30",
                description="B2 左右横移 3/3 实测有效速度上限。",
            ),
            DeclareLaunchArgument(
                "max_angular_z",
                default_value="0.35",
                description="B2 实测转向速度上限。",
            ),
            DeclareLaunchArgument(
                "min_effective_linear_x",
                default_value="0.30",
                description="B2 实测能够可靠形成步态的最低前进速度。",
            ),
            DeclareLaunchArgument(
                "min_effective_linear_y",
                default_value="0.30",
                description="B2 实测能够产生明确位移的横移速度。",
            ),
            DeclareLaunchArgument(
                "min_effective_angular_z",
                default_value="0.35",
                description="B2 实测能够产生明确转向的角速度。",
            ),
            DeclareLaunchArgument(
                "allow_reverse",
                default_value="true",
                description="后退已于 2026-09-09 完成 3/3 单轴实机验收。",
            ),
            DeclareLaunchArgument(
                "allow_lateral",
                default_value="true",
                description="左右横移已于 2026-09-09 各完成 3/3 单轴实机验收。",
            ),
            DeclareLaunchArgument(
                "allow_combined_translation",
                default_value="false",
                description="是否允许前后和横移同时非零；组合运动尚未实机验收。",
            ),
            robot_description,
            lidar_frame_alias,
            rviz,
            safe_twist_bridge,
            driver,
        ]
    )
