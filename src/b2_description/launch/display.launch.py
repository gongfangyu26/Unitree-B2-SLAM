"""加载 B2 URDF，并按需启动关节状态发布节点."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """生成机器人模型启动描述."""
    package_share = Path(get_package_share_directory("b2_description"))
    urdf_path = package_share / "urdf" / "b2_description.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    use_joint_state_publisher = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="true",
        description="是否启动通用关节状态发布节点；连接真机时应设为 false。",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
        output="screen",
    )

    return LaunchDescription(
        [
            use_joint_state_publisher,
            robot_state_publisher,
            joint_state_publisher,
        ]
    )
