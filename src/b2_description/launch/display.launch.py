from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from lauch.substitutions import FindExecutable
# 参数声明与获取
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关
# from launch.event_handlers import OnProcessExit, OnProcessStart
# from launch.actions import RegisterEventHandler, ExecuteProcess, LogInfo
# 获取功能包下share目录路径
# from ament_index_python.packages import get_package_share_directory

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取功能包路径
    b2_description_pkg = get_package_share_directory("b2_description")
    use_joint_state_publisher = DeclareLaunchArgument(
        name = "use_joint_state_publisher",
        default_value = "true"
    )
    robot_desc = ParameterValue(Command(["xacro ", os.path.join(b2_description_pkg, "urdf", "b2_description.urdf")]))
    # robot_state_publisher节点 === 加载机器人URDF文件
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )
    # joint_state_publisher节点 === 发布关节状态
    # 以后更合理的方式是由程序动态获取关节信息并发布，这个节点的启动是有条件的
    joint_state_publisher = Node(
        package="joint_state_publisher",    
        executable="joint_state_publisher",
        condition = IfCondition(LaunchConfiguration("use_joint_state_publisher"))
    )

    return LaunchDescription([
        use_joint_state_publisher,
        robot_state_publisher, 
        joint_state_publisher
    ])
