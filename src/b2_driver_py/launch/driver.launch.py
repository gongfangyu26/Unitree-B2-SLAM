from launch import LaunchDescription
from launch_ros.actions import Node
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

import os
"""
    launch文件需要集成如下功能:
        1.机器人模型可视化
        2.速度消息桥接
        3.里程计消息发布、广播里程计坐标变换、发布关节状态信息
"""

def generate_launch_description():

    b2_desc_pkg = get_package_share_directory("b2_description")
    b2_driver_pkg = get_package_share_directory("b2_driver_py")

    # 为rviz2添加启动开关
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true"
    )

    return LaunchDescription([
        # 声明参数必须放在最前面
        use_rviz,
        # 1.机器人模型可视化
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(b2_desc_pkg,"launch","display.launch.py")
            ),
            # 关节状态由driver节点发布, 不需要使用默认
            launch_arguments=[("use_joint_state_publisher", "false")] 
        ),
        # 雷达坐标系的映射
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_lidar",
            arguments=[
                "0", "0", "0",          # x y z 平移
                "0", "0", "0",          # roll pitch yaw 旋转
                "lidar_link",           # 父坐标系
                "rslidar"               # 子坐标系
            ]
        ),
        # 包含rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(b2_driver_pkg,"rviz","display.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),
        # 2.速度消息桥接
        Node(
            package="b2_twist_bridge",
            executable="twist_bridge"
        ),
        # 3.里程计消息发布、广播里程计坐标变换、发布关节状态信息
        Node(
            package="b2_driver_py",
            executable="driver",
            # 加载动态参数yaml文件
            parameters=[os.path.join(b2_driver_pkg,"params","driver.yaml")]
        )
    ])