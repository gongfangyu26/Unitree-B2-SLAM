"""
一键启动建图: 驱动 + 传感器处理 + SLAM + RViz (SLAM 专用视图)

启动流程 (按执行顺序):
  1. b2_driver                   — 启动机器狗驱动，提供 odom/TF，关闭自带 RViz
  2. pointcloud_timestamp_fix    — 修复雷达点云的时间戳
  3. pointcloud_to_laserscan     — 将 3D 点云转为 2D /scan
  4. async_slam_toolbox_node     — SLAM Toolbox 建图
  5. rviz2                       — 可视化 (Fixed Frame = odom)

Go2  vs  B2  的关键区别:
  Go2: 雷达话题 /utlidar/cloud_deskewed, lidar_frame = radar
  B2:  雷达话题 /rslidar_points,        lidar_frame = rslidar (RoboSense SDK)
  B2  URDF: base_link → lidar_link → rslidar (static TF 零偏移)
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 本包 share 目录
    slam_share = Path(get_package_share_directory("b2_slam"))

    # 配置文件路径
    slam_params = str(slam_share / "config" / "slam_toolbox_params.yaml")
    scan_params = str(slam_share / "config" / "pointcloud_to_laserscan_params.yaml")
    rviz_cfg = str(slam_share / "config" / "slam.rviz")

    # 1) B2 驱动 — 提供 odom / TF
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("b2_driver"))
                / "launch"
                / "driver.launch.py"
            )
        ),
        # 关闭 driver 自带 rviz，避免与 SLAM rviz 冲突
        launch_arguments={"use_rviz": "false"}.items(),
    )

    # 2) 时间戳修复 — B2 雷达话题是 /rslidar_points (RoboSense)
    timestamp_fix = Node(
        package="b2_slam",
        executable="pointcloud_timestamp_fix",
        name="pointcloud_timestamp_fix",
        parameters=[
            {
                "input_topic": "/rslidar_points",
                "output_topic": "/utlidar/cloud_fixed",
            }
        ],
        output="screen",
    )

    # 3) 3D 点云 → 2D 激光扫描
    pc_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[scan_params],
        remappings=[
            ("cloud_in", "/utlidar/cloud_fixed"),  # 时间戳修复后的点云
            ("scan", "/scan"),                      # 输出标准 /scan
        ],
        output="screen",
    )

    # 4) SLAM Toolbox 建图
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
    )

    # 5) RViz — 加载 SLAM 专用配置
    rviz_args = ["-d", rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=rviz_args,
        output="screen",
    )

    return LaunchDescription([driver, timestamp_fix, pc_to_scan, slam, rviz])
