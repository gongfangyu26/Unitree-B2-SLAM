#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

# 该脚本只安装 Ubuntu 22.04 + ROS2 Humble 下的系统依赖。
# 宇树 unitree_ros2 工作空间仍需按官方文档单独编译。
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-pcl-ros \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-opencv \
  python3-rosdep \
  python3-yaml

echo
printf '%s\n' '巡检工程的 ROS2 和 Python 依赖已安装。'
printf '%s\n' '推荐通信环境：'
printf '%s\n' '  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
printf '%s\n' '  export ROS_DOMAIN_ID=0'
