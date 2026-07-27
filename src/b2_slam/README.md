# b2_slam 功能包

## 概述

`b2_slam` 是一个专为 Unitree B2 机器狗设计的 ROS 2 SLAM（同步定位与建图）功能包。该包实现了完整的激光 SLAM 建图流程，包括点云时间戳修复、3D点云到2D激光扫描的投影转换，以及基于 `slam_toolbox` 的异步建图功能。

### 核心功能

| 功能模块 | 说明 |
|---------|------|
| **时间戳修复** | 修复雷达传感器时间与系统时间不同步的问题 |
| **3D→2D转换** | 将 RoboSense 3D 点云投影为 2D 激光扫描 |
| **SLAM 建图** | 基于 slam_toolbox 的异步在线建图 |
| **可视化** | 集成 RViz 实时显示建图过程 |

### 支持的雷达

| 平台 | 雷达话题 | LiDAR Frame |
|------|---------|-------------|
| Unitree B2 | `/rslidar_points` | `rslidar` |
| Unitree Go2 | `/utlidar/cloud_deskewed` | `radar` |

---

## 目录结构

```
b2_slam/
├── b2_slam/                    # Python 源代码目录
│   ├── __init__.py
│   └── pointcloud_timestamp_fix.py  # 点云时间戳修复节点
├── config/                     # 配置文件目录
│   ├── pointcloud_to_laserscan_params.yaml  # 点云转激光参数
│   ├── slam.rviz               # SLAM 专用 RViz 配置
│   └── slam_toolbox_params.yaml           # slam_toolbox 参数
├── launch/                     # 启动文件目录
│   └── slam.launch.py          # 一键启动建图
├── resource/                   # 资源文件
├── test/                       # 测试文件
├── package.xml                 # ROS 包声明
├── setup.cfg                   # Python 安装配置
└── setup.py                    # Python 包安装脚本
```

---

## 安装与依赖

### 依赖包

```xml
<!-- 核心依赖 -->
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>

<!-- 执行依赖 -->
<exec_depend>pointcloud_to_laserscan</exec_depend>
<exec_depend>slam_toolbox</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>ros2launch</exec_depend>
```

### 编译

```bash
cd /home/nvidia/b2_code/b2_ws
colcon build --packages-select b2_slam --symlink-install
source install/setup.bash
```

---

## 启动方式

### 一键启动完整建图流程

```bash
ros2 launch b2_slam slam.launch.py
```

### 启动流程详解

启动顺序（按执行顺序）：

1. **b2_driver** - 启动机器狗驱动，提供 odom/TF，关闭自带 RViz
2. **pointcloud_timestamp_fix** - 修复雷达点云的时间戳
3. **pointcloud_to_laserscan** - 将 3D 点云转为 2D `/scan`
4. **async_slam_toolbox_node** - SLAM Toolbox 建图
5. **rviz2** - 可视化 (Fixed Frame = odom)

---

## 核心节点说明

### 1. pointcloud_timestamp_fix

**功能**：修复传感器时间与系统时间不同步导致 TF 坐标变换失败的问题

**工作原理**：复制整帧点云数据，仅将时间戳替换为当前系统时间

**参数配置**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_topic` | `/rslidar_points` | 输入点云话题（B2 RoboSense） |
| `output_topic` | `/utlidar/cloud_fixed` | 输出修复后的点云话题 |

**QoS 配置**：
- 可靠性：RELIABLE
- 持续性：VOLATILE
- 历史记录：KEEP_LAST (depth=10)

**调试信息**：每 5 秒输出一次状态报告，包括接收消息数量和频率

---

## 配置文件详解

### slam_toolbox_params.yaml 关键参数

```yaml
# 坐标系配置
odom_frame: odom       # 里程计坐标系
map_frame: map         # 地图坐标系
base_frame: base_link  # 机器人基坐标系
scan_topic: /scan      # 激光扫描话题

# 建图参数
mode: mapping                    # 工作模式：建图
resolution: 0.05                # 地图分辨率 5cm
max_laser_range: 10.0           # 最大激光距离 10m
map_update_interval: 0.4        # 地图更新间隔

# 关键帧添加条件
minimum_travel_distance: 0.15   # 移动 15cm 添加关键帧
minimum_travel_heading: 0.2     # 转动 0.2rad 添加关键帧

# 回环检测
do_loop_closing: true           # 启用回环检测
loop_search_maximum_distance: 5.0  # 回环搜索最大距离
```

### pointcloud_to_laserscan_params.yaml

用于将 3D 点云投影为 2D 激光扫描，配置激光高度范围、角度范围等参数。

---

## 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | B2 雷达原始点云 |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/utlidar/cloud_fixed` | `sensor_msgs/PointCloud2` | 时间戳修复后的点云 |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光扫描数据 |

### TF 变换

| 变换关系 | 说明 |
|----------|------|
| `map → odom` | SLAM 输出的地图到里程计变换 |
| `odom → base_link` | 里程计到机器人基座变换 |
| `base_link → lidar_link → rslidar` | 基座到雷达的静态变换 |

---

## 使用指南

### 建图流程

1. **启动建图**
   ```bash
   ros2 launch b2_slam slam.launch.py
   ```

2. **遥控机器人移动**
   使用手柄或键盘控制机器人在环境中移动，SLAM 会自动构建地图

3. **保存地图**
   ```bash
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: my_map, map_type: 0}"
   ```

### 常见问题排查

**问题1：点云时间戳不同步**
- 现象：TF 变换失败，RViz 显示异常
- 解决方案：确保 `pointcloud_timestamp_fix` 节点正常运行

**问题2：无点云数据**
- 检查命令：
  ```bash
  ros2 topic info /rslidar_points
  ros2 topic echo /rslidar_points --no-header
  ```
- 确认雷达驱动已启动，QoS 配置匹配（需要 RELIABLE）

**问题3：建图质量差**
- 调整 `slam_toolbox_params.yaml` 中的分辨率和关键帧参数
- 确保机器人移动速度适中，避免过快

---

## 技术细节

### Go2 vs B2 关键区别

| 对比项 | Go2 | B2 |
|--------|-----|----|
| 雷达话题 | `/utlidar/cloud_deskewed` | `/rslidar_points` |
| LiDAR Frame | `radar` | `rslidar` |
| TF 链 | `base_link → radar` | `base_link → lidar_link → rslidar` |
| 雷达类型 | 速腾聚创 | RoboSense |

### 时间戳修复原理

由于传感器时间与 ROS 系统时间可能存在偏差，直接使用原始点云时间戳会导致：
- TF 坐标变换失败（时间戳超出 TF buffer）
- 激光数据与里程计数据不同步
- 建图漂移增大

修复方案：复制点云数据，仅将 `header.stamp` 替换为当前 ROS 系统时间。

---

## 参考资料

- [Go2 Tutorial](https://ztl3106742440-hub.github.io/go2-tutorial/04-perception/10-lidar/)
- [slam_toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)
