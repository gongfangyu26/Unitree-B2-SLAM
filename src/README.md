# 宇树B2机器狗SLAM算法

> 🐕 基于宇树B2平台的激光SLAM建图与导航实现，ROS2 Humble框架

---

## 一、目录结构

```
src/
├── b2_description/        # 机器人URDF模型
├── b2_driver/             # C++驱动（发布odom/TF/joint_states）
├── b2_driver_py/          # Python驱动（备用）
├── b2_navigation/         # Nav2导航栈配置
├── b2_slam/               # SLAM建图（时间戳修复+点云转激光+slam_toolbox）
├── b2_teleop_ctrl_keyboard/# 键盘控制
├── b2_twist_bridge/       # C++速度桥接（Twist→unitree_api）
├── b2_twist_bridge_py/    # Python速度桥接（备用）
└── maps/                  # 预设地图文件
```

**核心节点**：
- `driver`：订阅机器狗状态，发布里程计和TF
- `pointcloud_timestamp_fix`：修复雷达点云时间戳
- `twist_bridge`：将ROS速度指令转为宇树API格式
- `slam_toolbox`：在线建图核心

---

## 二、环境配置

### 2.1 硬件要求

| 组件 | 要求 |
|------|------|
| 机器狗 | 宇树B2 |
| 系统 | Ubuntu 22.04 + ROS2 Humble |

### 2.2 依赖安装

```bash
# 1. ROS2 Humble（已安装可跳过）
sudo apt install -y ros-humble-desktop-full python3-colcon-common-extensions

# 2. CycloneDDS
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# 3. 宇树SDK（参考官方文档）
# https://support.unitree.com/home/zh/developer/ROS2_service

# 4. SLAM与导航依赖
sudo apt install -y ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2

# 5. 设置ROS_DOMAIN_ID（需与机器狗一致）
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 2.3 环境验证

```bash
ros2 --version                      # 确认Humble
echo $RMW_IMPLEMENTATION            # 确认CycloneDDS
ros2 pkg list | grep unitree        # 确认宇树包
ros2 pkg list | grep slam_toolbox   # 确认SLAM包
```

---

## 三、编译步骤

```bash
# 1. 创建工作空间
mkdir -p ~/b2_ws/src
cd ~/b2_ws/src

# 2. 复制项目代码到src目录

# 3. 编译
cd ~/b2_ws
colcon build --symlink-install

# 4. 设置环境变量
source ~/b2_ws/install/setup.bash
echo "source ~/b2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 四、运行指南

### 4.1 启动前准备

```bash
source /opt/ros/humble/setup.bash
source ~/b2_ws/install/setup.bash
ping b2  # 确认网络连接
```

### 4.2 一键启动SLAM建图

```bash
ros2 launch b2_slam slam.launch.py
```

启动流程：`driver` → `pointcloud_timestamp_fix` → `pointcloud_to_laserscan` → `slam_toolbox` → `rviz2`

### 4.3 键盘控制

```bash
ros2 run b2_teleop_ctrl_keyboard b2_teleop_ctrl_keyboard
```

| 按键 | 功能 |
|------|------|
| `w/s` | 前后移动 |
| `a/d` | 左右转 |
| `Q/E` (Shift) | 左右平移 |
| `r/t` | 增减速度 |
| `h/n/m/j` | 动作（打招呼/坐下/站起/前跳） |
| `Ctrl+C` | 退出并站立 |

### 4.4 保存地图

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: /home/nvidia/b2_ws/src/maps/my_map, map_type: 0}"
```

### 4.5 启动自主导航

```bash
ros2 launch b2_navigation navigation.launch.py map:=/home/nvidia/b2_ws/src/maps/my_map.yaml
```

在RViz中使用 `2D Pose Estimate` 设置初始位置，`2D Nav Goal` 设置目标点。

---

## 五、算法说明

### 5.1 SLAM流程

```
/rslidar_points → 时间戳修复 → /utlidar/cloud_fixed → 点云转激光 → /scan → slam_toolbox → map
```

- **时间戳修复**：将点云时间戳替换为系统时间（回退0.05秒），解决TF匹配问题
- **点云转激光**：高度裁剪(-0.25m~1.0m)，投影为2D扫描
- **slam_toolbox**：基于Ceres Solver的Graph SLAM，支持回环检测

### 5.2 TF变换链

```
map → odom → base_link → lidar_link → rslidar
```

### 5.3 导航系统

- **定位**：AMCL粒子滤波
- **规划**：NavfnPlanner全局规划 + DWBLocalPlanner局部规划
- **控制**：Twist消息 → twist_bridge → 宇树API

---

## 六、常见问题

| 问题 | 解决方案 |
|------|----------|
| 连接失败 | 检查网络、ROS_DOMAIN_ID、CycloneDDS配置 |
| 无点云数据 | 检查雷达驱动、话题`/rslidar_points`、QoS配置 |
| 建图漂移 | 降低移动速度、调整关键帧参数 |
| SLAM无响应 | 检查QoS配置、TF变换链 |
| 定位失败 | 在RViz中设置初始位置(2D Pose Estimate) |
| 编译找不到unitree包 | 参考宇树官方文档重新安装SDK |

---

## 七、结果验证

### 7.1 实时验证

```bash
ros2 topic hz /scan            # 激光频率（~10Hz）
ros2 topic echo /odom          # 里程计数据
ros2 tf tree                   # TF变换链
```

### 7.2 离线验证

```bash
# 检查保存的地图
cat ~/b2_ws/src/maps/my_map.yaml
eog ~/b2_ws/src/maps/my_map.pgm
```

**地图质量指标**：
- 分辨率：5cm
- 完整性：所有障碍物正确识别
- 一致性：无明显漂移
- 回环检测：回到起点时地图闭合

### 7.3 导航验证

| 指标 | 目标值 |
|------|--------|
| 定位误差 | < 0.3m |
| 路径跟踪误差 | < 0.2m |

---

## 附录：命令速查

| 功能 | 命令 |
|------|------|
| 启动建图 | `ros2 launch b2_slam slam.launch.py` |
| 启动导航 | `ros2 launch b2_navigation navigation.launch.py map:=xxx.yaml` |
| 键盘控制 | `ros2 run b2_teleop_ctrl_keyboard b2_teleop_ctrl_keyboard` |
| 保存地图 | `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: xxx, map_type: 0}"` |
| 查看节点 | `ros2 node list` |
| 查看TF | `ros2 tf tree` |

---

**参考文档**：
- [宇树ROS2开发文档](https://support.unitree.com/home/zh/developer/ROS2_service)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2](https://navigation.ros.org/)