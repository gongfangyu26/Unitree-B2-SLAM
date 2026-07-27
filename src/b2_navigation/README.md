# b2_navigation 功能包

## 概述

`b2_navigation` 是专为 Unitree B2 机器狗设计的 ROS 2 导航功能包，基于 **Nav2** 框架实现完整的 2D 自主导航能力。该包集成了 AMCL 定位、全局路径规划、局部路径跟踪和行为树导航等核心功能。

### 核心功能

| 功能模块 | 说明 |
|---------|------|
| **AMCL 定位** | 自适应蒙特卡洛定位，实现机器人在已知地图中的位姿估计 |
| **全局规划** | 基于 NavFn 的全局路径规划，生成从起点到目标点的最优路径 |
| **局部控制** | DWB 局部规划器，实现动态避障和路径跟踪 |
| **行为树导航** | 基于 BT 的导航行为编排，支持多种导航任务 |
| **生命周期管理** | Nav2 生命周期节点管理，确保节点正确启动和关闭 |

---

## 目录结构

```
b2_navigation/
├── b2_navigation/              # Python 源代码目录
│   └── __init__.py
├── config/                     # 配置文件目录
│   └── nav2_params.yaml        # Nav2 完整参数配置
├── launch/                     # 启动文件目录
│   └── navigation.launch.py    # 一键启动导航
├── rviz/                       # RViz 配置目录
│   └── navigation.rviz         # 导航专用 RViz 配置
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

<!-- Nav2 核心组件 -->
<exec_depend>nav2_amcl</exec_depend>
<exec_depend>nav2_map_server</exec_depend>
<exec_depend>nav2_planner</exec_depend>
<exec_depend>nav2_controller</exec_depend>
<exec_depend>nav2_bt_navigator</exec_depend>
<exec_depend>nav2_behaviors</exec_depend>
<exec_depend>nav2_lifecycle_manager</exec_depend>

<!-- 辅助依赖 -->
<exec_depend>pointcloud_to_laserscan</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>ros2launch</exec_depend>
```

### 编译

```bash
cd /home/nvidia/b2_code/b2_ws
colcon build --packages-select b2_navigation --symlink-install
source install/setup.bash
```

---

## 启动方式

### 一键启动完整导航系统

```bash
ros2 launch b2_navigation navigation.launch.py map:=/path/to/your/map.yaml
```

### 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map` | 无（必填） | 地图 YAML 文件路径 |
| `use_rviz` | `true` | 是否启动 RViz 可视化 |

### 启动流程详解

启动顺序（按执行顺序）：

1. **b2_driver** - 底层驱动，提供 `/odom`、TF、`/joint_states`
2. **pointcloud_timestamp_fix** - 修复雷达点云时间戳
3. **pointcloud_to_laserscan** - 3D点云转2D激光扫描
4. **map_server** - 加载地图文件
5. **amcl** - AMCL 定位节点
6. **localization_lifecycle** - 定位模块生命周期管理
7. **(延迟3秒)** planner_server + controller_server + behavior_server + bt_navigator
8. **navigation_lifecycle** - 导航模块生命周期管理
9. **rviz2** - 可视化 (Fixed Frame = map)

> **关键设计**：导航节点延迟 3 秒启动，确保 map_server 和 amcl 先进入 ACTIVE 状态

---

## 核心组件配置

### 1. AMCL 定位参数

```yaml
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::OmniMotionModel"  # 全向运动模型
    laser_model_type: "likelihood_field"             # 似然场模型
    
    # 运动模型噪声参数
    alpha1: 0.3  # 旋转噪声
    alpha2: 0.3  # 旋转噪声
    alpha3: 0.1  # 平移噪声
    alpha4: 0.1  # 平移噪声
    alpha5: 0.2  # 额外噪声
    
    # 粒子滤波器参数
    min_particles: 800    # 最小粒子数
    max_particles: 3000   # 最大粒子数
    update_min_d: 0.1     # 平移更新阈值 (m)
    update_min_a: 0.1     # 旋转更新阈值 (rad)
```

### 2. 全局规划器

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5        # 目标容差 (m)
      use_astar: false      # 使用 Dijkstra 算法
      allow_unknown: true   # 允许穿越未知区域
```

### 3. 局部控制器 (DWB)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # 速度限制 (适配 B2 机身)
      min_vel_x: -0.30
      max_vel_x: 0.30
      min_vel_y: -0.30
      max_vel_y: 0.30
      max_vel_theta: 0.50
      
      # 加速度限制
      acc_lim_x: 0.30
      acc_lim_y: 0.30
      acc_lim_theta: 1.0
      
      # 评价器权重
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

### 4. 代价地图配置

#### 局部代价地图

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 4.0           # 宽度 (m)
      height: 4.0          # 高度 (m)
      resolution: 0.05     # 分辨率 (m)
      robot_radius: 0.48   # 机器人半径 (适配 B2)
      rolling_window: true # 滚动窗口模式
```

#### 全局代价地图

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.05     # 分辨率 (m)
      robot_radius: 0.48   # 机器人半径
      track_unknown_space: true
```

---

## B2 机身适配参数

B2 机身尺寸：**1098mm × 450mm × 645mm**

| 参数 | 值 | 说明 |
|------|-----|------|
| `robot_radius` | 0.48m | 机器人碰撞半径（含安全余量） |
| `max_vel_x` | 0.30 m/s | 最大线速度 |
| `max_vel_y` | 0.30 m/s | 最大横向速度 |
| `max_vel_theta` | 0.50 rad/s | 最大角速度 |
| `obstacle_min_range` | 0.50m | 障碍物最小检测距离（过滤雷达近距噪声） |

---

## 话题接口

### 关键话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/scan` | `sensor_msgs/LaserScan` | 激光扫描数据 |
| `/odom` | `nav_msgs/Odometry` | 里程计数据 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL 估计位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 地图数据 |

### 服务接口

| 服务名 | 服务类型 | 说明 |
|--------|---------|------|
| `/navigate_to_pose` | `NavigateToPose` | 导航到指定位姿 |
| `/navigate_through_poses` | `NavigateThroughPoses` | 多点导航 |
| `/reinitialize_global_localization` | `ReinitializeGlobalLocalization` | 重新初始化定位 |
| `/clear_costmap` | `ClearCostmap` | 清除代价地图 |

---

## 使用指南

### 1. 准备地图

确保已有通过 `b2_slam` 创建的地图文件（`.yaml` + `.pgm`）。

### 2. 启动导航

```bash
ros2 launch b2_navigation navigation.launch.py map:=/path/to/map.yaml
```

### 3. 设置初始位姿

在 RViz 中使用 **2D Pose Estimate** 工具设置机器人初始位置，或通过命令行：

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
  }
}" --once
```

### 4. 发送导航目标

在 RViz 中使用 **2D Nav Goal** 工具点击目标位置，或通过命令行：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  goal: {
    pose: {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 5.0, y: 3.0, z: 0.0},
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      }
    }
  }
}"
```

---

## 常见问题排查

### 问题1：导航无法启动

**现象**：`lifecycle_manager_navigation` 节点报错

**排查步骤**：
```bash
# 检查节点状态
ros2 lifecycle list /lifecycle_manager_localization
ros2 lifecycle list /lifecycle_manager_navigation

# 检查 map_server 是否正确加载
ros2 topic echo /map --no-header
```

**解决方案**：确保地图文件路径正确，map.yaml 中 pgm 路径正确

### 问题2：定位漂移

**现象**：机器人估计位置与实际位置偏差逐渐增大

**排查步骤**：
- 检查激光数据质量：`ros2 topic echo /scan`
- 检查 TF 变换：`ros2 tf tree`
- 调整 AMCL 参数：增加粒子数，调整运动模型噪声

### 问题3：避障不及时

**现象**：机器人无法及时避开障碍物

**解决方案**：
- 调整局部代价地图参数，增大 `obstacle_max_range`
- 调整 DWB 控制器的速度和加速度限制
- 检查 `inflation_radius` 参数

### 问题4：自身被识别为障碍物

**现象**：机器人周围出现障碍区域

**解决方案**：
- 确认 `robot_radius` 参数正确（B2 建议 0.48m）
- 检查 `obstacle_min_range` 参数，过滤雷达近距噪声

---

## 性能优化建议

### 1. 计算资源优化

| 参数调整 | 说明 |
|----------|------|
| 降低 `controller_frequency` | 从 20Hz 降至 15Hz，减少 CPU 负载 |
| 增大代价地图 `resolution` | 从 0.05m 增至 0.1m，减少计算量 |
| 减小局部代价地图 `width/height` | 从 4m 降至 3m |

### 2. 导航精度优化

| 参数调整 | 说明 |
|----------|------|
| 减小 `xy_goal_tolerance` | 提高目标到达精度 |
| 增大 AMCL `min_particles` | 提高定位稳定性 |
| 调整 DWB `sim_time` | 增加轨迹模拟时间 |

---

## 参考资料

- [Nav2 Documentation](https://navigation.ros.org/)
- [Go2 Tutorial Navigation](https://ztl3106742440-hub.github.io/go2-tutorial/04-perception/12-nav2-baseline/)
- [AMCL Wiki](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [DWB Local Planner](https://navigation.ros.org/configuration/packages/configuring-dwb.html)
