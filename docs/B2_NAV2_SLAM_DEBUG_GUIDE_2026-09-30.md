# B2 运动建图与 Nav2 首次实机调试手册

日期：2026-09-30（计划）\
适用设备：Unitree B2、Jetson Orin Nano、RoboSense LiDAR、ROS2 Humble\
目标：完成“运动建图 → 保存三维数据库和二维地图 → 加载定位 → 人工点击近距离目标”
的首次安全验证。

> 本文中的运动建图、地图质量、AMCL/RTAB-Map 定位和 Nav2 控制尚未实机验证。
> 必须按顺序测试，不得跳过只读检查和默认锁定阶段。

## 0. 现场安全条件

1. 先在平整、宽阔、无行人区域测试；首次导航目标距离不超过 0.5 m。
2. 一人操作 Jetson/PC，一人看护 B2；原厂遥控器和急停必须随时可用。
3. 第一次运动建图使用原厂遥控器低速带机器人行走，不用 Nav2 控制。
4. 平地定位和导航未连续成功 3 次前，不做坡道自主导航。
5. 看到方向异常、连续转圈、地图撕裂、TF 跳变或停止失效，立即人工停车。

立即停止 Nav2 控制的命令必须提前复制到独立终端：

```bash
ros2 service call /b2/control/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /b2/navigation/set_armed \
  std_srvs/srv/SetBool "{data: false}"
```

## 1. 每个 Jetson 终端先加载环境

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/Unitree-B2-SLAM/install/setup.bash
```

确认 DDS 指向真实 B2 网口 `enP8p1s0`：

```bash
ip -br address
grep -n "NetworkInterface" \
  /home/jetson/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
```

预期：`enP8p1s0` 为 `192.168.123.211`，XML 中也是 `enP8p1s0`。若现场地址发生
变化，以实际配置为准并记录，不得直接假定。

## 2. 同步后编译和安装检查

在 Jetson 已经收到电脑端本轮源码后执行：

```bash
cd ~/Unitree-B2-SLAM
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select \
  b2_driver_py b2_twist_bridge_py b2_slam b2_slam_3d b2_navigation \
  --symlink-install
source install/setup.bash
```

确认新增节点和 Nav2 已安装：

```bash
ros2 pkg executables b2_slam_3d
ros2 pkg executables b2_navigation
ros2 pkg prefix nav2_bringup
ros2 pkg prefix pointcloud_to_laserscan
```

预期至少包含：

```text
b2_slam_3d occupancy_grid_preflight
b2_slam_3d sensor_time_sync
b2_slam_3d slam_preflight
b2_navigation navigation_gate
b2_navigation navigation_safety_monitor
```

## 3. 清理重复进程并核对基础数据

不要看到重复节点后继续启动。先列出节点：

```bash
ros2 node list | sort
```

如果发现同名的 `/b2_driver_py`、`/b2_safe_twist_bridge`、
`/static_tf_lidar_frame_alias` 或 `/rtabmap/rtabmap`，回到对应启动终端按
`Ctrl-C` 正常停止。不要用大范围 `pkill` 误杀其他实验。

仅检查现有真实数据：

```bash
ros2 topic echo /lf/sportmodestate --once
ros2 topic hz /rslidar_points
ros2 topic type /rslidar_points
```

预期：B2 状态可输出，`/rslidar_points` 类型为
`sensor_msgs/msg/PointCloud2`，频率约 8～10 Hz。频率明显异常时先修通信，不进入
运动阶段。

## 4. 阶段一：平地运动三维建图

### 4.1 建立实验目录

```bash
mkdir -p /home/jetson/b2_maps
mkdir -p /home/jetson/b2_bags
```

每次新实验使用新文件名，避免覆盖历史证据。

### 4.2 启动新地图

确认第 3 节没有驱动节点后，启动包含驱动的建图：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py \
  delete_db:=true \
  database_path:=/home/jetson/b2_maps/b2_flat_loop_20260930.db \
  use_rtabmap_viz:=false \
  start_driver:=true \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  grid_topic:=/rtabmap/grid_prob_map
```

如果唯一的驱动已在另一个受控终端运行，命令中的 `start_driver` 必须改为
`false`。

### 4.3 运动前强制检查

另开终端加载环境后执行：

```bash
ros2 topic echo /b2_slam/preflight_ok
ros2 topic echo /b2_slam/preflight_report --once
ros2 topic echo /b2_navigation/map_report --once
ros2 topic hz /b2_slam_3d/cloud_sync
ros2 topic hz /b2_slam_3d/odom_sync
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link rslidar
ros2 run tf2_ros tf2_echo map odom
```

通过条件：

- `/b2_slam/preflight_ok` 连续为 `true`；
- 点云和里程计频率稳定；
- `odom -> base_link`、`base_link -> rslidar`、`map -> odom` 都存在；
- 没有持续出现 TF extrapolation、点云为空或 ICP 注册错误。

不满足时禁止移动，并保存完整的 `preflight_report`。

### 4.4 录制运动证据

```bash
ros2 bag record \
  -o /home/jetson/b2_bags/b2_flat_loop_20260930 \
  /lf/sportmodestate \
  /rslidar_points \
  /odom \
  /b2_slam_3d/cloud_sync \
  /b2_slam_3d/odom_sync \
  /b2_slam/preflight_ok \
  /b2_slam/preflight_report \
  /tf \
  /tf_static
```

### 4.5 用原厂遥控器走小回路

路线建议：起点静止 10 秒 → 直行约 2 m → 小角度转弯 → 形成约 3～5 m 的闭合
回路 → 回到起点附近静止 10 秒。第一次不要加入坡道和动态行人。

观察真实话题，不假定是否带 `/rtabmap` 前缀：

```bash
ros2 topic list -t | grep -E 'cloud_map|grid_prob_map|(^|/)map'
ros2 run tf2_ros tf2_echo map base_link
```

在查到真实点云地图名称后，对该话题执行：

```bash
ros2 topic hz /rtabmap/cloud_map
ros2 topic echo /rtabmap/cloud_map --once --field width
```

如果现场真实名称是 `/cloud_map`，把命令中的名称替换为 `/cloud_map`，并在记录中
写清楚。地图应随运动累积，不应出现明显双墙、撕裂、折叠或周期跳变。

### 4.6 结束建图

先让 B2 完全静止，再停止 rosbag；建图节点保持运行，先完成下一节二维地图保存。

## 5. 阶段二：确认和保存二维导航地图

### 5.1 不猜话题，检查消息类型

```bash
ros2 topic list -t | grep -E 'map|grid'
ros2 topic type /rtabmap/grid_prob_map
ros2 topic echo /rtabmap/grid_prob_map --once --field info
ros2 topic echo /b2_navigation/map_report --once
```

只有消息类型为 `nav_msgs/msg/OccupancyGrid`、宽高和分辨率非零、已知区域和障碍区域
都存在时，才保存为 Nav2 地图。若 `/rtabmap/grid_prob_map` 不存在，逐个检查实际
候选话题，不能继续照抄命令。

### 5.2 保存二维地图

RTAB-Map 仍在运行时执行：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /rtabmap/grid_prob_map \
  -f /home/jetson/b2_maps/b2_flat_loop_20260930_2d
```

然后正常停止 RTAB-Map，检查文件：

```bash
ls -lh /home/jetson/b2_maps/b2_flat_loop_20260930.db
ls -lh /home/jetson/b2_maps/b2_flat_loop_20260930_2d.yaml
ls -lh /home/jetson/b2_maps/b2_flat_loop_20260930_2d.pgm
ros2 bag info /home/jetson/b2_bags/b2_flat_loop_20260930
```

记录数据库大小、地图尺寸、分辨率、bag 时长和消息数量。保存失败时不要删除数据库
或 bag。

## 6. 阶段三：Nav2 锁定观察模式

先确认 RTAB-Map 建图节点、旧驱动和旧桥已经停止：

```bash
ros2 node list | sort
```

启动 AMCL 模式：

```bash
ros2 launch b2_navigation navigation.launch.py \
  map:=/home/jetson/b2_maps/b2_flat_loop_20260930_2d.yaml \
  localization_mode:=amcl \
  start_driver:=true \
  use_rviz:=false \
  max_armed_duration_sec:=60.0
```

### 6.1 确认启动时绝对锁定

```bash
ros2 topic echo /b2_navigation/armed --once
ros2 topic echo /b2/control/enabled --once
ros2 topic echo /b2/control/status --once
```

前两项必须为 `false`。如果任一为 `true`，立即关闭服务并停止测试。

### 6.2 检查地图、雷达、TF 和生命周期

```bash
ros2 topic echo /b2_navigation/map_report --once
ros2 topic hz /scan
ros2 topic echo /b2/autonomy_reason
ros2 topic echo /b2/autonomy_ok
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list | grep navigate_to_pose
```

所有生命周期节点应为 `active`。AMCL 首次使用时，在 PC 的 RViz 中用
`2D Pose Estimate` 设置真实初始位置和朝向，然后等待点云与地图对齐。

### 6.3 只观察 Nav2 速度，不让 B2 动

保持两级锁都是 `false`。在 RViz 设置一个 0.3～0.5 m 的近目标，并同时观察：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /api/sport/request
```

允许 `/cmd_vel_nav` 出现规划速度；`/cmd_vel` 不应转发非零速度，B2 不应运动。
重点记录 Nav2 的最小/最大 `linear.x` 和 `angular.z`。当前速度桥会把非零小线速度
整形为 0.30 m/s，这一行为对连续 Nav2 控制尚未实机验证。

检查结束后在 RViz 取消目标，确认 Nav2 停止输出。

## 7. 阶段四：第一次近距离点击导航

只有第 6 节全部通过、B2 周围清场且两人就位时才能执行。

### 7.1 解锁前最后检查

```bash
ros2 topic echo /b2/autonomy_ok --once
ros2 topic echo /b2/autonomy_reason --once
ros2 topic echo /b2_navigation/armed --once
ros2 topic echo /b2/control/enabled --once
```

要求：`autonomy_ok=true`，两级锁均为 `false`。

### 7.2 按顺序双重解锁

```bash
ros2 service call /b2/navigation/set_armed \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /b2/control/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

两个服务都必须返回 `success: true`。导航门最长 60 秒后自动锁定。

### 7.3 发送近目标

在 RViz 中点击距机器人 0.3～0.5 m、正前方无遮挡位置。首次只验证前进和小角度
修正，不测试横移、后退恢复或坡道。

同时观察：

```bash
ros2 topic echo /b2/autonomy_reason
ros2 topic echo /b2/control/status
ros2 topic echo /cmd_vel_nav
ros2 topic echo /odom
```

记录：目标距离、实际路线、到点误差、最大速度、是否摆动、是否可靠停车、障碍物
距离和完成耗时。连续完成 3 次，才能把“平地近距离 Nav2”记为实机通过。

### 7.4 每次目标后主动锁定

```bash
ros2 service call /b2/control/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /b2/navigation/set_armed \
  std_srvs/srv/SetBool "{data: false}"
```

不要依赖 60 秒自动超时作为日常停车方法。

## 8. RTAB-Map 定位模式验证

AMCL 平地测试通过后，停止 AMCL 启动文件，再启动：

```bash
ros2 launch b2_navigation navigation.launch.py \
  map:=/home/jetson/b2_maps/b2_flat_loop_20260930_2d.yaml \
  localization_mode:=rtabmap \
  database_path:=/home/jetson/b2_maps/b2_flat_loop_20260930.db \
  start_driver:=true \
  use_rviz:=false
```

先只观察，不解锁：

```bash
ros2 topic echo /b2_slam/preflight_ok
ros2 topic echo /b2/autonomy_reason
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

比较 AMCL 和 RTAB-Map 定位的启动时间、位置跳变、回到旧位置后的重定位能力。
RTAB-Map 模式未通过前，不进入坡道导航。

## 9. 平地通过后再做坡道

顺序固定为：

1. 原厂遥控器上坡/下坡运动建图；
2. 检查点云是否出现双层地面、坡面断裂和里程计姿态异常；
3. 保存独立的坡道数据库和二维地图；
4. 锁定模式验证定位；
5. 最后才进行短距离自主上坡，再单独进行下坡。

坡道实验至少记录坡度、坡长、表面材质、上/下坡方向、速度、地图漂移、定位跳变、
停车距离和是否触发安全锁。平地参数不能直接宣称适用于坡道。

## 10. 当天必须保存的结果

- `ros2 bag info` 输出；
- 三维数据库和二维 YAML/PGM 的路径与文件大小；
- `/b2_slam/preflight_report` 和 `/b2_navigation/map_report`；
- 真实地图、点云地图和 TF 截图；
- 每次导航目标、结果、到点误差、异常和人工接管情况；
- 当天使用的 Git commit SHA 和所有参数；
- 无论成功或失败，都更新 `docs/B2_DEBUG_LOG_2026-09-30.md`。

## 11. 通过判定

本次只能按以下粒度分别判定：

- **运动三维建图通过**：真实 B2 移动、地图连续累积、回环后无明显撕裂，并保存
  可重载数据库；
- **二维地图保存通过**：真实 OccupancyGrid 成功保存 YAML/PGM 并能被
  `map_server` 加载；
- **定位通过**：静止和小范围移动时 `map -> base_link` 连续且与现场一致；
- **Nav2 近距离通过**：至少 3 次近目标产生正确实际位移并可靠停车；
- **坡道通过**：必须另行完成上坡和下坡实测，不能由平地结果替代。

只看到节点、话题、Publisher 或命令返回成功，不能写成机器人运动成功。
