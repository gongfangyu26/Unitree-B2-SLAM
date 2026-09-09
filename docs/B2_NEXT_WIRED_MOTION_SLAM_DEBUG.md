# B2 下一次有线运动 3D SLAM 调试手册

## 1. 本次只验证什么

本手册以 `docs/2026-09-09_B2_Jetson_实机调试记录.md` 为最高优先级。

已经通过、不再重复验收的项目：

- B2 有线 ROS2 通信和 Sport 接口；
- 站立、趴下、平衡站立、前进和转向；
- 后退 `0.30 m/s`，3/3 成功；
- 左右横移 `|vy|=0.30 m/s`，各 3/3 成功；
- `safe_motion_test.py` 单次 `MOVE + watchdog STOPMOVE`；
- `/rslidar_points` 约 10 Hz；
- 项目 `/odom` 约 20 Hz；
- `odom → base_link → lidar_link → rslidar` TF；
- `sensor_time_sync`、`slam_preflight` 和静止 RTAB-Map 数据链；
- 静止时非空 `/cloud_map`。

下一次只完成一个新目标：在平地进行小范围人工控制运动，同时录制 rosbag，验证
运动状态 RTAB-Map 是否能够连续建图。

无线通信因合同和采购尚未确定而延期。无线未完成不阻塞本次有线调试，也不能把
最终双网卡拓扑标记为完成。

本次禁止进行 Nav2 自主导航、自动避障、自动巡检、无人值守、坡道和工厂现场测试。

## 2. 选择运行平台

### 2.1 推荐：PC 有线直连 B2

如果 Jetson 还没有可靠的板载供电、固定支架或本地显示器，本次使用已经验证过的
Ubuntu PC 有线连接 B2。由一人操作，一人负责遥控急停和整理网线。

### 2.2 可选：Jetson 本地运行

只有同时满足以下条件才使用 Jetson：

- Jetson 使用厂家适配器稳定供电；
- Jetson、硬盘和线缆固定牢靠；
- 操作者能通过 HDMI 显示器和 USB 键盘在 Jetson 本机操作；
- 电源线和网线不会进入 B2 腿部摆动区。

Jetson 没有可靠板载电源时，不得把开发板松放在机器人背上，也不得拖着可能绊入
腿部的电源线运动，应改用 PC 方案。

如果实验室已有普通千兆交换机，也可以把 B2、Jetson 和 PC 接入同一交换机；
本次不为这个方案额外采购设备，不配置 Linux 网桥、NAT 或互联网共享。

## 3. 场地与人员

- 使用平整、干燥、防滑地面，第一次不测试坡道。
- 可活动区域至少约 `4 m × 4 m`，四周保留急停缓冲区。
- 场景应有墙角、柱子、纸箱或桌腿等静止三维几何特征。
- 避免纯空旷场地、大面积玻璃、重复平行墙和大量移动人员。
- 不搭建 A/B 拍照目标，本次只验证运动 3D SLAM。
- 一人输入命令，一人全程手持原厂遥控器或急停装置。
- 用地面胶带标记机器人初始位置和机头方向。
- 所有运动前确认前、后、左、右至少各有 1.5 m 空间。

出现跌倒趋势、打滑、地图剧烈跳变、数据中断或线缆接近腿部时，立即停车并结束本轮。

## 4. 同步代码时的注意事项

当前 Git 工作区包含大量历史修改和未跟踪文件。不要使用：

```bash
git add .
git restore .
git clean -fd
```

PC 项目目录：

```text
/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM
```

Jetson 实机记录中的项目目录：

```text
/home/nvidia/b2_ws
```

如果使用 Jetson，应通过 U 盘或已有传输方式逐个同步本次修改文件，不要用旧目录覆盖
Jetson 当天的实机日志。

## 5. 编译与软件测试

以下命令在实际运行 SLAM 的电脑上执行。

### 5.1 PC

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM
colcon build --symlink-install --packages-select b2_twist_bridge_py b2_driver_py b2_slam_3d b2_factory_inspection
source install/setup.bash
```

### 5.2 Jetson

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /home/nvidia/b2_ws
colcon build --symlink-install --packages-select b2_twist_bridge_py b2_driver_py b2_slam_3d b2_factory_inspection
source install/setup.bash
```

作用：只编译本轮需要的 Python 控制、里程计和 3D SLAM 包。

运行不控制机器人的测试：

```bash
python3 -m pytest -q \
  src/b2_twist_bridge_py/test/test_sport_model.py \
  src/b2_twist_bridge_py/test/test_velocity_policy.py \
  src/b2_slam_3d/test/test_health_core.py \
  src/b2_factory_inspection/test/test_quality_core.py \
  src/b2_factory_inspection/test/test_route_config.py \
  src/b2_factory_inspection/test/test_target_config.py
```

作用：只检查速度限幅、配置读取、点云健康规则和图像质量核心逻辑，不连接 B2，
不会发送运动请求。当前电脑的验证结果应为 `31 passed`。

如果提示没有 `pytest` 模块，先记录问题，不要因此阻塞实机数据采集。完整的 ROS
代码风格测试还依赖 `ament_copyright`、`ament_flake8` 和 `ament_pep257`，本轮不把这些
开发工具是否安装当作机器人功能验收条件。

## 6. 每个终端都要加载的环境

根据运行平台二选一，不要混用网口名。

### 6.1 PC

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export B2_NET_IF=enp8s0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${B2_NET_IF}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
```

### 6.2 Jetson

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /home/nvidia/b2_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export B2_NET_IF=enP8p1s0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${B2_NET_IF}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
```

作用：让 ROS2/CycloneDDS 只使用连接 B2 的有线网口。

## 7. 调试前最小检查

检查时间和磁盘：

```bash
date
df -h
```

作用：确认 rosbag 时间正确且至少预留 20 GB。Jetson RTC 曾出现 1970 年；如果时间错误，
先手动设置当前时间。下面只是格式示例，必须换成现场真实时间：

```bash
sudo date -s "2026-09-10 14:30:00"
```

快速检查已验证的数据流：

```bash
ros2 topic hz /rslidar_points
```

作用：确认 RoboSense 点云仍约 10 Hz，稳定后按 Ctrl+C。

```bash
ros2 topic hz /lf/sportmodestate
```

作用：确认 B2 运动状态仍在发布，稳定后按 Ctrl+C。

这只是开始运动前的健康门槛，不重新做 ping、站立、后退或横移验收。

## 8. 启动新的 RTAB-Map 数据库

打开终端 1，加载第 6 节环境。

### 8.1 PC

```bash
mkdir -p /home/gongfangyu/b2_data/motion_slam_01
ros2 launch b2_factory_inspection mapping_3d.launch.py \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  cloud_backdate_sec:=0.05 \
  delete_db:=true \
  database_path:=/home/gongfangyu/b2_data/motion_slam_01/b2_motion_3d.db \
  use_rtabmap_viz:=true
```

### 8.2 Jetson

```bash
mkdir -p /home/nvidia/b2_data/motion_slam_01
ros2 launch b2_factory_inspection mapping_3d.launch.py \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  cloud_backdate_sec:=0.05 \
  delete_db:=true \
  database_path:=/home/nvidia/b2_data/motion_slam_01/b2_motion_3d.db \
  use_rtabmap_viz:=true
```

作用：创建本次专用数据库。`delete_db:=true` 只用于这个新路径，不要指向已有数据库。
启动后机器人不会自动行走，安全速度桥保持锁定。

## 9. 运动前确认 preflight

打开终端 2，加载第 6 节环境，然后逐项执行。

```bash
ros2 topic echo /b2_slam/preflight_ok
```

作用：必须持续看到 `data: true`，然后按 Ctrl+C。

```bash
ros2 topic echo /b2_slam/preflight_report --once
```

作用：记录点云频率、点数、里程计频率、坐标系和失败原因。

```bash
ros2 run tf2_ros tf2_echo map odom
```

作用：确认 RTAB-Map 已发布 `map → odom`。静止时接近单位变换正常，看到连续输出后按 Ctrl+C。

```bash
ros2 topic info /cloud_map -v
ros2 topic echo /cloud_map --once --field width
```

作用：确认 `/cloud_map` 有 RTAB-Map 发布者且点数非零。任一项失败都不要让机器人运动。

## 10. 录制 rosbag

打开终端 3，加载第 6 节环境。必须先看到 rosbag 开始订阅，再让机器人移动。

### 10.1 PC

```bash
ros2 bag record -o /home/gongfangyu/b2_data/motion_slam_01/rosbag_motion_01 \
  /rslidar_points /odom /lf/sportmodestate /lf/lowstate \
  /api/sport/request /api/sport/response /tf /tf_static \
  /b2_slam_3d/cloud_sync /b2_slam_3d/odom_sync \
  /b2_slam/preflight_ok /b2_slam/preflight_report /cloud_map
```

### 10.2 Jetson

```bash
ros2 bag record -o /home/nvidia/b2_data/motion_slam_01/rosbag_motion_01 \
  /rslidar_points /odom /lf/sportmodestate /lf/lowstate \
  /api/sport/request /api/sport/response /tf /tf_static \
  /b2_slam_3d/cloud_sync /b2_slam_3d/odom_sync \
  /b2_slam/preflight_ok /b2_slam/preflight_report /cloud_map
```

作用：保存原始输入、同步后输入、控制请求、TF、健康状态和三维地图输出。

## 11. 第一次运动路线

优先使用熟悉的原厂遥控器低速人工控制。如果需要可重复的短运动，打开终端 4，加载
第 6 节环境后执行：

```bash
ros2 run b2_twist_bridge_py safe_motion_test
```

当前默认参数：

- 前后 `0.30 m/s`，普通动作脉冲 `0.60 s`；
- 左右横移 `0.30 m/s`，横移脉冲 `1.00 s`；
- 每次按键只发送一次 `MOVE`；
- watchdog 到时发送 `STOPMOVE`；
- 启动后默认锁定。

建议执行以下小矩形，不做连续按键：

1. 按 `i` 解锁。
2. 按 `w` 一次，等待自动停车和 2 秒静止。
3. 再按 `w` 两次，每次都等待自动停车和 2 秒静止。
4. 按 `a` 一次，等待自动停车和 2 秒静止。
5. 按 `s` 三次，每次分别等待。
6. 按 `d` 一次，等待自动停车。
7. 按 `x` 明确停车。
8. 按 `i` 重新锁定。

这条路线只使用已通过的纯前后或纯横移，不包含对角线组合运动。实际位移以现场为准，
不要求严格回到胶带中心。任何时候都可以按 `x`，异常时优先使用原厂遥控急停。

不得同时让原厂遥控器和 `safe_motion_test.py` 竞争发送运动命令，二者只选一个运动源。

## 12. 运动中观察项目

- RTAB-Map 不持续报告 ICP registration failure。
- 不持续出现 TF extrapolation 或 QoS incompatible。
- 机器人移动后地图节点开始增加，不再停留在静止时的一个节点。
- `/cloud_map` 随位移更新。
- 墙角、箱体边缘没有明显双重平行重影。
- 地面没有明显分层、倾斜跳变或突然折叠。
- 回到起点附近时地图不会大幅跳动。

可在终端 2 重复执行：

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /odom
ros2 topic echo /b2_slam/preflight_report --once
ros2 run tf2_ros tf2_echo map odom
```

预期仍为点云约 10 Hz、里程计约 20 Hz、preflight PASS。

## 13. 正确停止顺序

1. 先用 `x` 或原厂遥控器让 B2 完全停车。
2. 如果运行了 `safe_motion_test.py`，按 `i` 锁定，再按 Ctrl+C。
3. 在 rosbag 终端按 Ctrl+C，等待保存完成。
4. 最后在 RTAB-Map launch 终端按 Ctrl+C。
5. 不要在数据库仍写入时直接断电。

修复后的 `slam_preflight` 在 Ctrl+C 时不应再出现 shutdown-only `RCLError`。

## 14. 检查结果

### PC

```bash
ros2 bag info /home/gongfangyu/b2_data/motion_slam_01/rosbag_motion_01
ls -lh /home/gongfangyu/b2_data/motion_slam_01
```

### Jetson

```bash
ros2 bag info /home/nvidia/b2_data/motion_slam_01/rosbag_motion_01
ls -lh /home/nvidia/b2_data/motion_slam_01
```

作用：确认 rosbag 有持续时间、消息数量和实际文件大小，数据库不是零字节。

还要保存 RTAB-Map 完整日志、运动前中后地图截图、场地照片和尺寸、运行平台、运动源、
每次按键时间、`preflight_report` 和 `ros2 bag info` 输出。

## 15. 本次通过标准

只有同时满足以下条件，才能写成“运动状态 3D 数据链初步通过”：

- rosbag 完整记录本轮运动；
- 运动期间点云和里程计没有持续中断；
- preflight 运动前通过，运动后仍通过；
- RTAB-Map 节点随位移增加；
- `map → odom → base_link` 连续有效；
- `/cloud_map` 非空并随运动更新；
- 地图没有明显双墙、撕裂、折叠或发散；
- B2 全程能够可靠停车。

这仍不等于 3D SLAM 最终完成。闭环精度、长距离漂移、坡道、动态人员、Nav2 和自主巡检
必须分别验收。

## 16. 常见故障

### 16.1 preflight 变为 false

立即停车并执行：

```bash
ros2 topic echo /b2_slam/preflight_report --once
```

不要修改阈值绕过失败，先判断是点云、里程计、时间戳还是 TF 中断。

### 16.2 QoS incompatible

```bash
ros2 topic info /b2_slam_3d/cloud_sync -v
ros2 param get /rtabmap/rtabmap qos_scan
```

预期同步点云为 Best Effort，`qos_scan` 为 `2`。不要删除该实机配置。

### 16.3 机器人运动但地图不增加

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /cloud_map --once --field width
```

先保存 rosbag，再离线检查 `RGBD/LinearUpdate=0.15`、里程计变化和 ICP 日志，不在现场
同时随机修改多个参数。

### 16.4 双墙或地面分层

结束本轮并保存数据，优先检查：

1. `base_link → lidar_link` 外参；
2. 点云与里程计时间偏差；
3. 里程计在横移和转向时的连续性；
4. ICP 体素和对应距离。

### 16.5 再次看到 `api_id=1034`

只记录出现时间及前后操作，不解释成错误或状态。除非从 Unitree 官方 SDK/协议找到
明确依据，否则代码不得根据 1034 改变控制状态。

## 17. 后续顺序

1. 平地小范围运动 rosbag。
2. 离线检查时间偏差、外参和地图重影。
3. 第二次平地闭环与重复性实验。
4. 缓坡 3D SLAM。
5. Nav2 只读启动和单目标导航。
6. 人员/障碍物局部避障。
7. A/B 目标拍照和候选观测位姿切换。
8. 自动巡检。
9. 无线硬件到位后，再验收 Jetson 双网卡和断网安全。

无线延期不会改变前八项的算法开发顺序；在无线完成前，不能宣称已经达到最终
“电脑只监控”的部署形态。
