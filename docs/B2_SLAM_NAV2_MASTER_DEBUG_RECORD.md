# Unitree B2 SLAM、Nav2 与自主巡航总调试记录

最后更新：2026-10-16\
设备：Unitree B2、Jetson Orin Nano、RoboSense LiDAR、Ubuntu 22.04、ROS 2 Humble

## 1. 文档用途

本文档是后续调试的唯一入口，汇总当前可信结论、关键文件、风险和下次
操作顺序。原始详细记录保留不删：

- `B2_SLAM_debug_record_2026-09-29.md`：DDS、B2 状态、里程计与
  RTAB-Map 数据链恢复；
- `B2_DEBUG_LOG_2026-09-29.md`：Nav2 安全门、地图检查和安全监视器；
- `B2_NAV2_SLAM_DEBUG_GUIDE_2026-09-30.md`：现场标准操作流程；
- `B2_DEBUG_LOG_2026-10-13.md`：本次建图、标点、导航与转向标定的详细数据。
- `B2_DEBUG_LOG_2026-10-15.md`：断电恢复、参数持久化、Nav2/RViz 恢复及
  下一轮坡道、楼梯和闭环误差测试入口。
- `B2_DEBUG_LOG_2026-10-16.md`：新室内平地运动建图、磁盘清理、
  Nav2 点云 QoS/局部代价地图修复以及 AMCL 初始位姿故障。

## 2. 最终目标与当前进度

| 阶段 | 状态 | 结论 |
|---|---|---|
| ROS2 通信与 B2 控制 | 已通过 | B2 状态、点云、里程计、Sport API 可用 |
| 3D SLAM | 已通过平地建图 | RTAB-Map 点云地图和数据库已保存 |
| 2D 导航地图 | 已通过 | 新室内地图 0.05 m/像素已保存 |
| 电脑端显示 | 已通过 | RTAB-Map GUI 和 Nav2 RViz 均可显示 |
| 地图上标点 | 已通过 | A、B、C 三点已保存，非安全点可拒绝 |
| 静态路径规划 | 已通过 | 当前位置→A、A→B、B→C、C→A 均成功 |
| 平地短距离自主行走 | 部分通过 | 可连续行走，转向参数刚完成单轴标定 |
| AMCL 新地图初始定位 | 故障调试中 | 已禁止零位姿伪定位，但尚未成功设定真实位姿 |
| Nav2 自主原地转向 | 待复测 | 单轴 `±0.35 rad/s` 已成功，尚未复测 Nav2 闭环 |
| A→B→C→A 巡航 | 待测 | 只完成规划，未执行完整闭环 |
| 上下坡 | 未开始 | 平地闭环稳定 3 次后再开始 |
| 上下楼梯 | 未开始 | 必须单独做地图、步态与安全验证 |
| 闭环误差测量 | 未开始正式数据 | 网线中断段和手工回到起点段不计入 |

## 3. 硬件、网络与 ROS2 基线

- Jetson 与 B2 有线端口：`enP8p1s0`；
- Jetson 有线地址：`192.168.123.211/24`；
- 2026-10-13 手机热点下 Jetson 无线地址曾为 `10.239.201.107/24`；
- 热点会重新分配地址，下次必须以 `ip -br address show wlP1p1s0`
  的实时结果为准；
- `ROS_DOMAIN_ID=0`；
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；
- CycloneDDS 建图/控制端只绑定 B2 有线网口 `enP8p1s0`；
- `/rslidar_points` 约 9–10 Hz，`frame_id=rslidar`；
- `/odom` 约 20 Hz，`odom -> base_link`；
- 网线松动时会出现 `NO-CARRIER` 和 CycloneDDS 网卡不存在错误。

每个 Jetson 终端的环境：

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS2CLI_DISABLE_DAEMON=1
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="enP8p1s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/Unitree-B2-SLAM/install/setup.bash
```

## 4. 已保存的地图和巡航点

- 2026-10-16 新 RTAB-Map 数据库：
  `/home/jetson/b2_maps/b2_room_clean_20261016_01.db`，约 18 MB；
- 2026-10-16 新二维地图：
  `/home/jetson/b2_maps/b2_room_clean_20261016_01_2d.yaml`；
- 2026-10-16 新地图图像：
  `/home/jetson/b2_maps/b2_room_clean_20261016_01_2d.pgm`，约 99 KB；
- 新地图分辨率 `0.05 m`，原点 `[-8.95, -8.15, 0]`；
- RTAB-Map 数据库：
  `/home/jetson/b2_maps/b2_flat_loop_20261013_01.db`，约 31 MB；
- 二维地图：
  `/home/jetson/b2_maps/b2_flat_loop_20261013_02_2d.yaml`；
- 地图图像：
  `/home/jetson/b2_maps/b2_flat_loop_20261013_02_2d.pgm`；
- 巡航点：
  `/home/jetson/b2_maps/b2_patrol_waypoints.yaml`；
- A：`(-1.122, 2.006)`，yaw `65.22°`；
- B：`(-0.753, 2.807)`，yaw `57.99°`；
- C：`(-0.368, 3.423)`，yaw `-118.05°`。

注意：数据库编号 `_01` 和二维地图编号 `_02` 不一致，可用于继续调试，
但正式误差实验必须使用统一的新编号。

## 5. 安全联锁与正确顺序

两级锁默认为 `false`：

- 导航门：`/b2/navigation/set_armed`；
- 底层速度桥：`/b2/control/set_enabled`。

正确解锁顺序：

1. 取消旧目标；
2. 确认 `/b2/state_ok=true`、`/b2/autonomy_ok=true`；
3. 解锁导航门；
4. 等待 0.5 s，让 `/inspection/control_heartbeat` 到达底层桥；
5. 启用底层控制锁；
6. 再发送目标；
7. 每次目标结束后主动关闭两级锁。

动作状态码：`4=SUCCEEDED`，`5=CANCELED`，`6=ABORTED`。

## 6. 实机速度标定

### 6.1 平移

- 前后有效值：`0.30 m/s`；
- 左右横移有效值：`0.30 m/s`；
- 更小值可能只有关节/重心动作而无明确位移。

### 6.2 原地转向（2026-10-13）

| 命令 | 持续 | 实际结果 |
|---|---:|---|
| `+0.25 rad/s` | 约 1.7 s | 无明确转向，约 `0.6°` |
| `+0.30 rad/s` | 10 帧，约 1 s | 无明确转向，约 `1.2°` |
| `+0.35 rad/s` | 10 帧，约 1 s | 左转约 `12.2°`，通过 |
| `-0.35 rad/s` | 10 帧，约 1 s | 右转约 `16.8°`，通过 |

当前结论：当前 B2、地面和运动模式下，自主速度桥的
`min_effective_angular_z` 和 `max_angular_z` 均使用 `0.35 rad/s`，
不再继续提高。

## 7. 2026-10-13 断电时的精确状态

- 机器人最后已关闭底层控制锁和导航门；
- Jetson 因电量耗尽断电，不是主动正常关机；
- 电脑端源码已固化 `0.35 rad/s`；
- 电脑端 14 项相关单元测试通过，三个 ROS2 包构建通过；
- `rsync` 因 `No route to host` 失败，Jetson 工作区尚未获得新默认值；
- Jetson 断电前运行的 `0.35` 来自启动时临时参数，重启后失效；
- Jetson 根分区曾为 99%，约剩 2.3 GB；
- `~/b2_bags` 约 32 GB，未确认备份前不删除。

## 8. 下次上电的准确顺序

### 8.1 电源与磁盘

1. 先给 Jetson 充电并使用稳定供电；
2. 确认 B2 站稳、周围无人、手持遥控/急停；
3. 检查突然断电后的系统与空间：

```bash
df -h /
ls -lh /home/jetson/b2_maps/b2_flat_loop_20261013_01.db /home/jetson/b2_maps/b2_flat_loop_20261013_02_2d.yaml /home/jetson/b2_maps/b2_flat_loop_20261013_02_2d.pgm /home/jetson/b2_maps/b2_patrol_waypoints.yaml
```

### 8.2 恢复网络

Jetson 上查看新无线地址：

```bash
ip -br address show wlP1p1s0
```

电脑上确认与新地址同网段，再进行 `ping` 和 SSH。

### 8.3 必须先同步和构建

电脑端将下面命令中 `<JETSON_WIFI_IP>` 替换为实时地址：

```bash
rsync -av --exclude '__pycache__' "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_twist_bridge_py" "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_driver_py" "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_navigation" jetson@<JETSON_WIFI_IP>:/home/jetson/Unitree-B2-SLAM/src/
```

Jetson 空闲终端中一行构建：

```bash
cd ~/Unitree-B2-SLAM && source /opt/ros/humble/setup.bash && source ~/unitree_ros2/cyclonedds_ws/install/setup.bash && colcon build --symlink-install --packages-select b2_twist_bridge_py b2_driver_py b2_navigation --allow-overriding b2_twist_bridge_py b2_driver_py b2_navigation
```

### 8.4 启动与定位

1. 驱动终端重新加载 `install/setup.bash`，不带角速度覆盖参数启动；
2. Nav2 终端重启定位与 RViz；
3. 用 RViz `2D Pose Estimate` 根据真实位置重新初始化 AMCL；
4. 如机器人断电后未被搬动，上次最后参考位姿约为
   `(-1.114, 2.382, -10.8°)`，但必须以 RViz 地图实际重合为准；
5. 确认 `map -> base_link`、`/scan`、地图和安全状态恢复。

### 8.5 明天的运动顺序

1. 两级锁保持关闭，先做只规划测试；
2. 用 Nav2 做约 `30°–45°` 的自主原地转向；
3. 观察是否在目标角度附近来回振荡；
4. 原地转向通过后，只执行当前位置→A；
5. 再分段执行 A→B、B→C、C→A；
6. 一次闭环通过后才录制正式 rosbag，连续完成 3 次平地闭环。

## 9. 闭环误差测量方法

正式测试开始前记录 `map -> base_link` 起点
`(x_s, y_s, yaw_s)`，结束后记录
`(x_e, y_e, yaw_e)`。

- 平面闭环位置误差：
  `e_p = sqrt((x_e-x_s)^2 + (y_e-y_s)^2)`；
- 闭环航向误差：
  `e_yaw = wrap(yaw_e-yaw_s)`，换算为度；
- 同时记录里程计起终点，但不用里程计代替 AMCL/TF 的地图坐标误差；
- 每次试验记录地面、负载、电量、路线、耗时、是否触发恢复行为和是否人工干预。

网线中断、定位丢失、手工搬动、手工回到起点或中途重新设定 AMCL 的试验
不计入正式误差数据。

## 10. 坡道和楼梯前置条件

不得从当前状态直接进入坡道或楼梯。必须先满足：

1. 平地 A→B→C→A 连续 3 次无碰撞、无定位丢失、无人工接管；
2. 实测制动距离、转向超调和障碍物安全余量；
3. 单独验证坡道/楼梯上的激光高度过滤、局部代价地图和机身俯仰角；
4. 上下坡先遥控低速走查，再做自主测试；
5. 楼梯测试必须单独设计步态切换、台阶几何检测、摔落防护和人工急停，
   不能直接把平地 Nav2 参数用于楼梯。

## 11. 立即停止条件

出现任意一项立即关闭两级锁：

- 有线网卡 `carrier=0`；
- `/scan`、`/odom`、`map -> base_link` 超时；
- `/b2/autonomy_ok=false`；
- 位置或航向明显跳变；
- 转向在目标附近反复振荡；
- 机器人接近楼梯边缘、坡道边缘、人或未标定障碍物；
- Jetson 电量、供电或温度异常。

## 12. 2026-10-15 恢复后的当前状态

- 电脑端 `b2_twist_bridge_py`、`b2_driver_py`、`b2_navigation` 已成功同步到
  Jetson；2026-10-13 记录的“尚未同步”只描述当时的历史状态；
- Jetson 运行节点和 Nav2 安装配置均确认使用
  `min_effective_angular_z=0.35`、`max_angular_z=0.35`；
- B2 `mode=9`，点云约 9.99 Hz，里程计约 20.02 Hz；
- 地图报告、自主条件、`/scan` 和 Nav2 五个生命周期节点均曾在本日通过；
- 驱动与 Nav2/RViz 已改为分离运行，避免 RViz 强制退出时影响驱动；
- 机器人已经被搬动，旧初始位姿无效；本日尚未完成新位置的 AMCL 初始化；
- 本日没有解锁、没有发送运动命令，最后可核对的两级安全锁均为 false；
- Jetson 根分区仍为 99%，电量再次耗尽，运动复测、完整巡航、坡道和楼梯均
  未执行；
- 下次准确起点见 `B2_DEBUG_LOG_2026-10-15.md` 第七节。

## 13. 2026-10-16 建图与定位调试后的当前状态

### 13.1 已完成的实机验证

- RTAB-Map 平地运动建图已完成，并返回起点附近；
- 建图开始位姿约 `(-2.031, 3.066, -12.60°)`，结束约
  `(-2.301, 3.032, -12.15°)`；
- 单次地图/TF 平面残差约 `0.272 m`，航向残差约 `0.46°`；
- 新数据库和 0.05 m 二维地图已保存；
- Jetson 磁盘可用空间由约 2.3 GB 恢复到约 9.3 GB；
- 导航时间戳修正节点使用 `reliable` 输入后，
  `/b2_navigation/cloud_fixed` 和 `/scan` 恢复到约 9.5～10 Hz；
- 局部代价地图已从高度范围不合适的 `VoxelLayer` 改为
  `ObstacleLayer`；
- AMCL 已改为订阅 `/b2_navigation/initialpose`，与厂商
  `/QtServer` 的公共 `/initialpose` 隔离；
- `set_initial_pose=false` 已在 Jetson 构建和运行，AMCL 不再启动时
  自动伪造 `(0,0,0)` 定位；
- 最后可核对的导航门和底层控制锁均为 `false`。

### 13.2 尚未解决的阻塞

- 通过 `/b2_navigation/initialpose` 发布已知位姿后，AMCL 仍未产生
  `/amcl_pose` 和 `map -> base_link`；
- `/request_nomotion_update` 未使定位恢复；
- 下次首先检查并调用 `/set_initial_pose`
  `nav2_msgs/srv/SetInitialPose` 服务，同时保留 AMCL 终端的
  `initialPoseReceived` / `Setting pose` 或拒绝原因；
- 曾经在 AMCL 原点伪定位时出现 `autonomy_ok=true`，因此现有安全
  监视器尚不能证明定位真实。修复前不得仅凭该布尔量解锁；
- 机器人电量耗尽，导航计划、自主运动、坡道和楼梯本日均未执行。

### 13.3 下次强制顺序

1. 稳定供电，恢复 Domain 0 和 CycloneDDS 有线绑定；
2. 启动驱动与 Nav2，两级锁保持 `false`；
3. 确认初始位姿前没有 `/amcl_pose` 和 `map -> base_link`；
4. 调用 `/set_initial_pose` 服务，参考位姿
   `(-2.301, 3.032, -12.15°)`；
5. 同时验证 AMCL 输出、TF、RViz 机器人位置以及 `/scan` 与地图重合；
6. 定位通过后只做 `ComputePathToPose`，不解锁；
7. 纯计划通过后再单独设计短距离、低风险、现场看护的实机运动；
8. 平地巡航连续 3 次通过后才考虑坡道；楼梯必须另行验证官方步态、
   台阶几何、防跌落和双人看护。
