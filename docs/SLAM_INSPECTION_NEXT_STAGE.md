# B2 SLAM 与巡检控制下一阶段调试手册

> 本文保留长期分阶段路线。下一次实验室操作请优先使用
> `docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md`；2026-09-09 已完成后退、左右横移和
> 静止 3D 数据链验收，无线通信因采购延期暂不作为运动 SLAM 的前置条件。

## 1. 已确认基线

本手册以 `docs/2026-09-09_B2_Jetson_实机调试记录.md` 为准。下列项目已经完成，不再重复：

- PC 有线网卡 `enp8s0` 与 B2 通信；
- ROS2 Humble、CycloneDDS、`ROS_DOMAIN_ID=0`；
- `/api/sport/request` 和 `/api/sport/response` 通信；
- Python 安全锁、站立、平衡站立、前进、左转和自动停车；
- 前进 `0.30 m/s` 能可靠形成位移，`0.10～0.20 m/s` 只有腿部动作；
- 左转 `0.20 rad/s` 和 `0.60 s` 短脉冲可用。
- 后退 `0.30 m/s` 3/3 成功；
- 左右横移 `0.30 m/s` 各 3/3 成功，单次 MOVE 即可，不依赖 20 Hz 重发；
- 静止状态 3D RTAB-Map 数据链和非空 `/cloud_map` 已通过。

除非机器人固件、网络拓扑或 Sport 请求代码发生变化，否则不要重新执行网络排查和
基础动作测试。下一次实机工作直接从只读传感器检查开始。

## 2. 现场记录给出的 SLAM 数据候选

2026-09-08 保存的 topic 清单同时包含：

| 用途 | 已发现话题 | 当前处理原则 |
|---|---|---|
| RoboSense 点云 | `/rslidar_points` | 有对应 `rslidar_sdk` 节点，作为当前默认输入 |
| Livox 数据 | `/livox/lidar` | 消息类型和坐标系尚未记录，不能直接切换 |
| 板载 LIO-SAM 里程计 | `/lio_sam_ros2/mapping/odometry` | 类型、父子坐标系和 TF 尚未记录 |
| 板载 LIO-SAM 点云 | `/lio_sam_ros2/mapping/cloud_registered` | 可用于对照和可视化，暂不作为 RTAB-Map 原始输入 |
| Unitree SLAM | `/unitree/slam_mapping/points` | 先确认类型和用途，不调用未知服务 |
| B2 状态里程计 | `/lf/sportmodestate` | 当前 Python 驱动转换为 `/odom` |

话题“存在”不等于数据能够直接接入。当前节点只接收
`sensor_msgs/msg/PointCloud2` 和 `nav_msgs/msg/Odometry`。

## 3. S1：只读确认点云、里程计和坐标系

### 3.1 加载已验证环境

每个新终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/gongfangyu/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export B2_NET_IF=enp8s0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${B2_NET_IF}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
export ROS_DOMAIN_ID=0
```

这些命令只是恢复已验证环境，不需要再执行 ping 或 Sport 测试。

### 3.2 确认两路雷达消息类型

```bash
ros2 topic type /rslidar_points
ros2 topic type /livox/lidar
```

当前优先使用 `/rslidar_points`。它必须输出：

```text
sensor_msgs/msg/PointCloud2
```

如果 `/livox/lidar` 输出 Livox 自定义消息，当前 RTAB-Map 不能直接订阅，需要先由
Livox 驱动输出标准 PointCloud2。不要只改话题名强行启动。

继续记录频率和消息头：

```bash
ros2 topic hz /rslidar_points
ros2 topic echo /rslidar_points --once --field header
ros2 topic info /rslidar_points -v
```

必须记录实际 `frame_id`。后续命令中的 `lidar_frame:=rslidar` 要替换为该值。

### 3.3 确认板载 LIO-SAM，但暂不切换

```bash
ros2 topic type /lio_sam_ros2/mapping/odometry
ros2 topic hz /lio_sam_ros2/mapping/odometry
ros2 topic echo /lio_sam_ros2/mapping/odometry --once --field header
ros2 topic echo /lio_sam_ros2/mapping/odometry --once --field child_frame_id
```

只有类型为 `nav_msgs/msg/Odometry`，并且父子坐标系能够无冲突地接入
`odom → base_link` TF 树时，才考虑将其作为 `odom_topic`。现阶段继续使用 `/odom`，
避免 B2 Python 驱动和板载 LIO-SAM 同时发布冲突的 TF。

### 3.4 编译本阶段代码

```bash
cd /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM
colcon build --symlink-install --packages-select b2_twist_bridge_py b2_driver_py b2_slam b2_slam_3d b2_navigation b2_factory_inspection
source install/setup.bash
```

## 4. S2：静止状态启动 3D SLAM 输入检查

S2 不需要搭建专用场景，也不需要让机器狗行走。车间、实验室或普通户外
场地都可以，只要雷达周围不是全空旷、没有人近距离挡住雷达，并且机器狗
能稳定站立即可。本步只验证话题、数据类型、频率、时间戳和 TF，不判断地图质量。

先只显示参数，不启动节点：

```bash
ros2 launch b2_factory_inspection mapping_3d.launch.py --show-args
```

确认点云类型和 `frame_id` 后启动。该命令不会自动开始巡检，安全速度桥保持锁定：

```bash
mkdir -p /home/gongfangyu/b2_maps
ros2 launch b2_factory_inspection mapping_3d.launch.py \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  delete_db:=true \
  database_path:=/home/gongfangyu/b2_maps/b2_3d_working.db \
  use_rtabmap_viz:=true
```

另开终端查看新增的只读诊断：

```bash
ros2 topic echo /b2_slam/preflight_report
ros2 topic echo /b2_slam/preflight_ok
ros2 topic hz /b2_slam_3d/cloud_sync
ros2 topic hz /b2_slam_3d/odom_sync
ros2 run tf2_ros tf2_echo base_link rslidar
```

`/b2_slam/preflight_ok` 必须稳定为 `true`。报告会检查：

- 点云是否包含 `x/y/z` 字段以及至少 1000 个点；
- 点云是否达到 5 Hz；
- 里程计是否达到 10 Hz；
- 点云和里程计时间戳是否新鲜；
- 里程计是否为 `odom → base_link`；
- `base_link` 到点云 `frame_id` 的 TF 是否存在。

检查失败时不要修改代码绕过，保存 `/b2_slam/preflight_report` 的完整原因。

## 5. S3：小范围三维建图

S2 通过后，使用原厂遥控器带机器人低速走一个平地小回路。本阶段不调用
`/inspection/start`，也不发布测试心跳。

S3 不需要搭 A/B 拍照目标，但不建议在空旷草地、大面积玻璃旁或只有重复
平行墙面的长走廊做第一张图。首选约 `5 m × 5 m` 以上、有墙角、柱子、箱体
或停放设备等静态三维几何特征的区域，能走成闭合回路即可。现场人员不要在
机器狗和墙面之间反复走动。坡道等 S3 平地闭环通过后再测，不在首次输入检查
时加入。

观察：

```bash
ros2 topic hz /cloud_map
ros2 topic hz /rtabmap/map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

合格标准：

- 点云地图连续，没有明显双墙、撕裂和周期跳变；
- 回到起点后能够产生闭环，地图不发生大幅错误折叠；
- `map → odom → base_link` 连续；
- RTAB-Map 无持续 TF extrapolation、空点云或 ICP 注册失败。

结束后先按 `Ctrl-C` 正常关闭，再保存二维投影：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /rtabmap/map \
  -f /home/gongfangyu/b2_maps/b2_2d_test
```

三维数据库已经由 `database_path` 持续写入，不要在下一次启动时继续使用
`delete_db:=true`。继续同一地图应显式设置 `delete_db:=false`。

## 6. S4：录制可离线复现的数据

在正式调参前录制一次小回路：

```bash
ros2 bag record -o b2_slam_small_loop \
  /rslidar_points \
  /lf/sportmodestate \
  /lf/lowstate \
  /dog_imu_raw \
  /lio_sam_ros2/mapping/odometry \
  /lio_sam_ros2/mapping/cloud_registered \
  /tf \
  /tf_static
```

如果相机已接入，再增加实际的彩色图像、深度图和相机内参话题。不要用
`/camera/image_raw` 占位名称代替实际话题。

完成后：

```bash
ros2 bag info b2_slam_small_loop
```

需要保存：topic 类型、帧数、时长和数据包总大小。后续 RTAB-Map、LIO-SAM 对照和
时间戳调整都优先在 rosbag 上进行，减少机器狗重复行走。

## 7. N1：第一次 Nav2 只读联调

2026-09-08 已安装 `ros-humble-pointcloud-to-laserscan`，不需要重复安装。
启动 Nav2 前只做下面的只读确认：

```bash
ros2 pkg prefix pointcloud_to_laserscan
```

作用：确认点云转二维激光包可被 ROS2 找到。当前已验证的输出是
`/opt/ros/humble`。

地图和定位验证完成后，先启动完整节点但保持任务停止：

```bash
ros2 launch b2_factory_inspection autonomous_inspection.launch.py \
  map:=/home/gongfangyu/b2_maps/b2_2d_test.yaml \
  database_path:=/home/gongfangyu/b2_maps/b2_3d_working.db \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  use_rviz:=false \
  mission_autostart:=false
```

只读确认：

```bash
ros2 topic echo /b2_slam/preflight_report
ros2 topic echo /b2/autonomy_reason
ros2 topic echo /b2/control/enabled
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
```

控制锁必须保持 `false`。在 `/b2_slam/preflight_ok`、`/b2/autonomy_ok`、`/scan`
和定位 TF 全部稳定前，不调用 `/inspection/start`。

## 8. 当前自主速度约束

根据 2026-09-08/09 实测，控制代码采用：

- 最大前进速度 `0.30 m/s`；
- 非零但低于 `0.30 m/s` 的正向 Nav2 指令提升到 `0.30 m/s`；
- 最大转向速度 `0.20 rad/s`；
- 后退和横移单轴方向已经允许，横移有效值和上限均为 `0.30 m/s`；
- 心跳、状态、SLAM 前置检查、二维雷达或定位任一超时都会关闭控制锁并停车。

单轴接口通过不代表 Nav2 闭环已经通过。第一次 Nav2 路线仍只使用前进和转向；
DWB 横移采样、BackUp 恢复行为和组合平移应分别进行安全验收。

## 9. 下一次需要带回的记录

下一次实机调试结束后，请保存：

1. `/rslidar_points` 和 `/livox/lidar` 的消息类型；
2. 两路雷达的频率、`frame_id` 和时间戳样例；
3. `/lio_sam_ros2/mapping/odometry` 的类型、父坐标系和子坐标系；
4. `/b2_slam/preflight_report`；
5. RTAB-Map 启动终端完整日志；
6. 小回路 rosbag；
7. 地图截图、数据库、二维 `.yaml/.pgm`；
8. 实际彩色图、深度图和相机内参话题。

有了这些数据后，再决定是继续使用 RTAB-Map + B2 状态里程计，还是把板载 LIO-SAM
作为里程计/建图对照。现在不应仅凭话题名称提前选择算法。
