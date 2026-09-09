# 宇树 B2 完整巡检系统分阶段调试手册

## 1. 使用范围

本手册覆盖 B2 状态转换、传感器检查、数据录制、3D SLAM、二维导航地图、Nav2
局部避障、巡检路线、拍照质量评价、候选观测位姿调整和电脑只读监控。

完整系统不能一次性直接上真机。必须按 G0 到 G5 的顺序通过；前一阶段未通过，不得启动后一阶段。

2026-09-08/09 的实机记录已确认 G0 网络、DDS、Sport、前进、后退、转向、左右横移
和自动停车通过；静止状态 3D 数据链也已跑通。最新记录位于
`docs/2026-09-09_B2_Jetson_实机调试记录.md`。下一次先按
`docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md` 采集运动 rosbag，不重复这些基础验收。

## 2. Python 功能包结构

```text
src/
├── b2_twist_bridge_py/       # B2 安全速度桥和首次运动测试
├── b2_driver_py/             # B2 状态转里程计、TF、关节状态
├── b2_slam_3d/               # RTAB-Map 3D 建图与定位
├── b2_navigation/            # Nav2 全局规划和局部避障
└── b2_factory_inspection/    # 巡检任务、图像评价、观测位姿切换
```

旧的 `b2_driver`、`b2_twist_bridge` 和 `fixed_route_3d_patrol_node.py` 保留用于对照，但新的启动流程不依赖旧 C++ 驱动，也不使用原来的激光扇区直接绕障逻辑。

## 3. G0：基础运动控制（已通过）

G0 已于 2026-09-08/09 完成。前后实测速度为 `0.30 m/s`；横移只有
`|vy|=0.30 m/s` 产生明确位移，左右各 3/3 成功；左转测试速度为
`0.20 rad/s`。横移保持单次 MOVE 和 watchdog STOPMOVE，不使用临时 20 Hz 重发。
本阶段不再重复，除非更换机器人、升级固件或修改 Sport 请求格式。

## 4. G1：检查状态、雷达、相机和坐标系

### 4.1 加载已经验证的环境

每个新终端都需要执行：

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

`enp8s0`、`192.168.123.99/24` 和 `ROS_DOMAIN_ID=0` 已由实机日志确认。本节只用于
给新终端恢复已有环境，不再执行 ping、DDS 发现或 Sport 动作测试。复制到 Jetson 后
须改成 Jetson 的实际网卡和工作空间路径。
当前 `~/unitree_ros2/setup.sh` 仍硬编码 ROS2 Foxy，Humble 环境不要直接加载它。

### 4.2 安装工程依赖

第一次配置 Jetson 时执行：

```bash
sudo rosdep init
rosdep update
```

作用：初始化 ROS2 依赖索引。`sudo rosdep init` 每台系统只执行一次；若提示已经存在，
不要删除配置，直接执行 `rosdep update`。

```bash
cd /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM
rosdep install --from-paths src --ignore-src -r -y
```

作用：根据各功能包的 `package.xml` 安装 ROS2 依赖。安装失败时保存完整错误，不要用随机软件包名称代替。

2026-09-08 已安装 `ros-humble-pointcloud-to-laserscan`，不要重复安装。

安装后确认三维建图和二维避障的关键依赖：

```bash
ros2 pkg prefix rtabmap_slam
ros2 pkg prefix pointcloud_to_laserscan
ros2 pkg prefix nav2_bt_navigator
```

三条命令都必须输出路径。

### 4.3 编译 Python 主流程

```bash
colcon build --symlink-install --packages-select b2_description b2_twist_bridge_py b2_driver_py b2_slam b2_slam_3d b2_navigation b2_factory_inspection
source install/setup.bash
```

### 4.4 启动 Python 驱动

```bash
ros2 launch b2_driver_py driver.launch.py use_rviz:=false
```

该命令启动机器人模型、Python 状态转换节点和 Python 安全速度桥。速度桥初始为锁定状态，不会因为启动驱动而让机器人移动。

另开终端检查：

```bash
ros2 topic hz /lf/sportmodestate
ros2 topic hz /lf/lowstate
ros2 topic hz /odom
ros2 topic echo /b2/state_ok
```

预期：两个宇树状态话题持续更新，`/odom` 有数据，`/b2/state_ok` 稳定为 `true`。

现场记录同时发现 `/rslidar_points`、`/livox/lidar` 和板载 LIO-SAM 话题，但仅凭
话题名不能确认消息类型和坐标系。先做只读检查：

```bash
ros2 topic type /rslidar_points
ros2 topic hz /rslidar_points
ros2 topic echo /rslidar_points --once --field header
ros2 topic type /livox/lidar
ros2 topic type /lio_sam_ros2/mapping/odometry
ros2 topic echo /lio_sam_ros2/mapping/odometry --once --field header
```

只有消息类型为 `sensor_msgs/msg/PointCloud2` 的点云才能直接接入当前流程。默认继续
使用现场有 `rslidar_sdk` 节点支撑的 `/rslidar_points`。记录真实 `frame_id`；如果
不是 `rslidar`，通过 launch 的 `lidar_frame` 参数传入，不能靠猜测修改 TF。

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link rslidar
```

三条 TF 都必须连续存在。当前 `lidar_link` 安装位姿来自 URDF，但仍需用机器人机械图纸或实际测量核对，不能把未经核对的零位姿当成真实外参。

检查相机：

```bash
ros2 topic list | grep -E 'camera|image'
ros2 topic hz <实际相机图像话题>
ros2 topic type <实际相机图像话题>
```

把实际话题名、消息类型、分辨率、帧率和 `frame_id` 记录下来。

### 4.5 录制第一份 rosbag

```bash
ros2 bag record -o b2_g1_sensors /lf/sportmodestate /lf/lowstate /rslidar_points /odom /dog_imu_raw /lio_sam_ros2/mapping/odometry /tf /tf_static <实际相机图像话题>
```

作用：保存真实传感器数据，之后可以离线检查时间戳、TF 和算法，不必让机器人反复行走。

结束录制使用 `Ctrl-C`。检查数据包：

```bash
ros2 bag info b2_g1_sensors
```

## 5. G2：3D SLAM 建图

### 5.1 启动建图

```bash
ros2 launch b2_factory_inspection mapping_3d.launch.py delete_db:=true use_rtabmap_viz:=true cloud_topic:=/rslidar_points odom_topic:=/odom lidar_frame:=rslidar database_path:=/home/gongfangyu/b2_maps/outdoor_3d_working.db
```

作用：使用 LiDAR 与 B2 高层状态里程计建立 RTAB-Map 六自由度三维地图。
`delete_db:=true` 会删除 `database_path` 指向的同名旧数据库并开始新地图，只能在明确
需要重建时使用。当前流程没有订阅原始 IMU，不能写成“激光—惯性紧耦合 SLAM”。

第一次执行前先创建保存目录：

```bash
mkdir -p /home/gongfangyu/b2_maps
```

### 5.2 建图行走原则

- 先在平地低速小回路验证，再进入坡道。
- 上坡、坡顶、下坡分别停留几秒，避免高速转头。
- 路线最终回到起点形成闭环。
- 地图出现重影、断层或跳变时立即停止，优先检查时间戳和雷达外参。

建图阶段建议由现场人员使用 B2 原厂遥控器低速带机器人行走；本阶段没有启动完整
定位/雷达安全许可，禁止为了建图而手工伪造速度桥心跳。

检查地图和定位关系：

```bash
ros2 topic echo /b2_slam/preflight_report
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /cloud_map
```

### 5.3 保存地图

确认本次指定的数据库存在：

```bash
ls -lh /home/gongfangyu/b2_maps/outdoor_3d_working.db
```

复制为实验版本，文件名包含日期和场景：

```bash
mkdir -p ~/b2_maps
cp /home/gongfangyu/b2_maps/outdoor_3d_working.db /home/gongfangyu/b2_maps/outdoor_3d_20260908.db
```

同时保存供 Nav2 使用的二维投影地图。当 RTAB-Map 节点位于
`/rtabmap` 命名空间时，其二维占据地图话题为 `/rtabmap/map`。先确认：

```bash
ros2 topic type /rtabmap/map
ros2 topic hz /rtabmap/map
```

存在后执行：

```bash
ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f /home/gongfangyu/b2_maps/outdoor_2d_20260908
```

最终应生成 `.yaml` 和 `.pgm` 两个文件。3D 数据库负责三维定位和地图研究，二维投影负责 Nav2 全局代价地图；两者来自同一次建图实验。

## 6. G3：Nav2 导航与局部避障

### 6.1 先进行只读检查

```bash
ros2 launch b2_factory_inspection autonomous_inspection.launch.py --show-args
```

作用：只显示启动参数，不控制机器人。

### 6.2 启动完整节点但不启动任务

```bash
ros2 launch b2_factory_inspection autonomous_inspection.launch.py map:=/home/gongfangyu/b2_maps/outdoor_2d_20260908.yaml database_path:=/home/gongfangyu/b2_maps/outdoor_3d_20260908.db use_rviz:=false use_aruco:=true image_topic:=/camera/image_raw
```

启动后巡检任务仍处于空闲状态，Python 安全速度桥仍处于锁定状态。先检查：

```bash
ros2 topic echo /b2/state_ok
ros2 topic echo /b2/autonomy_ok
ros2 topic echo /b2/autonomy_reason
ros2 topic echo /b2_slam/preflight_report
ros2 topic echo /b2/control/enabled
ros2 topic hz /scan
ros2 action list
ros2 lifecycle nodes
ros2 run tf2_ros tf2_echo map base_link
```

必须满足：`/b2/state_ok` 和 `/b2/autonomy_ok` 稳定为 `true`、安全原因显示条件满足、
控制锁为 `false`、`/scan` 连续、存在 `/navigate_to_pose`、Nav2 生命周期节点为 active、
`map` 到 `base_link` 连续。任何一项不满足都不能解锁。

### 6.3 空场导航验证

第一次导航不要启动巡检任务。打开一个专门的终端发布有监督测试心跳：

```bash
ros2 topic pub -r 5 /inspection/control_heartbeat std_msgs/msg/Bool "{data: true}"
```

作用：以每秒 5 次模拟任务管理器心跳；关闭此终端后安全速度桥会在 0.60 秒内自动锁定。

另开终端确认机器人状态正常后启用控制：

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: true}"
```

然后在 RViz 中选择距离机器人约 0.5 米、无遮挡的目标点，观察规划路径和机器人是否可随时停车。若需要立即锁定控制：

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: false}"
```

结束测试时先关闭控制锁，再用 `Ctrl-C` 关闭测试心跳。不得让测试心跳在无人监督时持续运行。

### 6.4 障碍物验证

依次进行静态泡沫箱、缓慢横穿假人和窄通道试验。每次只改变一个变量。检查：

```bash
ros2 topic echo /local_costmap/costmap
ros2 topic echo /cmd_vel
ros2 topic echo /b2/control/status
```

当前 Nav2 参数只是保守基线，机器人 footprint、膨胀半径、坡度和制动距离必须用实测数据重新标定。

## 7. G4：拍照和候选观测位姿

### 7.1 配置路线

编辑：

```text
src/b2_factory_inspection/config/patrol_waypoints.yaml
```

每个巡检目标的第一项是正常拍摄位姿，后续项是图像不合格时依次尝试的候选位姿。所有候选位姿都必须提前在 RViz 中确认可通行，禁止用“沿相机方向盲目前进”生成位姿。

### 7.2 户外验证目标

把 ArUco 10 号标记固定在目标 A 板中心附近，把 20 号标记固定在目标 B 板中心附近。
映射和目标区域扩展配置位于：

```text
src/b2_factory_inspection/config/inspection_targets.yaml
```

ArUco 只用于功能验证。进入工厂后，真实目标检测程序只需发布：

```text
/inspection/target_bbox
std_msgs/msg/Float32MultiArray
[中心x, 中心y, 宽, 高, 置信度]
```

前四项均为 0 到 1 的归一化数值。

`roi_scale_x/y` 表示以标记宽高为单位向外扩展的目标区域，目的是评价标记周围的仪表、
文字或设备板，而不是只评价黑白标记是否清楚。若标记不在目标中心，使用
`roi_offset_x/y` 调整；每次修改后要在保存照片上人工核对区域是否覆盖真实目标。

### 7.3 启动巡检

确认场地安全、定位正常、路线坐标已经实测后执行：

```bash
ros2 service call /inspection/start std_srvs/srv/Trigger "{}"
```

查看状态和质量报告：

```bash
ros2 topic echo /inspection/state
ros2 topic echo /inspection/photo_quality_report
ros2 topic echo /b2/control/status
```

照片目录中每张 `.jpg` 会同时生成同名 `.json` 质量报告，包含清晰度、曝光、目标面积、
置信度、是否合格及失败原因，后续论文实验必须保留二者。

随时暂停：

```bash
ros2 service call /inspection/pause std_srvs/srv/Trigger "{}"
```

暂停会取消当前导航目标、关闭控制锁并发送停车指令。

`mission_autostart` 在调试阶段默认为 `false`。路线、外参、制动距离和所有
候选位姿逐个验证前，不得打开自动启动。

## 8. G5：Jetson 自主运行与电脑只读监控

最终所有决策节点运行在 Jetson：状态转换、3D 定位、Nav2、图像质量、候选位姿和任务管理器。电脑只运行 RViz 和状态查看命令，不发布 `/cmd_vel`、`/api/sport/request` 或任务服务。

只有 G0～G4 全部通过后，才在 Jetson 本机使用自动启动：

```bash
ros2 launch b2_factory_inspection autonomous_inspection.launch.py map:=/home/gongfangyu/b2_maps/outdoor_2d_20260908.yaml database_path:=/home/gongfangyu/b2_maps/outdoor_3d_20260908.db use_rviz:=false mission_autostart:=true mission_autostart_delay_sec:=20.0
```

作用：节点启动至少 20 秒后，仍需确认机器人状态、二维雷达、
`map→base_link` 定位和 Nav2 全部就绪，任务管理器才会在 Jetson 内部解锁并开始。
任一安全条件不满足时会继续等待，不会强行行走。

电脑端只读检查：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
rviz2
```

生产运行前，应使用 CycloneDDS 配置限制接口和允许的节点，并通过防火墙阻止监控电脑发布运动话题。仅靠操作约定不能证明电脑是只读的。

## 9. 无线板卡验收要求

当前状态：合同和采购尚未确定，无线测试延期。该延期不阻塞有线运动 3D SLAM、
rosbag 和离线算法验证；无线到位前不得宣称最终双网卡拓扑已完成。

无线板卡应明确支持 Ubuntu 22.04 当前内核，优先选择主线内核已有驱动的芯片；同时确认 Jetson 的接口、电压、天线接口和机身安装空间。购买前在 Jetson 执行：

```bash
uname -a
lspci -nn
lsusb
rfkill list
```

安装后至少测试连续 ping、ROS2 话题频率、1 小时数据传输和断网安全停车。具体型号必须等 Jetson 型号、可用接口和内核版本确认后再选择。

## 10. 当前不能跳过的实机标定

- B2 速度正方向和最大安全速度；
- LiDAR 到 `base_link` 的六自由度外参；
- 相机到 `base_link` 的六自由度外参；
- 点云和机器人状态的真实时间戳关系；
- B2 在平地、坡道和湿滑地面的制动距离；
- 机器人真实 footprint 与动态摆腿空间；
- 图像清晰度、目标面积、曝光阈值；
- 每个候选观测位姿的碰撞安全性。

这些数据无法仅靠离线代码猜测。没有实测记录时，代码只能算“已实现待标定”，不能称为完整验收。
