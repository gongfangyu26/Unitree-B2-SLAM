# B2 三维激光建图说明

该功能包使用 B2 里程计、RoboSense `PointCloud2` 和 RTAB-Map 建立六自由度
三维地图，目标场景包含平地、上坡、下坡和高度变化。

## 当前证据边界

2026-09-09 至 2026-10-16 的实机记录已经确认：

- `/rslidar_points` 有真实点云；
- `/odom`、`odom -> base_link` 和雷达 TF 可用；
- `sensor_time_sync`、`slam_preflight` 可运行；
- RTAB-Map 能接收真实点云并输出非空三维点云地图；
- 平地运动建图、一次返回起点和数据库/二维地图保存已通过实机验证。

坡道/楼梯地图质量、漂移、闭环和楼梯运动安全性仍为**待实机验证**；
不得用平地结果替代地形实验。

## 走廊三维点云导出模式（2026-10-30）

默认 `grid_3d=false` 沿用历史 Nav2 投影配置；这会把用于地图发布的局部点云
投到 XY 平面。要保存具有高度结构的走廊地图，应在全新数据库上显式使用：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py \
  start_driver:=false delete_db:=false \
  database_path:=/home/jetson/b2_maps/corridor3d_20261030_01/corridor_3d.db \
  grid_3d:=true grid_max_obstacle_height:=0.0 grid_ray_tracing:=false \
  use_rtabmap_viz:=false use_rviz:=false
```

先在现场确认停稳，正常退出房间 SLAM，保留其数据库；移到走廊起点后，再创建
不存在的新实验目录并运行上述命令。`start_driver=false` 要求唯一驱动已在运行。
`Grid/3D=true` 保留高度；障碍高度上限 0 关闭该裁剪；关闭体素射线追踪用于
减少三维表面采集的计算/内存，不代表已生成合格的二维可通行地图。
**实机已验证（2026-10-30，走廊局部采集）：** 新数据库路径及上述三个运行参数
已查询确认；遥控移动后地图包含多个图节点和高度结构，Jetson 成功导出
`pilot_02_cloud.pcd`，有效点数 43,566、文件约 681 KiB。
百米全程、定量地图精度、漂移、闭环和导航仍待验证，详见
[当日调试记录](../../docs/B2_DEBUG_LOG_2026-10-30.md)。

RTAB-Map 会按既有范围、体素、地面分割及噪声规则生成过滤后的地图点云，
PCD 不是未经处理的全部原始扫描。采集时保持 RViz 订阅地图点云，确认地图
随走廊运动而扩展，停稳后核对完整路线已显示，再运行下文的 `pointcloud_snapshot`。
保存 `.pcd` 的同时保留 `.db`，后者用于后续重处理和定位。

三维采集模式得到的二维投影可能含高处结构，不能直接当作已验收的 Nav2 地图。
后续自主巡逻需另行生成并验证符合障碍高度条件的二维地图。

## 新建三维地图

启动前确保同一 ROS_DOMAIN_ID 没有重复的驱动、TF 或 RTAB-Map 节点。新地图会
删除 `database_path` 指向的同名旧数据库，必须使用新的实验文件名：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py \
  delete_db:=true \
  database_path:=/home/jetson/b2_maps/b2_small_loop_3d.db \
  use_rtabmap_viz:=false \
  use_rviz:=true \
  start_driver:=true \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar
```

已有驱动正在运行时必须把 `start_driver` 改为 `false`。
项目 RViz 配置以 `map` 为固定坐标系，默认显示
`/rtabmap/cloud_map`，并保留一个默认关闭的实时雷达层。不建议在 Jetson 上同时打开
RTAB-Map GUI 和 RViz。

## 继续已有地图

```bash
ros2 launch b2_slam_3d slam_3d.launch.py \
  delete_db:=false \
  database_path:=/home/jetson/b2_maps/b2_small_loop_3d.db \
  use_rtabmap_viz:=false
```

要保留的数据库不得再次使用 `delete_db:=true`。

## 保存三维 PCD 点云

建图运行中，先确认真实话题和点数：

```bash
ros2 topic info /rtabmap/cloud_map -v
ros2 topic echo /rtabmap/cloud_map --once --field width
```

然后另开终端保存当前全局点云：

```bash
ros2 run b2_slam_3d pointcloud_snapshot --ros-args \
  -p cloud_topic:=/rtabmap/cloud_map \
  -p output_path:=/home/jetson/b2_maps/b2_slope_stairs_20261024_01.pcd \
  -p min_points:=100
```

程序会过滤 xyz 为 NaN/无穷大的点、保留其他可支持字段，并以 binary PCD
原子写入。默认拒绝覆盖同名文件，不应为了重试而打开 `overwrite`，应使用新的
实验编号。保存后同时保留 RTAB-Map `.db`；PCD 不能代替原始数据库。
`pointcloud_snapshot` 默认使用实机已确认的 `RELIABLE + TRANSIENT_LOCAL`
QoS，因此可接收 RTAB-Map 保留的最新一帧地图。

## 启动后的健康检查

```bash
ros2 topic echo /b2_slam/preflight_report --once
ros2 topic echo /b2_slam/preflight_ok
ros2 topic echo /b2_navigation/map_report --once
ros2 topic hz /b2_slam_3d/cloud_sync
ros2 topic hz /b2_slam_3d/odom_sync
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

`slam_preflight` 检查点云字段、点数、频率、时间戳、里程计 frame 和传感器 TF。
`occupancy_grid_preflight` 默认检查 `/rtabmap/grid_prob_map` 的结构、已知区域和
障碍栅格，并发布：

```text
/b2_navigation/map_ok
/b2_navigation/map_report
```

建图模式要求二维地图持续更新；更新超时会把 `map_ok` 置为 `false`。

## 查明并保存二维地图

不同 RTAB-Map 版本和命名空间可能导致话题名称不同，禁止只凭旧文档假定
`/rtabmap/map` 一定存在。RTAB-Map 运行期间先执行：

```bash
ros2 topic list -t | grep -E 'map|grid'
ros2 topic type /rtabmap/grid_prob_map
ros2 topic echo /rtabmap/grid_prob_map --once --field info
```

只有实际消息类型是 `nav_msgs/msg/OccupancyGrid` 才能交给 `map_saver_cli`。当前启动
文件默认检查 `/rtabmap/grid_prob_map`：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /rtabmap/grid_prob_map \
  -f /home/jetson/b2_maps/b2_small_loop_2d
```

若现场检查确认真正的 OccupancyGrid 是其他话题，则把 `-t` 和启动参数
`grid_topic` 一起改为该真实话题，并在当天调试记录中写明类型和名称。

保存后检查：

```bash
ls -lh /home/jetson/b2_maps/b2_small_loop_3d.db
ls -lh /home/jetson/b2_maps/b2_small_loop_2d.yaml
ls -lh /home/jetson/b2_maps/b2_small_loop_2d.pgm
```

时间兼容节点默认把点云改为主机接收时间，里程计保留驱动时间戳。这只是当前 TF
兼容方案，不是硬件时间同步的替代品；运动 rosbag 中仍需检查原始时间差。

完整运动建图、保存、重载与 Nav2 测试步骤见
`docs/B2_NAV2_SLAM_DEBUG_GUIDE_2026-09-30.md`。
