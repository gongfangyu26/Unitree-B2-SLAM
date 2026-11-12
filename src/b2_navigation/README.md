# B2 二维自主导航基础包

`b2_navigation` 将已知二维占据地图、B2 里程计、RoboSense 点云和 Nav2
连接起来。当前实现支持两种定位方式：

- `amcl`：加载二维地图，使用点云投影得到的 `/scan` 定位；
- `rtabmap`：加载既有三维 RTAB-Map 数据库，由 RTAB-Map 提供
  `map -> odom`，二维地图仍由 `map_server` 提供给 Nav2。

2026-10-13/16 已有实机阶段调试记录；10-30 走廊录点受代价图阻碍、RViz 卡顿，
本轮自主巡航未执行。稳定定位、转向、避障及整圈任务仍未验收。
最新状态见[代码交接说明](../../docs/B2_CODE_REVIEW_2026-11-12.md)，
下一轮按[11 月 13 日清单](../../docs/B2_TEST_PLAN_2026-11-13.md)分级检查。

## 1. 数据和控制路径

```text
/rslidar_points -> 时间戳兼容 -> pointcloud_to_laserscan -> /scan
二维地图 YAML -> map_server -> /map
/odom + /scan -> AMCL 或 RTAB-Map -> map -> odom

Nav2 -> /cmd_vel_nav
     -> navigation_gate（默认锁定）
     -> /cmd_vel
     -> b2_safe_twist_bridge（默认锁定）
     -> /api/sport/request
```

安全链同时检查：

- `/b2/state_ok`：B2 状态是否有效且新鲜；
- `/b2_navigation/map_ok`：二维地图结构是否通过检查；
- `/scan`：二维雷达是否有效且新鲜；
- `map -> base_link`：定位 TF 是否存在且新鲜；
- RTAB-Map 定位模式下还检查 `/b2_slam/preflight_ok`。

只有 `/b2/autonomy_ok=true` 时才能解锁导航安全门。即使导航安全门已解锁，
`b2_safe_twist_bridge` 仍保持锁定，必须由现场人员再次显式解锁。

## 2. 编译

```bash
cd ~/Unitree-B2-SLAM
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  b2_driver_py b2_twist_bridge_py b2_slam b2_slam_3d b2_navigation \
  --symlink-install
source install/setup.bash
```

Jetson 上的仓库路径若不同，以实际路径为准，不要照抄电脑路径。

## 3. 启动 AMCL 导航

首次必须保持默认锁定，只观察数据，不发送目标：

```bash
ros2 launch b2_navigation navigation.launch.py \
  map:=/home/jetson/b2_maps/b2_small_loop_2d.yaml \
  localization_mode:=amcl \
  start_driver:=true \
  use_rviz:=false
```

如果同一 ROS_DOMAIN_ID 中已经存在唯一的 `b2_driver_py` 和
`b2_safe_twist_bridge`，改用：

```bash
ros2 launch b2_navigation navigation.launch.py \
  map:=/home/jetson/b2_maps/b2_small_loop_2d.yaml \
  localization_mode:=amcl \
  start_driver:=false \
  use_rviz:=false
```

`start_driver` 不能凭感觉选择。启动前先执行 `ros2 node list`，不得出现重复驱动
或重复 TF 发布者。

## 4. 启动 RTAB-Map 定位导航

该模式必须同时提供已有二维地图和已有三维数据库：

```bash
ros2 launch b2_navigation navigation.launch.py \
  map:=/home/jetson/b2_maps/b2_small_loop_2d.yaml \
  localization_mode:=rtabmap \
  database_path:=/home/jetson/b2_maps/b2_small_loop_3d.db \
  start_driver:=true \
  use_rviz:=false
```

启动文件会在启动节点前检查 YAML 和数据库文件是否存在。该模式使用
`Mem/IncrementalMemory=false` 和 `Mem/InitWMWithAllNodes=true`，用于定位而不是
继续建图。不得同时启动 AMCL 和另一个发布 `map -> odom` 的定位节点。

## 5. 启动后只读检查

```bash
ros2 topic echo /b2_navigation/map_report --once
ros2 topic echo /b2/autonomy_reason
ros2 topic echo /b2/autonomy_ok
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list | grep navigate_to_pose
```

在所有检查通过前，禁止调用解锁服务。导航安全门默认状态可用以下命令确认：

```bash
ros2 topic echo /b2_navigation/armed --once
ros2 topic echo /b2/control/enabled --once
```

两者都应为 `false`。

## 6. 纯地图视图与巡航点采集

`navigation.launch.py` 默认打开 `rviz/navigation_clean.rviz`。该视图只显示二维
地图、机器人、规划路径和已记录巡航点，不显示红色实时激光、AMCL 粒子、代价
地图或 TF。完整诊断视图仍保留在 `rviz/navigation.rviz`，可通过
`rviz_config:=<文件路径>` 切换。

纯地图工具栏中的 `Publish Point` 只向 `/clicked_point` 发布坐标，不会发送导航
目标，也不会解锁机器人。`waypoint_recorder` 会使用全局代价地图拒绝地图外、障碍
区、未知区和膨胀区内的点击，并以 A、B、C 的顺序显示接受的点。默认按闭环路线
让每个点朝向下一个点。

```bash
# 撤销最后一个点
ros2 service call /b2_navigation/waypoints/undo std_srvs/srv/Trigger "{}"

# 清空全部点
ros2 service call /b2_navigation/waypoints/clear std_srvs/srv/Trigger "{}"

# 保存为现有巡检任务可读取的 YAML
ros2 service call /b2_navigation/waypoints/save std_srvs/srv/Trigger "{}"
```

默认输出文件为 `~/b2_maps/b2_patrol_waypoints.yaml`。保存后仍须在控制锁关闭状态
下逐段调用规划器验证，不得直接作为实机路线运行。

## 7. 双重人工解锁和立即停车

以下命令只能在现场清场、遥控器/急停可用、目标距离不超过 0.5 m、
`/b2/autonomy_ok` 稳定为 `true` 后执行。

先解锁导航安全门：

```bash
ros2 service call /b2/navigation/set_armed \
  std_srvs/srv/SetBool "{data: true}"
```

再解锁 B2 速度桥：

```bash
ros2 service call /b2/control/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

任何异常先关闭底层速度桥，再关闭导航门：

```bash
ros2 service call /b2/control/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /b2/navigation/set_armed \
  std_srvs/srv/SetBool "{data: false}"
```

导航门默认最多解锁 60 秒；状态、地图、雷达、定位 TF 或命令超时会自动锁定并
发送零速度。节点退出也会发送零速度，但现场仍必须保留人工急停手段。

## 8. 主要启动参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `map` | 必填 | 二维地图 YAML |
| `localization_mode` | `amcl` | `amcl` 或 `rtabmap` |
| `database_path` | `~/.ros/b2_rtabmap_3d.db` | RTAB-Map 定位数据库 |
| `start_driver` | `true` | 已有驱动时必须设为 `false` |
| `use_rviz` | `true` | Jetson 无显示时设为 `false` |
| `rviz_config` | `navigation_clean.rviz` | 纯地图视图或完整调试视图 |
| `start_waypoint_recorder` | `true` | 启动只读巡航点采集工具 |
| `cloud_topic` | `/rslidar_points` | 输入点云 |
| `lidar_frame` | `rslidar` | 点云实际坐标系 |
| `max_linear_x/y` | `0.30` | 速度桥线速度上限 |
| `max_angular_z` | `0.35` | 速度桥角速度上限 |
| `min_effective_angular_z` | `0.35` | 2026-10-13 实测可形成明确左右转的最小角速度 |
| `max_armed_duration_sec` | `60.0` | 导航门单次最长解锁时间 |

## 9. 当前限制

- 2026-09-09 的单次短运动表明 B2 在 `0.30 m/s` 才形成可靠前后/横向位移，
  但 Nav2 连续小速度经过最小有效速度整形后的闭环行为尚未验证；首次实机必须先
  观察 `/cmd_vel_nav`，再决定是否解锁。
- DWB 首轮关闭横移采样，先验证前进和小角度转向；左右横移接口成功不等于
  Nav2 全向控制已经成功。
- `base_link` 在坡道上的姿态、二维激光高度过滤、地图漂移和坡道占据栅格均需
  单独验证。
- 本包中的 `twist_mux.yaml` 当前未被主启动文件使用，实际安全路径由
  `navigation_gate` 和 `b2_safe_twist_bridge` 两级锁实现。

完整实机步骤见 `docs/B2_NAV2_SLAM_DEBUG_GUIDE_2026-09-30.md`。
