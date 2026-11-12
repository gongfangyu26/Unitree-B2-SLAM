# B2 大走廊建图与有限圈数闭环巡逻

日期：2026-10-30；开发/记录：用户与 Codex。适用 Ubuntu 22.04、ROS2 Humble、
B2、RoboSense 和 Jetson。当前实现为 **RTAB-Map 三维建图 → 保存二维地图与三维
数据库 → AMCL 二维定位 → Nav2 逐点闭环巡逻**。建图由现场人员遥控完成，
程序不做自主探索，不会自动站立、切换步态或解锁。

当日现场澄清：此前检查的是房间地图，机器狗仍在房间。房间数据库保留；
本次走廊必须在走廊起点建立独立新库，并保存保留高度的 PCD。三维采集与后续
导航用二维地图分别验收，不能把房间图或未经高度过滤的三维投影直接交给巡逻。

**仅代码检查，尚未实机验证：本次新增的走廊巡逻流程。**
历史实机记录中，2026-10-13/16 已有平地运动建图、保存地图与短距离导航证据；
10-16 最后一次 AMCL 初始化失败，完整走廊闭环和稳定转向尚未验收。
先恢复定位，再进入运动测试。不能把本机假传感器测试写成 B2 实机结果。

**2026-10-30 当日收尾更新：**

- 用户反馈已完成新走廊整圈遥控采集与完整地图保存，局部 PCD 导出有明确日志
  （43,566 点、约 681K）。完整地图保存输出和本轮实际目录未贴出，不补写数值。
- Nav2 已进入界面配置/录点尝试，RViz 全局状态曾恢复 Ok；用户报告青蓝代价区
  阻碍所需点位选择、Jetson RViz 卡顿和显示延迟高，故结束当天测试。
- 尚未完成可用巡航路线、逐段规划、短目标自动到点/停车/转向或整圈自主巡航。
  不按后续解锁章节直接运行百米任务。
- 下次先保留原库，在副本上核对人员残影、三维高度投影与正常膨胀区，
  准备经现场核对的二维导航图；取消 RViz 显示勾选或清空内存代价图不会修改底图。
- 优先安排电脑本地 RViz、Jetson 无界面感知与导航，先只读验证无线可视化
  DDS 通路并测显示延迟，再恢复小范围实验。
- 用户已授权清理旧房间地图和 `b2_bags`，今天走廊数据保留。
  清理后用户回传磁盘可用 41G、使用率 76%，三个当天目录仍在。
  精确命令与最新空间分析见[当日调试记录](B2_DEBUG_LOG_2026-10-30.md)。

## 1. 场地、人员和停车

- 选择平坦、无台阶、无坡道的大走廊，先隔离人流，清除低矮/悬空障碍。
  转弯点和终点须有机身、摆腿及制动净空；大走廊中的重复墙面可能造成定位歧义。
- 至少一名现场操作员全程看护，遥控器/急停随时可用。有线阶段由专人管理线缆，
  不得让线缆绷紧或绕腿；PC 与 Jetson 的无线链路须以当日实际状态为准。
- 按用户要求，走廊巡逻前进速度上限设为 `0.60 m/s`，DWB 与速度桥使用同一
  `max_linear_x`。角速度上限仍为 `0.35 rad/s`；小速度起步整形仍为 `0.30 m/s`，
  可以在 `0.30~0.60` 之间调速，不把每个非零指令强制为 `0.60`。
  `0.60 m/s` 的导航、制动距离和避障均**尚未实机验证**，不是官方通用安全阈值。
- 本次实际目标为约 `100 m`、`1 圈`。按 `0.60 m/s` 上限计算，纯行走至少约
  `167 s`（2 分 47 秒），实际还需转弯、减速、规划等时间。走廊入口默认单目标
  `60 s`、总任务 `600 s`（10 分钟），完成一圈立即停车，不等满时限。
  该预算尚待现场验证；总任务时间包括全部圈数，不是每圈的时间。
- 短距离首测的单目标不超过约 `0.5 m`，可显式使用单目标 `25 s`、总任务 `60 s`。
  验证停车、转向和定位后，再逐步扩大到本次百米实机路线。
- 任务失败、数据失效、人工停止、正常退出会发零速度并请求关闭两级锁；
  速度桥将零速度转换为 `STOPMOVE`。巡逻心跳丢失超过 `0.60 s` 后导航门锁定，
  现有速度桥还保留 `0.35 s` 指令超时。调度/DDS 延迟和实物制动时间须实测。
  断电、系统冻结等情况仍依靠现场遥控/急停。
- 不与 `mission_manager`、`safe_motion_test`、地形测试或其他速度源同时运行。
  本入口使用唯一 AMCL 发布 `map -> odom`，不能同时运行 RTAB-Map 建图/定位。

## 2. 编译与实验目录

在 Jetson 的实际仓库目录执行，路径以现场为准：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  b2_driver_py b2_twist_bridge_py b2_slam b2_slam_3d \
  b2_navigation b2_factory_inspection --symlink-install
source install/setup.bash
git status --short --branch
git rev-parse --short HEAD
```

每个终端均先 source 相同 ROS 与安装环境，保持实际 DDS 网卡、Domain 和 RMW
一致。新建实验目录；下列变量在后续需要的终端中分别设置：

```bash
export B2_CORRIDOR_DIR="$HOME/b2_maps/corridor3d_20261030_01"
mkdir -p "$HOME/b2_maps"
mkdir "$B2_CORRIDOR_DIR"
```

若该编号已用过，换 `02` 等新编号；不覆盖旧数据库、地图、路线或 rosbag。
上条 mkdir 提示目录存在时，保留其内容并先选新编号，不继续使用已有库。
保存 `git diff` 和本轮新增源码快照，记录 HEAD 与 dirty 状态，因为 HEAD 本身
不能复现未提交工作区。不要把 bag、数据库或缓存加入 Git。

## 3. 先建图

若仍在房间，先在检查终端读取
`ros2 param get /rtabmap/rtabmap database_path` 留档，再正常 Ctrl+C 退出旧 SLAM。
控制锁保持关闭。将机器人移到走廊起点的过程不纳入新的走廊建图实验：现场清场、
专人看护、遥控/急停可用，低速短段遥控，建议不超过约 0.30 m/s、每段不超过
1 m 或 3 s，松摇杆后确认停稳，异常用人工停车手段。移到走廊起点停稳后再开新库。

先 `ros2 node list`，确认驱动、速度桥和 TF 发布者都只有一份。可在终端 1
单独运行驱动，以便建图结束后继续保持同一里程计会话：

```bash
ros2 launch b2_driver_py driver.launch.py use_rviz:=false max_linear_x:=0.60
```

这里提前配置后续巡逻的速度上限，速度桥仍默认锁定。若驱动已按旧的 `0.30`
配置运行，先在现场确认停稳、双锁关闭，再退出旧驱动并用上面的命令重新启动；
不要重复启动。仅修改文件或 `ros2 param set` 不会更新该速度桥启动时缓存的
`VelocityPolicy`。若重启导致里程计会话改变，后续必须重新确认地图定位。

终端 2（本次 `.db` 必须尚不存在；保留 `delete_db=false` 以免误删）：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py \
  start_driver:=false delete_db:=false \
  database_path:="$B2_CORRIDOR_DIR/corridor_3d.db" \
  grid_3d:=true grid_max_obstacle_height:=0.0 grid_ray_tracing:=false \
  use_rtabmap_viz:=false use_rviz:=false
```

这是三维表面点云采集配置：保留 Z、高处点不被原先 1 m 障碍高度上限裁掉，
关闭三维体素射线追踪以减少计算和内存开销。原先默认 `grid_3d=false` 是二维
导航投影，不能据其 `cloud_map` 非空宣称保留了三维高度。新选项需先同步 launch
并在 Jetson 构建对应包，不能只向旧启动文件传入尚不存在的参数。

启动后核对实际值，再查看真实点云形状与 Z 范围：

```bash
ros2 param get /rtabmap/rtabmap database_path
ros2 param get /rtabmap/rtabmap Grid/3D
ros2 param get /rtabmap/rtabmap Grid/MaxObstacleHeight
ros2 param get /rtabmap/rtabmap Grid/RayTracing
```

只读检查：

```bash
ros2 topic echo /b2_slam/preflight_report --once
ros2 topic echo /b2_navigation/map_report --once
ros2 topic hz /rslidar_points
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

用遥控器低速分段走完整条拟巡逻路线，并返回可识别的起点。当前软件控制锁
保持关闭；遥控器自主控制的安全性由现场操作员负责。大走廊要覆盖两侧墙面、
转弯和末端，观察地图是否拉重影、错层或跳变；异常则停稳并保留失败数据。
建图阶段可在另一终端记录原始数据（输出目录必须不存在）：

```bash
ros2 bag record -o "$B2_CORRIDOR_DIR/mapping_bag" \
  /rslidar_points /odom /tf /tf_static /lf/sportmodestate
```

停稳后保持建图和地图可视化运行，确认画面包含新走廊整条路线，点云有真实高度
结构，且最后一次运动对应的地图已经处理完成，再保存 PCD。持续订阅和画面核对
用于避免只导出早先保留的一帧旧图。`pointcloud_snapshot` 保留收到的 XYZ，
不会把二维投影补成三维。默认拒绝覆盖已有同名文件。

同时可检查并留档当前 OccupancyGrid；三维采集参数的投影**尚不是已验收的导航地图**：

```bash
ros2 topic type /rtabmap/grid_prob_map
ros2 topic echo /rtabmap/grid_prob_map --once --field info
ros2 run nav2_map_server map_saver_cli \
  -t /rtabmap/grid_prob_map -f "$B2_CORRIDOR_DIR/corridor_2d"
ros2 run b2_slam_3d pointcloud_snapshot --ros-args \
  -p output_path:="$B2_CORRIDOR_DIR/corridor_3d.pcd"
```

实际地图话题若不同，先核对 `nav_msgs/msg/OccupancyGrid` 再替换，不凭名称猜测。
记录最终 `map -> base_link` 位姿与地面位置/朝向，停止 bag，然后正常 Ctrl+C
退出终端 2，让 RTAB-Map 关闭数据库。保留终端 1 驱动。

```bash
ls -lh "$B2_CORRIDOR_DIR"
ros2 node list
```

确认 `.db`、`.yaml`、地图图像均存在、非空，且旧建图节点确已退出。
不要运行记录中曾失败的旧 `autonomous_inspection.launch.py` 来代替下面入口。

## 4. 地图定位与路线采集

进入本节前，必须单独准备经过障碍高度过滤与可通行区域核对的走廊二维地图。
第 3 节三维采集生成的投影仅先留档，不能因成功保存 YAML 就视为适合导航。
后续二维重生成及质量检查未完成时，停留在建图与数据保存阶段。

下面只启动待命流程，不会开始巡逻。驱动已在终端 1 时使用 `start_driver=false`；
若已关闭驱动并确认无重复节点，则改为 `true`。本次约百米路线采用 1 圈、
10 分钟总任务预算：

```bash
ros2 launch b2_factory_inspection corridor_patrol.launch.py \
  map:="$B2_CORRIDOR_DIR/corridor_2d.yaml" \
  waypoints_file:="$B2_CORRIDOR_DIR/waypoints.yaml" \
  start_driver:=false use_rviz:=false \
  laps:=1 max_linear_x:=0.60 \
  goal_timeout_sec:=60.0 mission_timeout_sec:=600.0 \
  code_revision:=a99cc7b-dirty
```

`code_revision` 换成当日真实 HEAD 和工作区标记，不照抄此示例。巡逻节点每秒
调用 AMCL 的只读 `/request_nomotion_update`（服务可用时），让静止定位也能
更新消息；这不会给机器人发送运动指令。

在监控电脑（相同 ROS 域且 DDS 已打通）打开完整诊断视图，并把初始化工具
输出映射到已隔离的话题：

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share b2_navigation)/rviz/navigation.rviz" \
  --ros-args -r /initialpose:=/b2_navigation/initialpose
```

用 `2D Pose Estimate` 指定机器人在新走廊地图中的**实际位置和朝向**。
禁止沿用旧场地坐标，禁止默认填零。核对实时 `/scan` 与地图墙体重合、机器人
朝向真实，检查 `/amcl_pose` 和 `map -> base_link` 连续有效，两级锁仍为 false。

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /b2/autonomy_reason --once
ros2 topic echo /b2_navigation/armed --once
ros2 topic echo /b2/control/enabled --once
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 param get /b2_safe_twist_bridge max_linear_x
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
```

本次三项速度上限均应为 `0.6`；检查速度桥的 `min_effective_linear_x` 仍为
`0.3`。`start_driver=false` 时新 launch 无法替已有驱动改参数，必须先按本节前述
步骤配置并重启驱动。Nav2 启动时会以 `max_linear_x` 覆盖 DWB 的两个上限，
基础 YAML 中的 `0.30` 不会再把走廊入口截断到旧速度。

若仍复现 10-16 的无定位问题，停留在锁定状态，保存 AMCL 日志，检查：

```bash
ros2 service list -t
ros2 interface show nav2_msgs/srv/SetInitialPose
```

`/set_initial_pose` 若存在，可用同一实际测得的 PoseWithCovarianceStamped
调用它排查 topic 初始化问题；消息字段为 `pose`。不要用旧记录坐标或伪造
`/amcl_pose` 绕过检查。初始化失败时不执行后面的解锁或导航。

定位可靠后，在 RViz 用 `Publish Point` 依次点击 A、B、C…；避开墙、门洞边缘
和膨胀区，每段规划路径默认最多 `5 m`，约 100 m 闭环通常至少需要约 20 个
点，拐角和绕障段还需加点；当前位置到 A 也必须在单段限制内。不要重复点击 A
作为末点：程序会自动添加回到 A 的闭环段。两点路线等于往返，必须有转身净空。
路线航向由程序按下一点重新计算，最终回到 A 朝向首段；YAML 中已有 yaw 不用于
闭环航向。转向净空仍由人员在现场核对。

```bash
ros2 service call /b2_navigation/waypoints/save std_srvs/srv/Trigger "{}"
```

路线文件与现有 recorder 格式一致，要求 `frame_id=map`、每点 `inspect=false`
且只有一个 pose。不得直接拿其他场地的路线或包含拍照候选的配置运行。
保存器沿用现有行为，会替换指定路线文件，因此每次实验使用独立新路径。

## 5. 锁定规划 → 分级测试 → 指定圈数

人员先确认定位/雷达重合，然后在两级锁关闭时执行：

```bash
ros2 topic echo /patrol/state
```

另一个终端：

```bash
ros2 service call /patrol/validate std_srvs/srv/Trigger "{}"
```

响应 success 只表示接收检查请求，必须等 `/patrol/state` 出现 `READY`。
检查覆盖“当前位置 → A → B → … → A”，多圈则展开全部目标；仅使用
`ComputePathToPose`，不发 `NavigateToPose`。可给 RViz 添加 Path 显示
`/patrol/checked_path` 查看每段；节点 JSONL 逐段记录长度。检查路径的首尾点
偏差不能超过 `0.25 m`，单段最长 `5 m`。这些阈值是项目初值。

**先进行约 0.5 m 单目标和独立转向首测**：停止入口后，用只有一个近目标的
单独路线文件，启动参数 `route_mode:=once laps:=1`；仍须 validate、定位核对
和下面的人工双解锁。首测通过后，再重新启动闭环入口并采集小闭环 1 圈。

进入 `0.60 m/s` 百米路线前，在开阔平地、净空充足、专人看护及独立遥控/急停
可用的条件下，用有限目标逐级测试 `max_linear_x:=0.30 → 0.45 → 0.60`。
0.5 m 首测只检查基础控制，通常不足以达到 0.6 m/s；后续可用 2–3 m 直线目标，
每次只执行一段并自动到点停车，测量实际速度、到点超调和停止距离，异常立即中止。
保持单目标 60 s、总任务 600 s 上限与全部停车联锁；切换上限需在锁定停车后
重启相应节点并重新定位/validate。原 DWB 的加减速度、footprint 和膨胀配置仍是
待复核初值，须据实测制动和转弯净空判断是否能进入完整路线，不以软件限速代替实测。

只有场地清空、看护人员及遥控/急停就绪、定位合理、两级锁与速度源唯一时，
才执行以下运动阶段命令。READY 后有效期 `60 s`，这是检查到启动之间的等待窗口，
与整圈运行的 `600 s` 独立；过期须重新检查。先开导航门，
等至少 `0.5 s` 心跳，再开速度桥，等待状态更新，最后启动：

```bash
ros2 service call /b2/navigation/set_armed std_srvs/srv/SetBool "{data: true}"
# 人工等待至少 0.5 秒，并检查上条 success=true
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /b2_navigation/armed --once
ros2 topic echo /b2/control/enabled --once
ros2 service call /patrol/start std_srvs/srv/Trigger "{}"
```

start 还会核对当前位置距检查起点不超过 `0.20 m`、航向偏差不超过 `0.20 rad`、
路线文件哈希未变化、安全数据新鲜且两级锁已开。程序不自动重试、不跳过失败点，
中止后必须重新 validate。Nav2 的正常路径控制可以转向，但此任务的行为树不调用
额外 Spin/BackUp 恢复动作。规划或跟踪失败即中止。

完成后等待 `STOPPED`，确认两个锁都为 false。闭环首轮成功后逐步增加路线和圈数，
例如 `laps:=3`；总时长按实际路线长度、实测平均速度、转向和制动耗时估算，
再做分级验证。默认 `mission_timeout_sec=600.0`，最大 `600 s`；该值同时
约束导航门，门从人工解锁时开始计时，任务从 start 开始计时，先到者停车。
不因时间不足自动延长或续锁。十分钟是本次百米试验的初始预算；若实际路线、
圈数或平均速度使预算不足，先停车并重新评估，不自动重启或续锁。

## 6. 任何异常立即停止

优先使用现场遥控/急停。ROS 仍通畅时先关闭底层速度桥，再关导航门并中止任务：

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /b2/navigation/set_armed std_srvs/srv/SetBool "{data: false}"
ros2 service call /patrol/stop std_srvs/srv/Trigger "{}"
```

`/patrol/stop` 返回只说明已发停车请求；`STOPPED` 才表示收到了新的双锁关闭状态。
停车实际距离需现场测量，不能把软件状态当作实物已经停稳的证明。

## 7. 实验留档与通过标准

节点自动创建 `~/b2_patrol_reports/corridor_<UTC时间>.jsonl`，记录路线 SHA256、
展开的目标、参数、代码版本标记、逐段路径长度、到点定位、完成/失败原因和锁关闭。
JSONL 的 `arrived` 是定位估计，不是地面真值；保存失败与人工接管记录。

巡逻时另录新 bag：

```bash
ros2 bag record -o "$B2_CORRIDOR_DIR/patrol_bag_01" \
  /rslidar_points /scan /odom /tf /tf_static /amcl_pose \
  /cmd_vel_nav /cmd_vel /b2/autonomy_ok /b2_navigation/map_ok \
  /b2_navigation/armed /b2/control/enabled /patrol/state /patrol/checked_path
```

当天调试记录还须填写：场景尺寸/走廊宽度、路线长度、地面坡度、障碍和人流，
软硬件版本/供电/网络、地图分辨率、footprint、膨胀半径、圈数、超时、代码快照
与原始文件位置。逐圈记录实际耗时、地面测量的到点/返回起点位置和航向误差、
转弯超调、制动距离、人工干预、失败原因及任务完成率。

小闭环至少保留 3 次同条件重复结果，任何失败都算入分母；通过后才扩大到大走廊。
静态障碍、动态避障分别验证；没有指标、重复和实机数据时不宣称完整自主巡逻成功。

## 8. 接口依据

规划检查与实际导航分别使用 Nav2 Humble 官方接口：
[ComputePathToPose](https://api.nav2.org/actions/humble/computepathtopose.html)
的 `use_start=true` 支持逐段指定起点，
[NavigateToPose](https://api.nav2.org/actions/humble/navigatetopose.html)
的 `behavior_tree` 支持为本次目标指定无额外恢复动作的行为树。
同时已对照本机 `/opt/ros/humble/share/nav2_msgs/action/` 定义核对字段。
