# Ubuntu → Windows Thesis Evidence Sync

本文件用于把 Ubuntu 侧 `Unitree-B2-SLAM` 仓库中截至 2026-09-11 可核对的科研证据同步到 Windows 论文工作区。它只整理仓库已有日志、源码和配置，不代表本次又执行了机器人实验。

状态只使用以下五类：

- `REAL_ROBOT`：存在明确日志，证明真实 B2、真实 Jetson 或真实传感器上完成了相应范围的验证。
- `SIMULATION`：存在仿真环境中的执行证据，但没有实机证据。
- `CODE_ONLY`：存在代码、launch、config 或软件检查，但没有可靠的对应实机证据。
- `PLANNED`：目前只有方案、调试手册或后续计划。
- `NEEDS_EVIDENCE`：材料不足、存在冲突，或无法确认某项连接/行为是否真正发生。

“命令成功发送”“收到了 API 响应”“topic 名称存在”均不能单独证明机器人产生了预期运动或完整功能已经跑通。

## 1. Sync Metadata

| Field | Value |
|---|---|
| Date | 2026-09-11（Asia/Shanghai） |
| Repository | `/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM` |
| Origin | `git@github.com:gongfangyu26/Unitree-B2-SLAM.git` |
| Upstream | `https://github.com/leiyw25/Unitree-B2-SLAM.git` |
| Branch | `master` |
| Git Commit SHA | `fabbc17a4f81cc65b89ad7cb7eb1b40894d79495` |
| Baseline Commit | `fabbc17 feat: establish B2 control, SLAM and inspection development baseline` |
| Working Tree Status Before Sync | 无 tracked 修改；已有 `AGENTS.md` 和 `2026-09-09_学习知识笔记.md` 两个 untracked 文件 |
| Purpose | 第一次 Ubuntu → Windows 论文科研证据同步；建立可引用事实、代码能力和证据缺口的严格边界 |

同步前已经存在、不是本任务产生的文件：

```text
?? AGENTS.md
?? 2026-09-09_学习知识笔记.md
```

本任务未运行 B2 控制节点、未执行新实机运动、未修改已有 tracked 文件，也未把软件实现当作实机结果。

## 2. Hardware / Software Environment

### 2.1 Unitree B2 与 Ubuntu PC

| Item | Recorded evidence |
|---|---|
| Robot | Unitree B2 |
| PC OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| RMW | `rmw_cyclonedds_cpp` |
| ROS domain | `ROS_DOMAIN_ID=0` |
| PC B2 Ethernet interface | `enp8s0` |
| PC B2 Ethernet address | `192.168.123.99/24` |
| B2 control topics | `/api/sport/request`、`/api/sport/response` |
| Unitree messages | `unitree_api/msg/Request`、`unitree_api/msg/Response`；状态使用 `unitree_go` 消息 |
| Primary LiDAR | RoboSense，真实输出 `/rslidar_points` |

主要依据：`docs/B2_DEBUG_LOG_2026-09-08.md`、`docs/2026-09-09_B2_Jetson_实机调试记录.md` 和 `docs/debug_logs/`。

### 2.2 Jetson

| Item | Recorded evidence | Evidence boundary |
|---|---|---|
| Board | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super；照片资料称创乐博 Makerobo CLB Developer Kit | 真实板卡已检查 |
| Architecture | `aarch64` | 真实板卡已检查 |
| L4T | R36.4.7 | 真实板卡已检查 |
| Kernel | `5.15.148-tegra` | 真实板卡已检查 |
| Ubuntu version | 项目目标环境写为 Ubuntu 22.04 | 现有实机记录没有保存 `/etc/os-release` 或 `lsb_release` 原始输出，不能仅由本文件进一步推断 |
| Ethernet | `enP8p1s0`，`192.168.123.211/24`；Realtek RTL8111/8168/8411，驱动 `r8168` | 有线接口和地址已记录，但不能据此自动推断已与 B2 直接完成 ROS2 通信 |
| Wi-Fi hardware | 背面照片记录 RTL8822CE、Main/Aux 双天线 | 硬件照片证据存在 |
| Wi-Fi software state | 没有可用 WLAN interface；`rtl8822ce.ko` 可加载但没有产生 wlan 接口 | 成功无线连接未验证 |
| Bluetooth | 设备存在 | 不是本项目主通信链路证据 |
| NVMe | 系统实机记录为 `SSSTC CL1-8D256-HP`、约 238.5 GB；照片说明文档称 HP FX700 256 GB | 型号记录冲突，Windows 论文引用前应以 Jetson `lsblk`/硬件标签复核 |
| Free root space | 约 176 GB | 真实板卡已检查 |
| Thermal / power mode | 约 46–48 °C；`nvpmodel` 25 W | 桌面短时基础验收，不是满载长期测试 |
| ROS2 software | ROS2 Humble、CycloneDDS、`unitree_api`、`unitree_go` 可用 | 真实 Jetson 环境已检查 |
| Project build | 源码复制到 `/home/nvidia/b2_ws` 后在 aarch64 原生编译通过 | 证明可构建，不证明 Jetson 已控制 B2 |
| Clock | RTC 曾为 1970；无互联网时 NTP 未同步，调试当日手动设置时间 | 长期时间保持未解决 |
| SSH | 没有找到成功登录、IP、命令回显或会话日志 | `NEEDS_EVIDENCE` |
| PC ↔ Jetson communication | 没有保存 ping、iperf 或 SSH 成功证据 | `NEEDS_EVIDENCE` |
| Jetson ↔ B2 direct communication | 没有找到从 Jetson 端发现 B2 topic、读取真实 B2 消息或收到 Sport response 的明确日志 | `NEEDS_EVIDENCE` |

## 3. Module Evidence Matrix

| Module | Status | Evidence | Relevant File / Log | Key Parameters | Known Limitation | Paper Relevance |
|---|---|---|---|---|---|---|
| 1. B2 基础通信 | REAL_ROBOT | Ubuntu PC 通过有线网发现 B2 ROS2 数据；`/api/sport/request` 有机器人侧订阅者 | `docs/B2_DEBUG_LOG_2026-09-08.md`；`docs/debug_logs/b2_sport_request_info.txt` | `enp8s0`；`192.168.123.99/24`；ROS domain 0 | 仅证明 PC ↔ B2 有线基线，不证明 Jetson ↔ B2 或无线链路 | 支撑所有实机控制实验的通信基线 |
| 2. ROS2 与 B2 Sport API | REAL_ROBOT | request/response topic、消息类型和实际 API 响应有记录 | `docs/B2_DEBUG_LOG_2026-09-08.md`；`src/b2_twist_bridge_py/b2_twist_bridge_py/sport_model.py` | `MOVE=1008`；`STOPMOVE=1003`；`BALANCESTAND=1002` | `api_id=1034` 含义未知；API 响应本身不等于运动成功 | 机器人执行接口与实验可重复性基础 |
| 3. B2 基础运动控制 | REAL_ROBOT | 前进、后退、左右横移和左转存在实际运动观察 | 两份实机日志；`safe_motion_test.py` | 单轴 MOVE；当前实测有效平移值 0.30 m/s | 右转、组合平移、完整姿态动作并未全部留下实机证据 | 四足平台控制可行性基线 |
| 4. 安全短运动测试 | REAL_ROBOT | 默认锁定、未解锁拒绝动作、单次 MOVE、watchdog 自动 STOPMOVE 均有记录 | `docs/B2_DEBUG_LOG_2026-09-08.md`；`docs/2026-09-09_B2_Jetson_实机调试记录.md`；`safe_motion_test.py` | 默认普通脉冲 0.60 s；横移 1.00 s；脉冲允许 0.05–1.00 s | 自动停车不能替代人工急停；节点退出和异常网络仍需系统级故障注入 | 安全实验设计和控制联锁证据 |
| 5. 前进 | REAL_ROBOT | 0.10–0.20 m/s 只有腿部动作/趋势；0.30 m/s 后明确前向位移；Python 链最终成功 | `docs/B2_DEBUG_LOG_2026-09-08.md` | `x=+0.30`；`motion_pulse_sec=0.60`；约 0.62 s 后 STOPMOVE | 0.30 m/s 是当前 B2/模式/场地实测值，不是官方通用阈值；没有卷尺位移原始表 | 速度阈值与“命令≠位移”的重要实验演变 |
| 6. 后退 | REAL_ROBOT | 实际方向正确，3 次均明确后退，3 次均自动 STOPMOVE | `docs/2026-09-09_B2_Jetson_实机调试记录.md` | `x=-0.30`；当日基础脉冲 0.30 s | 没有原始位移数值、视频索引和不同地面重复实验 | 后续调整拍摄距离和 Nav2 recovery 的动作基础 |
| 7. 左横移 | REAL_ROBOT | 早期 0.08–0.25 m/s 无明确位移；0.30 m/s 后明确左移，最终 3/3 | 同上，第 4、13 节 | `y=+0.30`；约 1.00 s；单次 MOVE 可成功 | 只验证纯横移；没有组合运动、坡面和负载实验 | 主动观测位姿横向调整的重要基础 |
| 8. 右横移 | REAL_ROBOT | 最终 `0.30 m/s` 下明确右移，3/3，自动 STOPMOVE | 同上，第 13 节 | `y=-0.30`；约 1.00 s；单次 MOVE | 早期阶段未测，成功证据来自当日后续实验；无定量位移表 | 主动观测位姿横向调整的重要基础 |
| 9. 左旋转 | REAL_ROBOT | 机器人产生幅度较小的左转并自动停止 | `docs/B2_DEBUG_LOG_2026-09-08.md` | `z=+0.20 rad/s`；0.60 s | 仅单次定性记录，没有 3/3、角度测量或不同地面重复 | 调整目标朝向与导航姿态的初步证据 |
| 10. 右旋转 | CODE_ONLY | 当前键盘程序将 `e` 映射为负角速度 | `src/b2_twist_bridge_py/b2_twist_bridge_py/safe_motion_test.py` | `z=-angular_speed`；默认 0.20 rad/s | 未找到机器人实际右转日志；必须实机复测 | 完整双向姿态调整仍缺证据 |
| 11. Jetson 接入 | REAL_ROBOT | 真实 Jetson 板卡完成型号、架构、L4T、内核、NVMe、Ethernet、温度、功耗模式、ROS2 和本机编译检查 | `docs/2026-09-09_B2_Jetson_实机调试记录.md`；`jetson_board_test.py` | Orin Nano Super；aarch64；L4T R36.4.7；25 W | 仅桌面基础验收；未完成机器人安装、板载供电、满载或无线验收 | 证明算法向板端迁移具有软件基础 |
| 12. Jetson 与 B2 通信 | NEEDS_EVIDENCE | Jetson 有线口和 B2 网段地址已记录，但没有 Jetson 端 B2 topic/response 原始日志 | 实机记录与 `docs/B2_JETSON_DEBUG_2026-09-09.md` | `enP8p1s0`；`192.168.123.211/24` | IP 配置不等于实际 ROS2/DDS 通信；没有 Jetson 端 discovery、消息或控制证据 | 最终板载自治部署的关键缺口 |
| 13. LiDAR | REAL_ROBOT | RoboSense `/rslidar_points` 有真实 PointCloud2 数据、约 10 Hz、frame `rslidar`、时间戳非零 | `docs/2026-09-09_B2_Jetson_实机调试记录.md`；`docs/debug_logs/b2_node_list.txt` | 约 9.94–10.01 Hz；一次 preflight 为 57600 points | Livox topic 的 publisher 为 0；雷达到机身外参只验证 TF 连通，未精标 | SLAM、避障和场景几何感知的真实输入证据 |
| 14. 深度相机 | PLANNED | 文档要求后续记录深度图；当前 RTAB-Map 明确 `subscribe_depth: false` | `docs/SLAM_INSPECTION_NEXT_STAGE.md`；`src/b2_slam_3d/config/rtabmap_3d.yaml` | 当前无有效深度 topic 参数 | 没有深度相机型号、驱动、topic、帧率、内参、外参或真实数据 | 后续近距观测和目标距离估计的可选传感器 |
| 15. SLAM（仅静止 3D 数据链） | REAL_ROBOT | 真实 B2 点云和项目 odom/TF 进入 RTAB-Map；`map→odom` 出现；`/cloud_map` 非空 | `docs/2026-09-09_B2_Jetson_实机调试记录.md`；`src/b2_slam_3d/` | 点云约 10 Hz；odom 约 20 Hz；`qos_scan=2`；`Reg/Force3DoF=false`；cloud map 1317 points | 机器人全程静止；没有运动建图、地图精度、漂移、闭环或坡道验证 | 证明真实 3D 数据链可运行，但不能作为完整建图性能结果 |
| 16. 导航 | CODE_ONLY | Nav2 launch、AMCL/RTAB-Map 定位模式、NavFn、DWB、代价地图与安全速度桥接口已配置 | `src/b2_navigation/`；`autonomous_inspection.launch.py` | 实际 YAML：`max_vel_x=0.30`、`max_vel_y=0`、`max_vel_theta=0.20` | 没有真实 B2 单目标导航、路径跟踪、到点误差或局部避障日志 | 完整自主巡检的核心执行层，必须后续量化 |
| 17. 固定路线巡检 | CODE_ONLY | 存在旧版直接速度控制源码，但入口已从安装和当前主流程移除 | `fixed_route_3d_patrol_node.py`；`src/代码检查报告.md` | 旧代码含 0.28/0.22/0.55 等未验收参数 | 没有当前安全联锁；参数与实测有效值冲突；不应作为实机入口 | 可作为历史基线，不可作为已完成系统证据 |
| 18. 巡检目标配置 | CODE_ONLY | 路线 YAML、目标 YAML 和严格解析代码存在 | `patrol_waypoints.yaml`；`inspection_targets.yaml`；`route_config.py`；`target_config.py` | 示例目标 `target_a/target_b`；ArUco 10/20；每目标多候选位姿 | 路线文件明确写明仅为格式示例，坐标不可直接用于真机 | 为后续可复现实验定义目标与候选位姿 |
| 19. 图像采集 | CODE_ONLY | 节点可订阅 Image/CompressedImage，接收拍照触发并保存 JPG/JSON | `photo_quality_node.py`；`autonomous_inspection.launch.py` | 默认 `/camera/image_raw`；等待新帧 2.0 s | 默认 topic 是占位名；没有真实相机帧、保存照片或时间同步证据 | 形成论文图像数据与质量报告的入口 |
| 20. 图像质量评价 | CODE_ONLY | 已实现目标置信度、面积、Laplacian 清晰度、亮度、暗部和过曝规则，并存在单元测试 | `quality_core.py`；`photo_quality_node.py`；`photo_quality.yaml`；相关 test | blur 80；亮度 45–215；面积 0.03–0.65；置信度 0.60 | 阈值是功能验证初值；没有真实相机/故障检测可用性标注和实测 ROC/成功率 | 论文主动重观测触发条件的核心候选方法 |
| 21. 目标定位 | CODE_ONLY | ArUco 检测器按当前目标名发布归一化目标框 | `aruco_target_locator.py`；`inspection_targets.yaml` | DICT_4X4_50；marker 10/20；ROI scale 4×3 | 没有真实相机、标记板、遮挡、距离或视角实验 | 户外 A/B 功能验证的目标代理，不代表工厂目标识别 |
| 22. 主动靠近目标 | CODE_ONLY | 当前主流程可在图像不合格后导航到下一个预定义候选观测位姿；旧节点另有沿 waypoint 朝向递增 approach 的原型 | `mission_manager.py`；`patrol_waypoints.yaml`；旧 `fixed_route_3d_patrol_node.py` | 顺序候选位姿；旧代码 `approach_step` 原型 | 当前没有根据距离/目标几何在线生成“靠近”动作，也未证明候选位姿确实更近且安全 | 论文“主动观测”方向的核心待验证能力 |
| 23. 重新采集 | CODE_ONLY | 到候选位姿后等待稳定、重新发 `capture_required`，并要求请求后的新图像和同批 bbox | `mission_manager.py`；`photo_quality_node.py` | settle 1.5 s；quality timeout 8.0 s；bbox tolerance 0.20 s | 没有真实相机和真实 B2 重拍记录 | 支撑从一次失败观测恢复的机制 |
| 24. 感知—决策—运动—再感知闭环 | CODE_ONLY | integrated launch、任务状态机、安全许可、Nav2 action、质量反馈和候选位姿切换代码存在 | `autonomous_inspection.launch.py`；`mission_manager.py`；`safety_monitor.py`；`twist_bridge.py` | 默认 `mission_autostart=false`；心跳/状态/定位/scan 超时联锁 | 未在 B2 上完整启动；没有端到端 rosbag、照片、轨迹或故障注入证据 | 论文系统贡献的最终验证对象 |
| 25. 工厂实地实验 | PLANNED | 现阶段只有工厂与户外模拟场景实验方案 | `src/项目路线与论文实验方案.md`；`OPERATION_GUIDE.md` | 计划多点、坡道、障碍、图像质量与重观测实验 | 尚未进厂；没有场地、路线、目标、坡度或结果数据 | 投稿前必须补齐的外部有效性证据 |

本仓库没有找到 Gazebo、Ignition 或其他仿真运行日志，因此当前没有可列为 `SIMULATION` 的模块。单元测试和 `launch --show-args` 属于软件检查，不属于机器人仿真。

## 4. Verified B2 Motion Experiments

### 4.1 2026-09-08：通信、安全锁、前进和左转

#### 通信与安全锁

- PC 使用 Ubuntu 22.04、ROS2 Humble、CycloneDDS、`enp8s0` 和 `192.168.123.99/24` 连接 B2。
- `/api/sport/request` 的机器人侧订阅数量为 1，request/response 消息类型已核对。
- `safe_motion_test` 启动时 `armed=False`。未解锁按 `u` 被程序拒绝，机器人没有动作，证明软件锁至少在该场景有效。

#### 姿态动作的证据边界

- `BALANCESTAND=1002`：记录到机器人轻微调整并保持稳定，可视为真实行为证据。
- `STANDUP=1004`：机器人当时已经站立，因此没有明显动作；同时观察到含义未知的 `api_id=1034`。这不能作为“从蹲姿成功站起”的证据。
- 没有找到 `STANDDOWN=1005` 或 `RECOVERYSTAND=1006` 的明确实机动作记录。部分后续手册笼统写“站立、趴下已验收”，但详细原始日志不足，论文证据应以保守结论为准。

#### 前进参数演变

| Stage | Command / parameter | Actual observed behavior | Evidence conclusion |
|---|---|---|---|
| Python 初测 | `x=+0.10 m/s`，0.30 s | 关节动作、有前进趋势、无明显位移 | 不能算前进成功 |
| Python 延长脉冲 | `x=+0.10 m/s`，0.60 s | 动作幅度增加，仍无明显位移 | 不能算前进成功 |
| 官方 C++ 对照 | `x=+0.10 m/s` | 腿动，不走 | 不能算前进成功 |
| 官方 C++ 对照 | `x=+0.20 m/s` | 动作幅度增加，仍无明显位移 | 不能算前进成功 |
| 官方 C++ 对照 | `x=+0.30 m/s` | 成功产生前向位移 | `REAL_ROBOT` |
| 最终 Python 复验 | `x=+0.30 m/s`，`motion_pulse_sec=0.60` | 明确前向位移；收到 MOVE 1008；约 0.62 s 后 STOPMOVE 1003 | Python 控制链和实际前进均得到证据 |

这组记录说明：非零命令或 API 响应不等于机身位移；实际观察必须单独记录。

#### 左转

- 按键 `q` 对应 `z=+angular_speed`。
- `angular_speed=0.20 rad/s`、`motion_pulse_sec=0.60 s`。
- 实际观察为机器人左转，幅度较小，并自动停车。
- 没有 3/3 重复、旋转角测量或右转对应实验。

### 4.2 2026-09-09：后退与横移

#### 后退

- 实际命令：`MOVE {'x': -0.3, 'y': 0.0, 'z': 0.0}`。
- 当日基础脉冲为 0.30 s。
- 三次方向均正确、均有明确位移、均自动 STOPMOVE。
- 最终结论：后退 3/3 `REAL_ROBOT`。

#### 左横移的实验演变

同一份当日日志先记录阶段性失败，随后继续做增量参数与 A/B 对照并得到最终成功。两段都必须保留：

| `vy` | Pulse / send mode | Actual observed behavior |
|---:|---|---|
| +0.08 m/s | 0.30 s，单次 MOVE | 向左动作趋势，无明确平移 |
| +0.08 m/s | 0.60 s，单次 MOVE | 有动作趋势，无明确位移 |
| +0.10 m/s | 0.60 s，单次 MOVE | 仍只有趋势 |
| +0.15 m/s | 0.60 s；另记录约 1.00 s | 动作幅度增加，仍无明确横向位移 |
| ±0.15 m/s | 临时约 20 Hz MOVE | 无明确位移 |
| ±0.20 m/s | 临时约 20 Hz MOVE | 无明确位移，动作幅度增加 |
| ±0.25 m/s | 临时约 20 Hz MOVE | 无明确位移 |
| ±0.30 m/s | 临时约 20 Hz MOVE | 出现明确横向实际位移，自动停车正常 |
| ±0.30 m/s | 恢复单次 MOVE，约 1.00 s | 仍有明确实际位移，证明 20 Hz 重发不是必要条件 |

最终重复性：

- 左横移：`y=+0.30 m/s`，3/3 明确位移。
- 右横移：`y=-0.30 m/s`，3/3 明确位移。
- 每次均能自动 `STOPMOVE`。
- 成功样本使用约 1.00 s 横移脉冲。

阶段性“左横移 FAIL、右横移 Pending”是当天前半段状态；同一日志后半段第 13 节是继续实验后的最终结论。不能删除前半段，也不能继续沿用阶段性失败作为当天最终状态。

### 4.3 当前代码中的安全参数与证据边界

`src/b2_twist_bridge_py/b2_twist_bridge_py/safe_motion_test.py` 当前状态：

```text
默认 linear_speed       = 0.30 m/s
默认 lateral_speed      = 0.30 m/s
默认 angular_speed      = 0.20 rad/s
默认 motion_pulse_sec   = 0.60 s
默认 lateral_pulse_sec  = 1.00 s
linear_speed 硬上限     = 0.30 m/s
lateral_speed 硬上限    = 0.30 m/s
angular_speed 硬上限    = 0.35 rad/s
脉冲允许范围            = 0.05–1.00 s
MOVE 发送方式           = 每按一次只发送一次
停止方式                = 0.05 s watchdog 检查截止时间并发送 STOPMOVE
```

需要特别记录的代码/实机边界：

- 实机只记录 `0.20 rad/s` 左转；安全测试程序目前允许通过参数或 `+` 键将角速度提高到 `0.35 rad/s`。`0.20–0.35 rad/s` 区间没有实机证据，不能写成已验证安全范围。
- `twist_bridge.py` 的自主控制上限更保守：`max_linear_x=0.30`、`max_linear_y=0.30`、`max_angular_z=0.20`。
- 自主桥默认锁定，`command_timeout_sec=0.35`，任务心跳、机器人状态和自主状态超时均默认为 0.60 s，紧急停车默认重复发送 3 次。
- `allow_combined_translation=false`，因为当前只验证了纯前后和纯横移，没有验证 `x/y` 同时非零。
- `api_id=1034` 未在当前 B2 白名单映射中定义；在找到 Unitree 官方协议依据前不得解释。

## 5. Jetson Evidence

### 5.1 已确认

- 真实板卡型号、aarch64、L4T R36.4.7、内核 5.15.148-tegra。
- 有线接口 `enP8p1s0` 和地址 `192.168.123.211/24`。
- Ethernet 芯片/驱动、NVMe 容量、剩余空间、温度和 25 W 功耗模式。
- ROS2 Humble、CycloneDDS、Unitree 消息包可用。
- 项目源码在 `/home/nvidia/b2_ws` 完成 aarch64 原生编译。
- `jetson_board_test.py` 在真实 Jetson 上暴露 sysfs `TypeError`，调整读取异常处理后，除 Wi-Fi 外基础报告基本通过。

### 5.2 已确认失败或未完成

- 没有可用 WLAN interface。
- `ip`、`nmcli`、`lspci` 没有发现可用 Wi-Fi 网络接口；加载已有模块后仍无 wlan。
- 最终 `B2 ↔ Jetson Ethernet + PC ↔ Jetson 5 GHz Wi-Fi` 拓扑未完成。
- RTC 保持问题未解决。
- 没有板载 DC-DC 供电、机械固定、振动、散热或长时间满载证据。

### 5.3 材料不足，不能确认

- 没有 SSH 成功日志。
- 没有 PC ↔ Jetson ping/iperf 结果。
- 没有 Jetson 端执行 `ros2 topic hz /rslidar_points`、读取 `/lf/sportmodestate` 或观察 `/api/sport/response` 的明确保存记录。
- 因此不能把 Jetson 的 B2 网段地址和本机编译成功写成“Jetson 已直接控制或接收 B2 数据”。
- Jetson 用户态 Ubuntu 版本应补录 `/etc/os-release` 原始输出，不能只依据 L4T 版本推断。
- NVMe 型号需复核系统枚举与照片标签的冲突。

## 6. Sensor Evidence

### 6.1 RoboSense LiDAR：`REAL_ROBOT`

- Topic：`/rslidar_points`。
- Type：`sensor_msgs/msg/PointCloud2`。
- Publisher：1。
- Frequency：约 9.94–10.01 Hz。
- Frame：`rslidar`。
- Header timestamp：非零。
- 相关已保存节点名称包括 `/rslidar_sdk/rslidar_end_node` 和 `/rslidar_sdk/rslidar_points_destination_0`。
- preflight 一次真实样本为 57600 points。

局限：`base_link→lidar_link` 使用项目 URDF 中 `xyz=0.34218 0 0.17851`，`lidar_link→rslidar` 是零变换 frame alias。TF 链连通不等于六自由度外参已经精确标定。

### 6.2 B2 状态与里程计：`REAL_ROBOT`

- `/dog_odom`：约 501.5 Hz，`odom→robot_center`，静止速度接近零。
- 项目 `b2_driver_py` 实际订阅 `/lf/sportmodestate` 和 `/lf/lowstate`，不是 `/dog_odom`。
- 项目 `/odom`：约 20 Hz，`odom→base_link`，并发布对应 TF。
- 完整静止 TF：`odom→base_link→lidar_link→rslidar`。

局限：当前 driver 协方差是保守初值，不是重复实测标定结果；运动状态下的里程计连续性和漂移尚未用 rosbag 量化。

### 6.3 Livox 与板载 LIO-SAM：未形成有效输入证据

- `/livox/lidar` 名称可见，但 Publisher = 0，等待约 1 分钟无数据。
- `/lio_sam_ros2/mapping/odometry` 类型可见，但 Publisher = 0，无数据。
- topic 名称存在不能作为真实传感器输出或 LIO-SAM 工作证据。

### 6.4 相机与深度相机

- 仓库内没有真实相机消息、图像文件、相机内参、帧率、编码、时间戳或外参记录。
- `/front_videohub/videostream` 等名称出现在 topic 清单中，但没有消息类型与实际帧证据，不能当作 RGB 或深度相机已经接入。
- 当前巡检代码只为通用 `sensor_msgs/Image` 或 `CompressedImage` 提供默认占位 topic。
- 当前 RTAB-Map 设置 `subscribe_rgb=false`、`subscribe_depth=false`、`subscribe_rgbd=false`、`subscribe_stereo=false`。
- 深度相机仍为 `PLANNED`。

### 6.5 原始证据文件缺失

当前仓库中没有找到 rosbag、`.db3`、`.mcap`、RTAB-Map `.db`、实拍 `.jpg/.png` 或质量 `.json`。现有真实证据主要是 Markdown 文字记录和少量 topic/node 文本快照，尚不能完成高质量的离线复算和论文定量分析。

## 7. SLAM / Navigation Evidence

### 7.1 3D SLAM 静止链路：`REAL_ROBOT`

数据链：

```text
/rslidar_points（约 10 Hz）
  → sensor_time_sync
  → /b2_slam_3d/cloud_sync
  → RTAB-Map
  → /cloud_map

/lf/sportmodestate
  → b2_driver_py
  → /odom（约 20 Hz）与 odom→base_link
  → sensor_time_sync
  → /b2_slam_3d/odom_sync
  → RTAB-Map
```

实机演变：

1. `sensor_time_sync` 成功转发真实点云约 10 Hz、里程计约 20 Hz。
2. `slam_preflight` 首次因 ROS2 Humble logger 调用方式崩溃，但数据本身显示点云、里程计、时间戳和 TF 正常。
3. 修复 logger 后，preflight 稳定输出 `通过=true`、点云约 9.7–10 Hz、57600 points、里程计约 20 Hz、`odom→base_link`、无失败原因。
4. RTAB-Map 第一次启动出现 Reliability QoS 不兼容并持续 5 秒收不到数据。
5. 配置改为 `qos_scan=2`（Best Effort）后，QoS 错误消失，RTAB-Map 连续处理约 1 Hz。
6. 静止时 `local map=1`、`WM=1`；`map→odom` 为单位变换，符合机器人静止条件。
7. `/cloud_map` 有真实非空 PointCloud2：`frame_id=map`、`width=1317`、`point_step=32`、`row_step=42144`。
8. 当时数据库约 724 KB，但该数据库没有保存在当前 Git 工作区中。

以上只证明静止状态真实 3D 输入/输出链，不证明运动建图质量。

### 7.2 当前 3D 配置：`CODE_ONLY` 与待验证参数

- 纯激光 ICP：`Reg/Strategy=1`。
- 保留六自由度：`Reg/Force3DoF=false`。
- 运动更新初值：线位移 0.15 m、角位移 0.08 rad。
- voxel 0.10 m，最大对应距离 1.0 m，30 次 ICP。
- 3D grid 开启；最大地面角初值 45°。
- 不使用 RGB、深度、RGB-D、stereo 或原始 IMU。
- `sensor_time_sync` 默认把点云改到主机接收时间附近并回退 0.05 s；这不是 PTP/PPS 或硬件同步。

上述数值是代码初值，不是运动、坡道或工厂场景标定结果。

### 7.3 运动 3D mapping：`NEEDS_EVIDENCE`

截至 2026-09-11 明确未执行：

- 小范围运动 RTAB-Map；
- 运动 rosbag；
- 地图节点随运动增长检查；
- 重影、分层、漂移和闭环评价；
- 重复路线误差；
- 坡道 6DoF 建图；
- 长期稳定性和 Jetson 资源占用。

### 7.4 Localization：`CODE_ONLY`

`autonomous_inspection.launch.py` 已配置 RTAB-Map localization 模式：

```text
Mem/IncrementalMemory=false
Mem/InitWMWithAllNodes=true
```

但没有在已有三维数据库上完成真实 B2 重定位、丢失恢复或定位误差实验。静止时出现 `map→odom` 单位变换不能替代 localization 验证。

### 7.5 Nav2 与局部避障：`CODE_ONLY`

已有代码包括：

- PointCloud2 → LaserScan；
- map server；
- AMCL 或外部 RTAB-Map localization；
- NavFn 全局规划；
- DWB 局部控制；
- local/global costmap；
- voxel/obstacle/inflation layer；
- NavigateToPose；
- 安全速度桥和自主状态监视器。

没有以下真实证据：

- `/scan` 在当前 B2 上的频率、范围和障碍投影；
- Nav2 lifecycle 全部 active；
- 单目标导航；
- 到点误差、路径跟踪、制动距离；
- 静态障碍、移动人员或窄通道避让；
- BackUp、Spin、横移采样和恢复行为。

文档/代码冲突：`src/b2_navigation/README.md` 的旧示例片段写有 `min_vel_x=-0.30`、`max_vel_y=0.30`、`max_vel_theta=0.50`，但当前实际 `config/nav2_params.yaml` 为 `min_vel_x=0.0`、`max_vel_y=0.0`、`max_vel_theta=0.20`。论文和实验应以实际 YAML 与运行时参数 dump 为准。

## 8. Inspection Pipeline Evidence

### 8.1 当前主流程代码结构

```text
真实/占位相机图像
  → ArUco 目标框（户外功能验证代理）
  → 目标区域质量评价
  → /inspection/photo_clear
  → mission_manager
  → 合格：下一个 waypoint
  → 不合格：下一个预定义候选观测位姿
  → Nav2 NavigateToPose
  → 稳定等待后重新采集
```

同时使用：

- `safety_monitor.py`：要求机器人状态、SLAM preflight、二维 scan 和 `map→base_link` 都新鲜。
- `twist_bridge.py`：要求任务心跳、机器人状态和自主许可，并在超时时锁定和 STOPMOVE。
- `mission_manager.py`：导航/图像/安全失败时取消目标、停止拍照请求并关闭控制锁。
- `mission_autostart=false`：调试阶段默认不会自动开始任务。

### 8.2 已有软件能力：`CODE_ONLY`

- 严格读取路线和候选观测位姿。
- 严格读取 ArUco 目标与 ROI 扩展配置。
- 调用 Nav2 `NavigateToPose`。
- 到点后等待 1.5 s 再请求新图像。
- 拍照请求后只接受新图像和时间接近的新 bbox。
- 根据目标置信度、面积、清晰度、亮度、暗部和过曝给出原因。
- 保存原图和同名 JSON 报告。
- 不合格时依次选择下一个候选观测位姿并再次采集。
- 所有候选失败、质量超时、导航失败或安全状态丢失时进入安全暂停。

### 8.3 不能作为实机证据的内容

- `patrol_waypoints.yaml` 明确注明坐标只是格式示例，不可直接用于真机。
- `inspection_targets.yaml` 的 ArUco 10/20 与 ROI 参数只是户外功能验证初值。
- `photo_quality.yaml` 的阈值没有真实相机数据和下游故障检测任务标定。
- 单元测试使用棋盘格和均匀图像等合成数组，只证明函数逻辑，不是 `SIMULATION` 或 `REAL_ROBOT`。
- `src/代码检查报告.md` 中“观测位姿已验证候选按顺序尝试”应理解为软件逻辑检查；2026-09-09 实机日志明确没有自动巡检，因此不能解释为 B2 实机验证。
- 旧 `fixed_route_3d_patrol_node.py` 包含直接速度与简易扇区避障原型，但入口已移除且不属于当前主流程。其 0.28 m/s 前进、0.22 m/s 横移和 0.55 rad/s 转向参数与当前实测/安全基线不一致，不得运行或引用为有效实验参数。

### 8.4 尚未形成的闭环实证

没有日志证明真实 B2 已完成以下任一完整序列：

```text
自主导航到目标
→ 真实相机拍照
→ 真实目标定位
→ 图像质量不合格
→ 自主移动到安全候选位姿
→ 重新拍照
→ 图像合格
→ 前往下一巡检点
```

因此固定路线、图像采集、质量评价、目标定位、主动调整、重采集和端到端闭环都不能写成 `REAL_ROBOT`。

## 9. Confirmed REAL_ROBOT Evidence

以下仅列有明确日志支撑的范围：

1. Ubuntu PC ↔ B2 有线网络、CycloneDDS 和 ROS2 topic 发现。
2. `/api/sport/request`、`/api/sport/response` 的 B2 Sport 通信链。
3. 未解锁时安全程序拒绝非 STOP 动作。
4. `BALANCESTAND` 引起轻微实际调整并保持稳定。
5. 前进：`x=+0.30 m/s`、0.60 s 明确位移；0.10–0.20 m/s 没有明确位移。
6. 左转：`z=+0.20 rad/s`、0.60 s，产生幅度较小的实际左转并自动停止。
7. 后退：`x=-0.30 m/s`，3/3 明确位移并自动停止。
8. 左横移：`y=+0.30 m/s`、约 1.00 s，3/3 明确位移。
9. 右横移：`y=-0.30 m/s`、约 1.00 s，3/3 明确位移。
10. 横移单次 MOVE A/B 对照成功；20 Hz 重发不是必要条件。
11. MOVE 1008 与 STOPMOVE 1003 响应及 watchdog 自动停车链。
12. Jetson Orin Nano Super 的真实板卡基础环境、Ethernet、ROS2 依赖和 aarch64 本机编译。
13. RoboSense `/rslidar_points` 真实 PointCloud2，约 10 Hz，frame `rslidar`。
14. `/dog_odom`、`/lf/sportmodestate`、`/lf/lowstate` 和项目 `/odom` 数据。
15. 静止状态 `odom→base_link→lidar_link→rslidar` TF 链。
16. `sensor_time_sync` 和 `slam_preflight` 接收真实 B2 数据并通过。
17. QoS 修复后 RTAB-Map 接收真实点云、发布静止 `map→odom` 和非空 `/cloud_map`。

不得把第 17 项缩写为“B2 运动 3D SLAM 已完成”。

## 10. CODE_ONLY Evidence

1. 右旋转键位和负角速度映射。
2. Python 自主速度桥的任务心跳、状态许可、控制锁、限幅和超时停车。
3. 运动状态 3D mapping 的 RTAB-Map launch/config 初值。
4. 已有数据库上的 RTAB-Map localization 模式。
5. PointCloud2 → LaserScan 配置。
6. Nav2 全局规划、DWB 局部控制、代价地图和恢复行为配置。
7. Nav2 单目标导航和局部避障链。
8. 旧固定路线直接速度巡检原型（已从当前安装入口移除）。
9. 巡检路线与候选观测位姿解析。
10. ArUco 目标配置与目标框发布。
11. 通用 RGB/压缩图像订阅、拍照触发和 JPG/JSON 保存。
12. 图像质量多阈值评价。
13. 图像不合格后的候选观测位姿顺序切换。
14. 到新候选位姿后的重新采集逻辑。
15. 感知—决策—运动—再感知的整体 launch 和状态机。
16. 安全状态监视、任务暂停、目标取消和关闭控制锁逻辑。

这些项目即使有单元测试、构建成功或 `--show-args` 成功，也仍然是 `CODE_ONLY`，直到获得对应实机数据。

## 11. PLANNED Work

1. 深度相机选型、驱动、内参/外参和真实深度数据接入。
2. Jetson 机器人端可靠供电、机械固定、防护、散热和振动验收。
3. Jetson 5 GHz WLAN、PC 无线监控、双网卡 DDS 和断网安全测试。
4. 运动状态 RTAB-Map、完整 rosbag、平地闭环与坡道 6DoF 建图。
5. 重复定位、地图精度、漂移和长期稳定性实验。
6. Nav2 单目标导航、静态障碍、移动人员、窄通道和恢复行为实验。
7. 户外 A/B 目标场景、真实 RGB 相机、ArUco 代理目标与质量阈值标定。
8. 基于图像质量和运动代价的候选观测位姿评分，而不只是固定顺序尝试。
9. 感知/定位/相机/任务/网络故障注入。
10. 工厂多点、坡道、人流和设备目标实地实验。

## 12. NEEDS_EVIDENCE / Needs Re-test

### 12.1 控制与动作

- 右旋转是否方向正确、可重复、能自动稳定停止。
- 真正从蹲姿执行 STANDUP、执行 STANDDOWN、执行 RECOVERYSTAND 的可观察结果。
- 左转的 3/3 重复、角度、角速度和制动结果。
- 前进/后退/横移每次实际位移、打滑和停止距离。
- `0.20–0.35 rad/s` 安全测试角速度区间；目前只有 0.20 rad/s 左转证据。
- `x/y` 组合平移、Nav2 BackUp、Spin 和横移控制。

### 12.2 Jetson 与网络

- Jetson `/etc/os-release` 或 `lsb_release` 原始输出。
- PC ↔ Jetson 的 ping、SSH 和吞吐证据。
- Jetson 端发现 B2 ROS2 topic、接收真实状态/点云和 Sport response 的证据。
- WLAN interface、5 GHz 连接、重启恢复、丢包、带宽和装机后覆盖。
- NVMe 型号冲突复核。
- RTC/NTP 长期同步。

### 12.3 传感器与 SLAM

- 一份带运动的完整 rosbag；当前仓库没有 bag。
- RoboSense 到 `base_link` 的精确六自由度外参。
- 点云与里程计的真实时钟偏差，以及是否需要 PTP/PPS。
- 运动中 odom 连续性、漂移和横移/旋转响应。
- RTAB-Map 平地闭环、重复性、地图重影、漂移、闭环误差和坡道高度误差。
- Livox 和板载 LIO-SAM 是否实际启用；当前 publisher 为 0。

### 12.4 导航与巡检

- `/scan` 真实数据和障碍投影质量。
- RTAB-Map localization 或 AMCL 的真实定位效果。
- Nav2 单目标导航、局部避障、到点精度和人工接管率。
- 相机型号、图像 topic、内参、外参、帧率、曝光和时间戳。
- ArUco 目标定位在距离、角度、光照和遮挡变化下的结果。
- 图像质量阈值与下游故障检测可用性的对应关系。
- 候选观测位姿的可达性、安全性和是否真正改善图像。
- 完整主动重观测闭环和工厂实地结果。

## 13. Evidence Files for Windows

### 13.1 第一优先级：事实日志

| Relative path | Why it matters |
|---|---|
| `docs/B2_DEBUG_LOG_2026-09-08.md` | PC↔B2 有线、DDS/Sport、安全锁、前进速度演变、左转和自动停止的原始文字证据 |
| `docs/2026-09-09_B2_Jetson_实机调试记录.md` | Jetson 基础验收、后退/横移、真实点云/odom/TF、preflight、RTAB-Map QoS 演变和静止 cloud map 的核心证据 |
| `docs/debug_logs/b2_node_list.txt` | B2 侧发现的节点文本快照 |
| `docs/debug_logs/b2_topic_list.txt` | B2 侧发现的话题文本快照 |
| `docs/debug_logs/b2_sport_request_info.txt` | `/api/sport/request` 类型、端点、QoS 和订阅者证据 |

### 13.2 第二优先级：当前可恢复基线与边界规则

| Relative path | Why it matters |
|---|---|
| `AGENTS.md` | 长期实验真实性、安全和 Git 规则；同步前已是 untracked 文件 |
| `README.md` | 当前项目入口与最新实机基线索引 |
| `docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md` | 下一次运动 3D SLAM 与 rosbag 采集方案；属于计划，不是已完成证据 |
| `docs/B2_REVERSE_LATERAL_TEST.md` | 后退/横移复现条件和最终参数摘要 |
| `docs/JETSON_WIRELESS_SELECTION.md` | Jetson 无线硬件照片判断与成功验收条件；当前成功无线仍未完成 |
| `src/代码检查报告.md` | 软件检查结果与历史代码边界；实机结论必须被 09-09 日志校正 |
| `src/项目路线与论文实验方案.md` | 论文方向、指标和后续实验设计，不是实机结果 |

### 13.3 第三优先级：可复核实现

| Relative path | Why it matters |
|---|---|
| `src/b2_twist_bridge_py/b2_twist_bridge_py/safe_motion_test.py` | 实际按键映射、单次 MOVE、watchdog、参数默认值和硬上限 |
| `src/b2_twist_bridge_py/b2_twist_bridge_py/sport_model.py` | B2 Sport API 编号与安全白名单 |
| `src/b2_twist_bridge_py/b2_twist_bridge_py/twist_bridge.py` | 自主速度桥、安全锁、心跳、状态、超时和 STOPMOVE |
| `src/b2_twist_bridge_py/b2_twist_bridge_py/velocity_policy.py` | 实测速度整形与组合平移禁用规则 |
| `src/b2_driver_py/b2_driver_py/driver.py` | `/lf/sportmodestate` 到 `/odom` 和 TF 的转换 |
| `src/b2_slam_3d/b2_slam_3d/sensor_time_sync.py` | 当前主机时钟兼容方法及其非硬件同步边界 |
| `src/b2_slam_3d/b2_slam_3d/slam_preflight.py` | 真实点云、odom、时间戳和 TF 健康检查 |
| `src/b2_slam_3d/config/rtabmap_3d.yaml` | `qos_scan=2`、6DoF 和 ICP/Grid 参数基线 |
| `src/b2_slam_3d/launch/slam_3d.launch.py` | 当前真实 3D 数据链组合方式 |
| `src/b2_navigation/config/nav2_params.yaml` | 实际 Nav2 运行参数，应优先于 README 旧示例 |
| `src/b2_factory_inspection/launch/autonomous_inspection.launch.py` | 定位、Nav2、相机、目标、质量、安全与任务的 CODE_ONLY 集成入口 |
| `src/b2_factory_inspection/b2_factory_inspection/mission_manager.py` | 候选观测位姿、重拍和安全暂停状态机 |
| `src/b2_factory_inspection/b2_factory_inspection/photo_quality_node.py` | 新帧/新 bbox 同步、保存和质量报告 |
| `src/b2_factory_inspection/b2_factory_inspection/quality_core.py` | 可解释图像质量指标实现 |
| `src/b2_factory_inspection/b2_factory_inspection/aruco_target_locator.py` | 户外代理目标框生成方式 |
| `src/b2_factory_inspection/config/patrol_waypoints.yaml` | 明确标为示例的候选位姿格式 |
| `src/b2_factory_inspection/config/inspection_targets.yaml` | ArUco 10/20 与 ROI 初值 |

当前不应只同步代码而遗漏日志。Windows 论文仓库引用实验结论时，应同时保存 Git SHA、事实日志路径和相应原始数据位置。

## 14. Current Research Baseline

### 14.1 截至 2026-09-11，真实已经完成了什么？

已经完成的是一个有真实设备支撑的“基础控制 + 静止 3D 数据链”基线：

- Ubuntu PC 能通过有线 ROS2/CycloneDDS 与 B2 Sport 接口通信。
- B2 前进、后退、左右横移和左转在记录参数下产生过预期实际运动，并有自动 STOPMOVE。
- 横移实验保留了从低速度无位移到 0.30 m/s 明确位移、再到单次 MOVE A/B 对照的完整演变。
- 真实 Jetson Orin Nano Super 的基础硬件、ROS2 环境和 aarch64 编译已检查。
- 真实 RoboSense 点云、B2 状态、项目 odom 和 TF 已接入。
- `sensor_time_sync`、`slam_preflight` 和 RTAB-Map 在机器人静止时跑通，产生非空 `/cloud_map`。

这仍然不是“自主巡检系统完成”。

### 14.2 距离完整的“自主巡检 + 观测质量驱动主动重观测闭环”还缺什么？

至少还缺五个连续证据层：

1. **运动 3D SLAM 层**：完整 rosbag、平地/坡道闭环、外参与时间同步、地图精度、漂移和重复性。
2. **定位与导航层**：真实数据库定位、PointCloud2→LaserScan、Nav2 单目标导航、到点精度、静态/动态障碍与恢复行为。
3. **真实视觉层**：相机型号、内外参、图像数据、真实目标框，以及质量阈值与故障检测可用性的标定。
4. **主动重观测层**：候选位姿可达性和安全性、图像改善量、移动代价、失败恢复，以及端到端重复实验。
5. **板载与工厂层**：Jetson↔B2 直接通信、可靠供电/安装、无线只读监控与断网安全，最后完成多点、坡道、人流和设备目标的工厂实验。

当前最准确的论文表述是：

> 已建立并实机验证 Unitree B2 的有线基础运动控制，以及基于真实 RoboSense 点云和 B2 状态里程计的静止 RTAB-Map 三维数据链；自主导航、真实视觉质量评价与主动重观测闭环已形成软件基线，但尚未完成 B2 端到端实机验证。
