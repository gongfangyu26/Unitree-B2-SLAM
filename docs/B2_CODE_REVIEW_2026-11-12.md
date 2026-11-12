# B2 代码审阅与交接说明

整理日期：2026-11-12（Asia/Shanghai）；整理人：用户 / Codex。
用途：供师兄审阅当前源码，并为 2026-11-13 实机调试确定检查顺序。
本次不修改控制逻辑，不连接或控制机器人。

## 1. 建议阅读顺序

1. 本文：先了解架构、已知问题与证据边界。
2. [10-30 调试记录第 40 节](B2_DEBUG_LOG_2026-10-30.md#40-当日收尾录点受阻与显示卡顿巡航未执行)：最近现场结论。
3. [导航启动文件](../src/b2_navigation/launch/navigation.launch.py)与
   [Nav2 参数](../src/b2_navigation/config/nav2_params.yaml)：定位、规划与速度流。
4. [导航安全门](../src/b2_navigation/b2_navigation/navigation_gate.py)、
   [底层速度桥](../src/b2_twist_bridge_py/b2_twist_bridge_py/twist_bridge.py)：锁、超时与停车。
5. [巡逻节点](../src/b2_factory_inspection/b2_factory_inspection/corridor_patrol.py)与
   [纯逻辑核心](../src/b2_factory_inspection/b2_factory_inspection/corridor_patrol_core.py)：规划检查、有限圈数与异常取消。
6. [明天现场清单](B2_TEST_PLAN_2026-11-13.md)：按前一关通过后才进入下一关。

## 2. 模块与控制关系

| 模块 | 主要入口 / 文件 | 职责 |
|---|---|---|
| `b2_description` | URDF、RViz 资源 | 驱动启动所需描述依赖 |
| `b2_driver_py` | `driver.py`、`driver.launch.py` | B2 状态转 `/odom`、TF、关节状态和 `/b2/state_ok` |
| `b2_twist_bridge_py` | `twist_bridge.py`、`velocity_policy.py` | 人工解锁、状态/心跳检查、限幅整形与 STOPMOVE |
| `b2_slam` | `pointcloud_timestamp_fix.py` | 导航点云接收时间兼容与输入 QoS |
| `b2_slam_3d` | `slam_3d.launch.py`、`sensor_time_sync.py`、`slam_preflight.py` | RTAB-Map 三维建图、输入健康检查 |
| `b2_slam_3d` | `occupancy_grid_preflight.py`、`pointcloud_snapshot.py` | 二维栅格检查与拒绝覆盖的 PCD 保存 |
| `b2_navigation` | `navigation.launch.py`、`navigation_safety_monitor.py` | AMCL 或 RTAB-Map 定位、Nav2 与感知安全条件 |
| `b2_navigation` | `navigation_gate.py`、`waypoint_recorder.py` | 导航门与 RViz 点击录点；录点不发送导航目标 |
| `b2_factory_inspection` | `corridor_patrol.launch.py`、`corridor_patrol.py` | 平地有限圈数巡逻；默认待命，不自动解锁 |
| `b2_factory_inspection` | `mission_manager.py`、`photo_quality_node.py` | 拍照巡检与图像质量反馈；实机闭环待验收 |
| `tools/b2_sdk_smoke_test` | `main.cpp`、`state_monitor.hpp`、`keyboard_control.hpp` | 独立官方 SDK 视觉模式对照，不属于 ROS 导航链 |

```text
RoboSense /rslidar_points ── 时间兼容 / 点云转激光 ── /scan
B2 状态 ── b2_driver_py ── /odom、odom -> base_link、state_ok
二维地图 + /scan + /odom ── AMCL ── map -> odom
                          （也可单独选择 RTAB-Map 定位）

路线 -> corridor_patrol -> Nav2 action -> /cmd_vel_nav
     -> navigation_gate -> /cmd_vel -> b2_safe_twist_bridge
     -> /api/sport/request -> B2
```

导航门和速度桥分别人工解锁。地图、状态、雷达、定位或心跳失效时阻止运动；
命令超时默认 0.35 s，走廊巡逻进程心跳超时默认 0.60 s。
这些是软件检查阈值，实际制动与通信中断时的响应仍须现场测量。
基础测试、地形测试和 SDK 工具直接走独立控制入口，不能依靠导航双锁保护它们，
也不能与导航或其他运动程序同时运行。

## 3. 已有证据与尚未完成事项

| 结论层级 | 结果 | 证据 |
|---|---|---|
| 实机已验证（历史） | 基础动作、前后/左右短运动和自动停车；横移 0.30 m/s、约 1 s 左右各 3/3 | [09-09 原始记录](2026-09-09_B2_Jetson_实机调试记录.md) |
| 实机已验证（历史） | 平地运动建图和地图保存已有记录；不代表定量漂移或闭环验收 | [10-13](B2_DEBUG_LOG_2026-10-13.md)、[10-16](B2_DEBUG_LOG_2026-10-16.md) |
| 实机已验证（有输出） | 走廊局部 `pilot_02_cloud.pcd` 导出，43,566 点、约 681 KiB | [10-30 第 30 节](B2_DEBUG_LOG_2026-10-30.md#30-后续局部采集与-jetson-pcd-导出成功弯曲轮廓符合现场反馈) |
| 实机已验证（用户反馈） | 走廊整圈遥控采集与完整地图保存；完整文件大小/点数和最终目录待核对 | [10-30 第 36–40 节](B2_DEBUG_LOG_2026-10-30.md#36-用户反馈整圈遥控采集完成准备保存完整地图并配置巡航点) |
| 实机已验证（历史） | SDK v2 的 10 s 视觉模式进入、保持、退出；该过程没有 Move | [10-28 第 28 节](B2_DEBUG_LOG_2026-10-28.md#28-v2_reference在jetson完成10秒电脑视觉模式流程) |
| 仅代码检查，尚未实机验证 | 有限圈数巡逻、任务异常取消、0.60 m/s 自主制动和完整任务 | [本次软件检查](B2_DEBUG_LOG_2026-11-12.md)；[10-30](B2_DEBUG_LOG_2026-10-30.md) |
| 未执行 / 待验证 | 新键盘视觉运动、电脑控制上下楼、图像质量调整的实机闭环 | [10-28 第 29 节](B2_DEBUG_LOG_2026-10-28.md#29-持续视觉键盘入口vision_keyboard_v3仅离线验证) |

日期较早的 README、总记录和 AGENTS 中的静止 SLAM 状态属于历史基线，
应以上表所列更晚的原始实机记录校正；本次不重写过去的失败过程。

## 4. 希望师兄重点检查的地方

1. **二维地图和代价图。** 上次存在底图黑点及录点受阻。分离人员残影、高处点投影、
   正常膨胀区与实际障碍；不直接把彩色区域全部擦掉。当前 footprint 为
   1.40 × 0.80 m、padding 0.05 m、inflation_radius 0.55 m，录点器默认只接受零代价。
   参数是否适合通道，应结合实际机身、摆腿净空、制动与地图误差评估。
2. **定位与时间。** AMCL 初始位姿隔离在 `/b2_navigation/initialpose`；
   不再默认初始化地图原点。检查唯一 `map -> odom`、激光与地图对齐、
   实际位姿误差，以及接收时间重打戳在运动时的影响。
3. **速度整形与控制器。** 默认线速度有效值 0.30 m/s、角速度 0.35 rad/s
   来自项目特定实测。小指令被抬升后，可能影响 DWB 预测、到点振荡和制动；
   这是需验证的控制问题。走廊入口默认上限 0.60 m/s，明天短测显式降为 0.30。
4. **任务停车。** 审核超时、迟到 action accepted、取消、节点退出、心跳丢失时
   是否有新的零速度与双锁关闭证据。正常软件退出不等于断电/系统冻结时可保证停车。
5. **RViz 与网络。** 优先 PC 本地 RViz、Jetson 无界面运行；先验证实际 DDS 路由，
   不为无线显示随意切换 B2 有线网卡配置。上次卡顿原因尚无量化证据。
6. **视觉状态定义。** SDK 工具的状态元组 2/3/1 只适用于已核对的本机参考。
   不据此解释所有非零状态；`api_id=1034` 的官方含义仍不确定。

## 5. 软件复现与发布范围

编译命令见[根 README](../README.md#源码与编译)，必须包含 `b2_description`，
并先 source ROS2 与 Unitree 消息工作区。SDK 工具单独按其
[README](../tools/b2_sdk_smoke_test/README.md)构建，使用已记录的 SDK 提交；
不要把最新下载的不同版本直接混入同一组实机实验。

本次检查结果、失败尝试和环境限制统一记录在
[11-12 调试记录](B2_DEBUG_LOG_2026-11-12.md)。所有本机测试均不能替代 B2 实机验收。

上传内容为现有相关源码、配置、测试、Markdown 历史记录及本次交接资料。
官方工单 `.txt` 原稿、Word 周报保留本地；不上传 bag、地图数据库、PCD、构建产物和缓存。
基线 `fabbc17` 保留；本次整理起点为 `a99cc7b` 加既有工作区改动，
发布到用户 `origin` 的 `codex/b2-review-2026-11-12` 分支。
