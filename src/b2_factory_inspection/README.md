# B2 工厂巡检 Python 包

主要程序：

- `mission_manager.py`：调用 Nav2，管理路线点、观测点、重拍和安全暂停；
- `photo_quality_node.py`：判断目标是否存在、大小是否合适、是否清晰和曝光正常；
- `aruco_target_locator.py`：在户外模拟试验中定位 A、B 目标；
- `safety_monitor.py`：检查机器人状态、SLAM 输入、二维雷达和全局定位新鲜度；
- `front_lidar_distance.py`：从 RoboSense 点云计算稳定的前向距离，不发送运动指令；
- `jetson_board_test.py`：只读检查 Jetson 型号、硬盘、网卡、温度和功耗模式；
- `mapping_3d.launch.py`：启动 3D 建图；
- `autonomous_inspection.launch.py`：启动 3D 定位、Nav2、图像评价和巡检任务。

路线配置使用 `config/patrol_waypoints.yaml`，ArUco 目标编号使用 `config/inspection_targets.yaml`。

`fixed_route_3d_patrol_node.py` 和 `fixed_route_3d_patrol.launch.py` 是原始直接速度控制基线，不用于新的实机流程。

调试时 `mission_autostart` 默认关闭，由操作者调用 `/inspection/start`。
最终 Jetson 部署可在 G0～G4 全部通过后打开该参数，让监控电脑保持只读。

最新现场状态：2026-10-30 走廊遥控建图与保存获用户反馈，录点受阻且 RViz 延迟，
完整自主巡航未执行。下一阶段先检查导航地图、定位和显示链路，再测短目标。
审阅见[代码交接说明](../../docs/B2_CODE_REVIEW_2026-11-12.md)，
现场见[11 月 13 日清单](../../docs/B2_TEST_PLAN_2026-11-13.md)。

详细命令和阶段通过标准见 [完整巡检系统分阶段调试手册](OPERATION_GUIDE.md)。

## 走廊有限圈数巡逻（2026-10-30 新增）

`corridor_patrol.launch.py` 复用地图、AMCL、Nav2 和双重安全锁，启动后保持待命。
路线由 RViz `Publish Point` 采集，支持 `laps:=N` 的闭环和 `route_mode:=once`
短段首测；闭环会自动返回首点。先 `/patrol/validate` 在锁定状态逐段规划，
READY 后由现场人员双重解锁并调用 `/patrol/start`。节点不自动解锁；失败、
超时、退出和 `/patrol/stop` 都请求停车；导航门额外监控巡逻进程存活心跳。

按约 100 m、1 圈实机试验设置，默认单目标上限 `60 s`、全部圈数总任务上限
`600 s`；一圈完成就停车，不必等满十分钟。短距离首测可显式设较短的时间。
走廊入口默认 `max_linear_x:=0.60`，同步设置 DWB 与随入口启动的速度桥；
沿用已有驱动时须先停车重启并核对其上限。0.6 m/s 的闭环制动/避障待实机验证。

**仅代码检查，尚未实机验证**。本次只使用 AMCL 平地定位，不能用于坡道/楼梯。
新场景必须先建图和验证实际定位。命令、停车方法和实验记录要求见
[大走廊建图与闭环巡逻手册](../../docs/B2_CORRIDOR_PATROL_GUIDE_2026-10-30.md)。

## 稳定前向 LiDAR 距离

`front_lidar_distance` 使用已经实测确认的 `rslidar +X` 前向定义。默认流程为：

```text
有限值过滤
→ x=0.30~5.00 m、|y|≤0.50 m、z=-0.20~1.50 m ROI
→ 搜索最近且至少包含 12 点的 0.20 m 距离簇
→ 单帧距离中位数
→ 最近 5 帧中至少 3 帧有效时输出多帧中位数
```

该算法用于替代“ROI 中单个最近点”，但所有阈值截至 2026-09-23 都只是代码初值，
**待不同实物距离的 B2 实机验证**。节点只读 PointCloud2，不会控制机器人：

```bash
ros2 run b2_factory_inspection front_lidar_distance
```

输出：

```text
/b2/perception/front_distance         std_msgs/msg/Float32
/b2/perception/front_distance_valid   std_msgs/msg/Bool
/b2/perception/front_distance_report  std_msgs/msg/String（JSON）
```

使用距离前必须同时检查 `front_distance_valid=true`，不能沿用上一次有效距离。地面阈值、
距离簇宽度和点数应根据明天的 0.5/1.0/1.5/2.0 m 实物数据调整，不得直接用于自动靠近。
