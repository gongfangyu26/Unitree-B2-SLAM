# B2 工厂巡检 Python 包

主要程序：

- `mission_manager.py`：调用 Nav2，管理路线点、观测点、重拍和安全暂停；
- `photo_quality_node.py`：判断目标是否存在、大小是否合适、是否清晰和曝光正常；
- `aruco_target_locator.py`：在户外模拟试验中定位 A、B 目标；
- `safety_monitor.py`：检查机器人状态、SLAM 输入、二维雷达和全局定位新鲜度；
- `jetson_board_test.py`：只读检查 Jetson 型号、硬盘、网卡、温度和功耗模式；
- `mapping_3d.launch.py`：启动 3D 建图；
- `autonomous_inspection.launch.py`：启动 3D 定位、Nav2、图像评价和巡检任务。

路线配置使用 `config/patrol_waypoints.yaml`，ArUco 目标编号使用 `config/inspection_targets.yaml`。

`fixed_route_3d_patrol_node.py` 和 `fixed_route_3d_patrol.launch.py` 是原始直接速度控制基线，不用于新的实机流程。

调试时 `mission_autostart` 默认关闭，由操作者调用 `/inspection/start`。
最终 Jetson 部署可在 G0～G4 全部通过后打开该参数，让监控电脑保持只读。

2026-09-09 已完成后退、左右横移和静止 3D 数据链实机验收。当前阶段使用
`docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md` 进行有线运动 3D SLAM 数据采集；无线通信因
采购延期而暂不作为进入条件，也不重复网络和 Sport 单轴动作测试。

详细命令和阶段通过标准见 [完整巡检系统分阶段调试手册](OPERATION_GUIDE.md)。
