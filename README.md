# 宇树 B2 工厂自主巡检工程

本工程采用 ROS2 Humble 和 Python 主流程，实现 B2 基础控制、3D 建图与定位、Nav2 导航避障、巡检点拍照、图像质量评价和候选观测位姿调整。

2026-09-09 已完成有线网络、B2 后退/左右横移和静止 RTAB-Map 3D 数据链验证。
后续开发以实机记录为基线，不重复已通过的网络、站立和单轴运动测试：

- [B2 实机调试基线](docs/B2_DEBUG_LOG_2026-09-08.md)
- [2026-09-09 B2 + Jetson 实机调试记录](docs/2026-09-09_B2_Jetson_实机调试记录.md)
- [下一次有线运动 3D SLAM 调试手册](docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md)
- [SLAM 与巡检长期分阶段手册](docs/SLAM_INSPECTION_NEXT_STAGE.md)
- [B2 后退与横移专项实机测试手册](docs/B2_REVERSE_LATERAL_TEST.md)
- [2026-09-09 B2 与 Jetson 历史调试清单](docs/B2_JETSON_DEBUG_2026-09-09.md)
- [Jetson 无线通信验收与升级条件](docs/JETSON_WIRELESS_SELECTION.md)
- [有线基础控制调试手册](src/b2_twist_bridge_py/有线基础控制调试手册.md)
- [完整巡检系统分阶段调试手册](src/b2_factory_inspection/OPERATION_GUIDE.md)
- [项目路线与论文实验方案](src/项目路线与论文实验方案.md)
- [本周文献阅读清单（中文翻译、总结、创新点与机理）](src/文献阅读/本周文献阅读清单.md)
- [代码检查报告](src/代码检查报告.md)

程序全部放在 `src/` 内。旧 C++ 驱动和速度桥仅保留作历史对照，默认编译已忽略它们。
当前 SLAM 默认使用已实机确认约 10 Hz 的 `/rslidar_points` 和约 20 Hz 的项目
`/odom`。无线通信采购与测试延期，不阻塞下一阶段有线运动 3D SLAM 数据采集。
