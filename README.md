# 宇树 B2 工厂自主巡检工程

本工程基于 Unitree B2，使用 Ubuntu 22.04、ROS2 Humble 和 Python 主流程，
研究三维 SLAM、自主导航与工厂巡检。Jetson 承担机器人端计算，电脑用于监控与分析。

## 代码审阅与下一次调试

- **师兄先看：[代码审阅与交接说明（2026-11-12）](docs/B2_CODE_REVIEW_2026-11-12.md)**：模块入口、控制链、验证依据和重点问题。
- **现场使用：[11 月 13 日调试清单](docs/B2_TEST_PLAN_2026-11-13.md)**：地图、定位、录点与短目标分级检查。
- [最近一次实机记录（2026-10-30）](docs/B2_DEBUG_LOG_2026-10-30.md#40-当日收尾录点受阻与显示卡顿巡航未执行)：从第 40 节看最终结论，前文保留实验过程。
- [本次整理和软件检查记录（2026-11-12）](docs/B2_DEBUG_LOG_2026-11-12.md)。

## 当前进展与限制

以下依据截至 2026-10-30 的仓库实机记录；2026-11-12 仅做本机软件检查。

| 项目 | 当前证据 |
|---|---|
| B2 基础动作与短脉冲停车 | 2026-09-08/09 有实机记录；参数只代表当次设备和场地 |
| RoboSense、里程计、TF、静止 SLAM 输入 | 有真实设备验证记录 |
| 运动建图与地图保存 | 10-13/16 有平地记录；10-30 局部三维 PCD 导出有日志，整圈遥控采集和完整保存有用户反馈 |
| AMCL / Nav2 | 有阶段调试记录；10-30 录点受代价图阻碍、RViz 延迟明显，未执行本轮自主巡航 |
| 有限圈数走廊巡逻 | 已有代码和本机测试，完整闭环、稳定转向、避障、0.60 m/s 制动尚未实机验收 |
| SDK 视觉模式 / 楼梯 | 10-28 的 10 秒视觉进入、保持、退出有记录；新键盘运动和电脑控制上下楼尚未验收 |
| 图像质量反馈巡检 | 已有程序与测试，实机任务闭环尚未验收 |

三维 PCD 与二维导航地图分别验收。地图保存成功、RViz 显示正常或测试通过，
均不能替代实机定位、导航和停车验收。上次失败项、证据层级和数据位置见原始日志。

## 源码与编译

主要包的职责和入口见 [src 源码说明](src/README.md)。旧 C++ 驱动和速度桥保留作
历史对照，默认编译已忽略；`tools/b2_sdk_smoke_test` 是另行构建的官方 SDK 对照工具。

在实际仓库根目录执行；Unitree 消息工作区路径按安装位置调整：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
colcon build --packages-select \
  b2_description b2_twist_bridge_py b2_driver_py b2_slam \
  b2_slam_3d b2_navigation b2_factory_inspection --symlink-install
source install/setup.bash
```

还需安装对应的 Nav2、RTAB-Map、点云转激光等运行依赖，清单见各包 `package.xml`。
编译不会启动机器人。现场启动顺序与条件见上面的调试清单；不得同时运行多个运动控制入口。

## 历史手册与研究资料

以下保留历史上下文；旧状态描述应以较新的实机记录校正：

- [B2 实机调试基线](docs/B2_DEBUG_LOG_2026-09-08.md)
- [2026-09-09 B2 + Jetson 实机调试记录](docs/2026-09-09_B2_Jetson_实机调试记录.md)
- [下一次有线运动 3D SLAM 调试手册](docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md)
- [2026-10-24 坡道、楼梯与三维点云调试手册](docs/B2_TERRAIN_3D_DEBUG_GUIDE_2026-10-24.md)
- [SLAM 与巡检长期分阶段手册](docs/SLAM_INSPECTION_NEXT_STAGE.md)
- [B2 后退与横移专项实机测试手册](docs/B2_REVERSE_LATERAL_TEST.md)
- [2026-09-09 B2 与 Jetson 历史调试清单](docs/B2_JETSON_DEBUG_2026-09-09.md)
- [Jetson 无线通信验收与升级条件](docs/JETSON_WIRELESS_SELECTION.md)
- [有线基础控制调试手册](src/b2_twist_bridge_py/有线基础控制调试手册.md)
- [完整巡检系统分阶段调试手册](src/b2_factory_inspection/OPERATION_GUIDE.md)
- [项目路线与论文实验方案](src/项目路线与论文实验方案.md)
- [本周文献阅读清单（中文翻译、总结、创新点与机理）](src/文献阅读/本周文献阅读清单.md)
- [代码检查报告](src/代码检查报告.md)

地图、rosbag、数据库和实验原始文件另存于设备的数据目录，不随源码上传。
2026-10-30 已有无线恢复反馈，但当前网卡、IP 和跨机 DDS 可视化仍须在现场复核。
