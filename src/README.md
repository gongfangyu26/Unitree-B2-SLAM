# `src` 源代码说明

审阅入口：[2026-11-12 代码交接说明](../docs/B2_CODE_REVIEW_2026-11-12.md)。
现场入口：[2026-11-13 调试清单](../docs/B2_TEST_PLAN_2026-11-13.md)。

## 当前 Python 主流程

```text
src/
├── b2_twist_bridge_py/
│   └── b2_twist_bridge_py/
│       ├── safe_motion_test.py   # 首次站立、趴下、短运动测试
│       ├── twist_bridge.py       # 带安全锁和超时停车的速度桥
│       ├── velocity_policy.py    # 实测前后/横移起步速度与安全限幅
│       ├── heading_control.py    # 可选航向反馈；默认不开启
│       ├── terrain_motion_test.py / stair_remote_control.py # 独立地形实验入口
│       └── sport_model.py        # 宇树 Sport API 编号
├── b2_driver_py/
│   └── b2_driver_py/driver.py    # 状态、里程计、TF 和关节状态
├── b2_slam_3d/                   # RTAB-Map、输入/栅格检查、PCD 保存
├── b2_navigation/                # AMCL/RTAB-Map 定位、Nav2、导航安全门、录点
└── b2_factory_inspection/
    └── b2_factory_inspection/
        ├── mission_manager.py        # 巡检任务状态机
        ├── corridor_patrol.py        # 有限圈数巡逻，默认待命
        ├── corridor_patrol_core.py   # 路线、规划结果与期限检查
        ├── front_lidar_distance.py   # 只读前向距离估计
        ├── photo_quality_node.py     # 目标区域图像质量评价
        └── aruco_target_locator.py   # 户外 A/B 目标定位
```

## 主要启动入口

```bash
ros2 launch b2_driver_py driver.launch.py use_rviz:=false
ros2 launch b2_factory_inspection mapping_3d.launch.py --show-args
ros2 launch b2_factory_inspection autonomous_inspection.launch.py --show-args
ros2 launch b2_factory_inspection corridor_patrol.launch.py --show-args
```

`--show-args` 只查看参数。实际启动要先核对唯一驱动、地图、定位源和控制锁。
默认速度链为 `Nav2 -> /cmd_vel_nav -> navigation_gate -> /cmd_vel ->
b2_safe_twist_bridge -> /api/sport/request`。基础测试、地形程序和仓库外层
`tools/b2_sdk_smoke_test` 使用独立控制入口，不经过整条导航安全链，不得同时运行。

2026-10-30 已有走廊遥控建图和保存反馈，但录点受阻且 RViz 卡顿，整圈自主巡航
未执行。下一步从二维导航图、跨机显示和静止定位复核开始，详见新的现场清单。

`b2_driver/`、`b2_twist_bridge/`、`b2_teleop_ctrl_keyboard/` 和原始固定路线节点属于旧版或对照代码，不用于新的实机主流程。

文献翻译、阅读顺序和论文方法对照见
[本周文献阅读清单](文献阅读/本周文献阅读清单.md)。
