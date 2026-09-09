# `src` 源代码说明

## 当前 Python 主流程

```text
src/
├── b2_twist_bridge_py/
│   └── b2_twist_bridge_py/
│       ├── safe_motion_test.py   # 首次站立、趴下、短运动测试
│       ├── twist_bridge.py       # 带安全锁和超时停车的速度桥
│       ├── velocity_policy.py    # 实测前后/横移起步速度与安全限幅
│       └── sport_model.py        # 宇树 Sport API 编号
├── b2_driver_py/
│   └── b2_driver_py/driver.py    # 状态、里程计、TF 和关节状态
├── b2_slam_3d/                   # RTAB-Map 3D 建图、定位和输入健康检查
├── b2_navigation/                # Nav2 规划、局部代价地图和避障
└── b2_factory_inspection/
    └── b2_factory_inspection/
        ├── mission_manager.py        # 巡检任务状态机
        ├── photo_quality_node.py     # 目标区域图像质量评价
        └── aruco_target_locator.py   # 户外 A/B 目标定位
```

## 主要启动入口

```bash
ros2 launch b2_driver_py driver.launch.py use_rviz:=false
ros2 launch b2_factory_inspection mapping_3d.launch.py --show-args
ros2 launch b2_factory_inspection autonomous_inspection.launch.py --show-args
```

基础 Sport 控制以及后退、左右横移已在 2026-09-08/09 完成实机验证。当前入口从
静止数据链复核和运动 3D 建图开始，具体命令见
`docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md`。

`b2_driver/`、`b2_twist_bridge/`、`b2_teleop_ctrl_keyboard/` 和原始固定路线节点属于旧版或对照代码，不用于新的实机主流程。

文献翻译、阅读顺序和论文方法对照见
[本周文献阅读清单](文献阅读/本周文献阅读清单.md)。
