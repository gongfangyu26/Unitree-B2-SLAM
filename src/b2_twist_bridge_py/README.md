# B2 Python 基础控制包

本目录就是基础控制测试程序所在位置，所有实际运行代码均位于：

```text
src/b2_twist_bridge_py/b2_twist_bridge_py/
├── safe_motion_test.py   # 已完成的站立、趴下、短距离移动复验程序
├── twist_bridge.py       # Nav2 速度到 B2 Sport 请求的安全转换
├── velocity_policy.py    # 实测最低起步速度和方向许可
└── sport_model.py        # B2 Sport API 编号
```

编译后使用 ROS2 可执行程序名运行，不直接使用 `python3 文件名`：

```bash
ros2 run b2_twist_bridge_py safe_motion_test
ros2 run b2_twist_bridge_py twist_bridge
```

安全速度桥默认处于锁定状态。只有机器人状态正常且持续收到任务心跳时才允许启用：

实机基线为前后 `0.30 m/s`、横移 `0.30 m/s`、左转 `0.20 rad/s`。
2026-09-09 已确认后退 3/3、左移 3/3、右移 3/3；横移使用单次
`MOVE`，不依赖临时测试过的 20 Hz 重发。速度桥仍然上电锁定，只有任务心跳、
机器人状态和自主安全状态同时有效时才能通过服务解锁。
前后与横移同时非零的组合平移尚未实机验证，因此默认仍会被速度策略阻止。

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: true}"
```

正式自主巡检由任务管理器发布心跳。人工 Nav2 调试心跳的使用方法见完整调试手册，禁止在无人监督时伪造持续心跳。

关闭控制并停车：

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: false}"
```
