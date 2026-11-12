# B2 Python 基础控制包

2026-10-28 增加独立的 [官方 C++ SDK 对照入口](../../tools/b2_sdk_smoke_test/README.md)，
直接链接并调用 unitree_sdk2 的 B2 SportClient。用户已在 Jetson 实测普通前进成功，
持续踏步路径失败。新增 `vision` / `vision-forward` 调用官方 `VisionWalk(true)`，
已在 Jetson 实机尝试，两次均在返回0后出现 error_code=2 并由程序联锁中止，
视觉前进入口尚未发送 Move；未验证进入正确视觉状态或爬楼成功。
随后遥控器只读对照同样出现2/3/1，用户确认持续踏步且停稳。
原生v2_reference的10秒电脑视觉观察已实机通过；1及其他未验证值仍中止。
新增vision_keyboard_v3的`keyboard 0`入口，用E解锁、W/S前后、A/D转向，空格请求
归零并锁住方向输入，Q退出；默认方向输入期限0.35秒，视觉模式持续保持。
新键盘路径及视觉模式下前后/转向/楼梯仍待实机验证，详见上述C++工具说明。
视觉模式使用上述独立 C++ 入口，旧 ROS2 `stair_remote_control` 仍是持续踏步测试。

本目录就是基础控制测试程序所在位置，所有实际运行代码均位于：

```text
src/b2_twist_bridge_py/b2_twist_bridge_py/
├── safe_motion_test.py   # 已完成的站立、趴下、短距离移动复验程序
├── terrain_motion_test.py # 坡道/楼梯双确认解锁、点动和 CSV 记录
├── terrain_safety.py      # 姿态解算、步态核对和安全限值
├── stair_remote_control.py # 官方 AI 持续踏步平地短测，楼梯移动保持锁定
├── stair_control.py       # 楼梯模式协议、参数硬限位和映射
├── heading_control.py    # 四元数航向、角度环绕和航向比例反馈纯算法
├── twist_bridge.py       # Nav2 速度到 B2 Sport 请求的安全转换
├── velocity_policy.py    # 实测最低起步速度和方向许可
└── sport_model.py        # B2 Sport API 编号
```

编译后使用 ROS2 可执行程序名运行，不直接使用 `python3 文件名`：

```bash
ros2 run b2_twist_bridge_py safe_motion_test
ros2 run b2_twist_bridge_py terrain_motion_test
ros2 run b2_twist_bridge_py stair_remote_control
ros2 run b2_twist_bridge_py twist_bridge
```

## AI 持续踏步平地短测（2026-10-28 官方文档纠正）

用户提供的 [B2 AI 运控服务接口](https://support.unitree.com/home/zh/B2_developer/AI%20Motion%20Interface)
明确给出 SwitchGait 请求枚举：0 锁定站立、1 盲走、2 持续踏步、
3 视觉辅助、4 平面行走。调用前须处于可移动状态并先调用 ClassicWalk。
旧版本误把通用状态表中的 `gait_type=3` 当成 AI 接口楼梯请求编号，
且遗漏 ClassicWalk 前置调用；此前直接发送 SwitchGait(3) 返回 -1
不能据此证明基础版 B2 不支持电脑切换。

当前 `stair_remote_control` 仅开放原地踏步短测，按官方 SDK 格式顺序调用：

1. CheckMode 只读确认当前 `form=0,name=ai`；不自动尝试其他运动服务。
2. BalanceStand：API 1002，参数 `{}`，解除锁定进入可移动状态。
3. ClassicWalk(true)：API 1049，参数 `{"data":true}`。
4. SwitchGait(2)：API 1011，参数 `{"data":2}`。
5. 按用户后续要求，默认从第 4 步发布时起计时 10.00 秒，随后请求 StopMove 和
   SwitchGait(0)，恢复锁定站立；watchdog 周期最高会带来约 0.10 秒调度延迟。

每一步均匹配响应通道、非零 identity.id、api_id，并记录返回码；单步响应
等待不超过 1 秒，失败/超时/停车取消后不执行后续激活动作。代码在清理时先
立即发布停车与锁定站立请求，再通过带 ID 的请求检查返回码。没有收到响应
不等于机器人没有执行，所以清理过程也适用于前置调用失败的情况。
节点退出、状态超时、姿态超限或错误码非零都会请求停车。初始化阶段另有
12 秒总时限；实际停车效果仍需现场确认。

根据现场 CSV，旧版三条激活请求在约 4.8 毫秒内收到成功响应，但约 28 毫秒
后出现 error_code=1 并停车。当前版本 ai_step_classic3s_v4 在 BalanceStand
应答后等待至少 1 秒；按用户提供的 AI 运控参考例程，在 ClassicWalk(true)
应答后等待至少 3 秒。两者均要求新鲜无错误状态，分别最多等待 2 秒、4 秒。
启动日志、status 和服务返回均包含 control_revision 以确认运行版本。
该等待只确认新状态正常，不能等同于动作已经完成；
“前置动作过快衔接导致失败”仍为待实机验证的假设。

本版本不创建 WirelessController 发布端，移除了尚未验证的遥控器重放。
`/b2/stairs/set_armed` 的 true 请求会拒绝；`up/down` 保持锁定。
不会因看到 `gait_type=2` 就宣称楼梯模式生效，也不将请求枚举与输出
SportModeState 的 gait_type 直接等同。持续踏步成功仍不代表上下楼梯验证成功。

### 同步和启动

在 PC 执行（Jetson IP 若变化，应替换）：

```bash
scp \
  "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_twist_bridge_py/b2_twist_bridge_py/stair_control.py" \
  "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_twist_bridge_py/b2_twist_bridge_py/stair_remote_control.py" \
  jetson@10.92.201.107:/home/jetson/Unitree-B2-SLAM/src/b2_twist_bridge_py/b2_twist_bridge_py/
```

先在 Jetson 的旧节点终端 Ctrl+C 退出，再同步代码和构建，避免在节点仍运行
时替换 symlink 指向的源码。保留已有 CycloneDDS 绑定 enP8p1s0、Domain 0
环境。停止其他可能发送 Sport 控制的程序后执行：

```bash
cd ~/Unitree-B2-SLAM
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select b2_twist_bridge_py --symlink-install \
  --allow-overriding b2_twist_bridge_py
source install/setup.bash
ros2 run b2_twist_bridge_py stair_remote_control --ros-args \
  -p step_probe_duration_sec:=10.0 \
  -p test_label:=ai_step_official_20261028
```

启动本身不会执行测试。只有在开阔平地、远离楼梯/边缘、现场有人看护且可靠
人工停车手段就绪时，在另一个已经加载相同 ROS2/DDS 环境的 SSH 终端调用：

```bash
ros2 service call /b2/stairs/probe_ai_step std_srvs/srv/Trigger "{}"
```

`activate_mode` 保留为上述短测的兼容别名，不再表示“进入并保持楼梯模式”。
全程不发送 Move。参数 `step_probe_duration_sec` 硬限制为 0.20–10.00 秒，
包含原地踏步/重心变化风险。观察是否出现踏步、是否自动停车，保存服务完整
响应和 CSV。若 StopMove 或 SwitchGait(0) 返回失败，服务返回 false 并提示
人工确认。软件停车服务和状态查询为：

```bash
ros2 service call /b2/stairs/stop std_srvs/srv/Trigger "{}"
ros2 service call /b2/stairs/status std_srvs/srv/Trigger "{}"
```

**仅代码检查，尚未实机验证**：本次修正版官方调用链、持续踏步和退出锁定。
下一阶段在获得实机返回码、现场动作及停车证据后，再确认与遥控器楼梯模式的
关系；不得直接在楼梯上执行该短测。

后续现场已确认上述五个 API 均返回 0，但节点报出
`SportModeState 错误码=1` 并提前停车。该错误码含义尚未确认，
不能将接口应答成功写成持续踏步/实际停车已验证。
延长到 10 秒不会绕过非零错误码；故障未确认解除前不再触发测试。
服务响应增加 stop_reason、last_fault、step_probe_duration_sec，
CSV 在探测期间也记录 stop_and_disarm 原因。

## 坡道和楼梯有监督测试

`terrain_motion_test` 与已验收的平地测试分离，启动后默认锁定。
它只允许前后单轴、单次最多 1 秒的 `MOVE`，到时自动
`STOPMOVE`并自动锁定；状态超时、非零错误码、横滚/俯仰超限或步态不符都会停车
并锁定。每一次点动前都需在 3 秒内连按两次 `i`。

旧通用状态表与 B2 AI 运控请求枚举不同，不得再把数字 3 作为这里的
AI 楼梯切换依据。`terrain_motion_test` 保持“只观察、不自动切换”的原设计；
坡道测试只能使用已经实机确认的状态判据，不能借此绕过楼梯模式验证。
若用于已验证场景，可锁定实测步态：

```bash
ros2 run b2_twist_bridge_py terrain_motion_test --ros-args \
  -p expected_gait_type:=<已实机确认的数值> \
  -p test_label:=slope_or_stairs_20261024
```

运行时姿态、位置、速度、足端力、`mode/gait_type` 和操作事件保存到
`~/b2_terrain_logs/*.csv`。详细顺序见
`docs/B2_TERRAIN_3D_DEBUG_GUIDE_2026-10-24.md`。

安全速度桥默认处于锁定状态。只有机器人状态正常且持续收到任务心跳时才允许启用：

实机基线为前后 `0.30 m/s`、横移 `0.30 m/s`、左右转 `0.35 rad/s`。
2026-09-09 已确认后退 3/3、左移 3/3、右移 3/3；横移使用单次
`MOVE`，不依赖临时测试过的 20 Hz 重发。速度桥仍然上电锁定，只有任务心跳、
机器人状态和自主安全状态同时有效时才能通过服务解锁。
自主导航中的非零小角速度会被提升到 `0.35 rad/s`，解决 B2 对
`0.20–0.30 rad/s` 只有重心/关节动作而不形成明确转向的死区问题；
最大角速度同样限定为 `0.35 rad/s`。
前后与横移同时非零的组合平移尚未实机验证，因此默认仍会被速度策略阻止。

## 航向保持测试状态

`safe_motion_test.py` 已加入基于 `/lf/sportmodestate` 的可选航向保持，但截至
2026-09-23 仅完成代码和单元测试，**待 Jetson+B2 实机验证**。默认参数
`yaw_hold_enabled=false`，因此不改变已经实机验证的“每次按键只发送一次 MOVE，
watchdog 到时 STOPMOVE”基线。

显式开启后，程序只在前进、后退或横移的短脉冲期间以 20 Hz 更新带航向修正的
MOVE；纯转向键仍使用原来的单次 MOVE。姿态丢失或超过时限会提前 STOPMOVE。
初始参数 `Kp=0.60`、死区 2 度、修正上限 `0.10 rad/s` 只是待验收起点，不能写成
已经整定完成。

只打印四元数航向和消息自带 RPY，不开启闭环：

```bash
ros2 run b2_twist_bridge_py safe_motion_test --ros-args \
  -p yaw_hold_enabled:=false \
  -p yaw_status_log_sec:=1.0
```

航向符号、左右修正和自动停车均确认后，才允许使用：

```bash
ros2 run b2_twist_bridge_py safe_motion_test --ros-args \
  -p yaw_hold_enabled:=true \
  -p yaw_status_log_sec:=1.0
```

不要把航向闭环所需的周期 MOVE 与 2026-09-09 横移 A/B 实验混淆：横移本身不依赖
20 Hz 重发；只有显式开启反馈后，控制器才需要周期更新 `z` 修正量。

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: true}"
```

正式自主巡检由任务管理器发布心跳。人工 Nav2 调试心跳的使用方法见完整调试手册，禁止在无人监督时伪造持续心跳。

关闭控制并停车：

```bash
ros2 service call /b2/control/set_enabled std_srvs/srv/SetBool "{data: false}"
```
