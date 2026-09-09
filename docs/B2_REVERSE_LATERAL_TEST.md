# B2 后退与横移专项实机测试手册

> 状态：已于 2026-09-09 完成，不属于下一次必测项目。最终结果为后退 3/3、
> 左移 3/3、右移 3/3；`|vy|=0.30 m/s` 有明确横移，`0.08～0.25 m/s`
> 无明确位移。单次 `MOVE + watchdog STOPMOVE` 已通过 A/B 对照，20 Hz 重发不是必要条件。

适用状态：2026-09-08 有线网络、ROS2 DDS、Sport 接口、站立、趴下、
`0.30 m/s` 前进和 `0.20 rad/s` 左转已验收。本文只补测后退、左横移和
右横移，不重复已通过的项目。

## 1. 测试场地与人员

- 选平整、干燥、防滑的硬质地面，机器狗后方至少留 `3 m`，左右各留
  `2 m`；不在坡道、楼梯边、坑洼、玻璃门旁或行人通道首测。
- 机器狗周围不放障碍物。这是运动接口验收，不是避障试验。
- 一人操作电脑，一人手持原厂遥控器或急停装置并盯住机器狗。
- 有线测试时把网线从机器狗后上方引出，另一人跟随理线，不让线缆
  进入腿部摆动区。
- 在机体初始中心、左右和后方用胶带做地面标记，准备卷尺或激光测距仪。

## 2. 打开测试程序

新建一个终端，逐行执行：

```bash
source /opt/ros/humble/setup.bash
```

作用：加载 ROS2 Humble。

```bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
```

作用：加载宇树 ROS2 消息和 CycloneDDS 工作空间。

```bash
source /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/install/setup.bash
```

作用：加载本项目中已编译的 Python 测试程序。

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

作用：使用已经实机验证的 CycloneDDS。

```bash
export B2_NET_IF=enp8s0
```

作用：使用已验证的 B2 有线网口。

```bash
export ROS_DOMAIN_ID=0
```

作用：使 ROS2 域与 2026-09-08 实机基线保持一致。

## 3. H1 后退短脉冲

首次只用 `0.30 s` 脉冲：

```bash
ros2 run b2_twist_bridge_py safe_motion_test --ros-args -p linear_speed:=0.30 -p lateral_speed:=0.08 -p angular_speed:=0.20 -p motion_pulse_sec:=0.30
```

作用：启动默认锁定、自动停车的短脉冲测试。

机器狗已平衡站立后，按键顺序为：

1. 按 `i`：解锁本次测试。
2. 按 `s` 一次：发送 `x=-0.30 m/s`，`0.30 s` 后程序自动发送 `StopMove`。
3. 按 `x`：再次确认停车。
4. 按 `i`：重新锁定。

记录后退方向是否正确、实际位移、脚步是否稳定、自动停车后是否继续滑行。
如果腿部有动作但位移不明显，不提高速度，只把命令中
`motion_pulse_sec:=0.30` 改为 `motion_pulse_sec:=0.60` 后重试一次。

## 4. H2 左、右横移短脉冲

只有控制代码或 B2 固件发生变化、确需回归时，才使用最终已验证参数启动：

```bash
ros2 run b2_twist_bridge_py safe_motion_test --ros-args -p linear_speed:=0.30 -p lateral_speed:=0.30 -p angular_speed:=0.20 -p motion_pulse_sec:=0.60 -p lateral_pulse_sec:=1.00
```

作用：复现 2026-09-09 已通过的单次横移参数；程序不会持续重发 `MOVE`。

先测左移：按 `i` 解锁，按 `a` 一次，等自动停车，按 `x`，再按 `i`
锁定。把机器狗放回地面标记处。

再测右移：按 `i` 解锁，按 `d` 一次，等自动停车，按 `x`，再按 `i`
锁定。

不要再从 `0.08 m/s` 开始重复摸索，也不要重新加入临时 20 Hz MOVE 重发。
今天只验证过单轴横移，不把对角线组合运动标记为通过。

## 5. 通过标准与结果记录

每个方向只有在下列条件全部满足时才记为通过：

- 连续 `3` 次都向预期方向位移，方向定义无反向；
- 没有绊倒、足端明显打滑、连续抬腿但机身不动等现象；
- 短脉冲到时后能收到 `api_id=1003` 的 `StopMove` 响应；
- 停车后没有明显持续滑行，且用卷尺记下每次位移。

现场按下表记录：

| 方向 | 速度 | 脉冲 | 3 次位移 | 方向正确 | 步态稳定 | StopMove 正常 | 结论 |
|---|---:|---:|---|---|---|---|---|
| 后退 |  |  |  |  |  |  |  |
| 左移 |  |  |  |  |  |  |  |
| 右移 |  |  |  |  |  |  |  |

如出现方向反了、未停车、摇晃、打滑或网线卷入腿部，立即按遥控急停，
不继续提速。

## 6. 验收后在主控制链中的处理

项目速度桥已经固化为：

```text
allow_reverse=true
allow_lateral=true
max_linear_y=0.30
min_effective_linear_y=0.30
```

这只表示速度桥允许已验证的单轴方向。速度桥上电仍保持锁定，并要求任务心跳、
机器人状态和自主安全状态。Nav2 横移采样、组合运动和动态避障尚未实机验收，
不能因为单轴测试通过就标记为已完成。
