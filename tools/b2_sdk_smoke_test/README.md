# B2 官方 C++ SDK 最小对照测试

直接调用用户指定的 [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
中的 B2 SportClient，不依赖 ROS2、自定义服务或手动拼装 API 请求。

已核对的官方文件：

- example/b2/b2_sport_client.cpp：指定网卡初始化、B2 专用 SportClient、
  BalanceStand、ClassicWalk、Move、StopMove、SwitchGait。
- include/unitree/robot/b2/sport/sport_client.hpp：真实 C++ 函数声明。
- example/b2/CMakeLists.txt：链接 unitree_sdk2。

本机实际编译使用的 SDK 提交：
d1abf001220d3e5f59e00a0c4aaa595e00089a4a，x86_64/GCC 11.4。
用户已在 Jetson 编译并实测基础前进成功；旧 step 路径实机失败。
vision / vision-forward 已在 Jetson/B2 实机尝试，两次均在 VisionWalk(true)
返回0后出现 error_code=2、mode=3、gait_type=1，触发测试程序联锁结束；
第二次在等待人工确认阶段退出，没有调用 Move。两次退出清理请求均返回0，
不将其写成视觉模式、实际停车或爬楼成功。状态值2的官方含义仍未确认。

随后用户已在Jetson编译并运行v2_reference的10秒视觉观察：匹配2/3/1，观察结束后
三条清理请求均返回0并恢复0/9/1，用户反馈“可以了”。这是电脑视觉进入/保持/退出
流程的实机证据，不代表视觉前进、倒退、转向或楼梯测试已完成。

## 持续视觉模式 + 键盘控制（vision_keyboard_v3）

新增 `keyboard 0` 入口；0明确表示持续运行到Q/人工中断，不设10秒自动退出。
复用已实机通过的视觉激活和停止路径，以及限定2/3/1参考、低层bit_flag=0x4、
12个腿部电机故障字全零、双话题0.30秒新鲜度、姿态等保护。键盘路径尚未实机验证。

先按下文同步整个工具目录并重新配置/编译；在交互式SSH终端执行，不使用nohup、
后台运行或无终端的远程命令。程序在连接DDS之前拒绝无交互终端的实际执行。
退出其他电脑运动控制进程，切英文输入法并让终端保持焦点，等READY后再操作：

```bash
bash ~/b2_sdk_smoke_test/build/run_sdk.sh enP8p1s0 keyboard 0 --execute
```

| 按键 | 请求行为 |
| --- | --- |
| E（点按） | 解锁方向输入，同时请求零速度；不会自行前进 |
| W / S | 单轴vx=+0.30 / -0.30 m/s，前后方向须先在视觉模式下平地确认 |
| A / D | 单轴vyaw=+0.20 / -0.20 rad/s，预期左/右转，先平地逐项确认 |
| 空格 | 请求Move(0,0,0)，锁住方向输入，不主动关闭视觉模式；再点E才能移动 |
| Q、Ctrl+C、Ctrl+D | 退出，尝试StopMove -> VisionWalk(false) -> StopMove |

不发送横移或合成前进转向；一批输入只采用最后一个方向。空格/ESC优先于同批
方向和解锁字节，Q优先退出。未知按键归零并锁方向输入；不要使用方向箭头。
终端不会提供松键事件，W/S/A/D靠按键连发续期：最后一个方向字节后0.35秒
请求零速度，视觉模式继续请求保持；相同方向只延长输入期限，不反复排队发送SDK
Move。按住时操作系统首次连发延迟可能造成一次短暂停顿；这是TTY输入限制，
不是保证按住就能无缝连续运动。每次空格后须重新点E解锁，单次轻点方向仅点动。
忽略的回车不延长方向输入期限。终端模式在正常退出和异常退出时恢复。

激活阶段SDK请求超时仍1秒，键盘运动环设为0.2秒，输入轮询约10毫秒。
0.35秒是软件输入期限，实际归零还受SDK阻塞、调度和网络延迟影响，不是硬实时
物理停车保证。SIGHUP、SIGPIPE及Ctrl+Z请求退出清理；SIGKILL、SIGSTOP、崩溃、
断电和无法通信时仍不能保证软件停车。SSH网络失联但未被检测到时，方向输入
过期会请求归零，视觉模式可能继续保持，必须有现场人员和独立可靠人工停车手段。

场地先用开阔平地，人与物远离足端；先分别核对前进、后退、两侧转向、松键归零、
空格归零与最终退出停车，确认零速度请求不会意外退出视觉模式，再考虑台阶。
本机视觉步态下0.30m/s并非已验证的楼梯安全速度，0.20rad/s也未实机验证；
不能直接套用普通模式方向和速度结果。实际楼梯须先确认踏步/梯面适配、净空、
看护与人工停车手段，楼梯段先只做前后单方向，转向在平地或平台单独验证。
Q/故障退出会请求关闭视觉模式，台阶上突然退出模式的效果尚未验证；正常退出
应选稳定位置，紧急情形以现场可靠人工停车为准。

需要留存终端事件时，可在同一交互终端用tee保存输出（stdin仍为TTY）：

```bash
mkdir -p ~/b2_terrain_logs
set -o pipefail
bash ~/b2_sdk_smoke_test/build/run_sdk.sh enP8p1s0 keyboard 0 --execute 2>&1 | tee "$HOME/b2_terrain_logs/b2_keyboard_$(date +%Y%m%d_%H%M%S).log"
```

KEY行含键盘阶段的单调时钟相对秒数、目标、解锁状态和SDK返回码；数值状态变化
继续打印。科研实验还需记录场景、台阶尺寸、方向真实响应、实际停车、重复次数，
不可把启动或请求成功当作上下楼成功。

## 遥控器视觉模式只读对照（当前诊断入口）

用户已明确允许一次遥控器正常视觉模式的只读对照，最终电脑控制目标不变。
新增 `read_visual_reference.py`，仅订阅 `/lf/sportmodestate` 和 `/lf/lowstate`，
没有 SportClient、控制消息发布或服务调用，不负责机器人停车，也不会因为
error_code=1/2/其他数值而发送停车或干预遥控器。它与原有运动测试程序分开。

当前修正版启动标记为 `read_visual_reference_v2`。首版在Jetson回调中遇到
ROS2数组的 NumPy float32 元素，JSON序列化失败；v2将rpy/velocity每个元素
显式转为Python float，采集异常也会记录capture_error并标记未完成。
已用真实NumPy float32的回调仿真及异常路径完成两项离线回归检查，尚待Jetson重试。
旧失败JSONL保留，不能因为末尾存在capture_end就当作有效对照。

v2已获得一次真实只读状态对照：停止时error=0/mode=9/gait=1；遥控器对照期间
出现error=2/mode=3/gait=1约13.45秒，随后恢复停止元组；bit_flag维持0x4、
腿部电机故障字全零。该结果与电脑视觉请求后的元组相同，提示原生程序的
非零状态联锁误杀了本次已确认的视觉参考状态；用户随后明确确认该期间
“是，持续踏步且已停稳”。
不能只凭这一数值对照允许所有非零值，或宣称已验证电脑视觉模式/爬楼。

只在开阔平地、远离楼梯边缘、现场有人看护并备好可靠人工停车手段时进行。
退出旧 stair_remote_control、twist_bridge、原生运动测试等控制进程，
先用人工方式确认机器人停止；本次不推动方向摇杆，不测试前进或上下楼。

将该 Python 文件同步到 Jetson，在已配置 ROS2 Humble、unitree_go 消息、
ROS_DOMAIN_ID=0、CycloneDDS 有线网卡的终端运行，不需要重新编译 C++：

```bash
python3 ~/b2_sdk_smoke_test/read_visual_reference.py
```

默认记录40秒，前5秒保持停止以采集基线。只有收到两个话题的新鲜数据后，
终端才提示可以用遥控器按已验证的方法进入视觉模式；没有数据则取消，
不要切模式。进入后只观察5–10秒，人工退出并确认停稳，不必等记录结束。
最后10秒提醒检查是否已停稳；脚本结束、Ctrl+C或数据中断均不会自动停车。

终端只输出 error/mode/gait、bit_flag、非零腿部电机 reserve[0] 的变化，
每条原始摘要数据带主机本地时间与单调时钟相对秒数，保存在
`~/b2_terrain_logs/b2_remote_visual_reference_日期_时间.jsonl`。
高低层数据来自独立订阅，终端组合行不代表同时采样；JSONL分别记录来源及时间。
反馈时只需给出 STATE 变化行，并说明是否出现与平常一致、可持续的视觉模式踏步
及是否人工停稳。不要把数值相同直接认定为正常模式，或据此放开原生运动联锁；
只读对照用于区分问题，不能替代厂家状态定义或验证电脑前进/爬楼。

## 视觉模式运动调试（vision_walk_v2_reference，当前入口）

v1两次在数值2出现后自动停车的历史记录仍保留。新的v2_reference不再在本次
已确认的视觉上下文中仅凭数值2停车；这不是官方公布的通用错误码定义，
只适用于本机、本次已核对的参考。更换机器人或固件后必须重新核对。

新增 `state_monitor.hpp`，通过真实回调的无DDS测试检查以下边界：

- 初始高层必须为0/9/1，底层bit_flag必须为0x4，前12个电机reserve[0]均为0。
- 只有本次VisionWalk请求期间允许高层精确组合2/3/1；其他非零值（含1及组合位6）、
  错误的mode/gait、非视觉路径中的2或未请求视觉前的2仍中止。
- VisionWalk返回0后最多等待2秒看到参考组合，继续检查0.3秒；随后才观察完整10秒，
  或进入现场MOVE确认。单纯RPC返回0、一直保持0/9/1均不代表激活成功。
- 看到视觉参考后，在请求退出前丢失该组合就中止。高低层话题均需保持0.30秒内
  新鲜；底层bit_flag偏离0x4、电机故障字非零、姿态超限、非法四元数或人工中断
  都继续触发退出清理，不提供任意非零值放行开关。
- 正常退出先请求StopMove -> VisionWalk(false) -> StopMove，再最多1秒检查是否
  回到高层停止参考0/9/1；关闭视觉例外后再监护0.3秒。超时提示人工停车。
  回到相同遥测元组仍不能替代现场物理停稳确认。

用户已实机通过v2_reference的10秒视觉观察；新增keyboard路径仍仅离线检查。
先重新同步整个
工具目录并重新构建，按下方vision命令只观察，不发送Move；现场确认成功后才
在平地单独点动，楼梯上下行仍须后续独立验证。

用户提供的官方遥控器截图明确写明 X（单击）对应视觉模式，并反馈该模式可爬楼。
官方 [B2 C++ 头文件](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/b2/sport/sport_client.hpp)
提供 VisionWalk(bool)；[API 定义](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/b2/sport/sport_api.hpp)
中编号为1101；[官方 ROS2 实现](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/common/ros2_b2_sport_client.cpp#L113)
使用 {"data": true/false}。截图中的按键与此接口是否在本机固件上产生同一状态，
仍须本次实机观察，不能仅凭返回0、mode/gait数值或看到小碎步判定已具备爬楼能力。

沿用下方同步/构建步骤后，先在开阔平地、远离楼梯边缘，现场有人看护并备妥
可靠人工停车手段。停止旧 stair_remote_control、twist_bridge 和其他控制进程，
确保只有一个测试程序发送运动请求。模式切换可能抬腿、踏步或造成位移。
使用隔离 DDS 库的启动器，避免此前混用 ROS DDS 库导致的断言崩溃：

```bash
bash ~/b2_sdk_smoke_test/build/run_sdk.sh enP8p1s0 vision 10 --dry-run
bash ~/b2_sdk_smoke_test/build/run_sdk.sh enP8p1s0 vision 10 --execute
```

vision 顺序：初始高低层参考检查 -> BalanceStand -> 0.5秒状态检查 -> VisionWalk(true) ->
匹配2/3/1参考并检查0.3秒 -> 完整观察10秒（只要检查条件持续满足）->
StopMove -> VisionWalk(false) -> StopMove -> 停止参考检查。
此视觉进入/保持/退出顺序已获得本机10秒实机证据，不是官方发布的完整爬楼例程。
不会调用 ClassicWalk、SwitchGait(2)、ContinuousGait 或 Move。
除上述限定视觉参考外的非零值、状态超时、姿态超限、RPC失败或 Ctrl+C
都提前结束并尝试清理；
10秒不会覆盖这些保护。退出会请求关闭视觉模式，不宣称已恢复锁定站立。

需要试视觉模式前进时，仍先在同样的开阔平地进行：

```bash
bash ~/b2_sdk_smoke_test/build/run_sdk.sh enP8p1s0 vision-forward 0.5 --execute
```

此入口先请求视觉模式，再在本次激活中等待现场确认。终端提示后，只有现场
确认机器人处于期望视觉模式、没有异常且路径净空，才输入大写 MOVE 并回车。
程序随后只调用一次 Move(+0.30,0,0)，持续0.5秒后自动执行上述清理；
其余输入、输入关闭或30秒内未确认均取消，不发送 Move。等待确认期间持续
检查状态，也可 Ctrl+C 停止。该速度沿用此前普通前进实测值，尚未验证视觉
模式下的实际速度、方向和地形表现，不是官方通用安全阈值。
首次不在台阶前直接发送该点动，不据此开放自动上下楼；下楼与倒退上/下楼
都需要独立的场景、方向和停车验证。

本机离线验收只执行不建立DDS连接的测试：

```bash
ctest --test-dir ~/b2_sdk_smoke_test/build --output-on-failure -E isolated_dds_write
```

保留的 step/forward/backward 为历史对照路径，下文旧 step 顺序已实机失败，
不要把它作为本轮视觉模式调试入口。

## 当前限制

已有机器人记录表明：SwitchGait(2) 成功应答后曾出现
SportModeState.error_code=1。更换为原生 SDK 不能证明这个状态无害，
也不能保证修复提前停车。该程序继续保留非零错误码、姿态超限和 0.30 秒
状态超时停车；不提供忽略 1 的开关。

step 是 AI 文档中的持续踏步请求，不是已经确认的楼梯模式。
官方 C++ 示例的 Move 菜单是旋转，不能直接当作楼梯例程；
本工具的 forward/backward 才分别是已知机身 x 方向 +/-0.30 m/s。
方向和效果仍需现场确认。

## 同步及构建

先 Ctrl+C 退出旧 stair_remote_control，并停止其他 Sport 控制程序。
在 PC 项目根目录执行：

```bash
scp -r tools/b2_sdk_smoke_test jetson@10.92.201.107:/home/jetson/
```

Jetson 上若尚无 ~/unitree_sdk2，才克隆；不覆盖已有 SDK：

```bash
test -d ~/unitree_sdk2 || git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
cmake -S ~/b2_sdk_smoke_test -B ~/b2_sdk_smoke_test/build -DUNITREE_SDK2_SOURCE_DIR="$HOME/unitree_sdk2"
cmake --build ~/b2_sdk_smoke_test/build -j2
```

不需要 sudo make install，也不需要修改 ROS2 包或 SDK 源码。
CMake 直接使用官方仓库自带、匹配系统架构的 SDK 静态库及 DDS 动态库。

## 先做离线检查

```bash
~/b2_sdk_smoke_test/build/b2_sdk_smoke_test enP8p1s0 step 10 --dry-run
ctest --test-dir ~/b2_sdk_smoke_test/build --output-on-failure
```

dry-run 在 DDS 初始化前退出，不连接机器人；所有 CTest 用例同样不连接机器人。

## 实机入口与操作边界

入口须明确提供 --execute，启动参数或模式不合法时在 DDS 初始化前退出。
本程序不负责更换机器人运动服务；使用前应已确认当前为 ai_sport。
在开阔平地、离开楼梯边缘、现场有人看护且可靠人工停车手段就绪时使用。
原地踏步也是运动，可能发生机身或足端位移；故障尚未确认时不应反复试动。

```bash
# 原地踏步：只请求一次步态切换，最多 10 秒。
~/b2_sdk_smoke_test/build/b2_sdk_smoke_test enP8p1s0 step 10 --execute
```

step 调用顺序：ClassicWalk(true) -> 3 秒受监护等待 ->
BalanceStand -> 0.5 秒受监护等待 -> SwitchGait(2) ->
定时等待 -> StopMove -> SwitchGait(0)。
该顺序是根据 AI 文档“ClassicWalk 前置”和“可移动状态”要求作出的
待实机验证修正：先选运控，再对所选运控解除锁定。此前先 BalanceStand
后 ClassicWalk 的 step 路径失败；不能把此改动写成已确认的故障修复。
forward/backward 保留实际前进成功的 BalanceStand -> ClassicWalk(true)
-> 等待3秒 -> Move 顺序。原地踏步成功后仍须单独确认楼梯模式，不能直接
让普通前进碰台阶。
调用返回码和 error/mode/gait 的变化直接输出到终端。
计时从 SwitchGait 调用前开始；不在后台重复排队发送运动指令。
接口等待上限 1 秒，检查周期 20 毫秒；SDK 调用阻塞期间人工停车仍须可用。
SIGINT/SIGTERM 仅设置退出标志，由正常控制流程执行停车；
SIGKILL、进程崩溃、断网和断电无法保证软件停车。

下面两个入口仅用于平地基础运动单独验收，不用于当前未确认的楼梯模式：

```bash
~/b2_sdk_smoke_test/build/b2_sdk_smoke_test enP8p1s0 forward 0.5 --execute
~/b2_sdk_smoke_test/build/b2_sdk_smoke_test enP8p1s0 backward 0.5 --execute
```

平移限制 0.20–1.00 秒、固定 |vx|=0.30 m/s；step 限制 0.20–10.00 秒。
任何失败均尝试 StopMove 和 SwitchGait(0)，并以非零进程返回值退出。
打印 code=0 或 Calls completed 不代表已验证实机动作/停车或已通过楼梯测试。
