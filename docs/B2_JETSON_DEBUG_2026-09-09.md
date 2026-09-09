# B2 与 Jetson 下午实机调试清单

> 本文是当天调试前清单，任务已经结束。实际结果以
> `docs/2026-09-09_B2_Jetson_实机调试记录.md` 为准；下一次操作使用
> `docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md`。

日期：2026-09-09

本手册接续 `B2_DEBUG_LOG_2026-09-08.md`。昨天已经通过的 PC-B2 有线网络、
ROS2 发现、站立、平衡站立、前进、左转和自动停车不再重复。今天只补尚未
验收的运动方向、Jetson 板卡、SLAM 输入和板载部署条件。

## 1. 今天的目标和停止点

按优先级完成：

1. Jetson 使用原装电源在桌面完成板卡、NVMe、Wi-Fi、有线网口和温度验收；
2. B2 补测后退、左横移和右横移；
3. 只读检查三维点云、里程计、时间戳和 TF；
4. 静止启动 RTAB-Map 输入检查；
5. 前四项都正常并且时间允许时，遥控走一个小闭环并录制 rosbag；
6. 记录 B2 实际扩展接口和电源接口照片，为板载供电设计取证。

今天不做自主 Nav2 放权、不做动态避障、不做坡道、不做自动巡检任务，也不把
Jetson 接到未经确认的 B2 电源端子。这样能避免多个新变量同时出现。

## 2. 照片已经确认的 Jetson 硬件

载板背面照片可确认：

- `J10 M.2 Key-E` 已安装 `RTL8822CE` 双频 Wi-Fi/蓝牙模块；
- `Main` 和 `Aux` 两根天线馈线都已连接；
- 已安装 `HP SSD FX700 256GB` NVMe 固态硬盘；
- 右下方空插槽是 M.2 Key-M 存储插槽，不是无线网卡插槽。

结论：今天不需要购买 AX210、USB Wi-Fi 或 4G 模块，也不要拆动现有网卡和天线。
先验收现有 RTL8822CE。如果装进机器狗后的覆盖实测不足，再考虑把天线移到金属
外壳外部；不要先换网卡。

## 3. 出发前携带物品

- Jetson、原装 DC 电源适配器和原装电源线；
- DisplayPort 显示器、键盘和鼠标，或者已经能够登录 Jetson 的电脑；
- 5 GHz 路由器或手机热点，优先使用独立路由器；
- 至少两根确认正常的千兆网线；
- 可选的小型非网管千兆交换机和第三根网线；
- B2 原厂遥控器或急停装置；
- 卷尺、地面胶带、手机录像和记录表；
- U 盘或移动硬盘，用于带回 rosbag 和报告。

不要携带未经确认的裸线去试接 B2 电源。今天只拍清楚机器人接口、丝印、线束
标签和原装适配器铭牌。

## 4. J0：Jetson 桌面供电与外观检查

1. Jetson 断电状态下检查网卡两根小馈线没有松脱，NVMe 固定螺丝存在；
2. 先连接显示器、键盘、鼠标和需要的网线；
3. 最后连接套件原装 `19 V` DC 圆孔适配器；
4. 看电源灯、风扇、系统启动和桌面是否正常；
5. 不用 USB-C 给这块参考载板供电，USB-C 主要用于数据/恢复模式；
6. 不热插拔 M.2 网卡、NVMe、CSI 相机或 40 针扩展设备。

NVIDIA P3768 参考载板的 DC 圆孔输入为 `9～20 V`，原装电源为 `19 V`，接口
中心为正极。当前开发阶段只用原装适配器，不能把 B2 的 `58 V` 电池或 `24 V`
附件电源直接插到 Jetson。

官方依据：

- <https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>
- <https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf>

## 5. J1：运行 Python 板卡验收程序

程序位于：

```text
src/b2_factory_inspection/b2_factory_inspection/jetson_board_test.py
```

先进入这份工程的根目录，再执行：

```bash
python3 src/b2_factory_inspection/b2_factory_inspection/jetson_board_test.py \
  --output jetson_board_report.txt
```

作用：只读检查设备树型号、ARM 架构、L4T、内核、NVMe、磁盘空间、无线驱动、
有线网口、温度、功耗模式和短时资源占用，并在当前目录保存中文报告。程序不会
改 IP、连接 Wi-Fi、切换功耗模式或控制机器人。

如果已经编译本项目，也可以执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run b2_factory_inspection jetson_board_test \
  --output jetson_board_report.txt
```

合格结果至少应包含：

- 型号包含 `Jetson Orin Nano`；
- 架构为 `aarch64`；
- 能读取 `/etc/nv_tegra_release`；
- 能枚举 `nvme` 设备，根分区最好剩余至少 `30 GiB`；
- 至少一个无线网口并绑定 Realtek/rtw88 对应驱动；
- 至少一个物理有线网口；
- 空载检查期间没有持续高温或反复降频、重启。

## 6. J2：人工复核 Wi-Fi 和网口

逐行执行并保存输出：

```bash
cat /proc/device-tree/model
```

作用：确认系统识别出的 Jetson 型号。

```bash
head -n 1 /etc/nv_tegra_release
```

作用：记录 Jetson Linux/L4T 版本。

```bash
uname -r
```

作用：记录内核版本。

```bash
lspci -nnk
```

作用：确认 `RTL8822CE` 被 PCIe 枚举，并查看 `Kernel driver in use`。常见驱动名
可能是 `rtw88_8822ce`、`rtl8822ce` 或当前 Jetson BSP 提供的对应名称。

```bash
ip -br link
```

作用：记录无线和有线网口的真实名称。后续不能照抄 PC 的 `enp8s0`。

```bash
nmcli device status
```

作用：查看 NetworkManager 是否管理无线网口。

```bash
rfkill list
```

作用：确认 Wi-Fi 的 `Soft blocked` 和 `Hard blocked` 都是 `no`。

```bash
nmcli device wifi list
```

作用：确认能扫描到 2.4 GHz 和 5 GHz 网络。

优先通过 Ubuntu 图形界面连接 5 GHz 网络。需要命令行时执行：

```bash
nmcli --ask device wifi connect "你的5GHz无线网络名称"
```

作用：密码会在本机交互输入，不会直接写进这条命令。不要把 Wi-Fi 密码发到
调试记录或聊天中。

连接后执行：

```bash
nmcli connection show --active
ip -br address
ip route
```

作用：保存无线连接、IP 地址和路由。电脑和 Jetson 应接入同一个 5 GHz 局域网。

先做 100 包延迟测试。把地址替换为监控电脑的无线 IP：

```bash
ping -c 100 监控电脑无线IP
```

验收建议：桌面近距离应无丢包；若丢包超过 `1%` 或频繁出现百毫秒级突增，先
检查天线、5 GHz 信道和摆放位置，不进入 ROS2 无线联调。

如果两端已有 `iperf3`，电脑执行：

```bash
iperf3 -s
```

Jetson 执行：

```bash
iperf3 -c 监控电脑无线IP -t 30 -P 4
iperf3 -c 监控电脑无线IP -t 30 -P 4 -R
```

作用：分别测试 Jetson 到电脑和电脑到 Jetson 的持续吞吐。当前架构在 Jetson
处理原始点云和图像，电脑只收状态、地图和按需画面；稳定性比峰值速率更重要。

## 7. J3：确认 Jetson 是否已经具备 ROS2 运行环境

Jetson 是 `aarch64`，不能把 PC 上已经编译好的 `build/`、`install/` 和 `log/`
目录直接复制过去使用。只能复制源码，然后在 Jetson 上重新编译。先执行：

```bash
ros2 --help
```

作用：确认 ROS2 命令是否存在。若提示找不到命令，今天不在 Jetson 上控制 B2，
只完成板卡验收和 PC 侧 SLAM 取数。

```bash
printenv ROS_DISTRO
```

作用：加载环境后应输出 `humble`。

```bash
ros2 pkg prefix rmw_cyclonedds_cpp
```

作用：确认 Jetson 已安装 CycloneDDS 的 ROS2 实现。

```bash
ros2 pkg prefix unitree_api
ros2 pkg prefix unitree_go
```

作用：确认宇树消息包已经在 Jetson 上按 ARM64 架构完成编译。如果任一包找不到，
不要把 B2 网口切给 Jetson后直接尝试控制；先带回系统信息，完成 Jetson 软件环境
安装和源码编译后再迁移。

```bash
ros2 pkg prefix b2_factory_inspection
ros2 pkg prefix b2_twist_bridge_py
```

作用：确认本项目已经在 Jetson 上编译。若项目还没有复制，板卡测试仍可直接运行
第 5 节的 Python 文件，不要求 ROS2。

## 8. H1/H2：今天唯一需要补做的 Sport 运动测试

先保持昨天的拓扑：PC 有线直连 B2。按
`docs/B2_REVERSE_LATERAL_TEST.md` 测后退、左横移和右横移，不重复站立、前进
和左转。

首次命令：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export B2_NET_IF=enp8s0
export ROS_DOMAIN_ID=0
ros2 run b2_twist_bridge_py safe_motion_test --ros-args \
  -p linear_speed:=0.30 \
  -p lateral_speed:=0.08 \
  -p angular_speed:=0.20 \
  -p motion_pulse_sec:=0.30
```

后退按键顺序：`i` 解锁、`s` 一次、等待自动停车、`x` 停车、`i` 重新锁定。

左移按键顺序：`i`、`a`、等待、`x`、`i`。

右移按键顺序：`i`、`d`、等待、`x`、`i`。

每个方向连续三次，记录速度、脉冲、位移、方向、步态和 `StopMove`。现场必须
平整、干燥，后方留 `3 m`，左右各留 `2 m`，一人控制、一人拿遥控急停并理线。

## 9. S1：只读检查三维 SLAM 输入

后退/横移结束后关闭测试程序，速度桥保持锁定。不要启动巡检任务。执行：

```bash
ros2 topic type /rslidar_points
ros2 topic hz /rslidar_points
ros2 topic echo /rslidar_points --once --field header
ros2 topic info /rslidar_points -v
```

作用：记录默认 RoboSense 点云的消息类型、频率、`frame_id`、时间戳和发布节点。
类型必须是 `sensor_msgs/msg/PointCloud2`。

```bash
ros2 topic type /livox/lidar
ros2 topic echo /livox/lidar --once --field header
```

作用：确认 Livox 话题是否为标准 PointCloud2。若是自定义类型，今天不强行接入
RTAB-Map。

```bash
ros2 topic type /lio_sam_ros2/mapping/odometry
ros2 topic hz /lio_sam_ros2/mapping/odometry
ros2 topic echo /lio_sam_ros2/mapping/odometry --once --field header
ros2 topic echo /lio_sam_ros2/mapping/odometry --once --field child_frame_id
```

作用：记录板载 LIO-SAM 里程计类型和父子坐标系，今天只观察，不替换 `/odom`。

## 10. S2：静止启动 RTAB-Map 输入检查

这一步不需要搭专用场景，也不需要机器人行走。实验室、车间或普通户外环境都
可以，但不要选全空旷草地、大面积玻璃旁或没有几何结构的长直走廊。雷达附近
应有墙角、箱体、柱子等静态结构。

把下面的 `rslidar` 替换成上一节实际记录的 `frame_id`：

```bash
mkdir -p /home/gongfangyu/b2_maps
ros2 launch b2_factory_inspection mapping_3d.launch.py \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  delete_db:=true \
  database_path:=/home/gongfangyu/b2_maps/b2_3d_working.db \
  use_rtabmap_viz:=true
```

另开终端执行：

```bash
ros2 topic echo /b2_slam/preflight_report
ros2 topic echo /b2_slam/preflight_ok
ros2 topic hz /b2_slam_3d/cloud_sync
ros2 topic hz /b2_slam_3d/odom_sync
```

再把 `rslidar` 替换为实际 `frame_id`：

```bash
ros2 run tf2_ros tf2_echo base_link rslidar
```

通过标准：`/b2_slam/preflight_ok` 稳定为 `true`，点云和里程计持续更新，TF
存在，终端没有持续的 extrapolation、空点云或时间戳错误。

## 11. S3/S4：时间允许才做小闭环和 rosbag

只有 S2 通过后，才用原厂遥控器低速走约 `5 m × 5 m` 的闭合路线。不发布
`/inspection/start`。同时录制：

```bash
ros2 bag record -o b2_slam_small_loop \
  /rslidar_points \
  /lf/sportmodestate \
  /lf/lowstate \
  /dog_imu_raw \
  /lio_sam_ros2/mapping/odometry \
  /lio_sam_ros2/mapping/cloud_registered \
  /tf \
  /tf_static
```

结束后按 `Ctrl-C`，再执行：

```bash
ros2 bag info b2_slam_small_loop
```

保存 rosbag、RTAB-Map 数据库、终端日志和地图截图。若 S2 未通过，今天不走小
闭环，先把前置检查报告带回来。

## 12. 机器人只有一个外露网口时的连接方式

### 12.1 今天最稳妥的分阶段方式

阶段 A：PC 直接接 B2，完成后退/横移和传感器取数。

阶段 B：结束 PC 控制后，把 B2 网线改接 Jetson；Jetson 仍使用原装 19 V
适配器供电，电脑通过 5 GHz Wi-Fi SSH 登录 Jetson。只有第 7 节的 ROS2、
CycloneDDS、Unitree 消息包和本项目都已存在时才做这一步；否则今天停在 PC
取数。此时电脑只操作 Jetson 终端，不直接占用 B2 网口。

最终拓扑：

```text
B2 唯一外露 RJ45 ─── Jetson 有线网口
                         │
                    Jetson Wi-Fi
                         │
                    5 GHz 路由器
                         │
                    监控电脑 Wi-Fi
```

这正好符合“Jetson 自主判断和控制、电脑只监控”。B2 与 Jetson 的控制链不依赖
Wi-Fi，电脑掉线时 Jetson 仍能安全停车或继续受控任务。

昨天 PC 使用 `192.168.123.99/24`。如果 PC 已完全拔掉 B2 有线网，Jetson 可以
在首次直连测试中临时使用同一地址，但必须先用 `ip -br link` 找到 Jetson 的
真实有线网口名称，不能照抄 `enp8s0`。在看到实际网口和 NetworkManager 连接名
之前，本手册不让初学者盲改永久网络配置。

### 12.2 需要 PC 和 Jetson 同时接入 B2 时

调试阶段可增加一个非网管千兆交换机：

```text
             ┌── PC 有线网口
B2 RJ45 ── 千兆交换机
             └── Jetson 有线网口
```

PC 和 Jetson 必须使用同网段且互不重复的静态 IP。PC 继续用 `.99` 时，Jetson
不能也用 `.99`。未核对 B2 保留地址和现场设备前，不在文档里猜一个永久地址。

不要把普通交换机当 PoE 供电器；普通 B2 网口和 Jetson RJ45 只承担数据。

### 12.3 ROS2 双网卡注意事项

现有 PC 调试配置把 CycloneDDS 只绑定到 `enp8s0`。这个配置不能原样用于最终
Jetson，因为 Jetson 需要同时使用：

- 有线网口：发现 B2 话题并发送 Sport 请求；
- Wi-Fi 网口：向电脑发布监控状态。

今天先记录 Jetson 两个真实网口名。取得输出后再生成双网卡 CycloneDDS 配置并
做断网安全测试；不要为了“让电脑看见”而把 B2 控制网与工厂 Wi-Fi 直接做 Linux
网桥或开放 IP 转发。

## 13. Jetson 如何从 B2 取电

宇树 B2 官方参数页列出的扩展能力包含千兆以太网、USB3.0，以及 `12 V`、`5 V`、
`24 V` 和电池电源接口，但官方同时说明不同配置会有差异。用户这台机器人目前
只看到一个外露网口，因此不能仅凭宣传参数认定某个电源口已经引出。

官方参数：<https://www.unitree.com/b2/>

### 13.1 今天只做取证，不接线

拍清楚并记录：

1. B2 顶部、侧面和舱内所有扩展接口及丝印；
2. 随机器人交付的扩展线束、连接器和标签；
3. B2 型号、配置单和用户开发计算机配置；
4. 向宇树或供应商确认 `24 V` 口的针脚、极性、连续/峰值电流、保险和上电时序；
5. Jetson 原装适配器铭牌的输出电压、电流和插头尺寸。

不要用万用表探测未知多针端子，也不要从 58 V 电池线束剥线取电。

### 13.2 计划中的安全电源链

若供应商确认这台 B2 有满足功率的 24 V 附件输出，推荐结构为：

```text
B2 受保护的 24 V 附件输出
        │
    独立保险丝
        │
车规/工业级 24 V → 19 V 降压模块
        │
锁紧、中心正极的 Jetson DC 插头
        │
Jetson P3768 DC 圆孔输入
```

DC-DC 模块应有宽输入、过压/欠压/过流/过温和反接保护，19 V 输出功率留足余量；
按原装适配器铭牌和所有 USB 外设的实际功耗核算，工程初选可按不低于 `80 W`
等级留余量，但最终额定值必须由 B2 端口允许电流和 Jetson 实测功耗共同确定。

禁止方案：

- `58 V BAT` 直接接 Jetson，超过 `9～20 V` 输入上限，会损坏板卡；
- `24 V` 直接接 Jetson，同样超过输入上限；
- 未确认电流就用 B2 的 `5 V` 口或 40 针 GPIO 给整板供电；
- 把 USB-C 当电源输入；
- 使用裸露的廉价 DC-DC 模块、面包板或杜邦线随机器人运动；
- 未经官方许可从电池/BMS 线束并线。

## 14. 机械安装建议

当前是开发套件，不是可直接裸装进工厂的工业整机。首次板载安装应满足：

- 安装在 B2 上部载荷平台，不进入腿部摆动和跌倒接触区域；
- 使用四点螺钉固定的保护盒和防松件，不只用双面胶或魔术贴；
- 保护盒与载板之间加绝缘支柱，底面不能接触螺钉头或金属平台；
- 使用适度减振垫，但不能让板卡大幅摆动；
- 风扇进出风口保持畅通，板卡周围留散热空间；
- 网线和电源线做应力释放，固定在机身上，不垂到腿部；
- DC-DC 模块与 Jetson 分开固定，电源线和相机/网线尽量分开走线；
- 两根 Wi-Fi 天线不要夹在金属外壳内，最终应位于外壳外或射频透明区域；
- 户外防雨需要独立防护设计。裸开发板和普通风扇不能继承 B2 的 IP67；
- 装好后重新做静止、原地姿态、小步、急停、振动、温升和无线覆盖测试。

NVIDIA 将开发套件定位为开发/测试平台。论文样机阶段可以使用，但进厂长期运行前
应评估工业载板、锁紧连接器、防尘防水外壳和宽温散热方案。

## 15. 今天必须带回的结果

1. `jetson_board_report.txt`；
2. `cat /proc/device-tree/model`、L4T、内核、`lspci -nnk`；
3. Jetson 有线/无线网口名、Wi-Fi 驱动名和 100 包 ping 结果；
4. 后退、左移、右移各三次的速度、脉冲和位移；
5. `/rslidar_points`、`/livox/lidar`、LIO-SAM 里程计的类型、频率和坐标系；
6. `/b2_slam/preflight_report` 和 RTAB-Map 完整日志；
7. 若 S2 通过：小闭环 rosbag、数据库和截图；
8. B2 扩展接口、线束标签、Jetson 原装电源铭牌照片；
9. 供应商对 B2 可用供电口针脚、电压、连续电流和线束型号的书面答复。

这些数据齐全后，下一步才是：固定 Jetson 有线 IP、生成 CycloneDDS 双网卡配置、
完成 B2-Jetson 控制迁移、设计 DC-DC 电源样机，然后进入平地 3D 建图与 Nav2
只读联调。
