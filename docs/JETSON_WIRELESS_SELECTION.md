# Jetson 无线通信验收与升级条件

> 2026-09-09 状态更新：Jetson 当前没有可用 WLAN interface；合同尚未确定，
> 因此无线采购和实机验收延期。本文件保留为后续硬件阶段参考，无线不再作为当前
> 有线运动 3D SLAM 与 rosbag 采集的前置条件。

## 1. 已确认的 Jetson 硬件

2026-09-08 正面照片确认开发套件为：

```text
创乐博 Makerobo
Jetson Orin Nano Super CLB Developer Kit
Jetson Orin Nano 模组
```

2026-09-09 背面照片进一步确认：

- `J10 M.2 Key-E 2230` 已安装 `RTL8822CE` 双频 Wi-Fi/蓝牙网卡；
- `Main` 和 `Aux` 两根天线馈线都已经连接；
- 已安装 `HP SSD FX700 256GB` NVMe 固态硬盘；
- 右下方空插槽属于 M.2 Key-M 存储插槽，不是无线插槽。

结论：当前不需要购买 Wi-Fi 网卡或天线。下一步是验证 Ubuntu/JetPack 能否
枚举 RTL8822CE、绑定驱动、连接 5 GHz 网络并达到稳定性要求。

硬件依据：

- 创乐博官网产品中心：<https://www.loborobot.com/>
- NVIDIA 参考载板硬件布局：
  <https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html>

## 2. 当前通信结论

本项目的主通信链路选 **Wi-Fi**，不选 4G。Wi-Fi 用于 Jetson 与监控电脑
在同一局域网内传输 ROS2 状态、点云、图像和地图。4G 仅在将来需要跨公网
远程查看低带宽状态时作备用。

原因：4G 存在运营商 NAT、时延和带宽波动，ROS2 DDS 的局域网发现不能
直接当作公网通信方案，还需要 VPN、鉴权和流量控制。对当前的近场调试和
工厂局域网监控，它不如专用 Wi-Fi 6 接入点合适。

## 3. 现有硬件与升级条件

### 3.1 当前方案：继续使用已安装的 RTL8822CE

照片中的原配模块信息为：

```text
芯片：RTL8822CE
常见模组：AzureWave AW-CB375NF
接口：M.2 Key A/E 2230
频段：2.4 GHz + 5 GHz
状态：模块、两根馈线和两根天线均已安装
```

NVIDIA 的 Orin Nano 参考套件默认使用 802.11ac 无线模块，NVIDIA 支持记录中
将 `RTL8822CE` 称为 Orin Nano 开发套件默认网卡。它是 Wi-Fi 5，不是 Wi-Fi 6，
但对“Jetson 运行全部算法、电脑只监控”的当前架构足够，而且兼容风险最低。

Linux Wireless 文档也列出 `rtw88` 是 RTL8822CE 的 PCIe 驱动：
<https://wireless.docs.kernel.org/en/latest/en/users/drivers/rtl819x.html>

因此先停止网卡采购。只有满足下面任一条件时才讨论升级：

- 装到机器狗最终位置后，5 GHz 覆盖测试持续丢包或带宽不足；
- 当前 JetPack 无法稳定驱动 RTL8822CE，且创乐博无法提供匹配 BSP；
- 工厂明确要求 Wi-Fi 6 漫游或现有模块无法达到论文实验的通信指标。

金属机身内信号差时，第一步应把两根天线移到机壳外或射频透明区域，不是直接
更换芯片。

### 3.2 未来备选：确认驱动后再考虑 AX210

如果 Jetson 载板有 **M.2 Key-E 2230** 插槽，Jetson 的 Linux 内核版本不低于
`5.10`，并且 `modinfo iwlwifi` 已确认目标内核包含驱动，Wi-Fi 6E 备选是：

```text
Intel Wi-Fi 6E AX210NGW（M.2 2230，Key A/E，2x2）
```

若未来确实升级，同时还需要：

- `2` 根与机器狗安装位置匹配的外置双频或三频天线；
- `2` 根与 AX210NGW 接口和天线座匹配的馈线，购买时让商家明确标注
  网卡端子型号，不凭图片猜接头；
- 固定螺丝和绝缘防松措施。

Intel 官方规格确认 AX210 是 M.2 2230、Wi-Fi 使用 PCIe、蓝牙使用 USB，
并列出 Linux 支持。Linux Wireless 的 `iwlwifi` 支持表列出 AX210 从内核
`5.10` 起受支持。这里的“免驱”指不需要下载第三方源码编译驱动；
系统仍需要自带的 `iwlwifi` 内核模块和 Intel firmware。

官方依据：

- NVIDIA Jetson Orin Nano 开发套件官方硬件布局：
  <https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html>
- Intel AX210 官方规格：
  <https://www.intel.com/content/www/us/en/products/sku/204836/intel-wifi-6e-ax210-gig/specifications.html>
- Intel AX210 官方产品说明：
  <https://www.intel.com/content/www/us/en/products/docs/wireless/wi-fi-6e-ax210-module-brief.html>
- Linux Wireless `iwlwifi` 支持列表：
  <https://wireless.docs.kernel.org/en/latest/en/users/drivers/iwlwifi.html>

## 4. 明确不要买的型号

不要买下列依赖 Intel CNVio/CNVio2 的网卡：

```text
AX201、AX211、AX411
```

这些卡虽然可能在外形上能插入 M.2 Key-E，但 Intel 官方说明它们是与
特定 Intel 处理器/芯片组配合的 CRF 模块。Jetson 是 ARM/NVIDIA 平台，不具备该
Intel CNVio 主机逻辑。

Intel 官方 CNVio 兼容说明：
<https://www.intel.com/content/www/us/en/support/articles/000026155/wireless.html>

也不建议为了便宜购买需要商家提供压缩包、每次内核升级后都重新编译
驱动的 USB Realtek 网卡。这类方案不符合“Ubuntu 22.04 免编译驱动”的要求。

## 5. 当前软件验收

硬件已经确认，当前还缺 JetPack/L4T、驱动和网络实测信息。在 Jetson 终端
逐行执行：

```bash
cat /proc/device-tree/model
```

作用：读取 Jetson 模组或开发套件型号。

```bash
head -n 1 /etc/nv_tegra_release
```

作用：读取 Jetson Linux/L4T 版本。

```bash
uname -r
```

作用：记录当前内核版本，为驱动问题定位提供依据。

```bash
modinfo rtw88_8822ce
```

作用：确认当前内核是否包含上游 RTL8822CE 驱动。

```bash
modinfo rtl8822ce
```

作用：查找 Jetson BSP 中可能使用的 NVIDIA/Realtek 配套模块名。

```bash
modinfo rtl88x2ce
```

作用：查找部分 Jetson BSP 中 RTL8822CE 使用的另一模块名。
上面三个命令有一个能输出驱动信息即可。命令名因 Jetson BSP 打包方式可能不同，
最终仍以 `lspci -nnk` 中的 `Kernel driver in use` 为准。

```bash
lspci -nnk
```

作用：确认 RTL8822CE 被 PCIe 枚举并已经绑定内核驱动。

```bash
ip -br link
```

作用：记录无线和有线网口的实际名称。

```bash
nmcli device status
rfkill list
nmcli device wifi list
```

作用：确认 NetworkManager 能看到设备、无线射频未被屏蔽并能扫描无线网络。

推荐通过 Ubuntu 图形界面连接 5 GHz 网络。需要命令行时使用：

```bash
nmcli --ask device wifi connect "你的5GHz无线网络名称"
```

密码在本机交互输入，不要把密码写进命令、截图或调试记录。

## 6. 工厂部署建议

- 调试阶段使用一台独立 Wi-Fi 6 路由器/接入点，Jetson 和电脑加入同一
  `5 GHz` 网络；暂不把 `6 GHz` 当作必需条件。
- 让路由器做 AP，Jetson 作为 STA 客户端连接，不把 Jetson 热点当作主要
  巡检网络。NVIDIA 的 RTL8822CE 测试记录显示 STA 模式吞吐正常，而 AP 模式
  需要额外调整频段和配置：
  <https://forums.developer.nvidia.com/t/slow-wi-fi-on-orin-nano-devkit-rtl8822ce-802-11ac/368697>
- 点云和图像优先在 Jetson 上处理，电脑只订阅降采样状态、缩略图或按需的
  RViz 数据，不要长期把全频原始点云和无压缩图像发到电脑。
- 进厂前做无线覆盖测绘，检查坡道、转角、金属设备后方和楼层切换处；
  多 AP 时优先使用有线回程。
- 无线中断不能破坏板载自主决策。任务、定位、避障、拍照判断和停车联锁
  都运行在 Jetson，电脑断网只影响监控。
- 如果将来需要 4G，把它作为第二链路，只传输健康状态、报警和压缩照片；
  先确认中国运营商频段、SIM、载板 M.2 B-Key/USB 接口和 VPN 方案，再选具体型号。

## 7. 现有 RTL8822CE 验收标准

当前网卡和天线已经安装，不拆卡。开机后逐行执行：

```bash
lspci -nnk
```

作用：确认 RTL8822CE 被 PCIe 枚举，并显示 `rtw88_8822ce`、`rtl8822ce`、
`rtl88x2ce` 或当前 Jetson BSP 提供的对应驱动名。

```bash
rfkill list
```

作用：确认 Wi-Fi 没有被硬件或软件屏蔽。

```bash
ip -br link
```

作用：查看新的无线网口名称。

```bash
nmcli device wifi list
```

作用：确认 NetworkManager 能扫描到 2.4/5 GHz 无线网络。

连接后还要完成：

```bash
ping -c 100 监控电脑无线IP
```

桌面近距离应无丢包；若超过 `1%` 或有频繁百毫秒级延迟尖峰，先排查天线和信道。
如果不需要下载第三方源码、不需要编译内核模块，重启后仍能自动识别和连接 5 GHz，
并且装到机器狗最终位置后网络仍稳定，才记为“Ubuntu 22.04/Jetson 无线验收通过”。

完整的当日板卡、网络、SLAM 和供电检查步骤见
[B2 与 Jetson 下午实机调试清单](B2_JETSON_DEBUG_2026-09-09.md)。
