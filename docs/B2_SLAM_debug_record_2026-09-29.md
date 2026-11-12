# Unitree B2 + Jetson Orin Nano + RTAB-Map SLAM 调试记录

日期：2026-09-29

## 一、项目目标

完成 Unitree B2 机器狗、Jetson Orin Nano、激光雷达与 ROS2 Humble
环境下的三维 SLAM 链路搭建。

------------------------------------------------------------------------

## 二、硬件与网络环境

### Jetson Orin Nano

-   Ubuntu 22.04.5
-   ROS2 Humble
-   RMW: rmw_cyclonedds_cpp

### 网络接口

B2通信网口：

    enP8p1s0
    IP: 192.168.123.211

实验室 WiFi：

    wlP1p1s0
    IP: 10.239.201.107

B2通信必须使用 enP8p1s0。

------------------------------------------------------------------------

# 三、初始故障现象

可以发现 ROS2 topic：

    /lf/sportmodestate

但是：

``` bash
ros2 topic echo /lf/sportmodestate --once
```

无数据输出。

同时：

``` bash
ros2 topic hz /odom
```

无输出。

但是：

``` bash
ros2 topic info /odom
```

显示存在 Publisher。

说明 ROS2 图发现正常，但是 DDS 数据通信异常。

------------------------------------------------------------------------

# 四、问题定位

## 根本原因

CycloneDDS 网络接口配置错误。

原配置：

``` xml
<NetworkInterface name="enp2s0"/>
```

实际 Jetson 网卡：

    enP8p1s0

由于接口名称错误：

-   可以发现节点和 topic
-   但是无法正常接收数据

------------------------------------------------------------------------

# 五、修改记录

修改文件：

    ~/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml

修改：

    enp2s0

为：

    enP8p1s0

最终配置：

``` xml
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="enP8p1s0"
                    priority="default"
                    multicast="default"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>
```

作用：

强制 CycloneDDS 使用 B2 通信网口。

------------------------------------------------------------------------

# 六、ROS2环境配置

每次启动前：

``` bash
export ROS_DOMAIN_ID=0

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash

source ~/unitree_ros2/cyclonedds_ws/install/setup.bash

source ~/Unitree-B2-SLAM/install/setup.bash

export CYCLONEDDS_URI=file:///home/jetson/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
```

------------------------------------------------------------------------

# 七、恢复结果

## 1. B2状态恢复

测试：

``` bash
ros2 topic echo /lf/sportmodestate --once
```

成功输出：

-   IMU状态
-   位姿 position
-   速度 velocity
-   yaw_speed

说明：

B2 DDS通信恢复。

------------------------------------------------------------------------

## 2. 里程计恢复

测试：

``` bash
ros2 topic hz /odom
```

结果：

约：

    20Hz

说明：

b2_driver_py 正常发布里程计。

------------------------------------------------------------------------

# 八、SLAM完整数据链路

    B2
     |
     | /lf/sportmodestate
     |
     v
    b2_driver_py
     |
     | /odom
     |
     v
    sensor_time_sync
     |
     +----------------+
     |
     | /b2_slam_3d/odom_sync
     |
     | /rslidar_points
     |
     v
    /b2_slam_3d/cloud_sync
     |
     v
    RTAB-Map
     |
     v
    /rtabmap/cloud_map

------------------------------------------------------------------------

# 九、SLAM测试结果

## 里程计同步

Topic：

    /b2_slam_3d/odom_sync

频率：

约：

    38~40Hz

------------------------------------------------------------------------

## 点云同步

Topic：

    /b2_slam_3d/cloud_sync

频率：

约：

    8~10Hz

------------------------------------------------------------------------

## RTAB-Map地图

Topic：

    /rtabmap/cloud_map

开始输出。

地图更新频率约：

    0.2Hz

说明 RTAB-Map 已经开始工作。

------------------------------------------------------------------------

# 十、当前存在的问题

发现重复节点：

    /b2_driver_py
    /b2_safe_twist_bridge
    /static_tf_lidar_frame_alias

原因：

多次启动 launch 导致。

后续需要统一启动流程。

------------------------------------------------------------------------

# 十一、电脑与Jetson代码同步要求

## 开发端

电脑 Ubuntu：

负责：

-   代码修改
-   Git管理
-   Codex辅助开发

## 运行端

Jetson Orin Nano：

负责：

-   编译
-   运行
-   实机调试

要求：

电脑端源码与 Jetson：

    ~/Unitree-B2-SLAM

保持同步。

------------------------------------------------------------------------

# 十二、需要Codex执行的任务

请 Codex：

1.  阅读本调试记录。

2.  保持电脑端 Unitree-B2-SLAM 与 Jetson 端同步。

3.  同步以下修改：

文件：

    unitree_ros2/cyclonedds_ws/src/cyclonedds.xml

修改：

    enp2s0

为：

    enP8p1s0

4.  不要重新修改已经验证通过的 ROS2 节点逻辑。

------------------------------------------------------------------------

# 十三、后续开发计划

1.  RViz查看三维地图。

2.  调整RTAB-Map参数：

-   ICP参数
-   voxel size
-   loop closure

3.  实验室环境自主建图。

4.  后续进入工厂巡检环境测试。

------------------------------------------------------------------------

# 总结

2026-09-29 完成 Unitree B2 三维SLAM基础通信链路打通。

已完成：

-   DDS通信恢复
-   B2状态获取
-   里程计发布
-   激光雷达同步
-   RTAB-Map建图启动

当前进入：

SLAM参数优化阶段。
