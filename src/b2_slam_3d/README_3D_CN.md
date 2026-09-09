# B2 三维激光建图说明

该功能包使用 RTAB-Map、B2 里程计和标准 PointCloud2 建立六自由度三维地图，适用于
含坡道和高度变化的场景。2026-09-09 已在 B2 静止状态确认
`/rslidar_points`、项目 `/odom`、TF、时间兼容节点、RTAB-Map 和非空
`/cloud_map` 数据链。该结果不代表运动建图、坡道地图或导航已经通过。

新建地图：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py delete_db:=true use_rtabmap_viz:=true
```

继续已有地图：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py delete_db:=false use_rtabmap_viz:=true
```

完整的传感器检查、建图、保存数据库和坡道验证步骤见：

```text
src/b2_factory_inspection/OPERATION_GUIDE.md
```

时间兼容节点默认只把点云改为主机接收时间，里程计保留驱动生成的
时间戳，使 `/odom` 与 `odom→base_link` TF 时刻一致。这不是硬件时间
同步的替代方案；取得真机 rosbag 后必须比较原始时间戳并修正驱动。

RTAB-Map 在该启动文件中使用 `/rtabmap` 命名空间，供 Nav2 保存的二维占据
地图话题为 `/rtabmap/map`。

启动后先查看只读健康报告：

```bash
ros2 topic echo /b2_slam/preflight_report
ros2 topic echo /b2_slam/preflight_ok
```

报告未通过时不要让机器人行走。下一次只进行有线、小范围、人工控制的运动建图，详细命令见
`docs/B2_NEXT_WIRED_MOTION_SLAM_DEBUG.md`。
