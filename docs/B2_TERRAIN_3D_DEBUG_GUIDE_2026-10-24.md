# Unitree B2 坡道、楼梯与三维点云调试手册

计划日期：2026-10-24\
当前状态：**仅代码检查，尚未实机验证**

## 1. 明天的目标与边界

明天只做有人看护的手动点动能力测试，同时检查 RTAB-Map 在高度变化场景中的
三维点云累积、显示和保存。不做 Nav2 自主爬坡、自主爬楼梯或无人值守。

当前已有实机证据是平地运动 3D 建图；坡道和楼梯仍是新实验。楼梯阶段只有
在坡道测试通过、步态来源可核对、并完成物理防跌落后才能开始。

## 2. 人员、场地和立即中止条件

- 至少两人：一人操作键盘，一人全程手持原厂遥控/急停；
- 机器人两侧布置不限制足端运动的防倾/保护措施，楼梯下方有缓冲区；
- 首轮使用干燥、防滑、宽度足够且两侧无落差的低坡；
- 记录坡度、坡长、表面、台阶级数、每级高度/深度/宽度、负载和电量；
- 线缆不得进入腿部、足端或台阶边缘；优先在 Jetson 本机操作。

出现任一项立即按 `x` 并用遥控/急停人工介入：打滑、横向偏移、足端踩空、
俯仰/横滚快速增大、机身碰台阶、线缆受拉、状态/点云中断、地图跳变或程序报错。

## 3. 代码同步与编译

仅同步本次涉及的包，不覆盖 Jetson 日志和地图：

```bash
rsync -av --exclude '__pycache__' \
  "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_twist_bridge_py" \
  "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_slam_3d" \
  "/home/gongfangyu/桌面/Unitree-B2-SLAM/Unitree-B2-SLAM/src/b2_factory_inspection" \
  jetson@<JETSON_WIFI_IP>:/home/jetson/Unitree-B2-SLAM/src/
```

Jetson 上：

```bash
cd ~/Unitree-B2-SLAM
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --symlink-install --packages-select \
  b2_twist_bridge_py b2_slam_3d b2_factory_inspection \
  --allow-overriding b2_twist_bridge_py b2_slam_3d b2_factory_inspection
source install/setup.bash
```

## 4. 上电后的只读检查

所有终端先加载项目已记录的 Domain 0、CycloneDDS 有线绑定和 ROS2 环境。
先检查磁盘：

```bash
df -h /
mkdir -p /home/jetson/b2_maps /home/jetson/b2_terrain_logs
```

2026-10-16 只剩约 9.3 GB，不允许盲目录制长时间原始点云。若可用空间少于 5 GB，
停止当天录包和建图，先备份；不删除尚未备份的历史数据。

## 5. 启动三维建图和 RViz

一个实验编号对应一个新 `.db`，不覆盖旧地图：

```bash
ros2 launch b2_slam_3d slam_3d.launch.py \
  delete_db:=true \
  database_path:=/home/jetson/b2_maps/b2_slope_stairs_20261024_01.db \
  start_driver:=true \
  cloud_topic:=/rslidar_points \
  odom_topic:=/odom \
  lidar_frame:=rslidar \
  use_rtabmap_viz:=false \
  use_rviz:=true
```

若驱动已单独运行，上面必须改为 `start_driver:=false`。同一 Domain 不得有重复驱动、
TF 或 RTAB-Map 节点。RViz 固定坐标系是 `map`，默认显示 `/rtabmap/cloud_map`。

机器人保持静止，另开终端执行：

```bash
ros2 topic echo /b2_slam/preflight_report --once
ros2 topic echo /b2_slam/preflight_ok --once
ros2 topic hz /b2_slam_3d/cloud_sync
ros2 topic hz /b2_slam_3d/odom_sync
ros2 topic info /rtabmap/cloud_map -v
ros2 topic echo /rtabmap/cloud_map --once --field width
ros2 run tf2_ros tf2_echo map base_link
```

任一输入不通过时不允许运动。

## 6. 启动地形点动测试

确认没有其他 `safe_motion_test`、Nav2 或键盘节点向 `/api/sport/request` 发送运动命令。
首次只观察实际步态，不猜测步态编号：

```bash
ros2 run b2_twist_bridge_py terrain_motion_test --ros-args \
  -p expected_gait_type:=-1 \
  -p motion_pulse_sec:=0.50 \
  -p max_abs_roll_deg:=22.0 \
  -p max_abs_pitch_deg:=35.0 \
  -p test_label:=terrain_20261024_01
```

启动后按 `p`，记录原厂方式切换地形模式前后真实 `mode/gait_type`。当场确认后，
先退出节点，再用真实数值替换 `-1` 重启；一旦步态发生变化，节点会停车锁定。

每轮顺序：

1. 机器人在平地、离坡道/首级台阶有足够停车距离时选阶段 `1/2/3/4`；
2. 按 `p` 确认姿态和 `mode/gait_type`；
3. 完成双人安全口头确认，3 秒内连按两次 `i`；
4. 第一次只按一次 `w`，观察 0.50 秒动作和自动 `STOPMOVE`；脉冲结束后程序会自动锁定；
5. 每次点动后等机器人稳定，检查 RViz 点云和终端日志；需要继续时必须重新双击 `i` 解锁；
6. 每轮结束按 `x`，确认已锁定。

不在坡面或台阶中途临时改速度、姿态阈值或步态。不在首轮测试中扩大
`motion_pulse_sec`。程序中的 22°/35° 是保守停车限值，不是 B2 官方能力指标。

## 7. 可选的短时 rosbag

仅在磁盘空间充足时录制，首轮每包不超过 30 秒：

```bash
mkdir -p /home/jetson/b2_bags/20261024
ros2 bag record -o /home/jetson/b2_bags/20261024/terrain_01 \
  /lf/sportmodestate /odom /rslidar_points \
  /b2_slam_3d/cloud_sync /b2_slam_3d/odom_sync \
  /rtabmap/info /tf /tf_static
```

不重复录制体积很大的全局 `/rtabmap/cloud_map`；全局点云用下一节导出一帧。

## 8. 保存三维点云和地图

机器人停稳、RTAB-Map 点云更新完成后：

```bash
ros2 topic echo /rtabmap/cloud_map --once --field width
ros2 run b2_slam_3d pointcloud_snapshot --ros-args \
  -p cloud_topic:=/rtabmap/cloud_map \
  -p output_path:=/home/jetson/b2_maps/b2_slope_stairs_20261024_01.pcd \
  -p min_points:=100
ls -lh \
  /home/jetson/b2_maps/b2_slope_stairs_20261024_01.db \
  /home/jetson/b2_maps/b2_slope_stairs_20261024_01.pcd
```

如果现场话题不是 `/rtabmap/cloud_map`，先用 `ros2 topic list -t | grep cloud`
和 `ros2 topic type` 查明真实 `sensor_msgs/msg/PointCloud2` 话题，再更改 `cloud_topic`，
不得因等待超时就宣称点云已保存。

为了离线复核 RTAB-Map 数据库，建图节点正常 Ctrl-C 退出后还可导出 PLY：

```bash
rtabmap-export --cloud --scan --opt 2 --voxel 0.05 \
  --output b2_slope_stairs_20261024_01 \
  --output_dir /home/jetson/b2_maps \
  /home/jetson/b2_maps/b2_slope_stairs_20261024_01.db
```

PCD、PLY 和 `.db` 都必须使用同一实验编号，并在当天调试日志中记录路径。

## 9. 当天收尾必须记录

- 每轮的地形几何、速度、脉冲时间、前进/后退、实际 `mode/gait_type`、结果和人工干预；
- CSV、rosbag、RTAB-Map DB、PCD/PLY 的绝对路径、文件大小和对应实验编号；
- 点云是否有重影、分层、高度跳变、丢帧，地图中楼梯/坡面是否可辨识；
- 分开写“实机已验证”“仅代码检查，尚未实机验证”“失败/中止”和“未执行”。

楼梯一次成功不构成能力结论。正式论文数据需要固定场景、参数和起终点，保留失败样本，
并按预先定义的重复次数和指标计算通过率。
