#!/usr/bin/env python3
"""
把需要兼容的 RoboSense 点云时间戳转换到 ROS 主机时钟.

该节点不修改位姿数值或点云内容。默认只重写点云接收时间，里程计时间戳原样
保留，以便和驱动发布的 odom→base_link TF 使用完全相同的时间。此方法不是硬件
时间同步，只是调试兼容措施；实机仍应检查 PTP、PPS 或传感器时钟配置。
"""

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2


class SensorTimeSync(Node):
    def __init__(self):
        super().__init__("sensor_time_sync")

        self.declare_parameter("cloud_in", "/rslidar_points")
        self.declare_parameter("cloud_out", "/b2_slam_3d/cloud_sync")
        self.declare_parameter("odom_in", "/odom")
        self.declare_parameter("odom_out", "/b2_slam_3d/odom_sync")
        self.declare_parameter("restamp_cloud", True)
        self.declare_parameter("restamp_odom", False)
        self.declare_parameter("cloud_backdate_sec", 0.05)
        self.declare_parameter("odom_backdate_sec", 0.0)

        cloud_in = self.get_parameter("cloud_in").value
        cloud_out = self.get_parameter("cloud_out").value
        odom_in = self.get_parameter("odom_in").value
        odom_out = self.get_parameter("odom_out").value
        self.restamp_cloud = bool(self.get_parameter("restamp_cloud").value)
        self.restamp_odom = bool(self.get_parameter("restamp_odom").value)
        self.cloud_backdate_sec = float(self.get_parameter("cloud_backdate_sec").value)
        self.odom_backdate_sec = float(self.get_parameter("odom_backdate_sec").value)
        backdates = (self.cloud_backdate_sec, self.odom_backdate_sec)
        if not all(math.isfinite(value) and value >= 0.0 for value in backdates):
            raise RuntimeError("时间回退参数不能为负数")

        # RoboSense 点云通常使用传感器数据服务质量配置。
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_out, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_in, self.cloud_callback, qos_profile_sensor_data
        )

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.create_publisher(Odometry, odom_out, odom_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_in, self.odom_callback, odom_qos)

        self.cloud_count = 0
        self.odom_count = 0
        self.last_log_ns = 0

        self.get_logger().info(
            f"时间兼容节点已启动：点云 {cloud_in} → {cloud_out}，"
            f"里程计 {odom_in} → {odom_out}；"
            f"重写点云={self.restamp_cloud}，重写里程计={self.restamp_odom}"
        )

    def _stamp(self, backdate_sec: float):
        now = self.get_clock().now()
        if backdate_sec > 0.0:
            now = now - Duration(seconds=backdate_sec)
        return now.to_msg()

    def cloud_callback(self, msg: PointCloud2):
        if self.restamp_cloud:
            out = copy.copy(msg)
            out.header = copy.copy(msg.header)
            out.header.stamp = self._stamp(self.cloud_backdate_sec)
        else:
            out = msg
        self.cloud_pub.publish(out)
        self.cloud_count += 1
        self._log_rate_limited()

    def odom_callback(self, msg: Odometry):
        if self.restamp_odom:
            out = copy.copy(msg)
            out.header = copy.copy(msg.header)
            out.header.stamp = self._stamp(self.odom_backdate_sec)
        else:
            out = msg
        self.odom_pub.publish(out)
        self.odom_count += 1
        self._log_rate_limited()

    def _log_rate_limited(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 5_000_000_000:
            self.last_log_ns = now_ns
            self.get_logger().info(
                f"累计转发：点云={self.cloud_count}，里程计={self.odom_count}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorTimeSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
