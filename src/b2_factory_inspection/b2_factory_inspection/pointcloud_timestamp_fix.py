#!/usr/bin/env python3
"""Republish PointCloud2 with host-clock timestamps for local patrol sensing."""

from __future__ import annotations

import copy

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudTimestampFix(Node):
    def __init__(self) -> None:
        super().__init__("factory_pointcloud_timestamp_fix")

        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("output_topic", "/factory_patrol/cloud_fixed")
        self.declare_parameter("backdate_sec", 0.10)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.backdate = Duration(seconds=float(self.get_parameter("backdate_sec").value))

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, sensor_qos
        )
        self.count = 0
        self.last_log_ns = 0
        self.get_logger().info(f"PointCloud timestamp fix: {input_topic} -> {output_topic}")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        out = copy.copy(msg)
        out.header = copy.copy(msg.header)
        out.header.stamp = (self.get_clock().now() - self.backdate).to_msg()
        self.publisher.publish(out)
        self.count += 1
        self._log_rate_limited()

    def _log_rate_limited(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns < 5_000_000_000:
            return
        self.last_log_ns = now_ns
        self.get_logger().info(f"Republished local patrol pointcloud count={self.count}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
