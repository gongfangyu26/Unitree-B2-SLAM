"""
点云接收时间兼容节点.

该节点复制整帧点云，仅把时间戳替换为主机接收时间。它可以临时规避传感器
时钟与 ROS 主机时钟不一致造成的 TF 查询失败，但不等同于硬件时间同步。

Go2 默认输入: /utlidar/cloud_deskewed
B2  默认输入: /rslidar_points (RoboSense 雷达)
"""

import copy
import math

import rclpy
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


class PointCloudTimestampFix(Node):
    """按主机接收时间重发点云，并报告输入频率."""

    def __init__(self):
        super().__init__("pointcloud_timestamp_fix")

        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("output_topic", "/utlidar/cloud_fixed")
        self.declare_parameter("backdate_sec", 0.05)
        self.declare_parameter("input_reliability", "best_effort")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        input_reliability = str(
            self.get_parameter("input_reliability").value
        ).strip().lower()
        reliability_policies = {
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
            "reliable": ReliabilityPolicy.RELIABLE,
        }
        if input_reliability not in reliability_policies:
            raise RuntimeError(
                "input_reliability 必须是 best_effort 或 reliable"
            )
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=reliability_policies[input_reliability],
            durability=DurabilityPolicy.VOLATILE,
        )

        backdate_sec = float(self.get_parameter("backdate_sec").value)
        if not math.isfinite(backdate_sec) or backdate_sec < 0.0:
            raise RuntimeError("backdate_sec 必须是非负有限数值")
        self.backdate = Duration(seconds=backdate_sec)

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            input_qos,
        )
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )

        # 每 5 秒报告一次接收情况，方便现场判断雷达是否断流。
        self._msg_count = 0
        self._last_log = self.get_clock().now()
        self._log_timer = self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f"点云接收时间兼容节点已启动：{input_topic} → {output_topic}"
            f"，输入可靠性={input_reliability}"
        )

    def _log_status(self):
        elapsed = (self.get_clock().now() - self._last_log).nanoseconds / 1e9
        rate = self._msg_count / elapsed if elapsed > 0 else 0
        if self._msg_count == 0:
            self.get_logger().warn(
                "连续 5 秒没有点云，请检查雷达驱动、话题名、网卡和 QoS"
            )
        else:
            self.get_logger().info(
                f"点云接收正常：{self._msg_count} 帧/{elapsed:.1f} 秒，"
                f"频率约 {rate:.1f} Hz"
            )
        self._msg_count = 0
        self._last_log = self.get_clock().now()

    def pointcloud_callback(self, msg: PointCloud2):
        """浅复制只读点云数据，并为新消息复制独立消息头."""
        self._msg_count += 1
        fixed_msg = copy.copy(msg)
        fixed_msg.header = copy.copy(msg.header)
        fixed_msg.header.stamp = (self.get_clock().now() - self.backdate).to_msg()
        self.publisher.publish(fixed_msg)


def main(args=None):
    """启动点云接收时间兼容节点."""
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
