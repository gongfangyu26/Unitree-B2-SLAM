#!/usr/bin/env python3
"""只读检查三维点云、里程计、时间戳和雷达 TF 是否可用于 SLAM."""

from collections import deque
import json
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .health_core import (
    estimate_rate_hz,
    validate_odometry_values,
    validate_pointcloud_layout,
)


class SlamPreflight(Node):
    """持续发布 SLAM 输入是否完整、新鲜且坐标一致."""

    def __init__(self) -> None:
        super().__init__("b2_slam_preflight")
        self.declare_parameter("cloud_topic", "/b2_slam_3d/cloud_sync")
        self.declare_parameter("odom_topic", "/b2_slam_3d/odom_sync")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("min_cloud_points", 1000)
        self.declare_parameter("min_cloud_rate_hz", 5.0)
        self.declare_parameter("min_odom_rate_hz", 10.0)
        self.declare_parameter("stream_timeout_sec", 0.60)
        self.declare_parameter("stamp_timeout_sec", 1.00)
        self.declare_parameter("warmup_sec", 2.0)
        self.declare_parameter("require_cloud_tf", True)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.min_cloud_points = int(self.get_parameter("min_cloud_points").value)
        self.min_cloud_rate_hz = float(
            self.get_parameter("min_cloud_rate_hz").value
        )
        self.min_odom_rate_hz = float(self.get_parameter("min_odom_rate_hz").value)
        self.stream_timeout_sec = float(
            self.get_parameter("stream_timeout_sec").value
        )
        self.stamp_timeout_sec = float(
            self.get_parameter("stamp_timeout_sec").value
        )
        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.require_cloud_tf = bool(
            self.get_parameter("require_cloud_tf").value
        )
        finite_nonnegative = (
            self.min_cloud_rate_hz,
            self.min_odom_rate_hz,
            self.stream_timeout_sec,
            self.stamp_timeout_sec,
            self.warmup_sec,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in finite_nonnegative):
            raise RuntimeError("SLAM 前置检查阈值必须是非负有限数值")
        if self.min_cloud_points <= 0 or self.stream_timeout_sec <= 0.0:
            raise RuntimeError("点数下限和数据流超时必须大于零")

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(
            PointCloud2, cloud_topic, self._cloud_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.ok_pub = self.create_publisher(Bool, "/b2_slam/preflight_ok", 10)
        self.report_pub = self.create_publisher(
            String, "/b2_slam/preflight_report", 10
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = time.monotonic()
        self.cloud_arrivals: deque[float] = deque(maxlen=100)
        self.odom_arrivals: deque[float] = deque(maxlen=200)
        self.last_cloud_reasons: list[str] = ["尚未收到三维点云"]
        self.last_odom_reasons: list[str] = ["尚未收到里程计"]
        self.last_cloud_stamp_reason = "尚未收到三维点云"
        self.last_odom_stamp_reason = "尚未收到里程计"
        self.cloud_frame = ""
        self.cloud_points = 0
        self.odom_frame_pair = ""
        self.last_report = ""
        self.last_report_time = 0.0
        self.create_timer(0.20, self._evaluate)
        self.get_logger().info(
            f"SLAM 只读前置检查已启动：点云={cloud_topic}，里程计={odom_topic}"
        )

    def _stamp_reason(self, stamp) -> str:
        ros_stamp = Time.from_msg(stamp)
        if ros_stamp.nanoseconds <= 0:
            return "消息时间戳无效"
        age = (self.get_clock().now() - ros_stamp).nanoseconds / 1.0e9
        if age < -0.10:
            return f"消息时间戳超前 {abs(age):.3f} 秒"
        if age > self.stamp_timeout_sec:
            return f"消息时间戳滞后 {age:.3f} 秒"
        return ""

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self.cloud_arrivals.append(time.monotonic())
        self.cloud_frame = str(msg.header.frame_id)
        self.cloud_points = int(msg.width) * int(msg.height)
        self.last_cloud_reasons = validate_pointcloud_layout(
            width=int(msg.width),
            height=int(msg.height),
            point_step=int(msg.point_step),
            row_step=int(msg.row_step),
            data_length=len(msg.data),
            field_names=[field.name for field in msg.fields],
            frame_id=self.cloud_frame,
            min_points=self.min_cloud_points,
        )
        self.last_cloud_stamp_reason = self._stamp_reason(msg.header.stamp)

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom_arrivals.append(time.monotonic())
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.odom_frame_pair = f"{msg.header.frame_id}->{msg.child_frame_id}"
        self.last_odom_reasons = validate_odometry_values(
            position=(position.x, position.y, position.z),
            quaternion=(rotation.x, rotation.y, rotation.z, rotation.w),
            linear_velocity=(linear.x, linear.y, linear.z),
            angular_velocity=(angular.x, angular.y, angular.z),
            frame_id=msg.header.frame_id,
            child_frame_id=msg.child_frame_id,
            expected_frame_id=self.odom_frame,
            expected_child_frame_id=self.base_frame,
        )
        self.last_odom_stamp_reason = self._stamp_reason(msg.header.stamp)

    def _cloud_tf_reason(self) -> str:
        if not self.require_cloud_tf:
            return ""
        if not self.cloud_frame:
            return "尚不知道点云坐标系"
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.cloud_frame, Time()
            )
        except TransformException as exc:
            return (
                f"缺少 {self.base_frame} 到 {self.cloud_frame} 的雷达外参：{exc}"
            )
        return ""

    def _evaluate(self) -> None:
        now = time.monotonic()
        cloud_rate = estimate_rate_hz(tuple(self.cloud_arrivals))
        odom_rate = estimate_rate_hz(tuple(self.odom_arrivals))
        reasons: list[str] = []
        if now - self.start_time < self.warmup_sec:
            reasons.append("SLAM 输入仍在预热")

        reasons.extend(self.last_cloud_reasons)
        reasons.extend(self.last_odom_reasons)
        if self.last_cloud_stamp_reason:
            reasons.append(f"点云{self.last_cloud_stamp_reason}")
        if self.last_odom_stamp_reason:
            reasons.append(f"里程计{self.last_odom_stamp_reason}")

        if not self.cloud_arrivals:
            reasons.append("点云数据流未建立")
        elif now - self.cloud_arrivals[-1] > self.stream_timeout_sec:
            reasons.append("点云数据流超时")
        elif len(self.cloud_arrivals) >= 3 and cloud_rate < self.min_cloud_rate_hz:
            reasons.append(
                f"点云频率过低：{cloud_rate:.2f} < {self.min_cloud_rate_hz:.2f} Hz"
            )
        if not self.odom_arrivals:
            reasons.append("里程计数据流未建立")
        elif now - self.odom_arrivals[-1] > self.stream_timeout_sec:
            reasons.append("里程计数据流超时")
        elif len(self.odom_arrivals) >= 3 and odom_rate < self.min_odom_rate_hz:
            reasons.append(
                f"里程计频率过低：{odom_rate:.2f} < {self.min_odom_rate_hz:.2f} Hz"
            )

        tf_reason = self._cloud_tf_reason()
        if tf_reason:
            reasons.append(tf_reason)
        reasons = list(dict.fromkeys(reasons))
        ok = not reasons
        report = {
            "通过": ok,
            "点云频率Hz": round(cloud_rate, 2),
            "点云点数": self.cloud_points,
            "里程计频率Hz": round(odom_rate, 2),
            "点云坐标系": self.cloud_frame,
            "实际里程计坐标系": self.odom_frame_pair,
            "期望里程计坐标系": f"{self.odom_frame}->{self.base_frame}",
            "原因": reasons,
        }
        report_text = json.dumps(report, ensure_ascii=False)
        self.ok_pub.publish(Bool(data=ok))
        if report_text != self.last_report or now - self.last_report_time >= 2.0:
            self.report_pub.publish(String(data=report_text))
            if report_text != self.last_report:
                if ok:
                    self.get_logger().info(report_text)
                else:
                    self.get_logger().warning(report_text)
            self.last_report = report_text
            self.last_report_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamPreflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS2 Humble 收到 Ctrl+C 后可能已经关闭 context。只有 context 仍有效时
        # 才发布退出状态，避免 shutdown-only RCLError。
        if rclpy.ok():
            node.ok_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
