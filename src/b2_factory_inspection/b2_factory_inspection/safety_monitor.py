#!/usr/bin/env python3
"""汇总机器人状态、二维雷达和定位 TF，生成自主运行安全许可."""

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class AutonomySafetyMonitor(Node):
    """只有运动状态、障碍感知和全局定位都新鲜时才发布真值."""

    def __init__(self) -> None:
        super().__init__("b2_autonomy_safety_monitor")
        self.declare_parameter("robot_state_topic", "/b2/state_ok")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_state_timeout_sec", 0.80)
        self.declare_parameter("slam_preflight_timeout_sec", 0.80)
        self.declare_parameter("scan_timeout_sec", 0.60)
        self.declare_parameter("scan_stamp_timeout_sec", 0.80)
        self.declare_parameter("transform_timeout_sec", 1.50)
        self.declare_parameter("min_scan_samples", 30)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_localization", True)
        self.declare_parameter("require_slam_preflight", True)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.robot_state_timeout_sec = float(
            self.get_parameter("robot_state_timeout_sec").value
        )
        self.slam_preflight_timeout_sec = float(
            self.get_parameter("slam_preflight_timeout_sec").value
        )
        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self.scan_stamp_timeout_sec = float(
            self.get_parameter("scan_stamp_timeout_sec").value
        )
        self.transform_timeout_sec = float(
            self.get_parameter("transform_timeout_sec").value
        )
        self.min_scan_samples = int(self.get_parameter("min_scan_samples").value)
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.require_localization = bool(
            self.get_parameter("require_localization").value
        )
        self.require_slam_preflight = bool(
            self.get_parameter("require_slam_preflight").value
        )
        timeout_values = (
            self.robot_state_timeout_sec,
            self.slam_preflight_timeout_sec,
            self.scan_timeout_sec,
            self.scan_stamp_timeout_sec,
            self.transform_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in timeout_values):
            raise RuntimeError("安全监视器超时参数必须是大于零的有限数值")
        if self.min_scan_samples < 2:
            raise RuntimeError("min_scan_samples 必须至少为 2")

        self.robot_state_ok = False
        self.slam_preflight_ok = False
        self.scan_valid = False
        self.scan_invalid_reason = "尚未收到二维雷达"
        self.last_robot_state_time = 0.0
        self.last_slam_preflight_time = 0.0
        self.last_scan_time = 0.0
        self.last_reason = ""
        self.last_reason_publish_time = 0.0

        self.ok_pub = self.create_publisher(Bool, "/b2/autonomy_ok", 10)
        self.reason_pub = self.create_publisher(
            String, "/b2/autonomy_reason", 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("robot_state_topic").value),
            self._robot_state_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/b2_slam/preflight_ok",
            self._slam_preflight_callback,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.10, self._evaluate)
        self.get_logger().info("自主安全监视器已启动，初始状态为禁止运动")

    def _robot_state_callback(self, msg: Bool) -> None:
        self.robot_state_ok = bool(msg.data)
        self.last_robot_state_time = time.monotonic()

    def _slam_preflight_callback(self, msg: Bool) -> None:
        """记录 SLAM 点云、里程计和雷达外参的综合健康状态."""
        self.slam_preflight_ok = bool(msg.data)
        self.last_slam_preflight_time = time.monotonic()

    def _scan_callback(self, msg: LaserScan) -> None:
        geometry_values = (
            float(msg.angle_min),
            float(msg.angle_max),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
        )
        reasons: list[str] = []
        if len(msg.ranges) < self.min_scan_samples:
            reasons.append("采样点数量不足")
        if not all(math.isfinite(value) for value in geometry_values):
            reasons.append("角度或量程字段含非法数值")
        elif not (
            msg.angle_increment > 0.0
            and msg.angle_max > msg.angle_min
            and msg.range_max > msg.range_min >= 0.0
        ):
            reasons.append("角度或量程字段关系错误")

        ranges_valid = all(
            (math.isinf(float(value)) and float(value) > 0.0)
            or (
                math.isfinite(float(value))
                and msg.range_min <= float(value) <= msg.range_max
            )
            for value in msg.ranges
        )
        if msg.ranges and not ranges_valid:
            reasons.append("距离数组含 NaN、负无穷或越界数值")

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds <= 0:
            reasons.append("时间戳无效")
        else:
            stamp_age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
            if stamp_age < -0.10:
                reasons.append("时间戳超前")
            elif stamp_age > self.scan_stamp_timeout_sec:
                reasons.append("消息时间戳已过期")

        self.scan_valid = not reasons
        self.scan_invalid_reason = "、".join(reasons)
        self.last_scan_time = time.monotonic()

    def _transform_reason(self) -> str | None:
        """返回定位 TF 的失败原因；返回空值表示定位有效."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except TransformException as exc:
            return f"缺少 {self.map_frame} 到 {self.base_frame} 的定位变换：{exc}"

        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds <= 0:
            return "定位变换没有有效时间戳"
        age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        if age < -0.10:
            return f"定位变换时间戳超前 {abs(age):.2f} 秒"
        if age > self.transform_timeout_sec:
            return f"定位变换已超时 {age:.2f} 秒"

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return "定位变换含 NaN 或无穷大"
        quaternion_norm = math.sqrt(
            rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
            + rotation.w * rotation.w
        )
        if quaternion_norm < 0.50 or quaternion_norm > 1.50:
            return "定位变换四元数长度异常"
        return None

    def _evaluate(self) -> None:
        now = time.monotonic()
        reasons: list[str] = []
        if now - self.last_robot_state_time > self.robot_state_timeout_sec:
            reasons.append("机器人状态超时")
        elif not self.robot_state_ok:
            reasons.append("机器人状态无效")

        if self.require_scan:
            if now - self.last_scan_time > self.scan_timeout_sec:
                reasons.append("二维雷达超时")
            elif not self.scan_valid:
                reasons.append(f"二维雷达无效：{self.scan_invalid_reason}")

        if self.require_slam_preflight:
            if (
                now - self.last_slam_preflight_time
                > self.slam_preflight_timeout_sec
            ):
                reasons.append("SLAM 前置健康状态超时")
            elif not self.slam_preflight_ok:
                reasons.append("SLAM 点云、里程计或雷达外参未通过检查")

        if self.require_localization:
            transform_reason = self._transform_reason()
            if transform_reason:
                reasons.append(transform_reason)

        autonomy_ok = not reasons
        reason = "自主运行条件已满足" if autonomy_ok else "；".join(reasons)
        self.ok_pub.publish(Bool(data=autonomy_ok))
        if reason != self.last_reason or now - self.last_reason_publish_time >= 1.0:
            self.reason_pub.publish(String(data=reason))
            if reason != self.last_reason:
                log = self.get_logger().info if autonomy_ok else self.get_logger().warn
                log(reason)
            self.last_reason = reason
            self.last_reason_publish_time = now


def main(args=None) -> None:
    """启动自主安全监视器."""
    rclpy.init(args=args)
    node = AutonomySafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ok_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
