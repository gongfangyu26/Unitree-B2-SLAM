#!/usr/bin/env python3
"""为 RViz 人工目标导航提供默认锁定、限时解锁的速度安全门."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .navigation_gate_core import NavigationGateConfig, NavigationGateState


class NavigationCommandGate(Node):
    """仅在地图和自主安全状态通过后转发 Nav2 速度."""

    def __init__(self) -> None:
        super().__init__("b2_navigation_command_gate")
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("autonomy_topic", "/b2/autonomy_ok")
        self.declare_parameter("map_ok_topic", "/b2_navigation/map_ok")
        self.declare_parameter(
            "heartbeat_topic", "/inspection/control_heartbeat"
        )
        self.declare_parameter("autonomy_timeout_sec", 0.80)
        self.declare_parameter("map_timeout_sec", 3.0)
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("max_armed_duration_sec", 60.0)
        self.declare_parameter("supervisor_heartbeat_topic", "")
        self.declare_parameter("supervisor_timeout_sec", 0.60)
        self.supervisor_topic = str(
            self.get_parameter("supervisor_heartbeat_topic").value
        )
        self.supervisor_timeout_sec = float(
            self.get_parameter("supervisor_timeout_sec").value
        )
        if not math.isfinite(self.supervisor_timeout_sec) or not (
            0.10 <= self.supervisor_timeout_sec <= 1.0
        ):
            raise RuntimeError("supervisor_timeout_sec 必须位于 0.10 到 1.0 秒")
        self.last_supervisor_time = 0.0

        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        if (
            not math.isfinite(self.command_timeout_sec)
            or self.command_timeout_sec <= 0.0
        ):
            raise RuntimeError("command_timeout_sec 必须大于零")
        try:
            config = NavigationGateConfig(
                autonomy_timeout_sec=float(
                    self.get_parameter("autonomy_timeout_sec").value
                ),
                map_timeout_sec=float(
                    self.get_parameter("map_timeout_sec").value
                ),
                max_armed_duration_sec=float(
                    self.get_parameter("max_armed_duration_sec").value
                ),
            )
        except ValueError as exc:
            raise RuntimeError(f"导航安全门参数无效：{exc}") from exc
        self.state = NavigationGateState(config)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        autonomy_topic = str(self.get_parameter("autonomy_topic").value)
        map_ok_topic = str(self.get_parameter("map_ok_topic").value)
        heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.heartbeat_pub = self.create_publisher(
            Bool, heartbeat_topic, 10
        )
        self.armed_pub = self.create_publisher(
            Bool, "/b2_navigation/armed", 10
        )
        self.status_pub = self.create_publisher(
            String, "/b2_navigation/status", 10
        )
        self.create_subscription(
            Twist, input_topic, self._command_callback, 10
        )
        self.create_subscription(
            Bool, autonomy_topic, self._autonomy_callback, 10
        )
        self.create_subscription(
            Bool, map_ok_topic, self._map_callback, 10
        )
        if self.supervisor_topic:
            self.create_subscription(
                Bool, self.supervisor_topic, self._supervisor_callback, 10
            )
        self.create_service(
            SetBool,
            "/b2/navigation/set_armed",
            self._set_armed_callback,
        )
        self.create_timer(0.10, self._watchdog_callback)

        self.last_command_time = 0.0
        self.motion_command_active = False
        self.last_status = ""
        self._publish_status("导航安全门已启动，默认锁定")
        self.get_logger().info(
            f"速度路径：{input_topic} → 安全门 → {output_topic}"
        )

    @staticmethod
    def _is_zero_twist(msg: Twist) -> bool:
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        return all(abs(float(value)) < 1.0e-9 for value in values)

    def _autonomy_callback(self, msg: Bool) -> None:
        self.state.update_autonomy(msg.data, time.monotonic())

    def _map_callback(self, msg: Bool) -> None:
        self.state.update_map(msg.data, time.monotonic())

    def _supervisor_callback(self, msg: Bool) -> None:
        self.last_supervisor_time = time.monotonic() if msg.data else 0.0
        if not msg.data and self.state.armed:
            self._disarm("巡逻程序撤销存活许可")

    def _supervisor_reason(self, now: float) -> str | None:
        if self.supervisor_topic and (
            self.last_supervisor_time <= 0.0
            or now - self.last_supervisor_time > self.supervisor_timeout_sec
        ):
            return "巡逻程序存活心跳尚未收到或已超时"
        return None

    def _command_callback(self, msg: Twist) -> None:
        now = time.monotonic()
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self._disarm("收到非法 Nav2 速度")
            return
        reason = self._supervisor_reason(now) or self.state.active_reason(now)
        if reason:
            if self.state.armed:
                self._disarm(reason)
            return
        self.cmd_pub.publish(msg)
        self.last_command_time = now
        self.motion_command_active = not self._is_zero_twist(msg)

    def _set_armed_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        now = time.monotonic()
        if request.data:
            supervisor_reason = self._supervisor_reason(now)
            if supervisor_reason:
                self._disarm(supervisor_reason)
                success, message = False, supervisor_reason
            else:
                success, message = self.state.request_arm(now)
            response.success = success
            response.message = message
            if success:
                self.last_command_time = now
                self.motion_command_active = False
                self._publish_status(
                    f"{message}，最长 "
                    f"{self.state.config.max_armed_duration_sec:.1f} 秒"
                )
            else:
                self._publish_status(f"拒绝解锁：{message}")
        else:
            self._disarm("收到人工锁定请求")
            response.success = True
            response.message = "导航安全门已锁定"
        self._publish_outputs()
        return response

    def _watchdog_callback(self) -> None:
        now = time.monotonic()
        if self.state.armed:
            reason = self._supervisor_reason(now) or self.state.active_reason(now)
            if reason:
                self._disarm(reason)
            elif (
                self.motion_command_active
                and now - self.last_command_time > self.command_timeout_sec
            ):
                self.cmd_pub.publish(Twist())
                self.motion_command_active = False
                self._publish_status("Nav2 速度超时，已转发零速度")
        self._publish_outputs()

    def _publish_outputs(self) -> None:
        self.heartbeat_pub.publish(Bool(data=self.state.armed))
        self.armed_pub.publish(Bool(data=self.state.armed))

    def _publish_status(self, text: str) -> None:
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text
        self.status_pub.publish(String(data=text))

    def _disarm(self, reason: str) -> None:
        was_armed = self.state.armed
        self.state.disarm()
        self.cmd_pub.publish(Twist())
        self.heartbeat_pub.publish(Bool(data=False))
        self.armed_pub.publish(Bool(data=False))
        self.motion_command_active = False
        prefix = "导航安全门自动锁定" if was_armed else "导航安全门保持锁定"
        self._publish_status(f"{prefix}：{reason}")

    def shutdown(self) -> None:
        """节点退出时发送零速度并撤销心跳."""
        self._disarm("节点退出")


def main(args=None) -> None:
    """运行人工点击导航速度安全门."""
    rclpy.init(args=args)
    node = NavigationCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
