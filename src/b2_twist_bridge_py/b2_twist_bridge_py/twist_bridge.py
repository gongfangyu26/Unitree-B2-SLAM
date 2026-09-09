#!/usr/bin/env python3
"""把 ROS2 速度指令安全地转换为宇树 B2 Sport 请求."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from unitree_api.msg import Request

from .sport_model import api_id
from .velocity_policy import VelocityPolicy


class SafeTwistBridge(Node):
    """带启停锁、限速和超时停车的 Python 速度桥."""

    def __init__(self) -> None:
        super().__init__("b2_safe_twist_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("request_topic", "/api/sport/request")
        # 2026-09-08/09 实机确认前后和横移均需 0.30 m/s 才可靠形成位移。
        self.declare_parameter("max_linear_x", 0.30)
        self.declare_parameter("max_linear_y", 0.30)
        self.declare_parameter("max_angular_z", 0.20)
        self.declare_parameter("min_effective_linear_x", 0.30)
        self.declare_parameter("min_effective_linear_y", 0.30)
        self.declare_parameter("deadband", 0.005)
        # 方向已经完成单轴实机验收，但节点上电仍锁定，不能自动开始运动。
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("allow_lateral", True)
        self.declare_parameter("allow_combined_translation", False)
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("heartbeat_timeout_sec", 0.60)
        self.declare_parameter("state_timeout_sec", 0.60)
        self.declare_parameter("autonomy_timeout_sec", 0.60)
        self.declare_parameter("require_autonomy_state", True)
        self.declare_parameter("stop_repetition", 3)

        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.min_effective_linear_x = float(
            self.get_parameter("min_effective_linear_x").value
        )
        self.min_effective_linear_y = float(
            self.get_parameter("min_effective_linear_y").value
        )
        self.deadband = float(self.get_parameter("deadband").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.allow_lateral = bool(self.get_parameter("allow_lateral").value)
        self.allow_combined_translation = bool(
            self.get_parameter("allow_combined_translation").value
        )
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        requested_start_enabled = bool(self.get_parameter("start_enabled").value)
        # 上电后始终保持锁定，必须由任务管理器在安全条件齐备后显式解锁。
        self.enabled = False
        self.heartbeat_timeout_sec = float(
            self.get_parameter("heartbeat_timeout_sec").value
        )
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.autonomy_timeout_sec = float(
            self.get_parameter("autonomy_timeout_sec").value
        )
        self.require_autonomy_state = bool(
            self.get_parameter("require_autonomy_state").value
        )
        self.stop_repetition = max(
            1, min(5, int(self.get_parameter("stop_repetition").value))
        )
        try:
            self.velocity_policy = VelocityPolicy(
                max_linear_x=self.max_linear_x,
                max_linear_y=self.max_linear_y,
                max_angular_z=self.max_angular_z,
                min_effective_linear_x=self.min_effective_linear_x,
                min_effective_linear_y=self.min_effective_linear_y,
                deadband=self.deadband,
                allow_reverse=self.allow_reverse,
                allow_lateral=self.allow_lateral,
                allow_combined_translation=self.allow_combined_translation,
            )
        except ValueError as exc:
            raise RuntimeError(f"速度策略参数无效：{exc}") from exc
        positive_values = (
            self.command_timeout_sec,
            self.heartbeat_timeout_sec,
            self.state_timeout_sec,
            self.autonomy_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise RuntimeError("速度上限和超时参数必须是大于零的有限数值")
        self.last_command_time = time.monotonic()
        self.last_heartbeat_time = 0.0
        self.last_state_time = 0.0
        self.last_autonomy_time = 0.0
        self.robot_state_ok = False
        self.autonomy_ok = False
        self.stop_already_sent = False

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        request_topic = str(self.get_parameter("request_topic").value)
        self.request_pub = self.create_publisher(Request, request_topic, 10)
        self.status_pub = self.create_publisher(String, "/b2/control/status", 10)
        self.enabled_pub = self.create_publisher(Bool, "/b2/control/enabled", 10)
        self.create_subscription(Twist, cmd_topic, self._twist_callback, 10)
        self.create_subscription(
            Bool, "/inspection/control_heartbeat", self._heartbeat_callback, 10
        )
        self.create_subscription(Bool, "/b2/state_ok", self._state_callback, 10)
        self.create_subscription(
            Bool, "/b2/autonomy_ok", self._autonomy_callback, 10
        )
        self.create_service(SetBool, "/b2/control/set_enabled", self._enable_callback)
        self.create_timer(0.05, self._watchdog_callback)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"安全速度桥已启动：订阅 {cmd_topic}，发布 {request_topic}，"
            f"初始启用状态={self.enabled}"
        )
        if requested_start_enabled:
            self.get_logger().warn(
                "为避免节点启动瞬间误动，start_enabled 参数已被忽略；"
                "请通过 /b2/control/set_enabled 显式解锁"
            )

    def _publish_request(self, api_name: str, parameter: dict | None = None) -> None:
        request = Request()
        request.header.identity.api_id = api_id(api_name)
        if parameter is not None:
            request.parameter = json.dumps(parameter, separators=(",", ":"))
        self.request_pub.publish(request)

    def _publish_stop(self, reason: str, force: bool = False) -> None:
        """发送幂等停车请求；紧急情况重复发送以降低单包丢失风险."""
        if self.stop_already_sent and not force:
            return
        repetitions = self.stop_repetition if force else 1
        for _ in range(repetitions):
            self._publish_request("STOPMOVE")
        self.stop_already_sent = True
        self.status_pub.publish(String(data=f"已停车：{reason}"))

    def _safety_ready(self, now: float | None = None) -> tuple[bool, str]:
        """检查任务心跳、机器人状态和自主导航感知链是否新鲜."""
        current = time.monotonic() if now is None else now
        if current - self.last_heartbeat_time > self.heartbeat_timeout_sec:
            return False, "任务心跳丢失或超时"
        if current - self.last_state_time > self.state_timeout_sec:
            return False, "机器人状态丢失或超时"
        if not self.robot_state_ok:
            return False, "机器人状态无效"
        if self.require_autonomy_state:
            if current - self.last_autonomy_time > self.autonomy_timeout_sec:
                return False, "自主导航安全状态丢失或超时"
            if not self.autonomy_ok:
                return False, "雷达或定位链未就绪"
        return True, "安全条件已满足"

    def _twist_callback(self, msg: Twist) -> None:
        self.last_command_time = time.monotonic()
        if not self.enabled:
            self._publish_stop("控制锁未启用")
            return

        ready, reason = self._safety_ready()
        if not ready:
            self.enabled = False
            self._publish_stop(reason, force=True)
            self._publish_state()
            return

        try:
            x, y, z, notes = self.velocity_policy.shape(
                msg.linear.x, msg.linear.y, msg.angular.z
            )
        except ValueError:
            self.get_logger().error("收到 NaN 或无穷大速度，已拒绝并停车")
            self.enabled = False
            self._publish_stop("速度数据非法，控制锁已关闭", force=True)
            self._publish_state()
            return

        if notes:
            self.status_pub.publish(String(data="；".join(notes)))

        if x == 0.0 and y == 0.0 and z == 0.0:
            self._publish_stop("收到零速度")
            return

        self._publish_request("MOVE", {"x": x, "y": y, "z": z})
        self.stop_already_sent = False

    def _watchdog_callback(self) -> None:
        now = time.monotonic()
        if self.enabled:
            ready, reason = self._safety_ready(now)
            if not ready:
                self.enabled = False
                self._publish_stop(f"{reason}，控制锁自动关闭", force=True)
                self._publish_state()
                return
        elapsed = time.monotonic() - self.last_command_time
        if self.enabled and elapsed > self.command_timeout_sec and not self.stop_already_sent:
            self.get_logger().warn(
                f"{elapsed:.2f} 秒未收到新速度指令，触发超时停车"
            )
            self._publish_stop("速度指令超时")

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        requested = bool(request.data)
        self.last_command_time = time.monotonic()
        ready, reason = self._safety_ready()
        if requested and not ready:
            self.enabled = False
            self._publish_stop(f"启用条件不满足：{reason}", force=True)
            response.success = False
            response.message = f"{reason}，拒绝启用"
            self._publish_state()
            return response
        self.enabled = requested
        if self.enabled:
            # 解锁只代表允许接收后续速度；先明确停车，避免继承旧控制源的运动。
            self._publish_stop("控制锁启用前确认停车", force=True)
            self.stop_already_sent = True
            response.message = "控制锁已启用，等待速度指令"
        else:
            self._publish_stop("控制锁被关闭", force=True)
            response.message = "控制锁已关闭，已发送停车指令"
        response.success = True
        self._publish_state()
        return response

    def _heartbeat_callback(self, msg: Bool) -> None:
        if msg.data:
            self.last_heartbeat_time = time.monotonic()
        else:
            self.last_heartbeat_time = 0.0

    def _state_callback(self, msg: Bool) -> None:
        self.robot_state_ok = bool(msg.data)
        self.last_state_time = time.monotonic()

    def _autonomy_callback(self, msg: Bool) -> None:
        self.autonomy_ok = bool(msg.data)
        self.last_autonomy_time = time.monotonic()

    def _publish_state(self) -> None:
        self.enabled_pub.publish(Bool(data=self.enabled))
        state = "已启用" if self.enabled else "已锁定"
        self.status_pub.publish(String(data=f"控制状态：{state}"))

    def shutdown(self) -> None:
        self.enabled = False
        self._publish_stop("节点退出", force=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeTwistBridge()
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
