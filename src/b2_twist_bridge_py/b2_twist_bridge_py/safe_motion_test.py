#!/usr/bin/env python3
"""宇树 B2 首次上机使用的低速、短脉冲键盘测试程序."""

import json
import math
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

from .sport_model import api_id


HELP = """
宇树 B2 有监督安全测试

  i 解锁/重新锁定测试（启动后默认锁定）

移动键（每按一次只运动很短时间，随后自动停车）：
  w 前进      s 后退
  a 左移      d 右移
  q 左转      e 右转
  x 立即停车

姿态键：
  u 站起      j 趴下/蹲下
  r 恢复站立  b 平衡站立

速度键：
  + 提高测试速度    - 降低测试速度

Ctrl-C：发送停车指令并退出。
实机测试时必须有人手持遥控器或急停装置。
"""


class SafeMotionTest(Node):
    """只在操作者按键后发出一段有限时长运动的测试节点."""

    def __init__(self) -> None:
        super().__init__("b2_safe_motion_test")
        self.declare_parameter("request_topic", "/api/sport/request")
        self.declare_parameter("response_topic", "/api/sport/response")
        # 默认值来自 2026-09-08/09 B2 实机测试。
        # 横移在 0.08～0.25 m/s 时无明确位移，0.30 m/s 左右各 3/3 成功。
        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("lateral_speed", 0.30)
        self.declare_parameter("angular_speed", 0.20)
        self.declare_parameter("motion_pulse_sec", 0.60)
        # 横移成功样本使用约 1.00 秒；与前后、转向脉冲分开，避免扩大其它动作。
        self.declare_parameter("lateral_pulse_sec", 1.00)

        request_topic = str(self.get_parameter("request_topic").value)
        response_topic = str(self.get_parameter("response_topic").value)
        self.pub = self.create_publisher(Request, request_topic, 10)
        self.create_subscription(Response, response_topic, self._response_callback, 10)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.lateral_speed = float(self.get_parameter("lateral_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.motion_pulse_sec = float(self.get_parameter("motion_pulse_sec").value)
        self.lateral_pulse_sec = float(
            self.get_parameter("lateral_pulse_sec").value
        )
        self._validate_parameters()
        self.armed = False
        self.motion_deadline: float | None = None
        self.create_timer(0.05, self._motion_watchdog)
        self.get_logger().info(f"向 {request_topic} 发送 B2 Sport 测试请求")

    def _validate_parameters(self) -> None:
        """拒绝可能导致首次实机测试幅度过大的启动参数."""
        limits = (
            ("linear_speed", self.linear_speed, 0.30),
            ("lateral_speed", self.lateral_speed, 0.30),
            ("angular_speed", self.angular_speed, 0.35),
        )
        for name, value, maximum in limits:
            if not math.isfinite(value) or not 0.0 < value <= maximum:
                raise RuntimeError(
                    f"{name} 必须是大于零且不超过 {maximum:.2f} 的有限数值"
                )
        pulses = (
            ("motion_pulse_sec", self.motion_pulse_sec),
            ("lateral_pulse_sec", self.lateral_pulse_sec),
        )
        for name, value in pulses:
            if not math.isfinite(value) or not 0.05 <= value <= 1.0:
                raise RuntimeError(f"{name} 必须位于 0.05 到 1.00 秒之间")

    def publish_api(self, name: str, parameter: dict | None = None) -> bool:
        """发布白名单动作；锁定时只允许停车，并返回是否已发布."""
        if name != "STOPMOVE" and not self.armed:
            self.get_logger().warn("测试仍处于锁定状态，请先按 i 解锁")
            return False
        request = Request()
        request.header.identity.api_id = api_id(name)
        if parameter is not None:
            request.parameter = json.dumps(parameter, separators=(",", ":"))
        self.pub.publish(request)
        self.get_logger().info(f"已发送 {name}：{parameter or '无参数'}")
        return True

    def move(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        pulse_sec: float | None = None,
    ) -> None:
        """只发送一次 MOVE，并登记由 watchdog 执行的停车截止时间。"""
        if self.publish_api("MOVE", {"x": x, "y": y, "z": z}):
            duration = self.motion_pulse_sec if pulse_sec is None else pulse_sec
            self.motion_deadline = time.monotonic() + duration

    def stop(self) -> None:
        self.motion_deadline = None
        self.publish_api("STOPMOVE")

    def set_armed(self, armed: bool) -> None:
        """切换测试解锁状态；重新锁定时立即停车."""
        self.armed = bool(armed)
        if not self.armed:
            self.stop()
            self.get_logger().info("测试已锁定并发送停车指令")
        else:
            self.get_logger().warn("测试已解锁，只能在有人看护时操作")

    def _motion_watchdog(self) -> None:
        if self.motion_deadline is not None and time.monotonic() >= self.motion_deadline:
            self.stop()
            self.get_logger().info("短运动时间结束，已自动停车")

    def _response_callback(self, msg: Response) -> None:
        api_id = msg.header.identity.api_id
        self.get_logger().info(f"收到 B2 响应：api_id={api_id}，data={msg.data}")


def get_key(settings):
    """读取一个按键，并立刻恢复终端设置."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print("错误：安全运动测试必须在可交互终端中运行。")
        return
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = SafeMotionTest()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == "\x03":
                break
            if key == "i":
                node.set_armed(not node.armed)
            elif key == "w":
                node.move(x=node.linear_speed)
            elif key == "s":
                node.move(x=-node.linear_speed)
            elif key == "a":
                node.move(y=node.lateral_speed, pulse_sec=node.lateral_pulse_sec)
            elif key == "d":
                node.move(y=-node.lateral_speed, pulse_sec=node.lateral_pulse_sec)
            elif key == "q":
                node.move(z=node.angular_speed)
            elif key == "e":
                node.move(z=-node.angular_speed)
            elif key == "x":
                node.stop()
            elif key == "u":
                node.publish_api("STANDUP")
            elif key == "j":
                node.publish_api("STANDDOWN")
            elif key == "r":
                node.publish_api("RECOVERYSTAND")
            elif key == "b":
                node.publish_api("BALANCESTAND")
            elif key == "+":
                node.linear_speed = min(0.30, node.linear_speed * 1.10)
                node.lateral_speed = min(0.30, node.lateral_speed * 1.10)
                node.angular_speed = min(0.35, node.angular_speed * 1.10)
                print(
                    f"当前速度：前后={node.linear_speed:.2f}，"
                    f"侧向={node.lateral_speed:.2f}，"
                    f"转向={node.angular_speed:.2f}"
                )
            elif key == "-":
                node.linear_speed = max(0.04, node.linear_speed * 0.90)
                node.lateral_speed = max(0.03, node.lateral_speed * 0.90)
                node.angular_speed = max(0.08, node.angular_speed * 0.90)
                print(
                    f"当前速度：前后={node.linear_speed:.2f}，"
                    f"侧向={node.lateral_speed:.2f}，"
                    f"转向={node.angular_speed:.2f}"
                )
            else:
                node.stop()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
