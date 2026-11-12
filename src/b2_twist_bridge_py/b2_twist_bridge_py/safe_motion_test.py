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
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

from .heading_control import (
    HeadingHoldConfig,
    quaternion_wxyz_to_yaw,
    wrap_angle,
)
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
航向保持默认关闭，只有显式设置 yaw_hold_enabled:=true 才会工作。
"""


class SafeMotionTest(Node):
    """只在操作者按键后发出一段有限时长运动的测试节点."""

    def __init__(self) -> None:
        super().__init__("b2_safe_motion_test")
        self.declare_parameter("request_topic", "/api/sport/request")
        self.declare_parameter("response_topic", "/api/sport/response")
        self.declare_parameter("sport_state_topic", "/lf/sportmodestate")
        # 默认值来自 2026-09-08/09 B2 实机测试。
        # 横移在 0.08～0.25 m/s 时无明确位移，0.30 m/s 左右各 3/3 成功。
        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("lateral_speed", 0.30)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("motion_pulse_sec", 0.60)
        # 横移成功样本使用约 1.00 秒；与前后、转向脉冲分开，避免扩大其它动作。
        self.declare_parameter("lateral_pulse_sec", 1.00)
        # 航向保持在完成符号和短脉冲实机验收前保持默认关闭。
        self.declare_parameter("yaw_hold_enabled", False)
        self.declare_parameter("yaw_kp", 0.60)
        self.declare_parameter("yaw_deadband_rad", math.radians(2.0))
        self.declare_parameter("yaw_max_correction", 0.10)
        self.declare_parameter("yaw_state_timeout_sec", 0.30)
        self.declare_parameter("yaw_status_log_sec", 0.0)
        self.declare_parameter("response_log_interval_sec", 0.50)

        request_topic = str(self.get_parameter("request_topic").value)
        response_topic = str(self.get_parameter("response_topic").value)
        sport_state_topic = str(self.get_parameter("sport_state_topic").value)
        self.pub = self.create_publisher(Request, request_topic, 10)
        self.create_subscription(
            Response, response_topic, self._response_callback, 10
        )
        self.create_subscription(
            SportModeState,
            sport_state_topic,
            self._sport_state_callback,
            qos_profile_sensor_data,
        )
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.lateral_speed = float(self.get_parameter("lateral_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.motion_pulse_sec = float(
            self.get_parameter("motion_pulse_sec").value
        )
        self.lateral_pulse_sec = float(
            self.get_parameter("lateral_pulse_sec").value
        )
        self.yaw_hold_enabled = bool(
            self.get_parameter("yaw_hold_enabled").value
        )
        self.yaw_state_timeout_sec = float(
            self.get_parameter("yaw_state_timeout_sec").value
        )
        self.yaw_status_log_sec = float(
            self.get_parameter("yaw_status_log_sec").value
        )
        self.response_log_interval_sec = float(
            self.get_parameter("response_log_interval_sec").value
        )
        try:
            self.heading_hold = HeadingHoldConfig(
                kp=float(self.get_parameter("yaw_kp").value),
                deadband_rad=float(
                    self.get_parameter("yaw_deadband_rad").value
                ),
                max_correction_rad_s=float(
                    self.get_parameter("yaw_max_correction").value
                ),
            )
        except ValueError as exc:
            raise RuntimeError(f"航向保持参数无效：{exc}") from exc
        self._validate_parameters()
        self.armed = False
        self.motion_deadline: float | None = None
        self.current_yaw: float | None = None
        self.reported_rpy_yaw: float | None = None
        self.current_yaw_speed: float | None = None
        self.last_yaw_time = 0.0
        self.motion_start_yaw: float | None = None
        self.yaw_target: float | None = None
        self.active_translation: tuple[float, float] | None = None
        self.max_abs_yaw_error = 0.0
        self.max_abs_yaw_correction = 0.0
        self.last_response_log_time = 0.0
        self.last_response_api_id: int | None = None
        self.create_timer(0.05, self._motion_watchdog)
        if self.yaw_status_log_sec > 0.0:
            self.create_timer(
                self.yaw_status_log_sec, self._yaw_status_callback
            )
        self.get_logger().info(f"向 {request_topic} 发送 B2 Sport 测试请求")
        self.get_logger().info(
            f"姿态输入 {sport_state_topic}；航向保持="
            f"{'开启（待实机验证）' if self.yaw_hold_enabled else '关闭'}"
        )

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
        if (
            not math.isfinite(self.yaw_state_timeout_sec)
            or not 0.10 <= self.yaw_state_timeout_sec <= 1.0
        ):
            raise RuntimeError("yaw_state_timeout_sec 必须位于 0.10 到 1.00 秒")
        log_intervals = (
            self.yaw_status_log_sec,
            self.response_log_interval_sec,
        )
        if not all(
            math.isfinite(value) and value >= 0.0 for value in log_intervals
        ):
            raise RuntimeError("航向和响应日志周期必须是非负有限数值")

    def publish_api(
        self,
        name: str,
        parameter: dict | None = None,
        log_request: bool = True,
    ) -> bool:
        """发布白名单动作；锁定时只允许停车，并返回是否已发布."""
        if name != "STOPMOVE" and not self.armed:
            self.get_logger().warn("测试仍处于锁定状态，请先按 i 解锁")
            return False
        request = Request()
        request.header.identity.api_id = api_id(name)
        if parameter is not None:
            request.parameter = json.dumps(parameter, separators=(",", ":"))
        self.pub.publish(request)
        if log_request:
            self.get_logger().info(f"已发送 {name}：{parameter or '无参数'}")
        return True

    def move(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        pulse_sec: float | None = None,
    ) -> None:
        """默认发送一次 MOVE；显式开启航向保持后由 watchdog 更新修正。"""
        self._clear_motion_tracking()
        translation = (x != 0.0 or y != 0.0) and z == 0.0
        if translation and self._yaw_state_fresh():
            self.motion_start_yaw = self.current_yaw
        if translation and self.yaw_hold_enabled:
            if not self._yaw_state_fresh():
                self.get_logger().error(
                    "航向保持已开启，但姿态数据不可用或超时，拒绝平移并停车"
                )
                self.stop()
                return
            self.yaw_target = self.current_yaw
            self.active_translation = (float(x), float(y))
            self.get_logger().info(
                f"记录航向目标 {math.degrees(self.yaw_target):.2f} 度；"
                "闭环参数尚待实机验证"
            )
        if self.publish_api("MOVE", {"x": x, "y": y, "z": z}):
            duration = (
                self.motion_pulse_sec if pulse_sec is None else pulse_sec
            )
            self.motion_deadline = time.monotonic() + duration
        else:
            self._clear_motion_tracking()

    def stop(self) -> None:
        was_moving = self.motion_deadline is not None
        start_yaw = self.motion_start_yaw
        final_yaw = self.current_yaw if self._yaw_state_fresh() else None
        hold_was_active = self.yaw_target is not None
        max_error = self.max_abs_yaw_error
        max_correction = self.max_abs_yaw_correction
        self.motion_deadline = None
        self._clear_motion_tracking()
        self.publish_api("STOPMOVE")
        if was_moving and start_yaw is not None and final_yaw is not None:
            yaw_change = wrap_angle(final_yaw - start_yaw)
            summary = f"本次平移航向变化={math.degrees(yaw_change):.2f} 度"
            if hold_was_active:
                summary += (
                    f"，闭环期间最大误差={math.degrees(max_error):.2f} 度，"
                    f"最大修正={max_correction:.3f} rad/s"
                )
            else:
                summary += "，航向保持关闭"
            self.get_logger().info(summary)

    def set_armed(self, armed: bool) -> None:
        """切换测试解锁状态；重新锁定时立即停车."""
        self.armed = bool(armed)
        if not self.armed:
            self.stop()
            self.get_logger().info("测试已锁定并发送停车指令")
        else:
            self.get_logger().warn("测试已解锁，只能在有人看护时操作")

    def _motion_watchdog(self) -> None:
        if self.motion_deadline is None:
            return
        now = time.monotonic()
        if now >= self.motion_deadline:
            self.stop()
            self.get_logger().info("短运动时间结束，已自动停车")
            return
        if self.active_translation is None or self.yaw_target is None:
            return
        if not self._yaw_state_fresh(now):
            self.get_logger().error("航向姿态数据超时，已提前停车")
            self.stop()
            return
        error, correction = self.heading_hold.correction(
            self.yaw_target, self.current_yaw
        )
        self.max_abs_yaw_error = max(self.max_abs_yaw_error, abs(error))
        self.max_abs_yaw_correction = max(
            self.max_abs_yaw_correction, abs(correction)
        )
        x, y = self.active_translation
        self.publish_api(
            "MOVE",
            {"x": x, "y": y, "z": correction},
            log_request=False,
        )

    def _sport_state_callback(self, msg: SportModeState) -> None:
        """读取真实 B2 姿态；Unitree 四元数顺序为 w、x、y、z。"""
        if int(msg.error_code) != 0:
            self.current_yaw = None
            return
        try:
            yaw = quaternion_wxyz_to_yaw(msg.imu_state.quaternion)
        except ValueError as exc:
            self.current_yaw = None
            self.get_logger().error(f"拒绝非法 IMU 四元数：{exc}")
            return
        self.current_yaw = yaw
        self.last_yaw_time = time.monotonic()
        yaw_speed = float(msg.yaw_speed)
        self.current_yaw_speed = (
            yaw_speed if math.isfinite(yaw_speed) else None
        )
        if len(msg.imu_state.rpy) >= 3:
            reported = float(msg.imu_state.rpy[2])
            self.reported_rpy_yaw = (
                reported if math.isfinite(reported) else None
            )

    def _yaw_state_fresh(self, now: float | None = None) -> bool:
        """判断航向数据是否存在且没有超过安全时限。"""
        current = time.monotonic() if now is None else now
        return (
            self.current_yaw is not None
            and self.last_yaw_time > 0.0
            and current - self.last_yaw_time <= self.yaw_state_timeout_sec
        )

    def _yaw_status_callback(self) -> None:
        """低频打印四元数航向和消息自带 rpy，供实机核对符号。"""
        if not self._yaw_state_fresh():
            self.get_logger().warn("航向状态不可用或已超时")
            return
        parts = [f"quaternion_yaw={math.degrees(self.current_yaw):.2f} 度"]
        if self.reported_rpy_yaw is not None:
            difference = wrap_angle(self.current_yaw - self.reported_rpy_yaw)
            parts.append(
                f"rpy_yaw={math.degrees(self.reported_rpy_yaw):.2f} 度"
            )
            parts.append(f"差值={math.degrees(difference):.2f} 度")
        if self.current_yaw_speed is not None:
            parts.append(f"yaw_speed={self.current_yaw_speed:.3f} rad/s")
        self.get_logger().info("，".join(parts))

    def _clear_motion_tracking(self) -> None:
        """清除一次短脉冲对应的航向目标和诊断统计。"""
        self.motion_start_yaw = None
        self.yaw_target = None
        self.active_translation = None
        self.max_abs_yaw_error = 0.0
        self.max_abs_yaw_correction = 0.0

    def _response_callback(self, msg: Response) -> None:
        response_api_id = msg.header.identity.api_id
        now = time.monotonic()
        should_log = (
            self.response_log_interval_sec == 0.0
            or response_api_id != self.last_response_api_id
            or now - self.last_response_log_time
            >= self.response_log_interval_sec
        )
        if should_log:
            self.get_logger().info(
                f"收到 B2 响应：api_id={response_api_id}，data={msg.data}"
            )
            self.last_response_log_time = now
            self.last_response_api_id = response_api_id


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
                node.move(
                    y=node.lateral_speed,
                    pulse_sec=node.lateral_pulse_sec,
                )
            elif key == "d":
                node.move(
                    y=-node.lateral_speed,
                    pulse_sec=node.lateral_pulse_sec,
                )
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
