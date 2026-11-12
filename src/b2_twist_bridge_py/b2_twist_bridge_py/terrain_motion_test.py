#!/usr/bin/env python3
"""Unitree B2 坡道和楼梯有监督、短脉冲实机测试节点."""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
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

from .sport_model import api_id
from .terrain_safety import TerrainSafetyLimits, quaternion_wxyz_to_rpy


PHASES = {
    "1": "slope_up",
    "2": "slope_down",
    "3": "stairs_up",
    "4": "stairs_down",
}

HELP = """
Unitree B2 坡道/楼梯有监督点动测试（启动后默认锁定）

  1 上坡    2 下坡    3 上楼梯    4 下楼梯
    选择/切换阶段会立即停车并重新锁定。

  i 解锁确认（需在 3 秒内连按两次）
  w 向前一次短脉冲（结束后自动锁定）
  s 向后一次短脉冲（结束后自动锁定）
  x 立即停车并锁定

  p 打印当前 mode/gait_type/姿态
  h 重新显示帮助
  Ctrl-C 停车、锁定并退出

注意：本程序不猜测、不自动切换未确认的 B2 爬楼梯步态。
先由现场操作者用原厂方式选择步态，用 p 核对真实 gait_type。
必须双人看护、手持遥控/急停，并有防跌落措施。
"""


class TerrainMotionTest(Node):
    """仅发送单次 MOVE，并在超时或姿态异常时停车锁定."""

    def __init__(self) -> None:
        super().__init__("b2_terrain_motion_test")
        self.declare_parameter("request_topic", "/api/sport/request")
        self.declare_parameter("response_topic", "/api/sport/response")
        self.declare_parameter("sport_state_topic", "/lf/sportmodestate")
        self.declare_parameter("forward_speed", 0.30)
        self.declare_parameter("reverse_speed", 0.30)
        self.declare_parameter("motion_pulse_sec", 0.50)
        self.declare_parameter("state_timeout_sec", 0.30)
        self.declare_parameter("max_abs_roll_deg", 22.0)
        self.declare_parameter("max_abs_pitch_deg", 35.0)
        self.declare_parameter("expected_gait_type", -1)
        self.declare_parameter("log_directory", "~/b2_terrain_logs")
        self.declare_parameter("test_label", "terrain_manual")

        self.forward_speed = float(
            self.get_parameter("forward_speed").value
        )
        self.reverse_speed = float(
            self.get_parameter("reverse_speed").value
        )
        self.motion_pulse_sec = float(
            self.get_parameter("motion_pulse_sec").value
        )
        self._validate_motion_parameters()
        try:
            self.safety_limits = TerrainSafetyLimits(
                max_abs_roll_rad=math.radians(
                    float(self.get_parameter("max_abs_roll_deg").value)
                ),
                max_abs_pitch_rad=math.radians(
                    float(self.get_parameter("max_abs_pitch_deg").value)
                ),
                state_timeout_sec=float(
                    self.get_parameter("state_timeout_sec").value
                ),
                expected_gait_type=int(
                    self.get_parameter("expected_gait_type").value
                ),
            )
        except ValueError as exc:
            raise RuntimeError(f"地形安全参数无效：{exc}") from exc

        request_topic = str(self.get_parameter("request_topic").value)
        response_topic = str(self.get_parameter("response_topic").value)
        sport_state_topic = str(
            self.get_parameter("sport_state_topic").value
        )
        self.request_pub = self.create_publisher(Request, request_topic, 10)
        self.create_subscription(
            Response, response_topic, self._response_callback, 10
        )
        self.create_subscription(
            SportModeState,
            sport_state_topic,
            self._state_callback,
            qos_profile_sensor_data,
        )

        self._lock = threading.RLock()
        self.armed = False
        self.arm_confirmation_deadline = 0.0
        self.phase = "unset"
        self.motion_deadline: float | None = None
        self.last_state_time = 0.0
        self.last_state: dict | None = None
        self.last_fault = ""
        self.csv_file, self.csv_writer, self.log_path = self._open_log()
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            f"地形点动测试已启动，日志：{self.log_path}"
        )
        self.get_logger().warn(
            "程序默认锁定；未确认 B2 官方步态编号，不会自动切换步态"
        )

    def _validate_motion_parameters(self) -> None:
        speeds = (self.forward_speed, self.reverse_speed)
        if not all(
            math.isfinite(value) and 0.0 < value <= 0.30
            for value in speeds
        ):
            raise RuntimeError("前进/后退速度必须大于零且不超过 0.30 m/s")
        if (
            not math.isfinite(self.motion_pulse_sec)
            or not 0.20 <= self.motion_pulse_sec <= 1.00
        ):
            raise RuntimeError("单次点动时间必须位于 0.20 到 1.00 秒")

    def _open_log(self):
        directory = Path(
            str(self.get_parameter("log_directory").value)
        ).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        label = str(self.get_parameter("test_label").value).strip()
        safe_label = "".join(
            char if char.isalnum() or char in "-_" else "_"
            for char in label
        ).strip("_") or "terrain_manual"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = directory / f"b2_{safe_label}_{stamp}.csv"
        handle = path.open("x", newline="", encoding="utf-8")
        fields = [
            "host_time_iso",
            "monotonic_sec",
            "event",
            "phase",
            "armed",
            "motion_active",
            "error_code",
            "mode",
            "gait_type",
            "progress",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "position_x",
            "position_y",
            "position_z",
            "velocity_x",
            "velocity_y",
            "velocity_z",
            "yaw_speed_rad_s",
            "foot_force_0",
            "foot_force_1",
            "foot_force_2",
            "foot_force_3",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        handle.flush()
        return handle, writer, path

    def _state_callback(self, msg: SportModeState) -> None:
        now = time.monotonic()
        try:
            roll, pitch, yaw = quaternion_wxyz_to_rpy(
                msg.imu_state.quaternion
            )
        except ValueError as exc:
            self._fault_stop(f"非法 IMU 四元数：{exc}")
            return

        position = tuple(float(value) for value in msg.position[:3])
        velocity = tuple(float(value) for value in msg.velocity[:3])
        foot_force = tuple(int(value) for value in msg.foot_force[:4])
        if len(position) != 3 or len(velocity) != 3 or len(foot_force) != 4:
            self._fault_stop("SportModeState 数组长度不足")
            return
        numeric = position + velocity + (float(msg.yaw_speed),)
        if not all(math.isfinite(value) for value in numeric):
            self._fault_stop("SportModeState 包含 NaN 或无穷大")
            return

        state = {
            "error_code": int(msg.error_code),
            "mode": int(msg.mode),
            "gait_type": int(msg.gait_type),
            "progress": float(msg.progress),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "position": position,
            "velocity": velocity,
            "yaw_speed": float(msg.yaw_speed),
            "foot_force": foot_force,
        }
        with self._lock:
            self.last_state_time = now
            self.last_state = state
            self._write_log("state", now=now)
            reasons = self.safety_limits.reasons(
                roll_rad=roll,
                pitch_rad=pitch,
                state_age_sec=0.0,
                error_code=state["error_code"],
                gait_type=state["gait_type"],
            )
        if reasons:
            self._fault_stop("；".join(reasons))

    def _state_reasons(self, now: float) -> list[str]:
        with self._lock:
            if self.last_state is None or self.last_state_time <= 0.0:
                return ["尚未收到 SportModeState"]
            state = self.last_state
            age = now - self.last_state_time
            return self.safety_limits.reasons(
                roll_rad=state["roll"],
                pitch_rad=state["pitch"],
                state_age_sec=age,
                error_code=state["error_code"],
                gait_type=state["gait_type"],
            )

    def select_phase(self, phase: str) -> None:
        if phase not in PHASES.values():
            raise ValueError(f"未知测试阶段：{phase}")
        self.stop_and_lock(f"切换测试阶段为 {phase}")
        with self._lock:
            self.phase = phase
            self._write_log("phase_selected")
        self.get_logger().warn(
            f"已选择 {phase}；请先核对场地和真实 gait_type，再双击 i 解锁"
        )

    def request_arm(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self.armed:
                self.stop_and_lock("操作者手动重新锁定")
                return
            if self.phase == "unset":
                self.get_logger().error("请先按 1/2/3/4 选择测试阶段")
                return
            reasons = self._state_reasons(now)
            if reasons:
                self.get_logger().error(
                    "不允许解锁：" + "；".join(reasons)
                )
                self.arm_confirmation_deadline = 0.0
                return
            if now > self.arm_confirmation_deadline:
                self.arm_confirmation_deadline = now + 3.0
                self.get_logger().warn(
                    "解锁待确认：检查双人看护、急停、防跌落和前后净空，3 秒内再按 i"
                )
                return
            self.armed = True
            self.arm_confirmation_deadline = 0.0
            self._write_log("armed", now=now)
        self.get_logger().warn(
            f"已解锁 {self.phase}；每次按键最多运动 {self.motion_pulse_sec:.2f} 秒"
        )

    def move(self, velocity_x: float) -> None:
        now = time.monotonic()
        with self._lock:
            if not self.armed:
                self.get_logger().warn("测试已锁定，拒绝运动")
                return
            if self.motion_deadline is not None:
                self.get_logger().warn("上一次点动尚未结束，拒绝叠加运动命令")
                return
            reasons = self._state_reasons(now)
            if reasons:
                self._fault_stop("发送运动前检查失败：" + "；".join(reasons))
                return
            request = Request()
            request.header.identity.api_id = api_id("MOVE")
            request.parameter = json.dumps(
                {"x": float(velocity_x), "y": 0.0, "z": 0.0},
                separators=(",", ":"),
            )
            self.request_pub.publish(request)
            self.motion_deadline = now + self.motion_pulse_sec
            self._write_log(
                "move_forward" if velocity_x > 0.0 else "move_reverse",
                now=now,
            )
        self.get_logger().warn(
            f"已发送单次 MOVE x={velocity_x:.2f} m/s，"
            f"{self.motion_pulse_sec:.2f} 秒后自动停车"
        )

    def _publish_stop(self) -> bool:
        if not rclpy.ok():
            self.get_logger().error("ROS2 context 已关闭，无法发送 STOPMOVE")
            return False
        request = Request()
        request.header.identity.api_id = api_id("STOPMOVE")
        try:
            self.request_pub.publish(request)
        except Exception as exc:  # ROS2 失效时只能现场急停。
            self.get_logger().error(f"STOPMOVE 发送失败：{exc}")
            return False
        return True

    def stop_and_lock(self, reason: str) -> None:
        with self._lock:
            self.motion_deadline = None
            self.armed = False
            self.arm_confirmation_deadline = 0.0
            stop_sent = self._publish_stop()
            self._write_log("stop_and_lock")
        if stop_sent:
            self.get_logger().warn(f"{reason}；已发送 STOPMOVE 并锁定")
        else:
            self.get_logger().error(
                f"{reason}；软件已锁定但 STOPMOVE 未能发送，"
                "必须立即使用遥控/急停"
            )

    def _fault_stop(self, reason: str) -> None:
        with self._lock:
            active = self.armed or self.motion_deadline is not None
            changed = reason != self.last_fault
            self.last_fault = reason
        if active:
            self.stop_and_lock(f"安全联锁触发：{reason}")
        elif changed:
            self.get_logger().error(f"安全状态不通过：{reason}")

    def _watchdog(self) -> None:
        now = time.monotonic()
        with self._lock:
            moving = self.motion_deadline is not None
            armed = self.armed
            expired = moving and now >= self.motion_deadline
        if expired:
            with self._lock:
                self.motion_deadline = None
                self.armed = False
                self.arm_confirmation_deadline = 0.0
                stop_sent = self._publish_stop()
                self._write_log("pulse_timeout_stop", now=now)
            if stop_sent:
                self.get_logger().warn(
                    "单次点动结束，已自动发送 STOPMOVE 并锁定"
                )
            else:
                self.get_logger().error(
                    "单次点动到期但 STOPMOVE 发送失败，"
                    "必须立即使用遥控/急停"
                )
            return
        if armed or moving:
            reasons = self._state_reasons(now)
            if reasons:
                self._fault_stop("；".join(reasons))

    def print_state(self) -> None:
        with self._lock:
            if self.last_state is None:
                self.get_logger().warn("尚未收到 SportModeState")
                return
            state = self.last_state
            age = time.monotonic() - self.last_state_time
            self.get_logger().info(
                f"phase={self.phase}, armed={self.armed}, "
                f"mode={state['mode']}, gait_type={state['gait_type']}, "
                f"roll={math.degrees(state['roll']):.1f}°, "
                f"pitch={math.degrees(state['pitch']):.1f}°, "
                f"yaw={math.degrees(state['yaw']):.1f}°, "
                f"state_age={age:.3f}s"
            )

    def _write_log(self, event: str, now: float | None = None) -> None:
        current = time.monotonic() if now is None else now
        state = self.last_state or {}
        position = state.get("position", ("", "", ""))
        velocity = state.get("velocity", ("", "", ""))
        forces = state.get("foot_force", ("", "", "", ""))
        row = {
            "host_time_iso": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "monotonic_sec": f"{current:.6f}",
            "event": event,
            "phase": self.phase,
            "armed": self.armed,
            "motion_active": self.motion_deadline is not None,
            "error_code": state.get("error_code", ""),
            "mode": state.get("mode", ""),
            "gait_type": state.get("gait_type", ""),
            "progress": state.get("progress", ""),
            "roll_deg": self._angle_or_empty(state.get("roll")),
            "pitch_deg": self._angle_or_empty(state.get("pitch")),
            "yaw_deg": self._angle_or_empty(state.get("yaw")),
            "position_x": position[0],
            "position_y": position[1],
            "position_z": position[2],
            "velocity_x": velocity[0],
            "velocity_y": velocity[1],
            "velocity_z": velocity[2],
            "yaw_speed_rad_s": state.get("yaw_speed", ""),
            "foot_force_0": forces[0],
            "foot_force_1": forces[1],
            "foot_force_2": forces[2],
            "foot_force_3": forces[3],
        }
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    @staticmethod
    def _angle_or_empty(value):
        return "" if value is None else math.degrees(value)

    def _response_callback(self, msg: Response) -> None:
        self.get_logger().info(
            "收到 B2 原始响应："
            f"api_id={msg.header.identity.api_id}，data={msg.data}"
        )

    def close(self) -> None:
        with self._lock:
            if not self.csv_file.closed:
                self._write_log("node_exit")
                self.csv_file.close()


def get_key(settings):
    """读取一个按键并立即恢复终端设置."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print("错误：地形实机测试必须在可交互终端中运行。")
        return
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TerrainMotionTest()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == "\x03":
                break
            if key in PHASES:
                node.select_phase(PHASES[key])
            elif key == "i":
                node.request_arm()
            elif key == "w":
                node.move(node.forward_speed)
            elif key == "s":
                node.move(-node.reverse_speed)
            elif key == "x":
                node.stop_and_lock("操作者立即停车")
            elif key == "p":
                node.print_state()
            elif key == "h":
                print(HELP)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            node.stop_and_lock("程序退出")
            time.sleep(0.10)
            rclpy.shutdown()
        spinner.join(timeout=1.0)
        node.close()
        node.destroy_node()


if __name__ == "__main__":
    main()
