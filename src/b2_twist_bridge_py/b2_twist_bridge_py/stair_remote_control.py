#!/usr/bin/env python3
"""Unitree B2 全电脑端上下楼梯有监护测试节点."""

import csv
from dataclasses import dataclass, field
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

from .sport_model import api_id
from .stair_control import (
    MOTION_SWITCHER_API_IDS,
    AI_STEP_GAIT,
    SWITCH_GAIT_API_ID,
    ai_step_activation_sequence,
    StairCommandLimits,
    motion_service_name,
    stair_motion_mode_is_active,
    parse_checked_mode,
    validate_step_probe_duration,
)
from .terrain_safety import TerrainSafetyLimits, quaternion_wxyz_to_rpy


CONTROL_REVISION = "ai_step_classic3s_v4"


@dataclass
class _PendingRpc:
    event: threading.Event = field(default_factory=threading.Event)
    code: int | None = None
    data: str = ""
    api: int = 0
    channel: str = ""


class StairRemoteControl(Node):
    """使用官方 MotionSwitcher、Move 和 StopMove 完成短时楼梯测试."""

    def __init__(self) -> None:
        super().__init__("b2_stair_remote_control")
        self.declare_parameter(
            "motion_switch_request_topic", "/api/motion_switcher/request"
        )
        self.declare_parameter(
            "motion_switch_response_topic", "/api/motion_switcher/response"
        )
        self.declare_parameter("sport_request_topic", "/api/sport/request")
        self.declare_parameter("sport_state_topic", "/lf/sportmodestate")
        self.declare_parameter("sport_response_topic", "/api/sport/response")
        self.declare_parameter("step_probe_duration_sec", 10.0)
        self.declare_parameter("up_velocity_x", 0.30)
        self.declare_parameter("down_velocity_x", -0.30)
        self.declare_parameter("motion_duration_sec", 0.50)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("arm_timeout_sec", 10.0)
        self.declare_parameter("state_timeout_sec", 0.30)
        self.declare_parameter("max_abs_roll_deg", 22.0)
        self.declare_parameter("max_abs_pitch_deg", 35.0)
        self.declare_parameter("log_directory", "~/b2_terrain_logs")
        self.declare_parameter("test_label", "stairs_remote")

        try:
            self.command_limits = StairCommandLimits(
                up_velocity_x=float(
                    self.get_parameter("up_velocity_x").value
                ),
                down_velocity_x=float(
                    self.get_parameter("down_velocity_x").value
                ),
                motion_duration_sec=float(
                    self.get_parameter("motion_duration_sec").value
                ),
                command_rate_hz=float(
                    self.get_parameter("command_rate_hz").value
                ),
                arm_timeout_sec=float(
                    self.get_parameter("arm_timeout_sec").value
                ),
            )
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
                expected_gait_type=-1,
            )
            self.step_probe_duration = validate_step_probe_duration(
                self.get_parameter("step_probe_duration_sec").value
            )
        except ValueError as exc:
            raise RuntimeError(f"楼梯测试参数无效：{exc}") from exc

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._sport_command_lock = threading.RLock()
        self._pending_lock = threading.Lock()
        self._rpc_serial_lock = threading.Lock()
        self._activation_lock = threading.Lock()
        self._activation_cancel = threading.Event()
        self._pending: dict[int, _PendingRpc] = {}

        motion_request_topic = str(
            self.get_parameter("motion_switch_request_topic").value
        )
        motion_response_topic = str(
            self.get_parameter("motion_switch_response_topic").value
        )
        sport_request_topic = str(
            self.get_parameter("sport_request_topic").value
        )
        sport_state_topic = str(
            self.get_parameter("sport_state_topic").value
        )
        self.motion_request_pub = self.create_publisher(
            Request, motion_request_topic, 1
        )
        self.sport_request_pub = self.create_publisher(
            Request, sport_request_topic, 10
        )
        self.create_subscription(
            Response,
            motion_response_topic,
            lambda msg: self._response_callback("motion", msg),
            1,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Response,
            str(self.get_parameter("sport_response_topic").value),
            lambda msg: self._response_callback("sport", msg),
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            SportModeState,
            sport_state_topic,
            self._state_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

        self.create_service(
            Trigger,
            "/b2/stairs/activate_mode",
            self._activate_mode_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/b2/stairs/probe_ai_step",
            self._activate_mode_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetBool,
            "/b2/stairs/set_armed",
            self._set_armed_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/b2/stairs/up",
            self._up_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/b2/stairs/down",
            self._down_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/b2/stairs/stop",
            self._stop_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/b2/stairs/status",
            self._status_callback,
            callback_group=self._callback_group,
        )

        self.armed = False
        self.arm_deadline = 0.0
        self.active_direction: str | None = None
        self.motion_deadline = 0.0
        self.last_state_time = 0.0
        self.last_state: dict | None = None
        self.last_fault = ""
        self.stair_motion_mode_verified_at = 0.0
        self.ai_step_probe_active = False
        self.probe_cancel_reason = ""
        self.probe_deadline = 0.0
        self.activation_results: list[dict] = []
        self.last_probe_result = "not_run"
        self.closed = False
        self.last_state_log_time = 0.0
        self.csv_file, self.csv_writer, self.log_path = self._open_log()

        self.create_timer(
            1.0 / self.command_limits.command_rate_hz,
            self._watchdog,
            callback_group=self._callback_group,
        )
        self.get_logger().warn(
            "楼梯节点已启动但处于锁定状态；不会自动切模式或运动"
        )
        self.get_logger().warn(
            "AI 接口修正版：activate_mode/probe_ai_step 仅原地踏步"
            "短测并自动恢复锁定站立；上下楼梯运动暂不开放"
        )
        self.get_logger().info(f"实验日志：{self.log_path}")
        self.get_logger().info(
            f"控制版本：{CONTROL_REVISION}；ClassicWalk 后等待 3 秒；"
            f"踏步上限 {self.step_probe_duration:.1f} 秒"
        )

    def _open_log(self):
        directory = Path(
            str(self.get_parameter("log_directory").value)
        ).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        label = str(self.get_parameter("test_label").value).strip()
        safe_label = "".join(
            char if char.isalnum() or char in "-_" else "_"
            for char in label
        ).strip("_") or "stairs_remote"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = directory / f"b2_{safe_label}_{stamp}.csv"
        handle = path.open("x", newline="", encoding="utf-8")
        fields = [
            "host_time_iso",
            "monotonic_sec",
            "event",
            "details",
            "armed",
            "active_direction",
            "command_velocity_x",
            "error_code",
            "mode",
            "gait_type",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "position_x",
            "position_y",
            "position_z",
            "velocity_x",
            "velocity_y",
            "velocity_z",
            "foot_force_0",
            "foot_force_1",
            "foot_force_2",
            "foot_force_3",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        handle.flush()
        return handle, writer, path

    def _write_log(
        self, event: str, details: str = "", now: float | None = None
    ) -> None:
        if self.closed:
            return
        now = time.monotonic() if now is None else now
        state = self.last_state or {}
        position = state.get("position", ("", "", ""))
        velocity = state.get("velocity", ("", "", ""))
        foot_force = state.get("foot_force", ("", "", "", ""))
        direction = self.active_direction or ""
        command_velocity = (
            self.command_limits.velocity_for(direction) if direction else ""
        )
        self.csv_writer.writerow(
            {
                "host_time_iso": datetime.now().astimezone().isoformat(),
                "monotonic_sec": f"{now:.6f}",
                "event": event,
                "details": details,
                "armed": self.armed,
                "active_direction": direction,
                "command_velocity_x": command_velocity,
                "error_code": state.get("error_code", ""),
                "mode": state.get("mode", ""),
                "gait_type": state.get("gait_type", ""),
                "roll_deg": self._degrees_or_blank(state.get("roll")),
                "pitch_deg": self._degrees_or_blank(state.get("pitch")),
                "yaw_deg": self._degrees_or_blank(state.get("yaw")),
                "position_x": position[0],
                "position_y": position[1],
                "position_z": position[2],
                "velocity_x": velocity[0],
                "velocity_y": velocity[1],
                "velocity_z": velocity[2],
                "foot_force_0": foot_force[0],
                "foot_force_1": foot_force[1],
                "foot_force_2": foot_force[2],
                "foot_force_3": foot_force[3],
            }
        )
        self.csv_file.flush()

    @staticmethod
    def _degrees_or_blank(value):
        return "" if value is None else f"{math.degrees(value):.3f}"

    def _response_callback(self, channel: str, msg: Response) -> None:
        request_id = int(msg.header.identity.id)
        with self._pending_lock:
            pending = self._pending.get(request_id)
            if pending is None or pending.channel != channel:
                return
            if pending.api != int(msg.header.identity.api_id):
                return
            pending.code = int(msg.header.status.code)
            pending.data = str(msg.data)
            pending.event.set()

    def _rpc_call(
        self, channel: str, api: int, parameter: str = "{}",
        timeout_sec: float = 1.0, cancellable: bool = False,
    ) -> tuple[int, str]:
        """匹配通道、identity.id 和 api_id，取消后禁止继续动作."""
        with self._rpc_serial_lock:
            request_id = time.monotonic_ns()
            pending = _PendingRpc(api=api, channel=channel)
            with self._pending_lock:
                self._pending[request_id] = pending
            try:
                request = Request()
                request.header.identity.id = request_id
                request.header.identity.api_id = int(api)
                request.parameter = parameter
                publisher = (
                    self.motion_request_pub
                    if channel == "motion" else self.sport_request_pub
                )
                with self._sport_command_lock:
                    if cancellable and self._activation_cancel.is_set():
                        return -2, "cancelled"
                    publisher.publish(request)
                deadline = time.monotonic() + timeout_sec
                while time.monotonic() < deadline:
                    if cancellable and self._activation_cancel.is_set():
                        return -2, "cancelled"
                    if pending.event.wait(timeout=0.02):
                        return int(pending.code), pending.data
                return -1, "response_timeout"
            finally:
                with self._pending_lock:
                    self._pending.pop(request_id, None)

    def _motion_switch_call(
        self, api: int, parameter: str = "", timeout_sec: float = 3.0
    ) -> tuple[int, str]:
        return self._rpc_call("motion", api, parameter, timeout_sec)

    def _probe_call(
        self, label: str, api: int, parameter: str,
        cancellable: bool = True,
    ) -> int:
        code, data = self._rpc_call(
            "sport", api, parameter, cancellable=cancellable
        )
        with self._lock:
            result = {
                "function": label, "api_id": api, "code": code, "data": data,
            }
            self.activation_results.append(result)
            self._write_log("ai_step_rpc", json.dumps(result))
        return code

    def _wait_precondition_state(self, label: str) -> str:
        """ClassicWalk 后等待参考例程的 3 秒，仍检查新的无错误状态."""
        settle_sec = 3.0 if label == "ClassicWalk(true)" else 1.0
        started = time.monotonic()
        while time.monotonic() - started < settle_sec + 1.0:
            if self._activation_cancel.is_set():
                return self.probe_cancel_reason or f"{label} 后被停车取消"
            now = time.monotonic()
            with self._lock:
                reasons = self._state_reasons(now)
                state = dict(self.last_state or {})
                has_new_state = self.last_state_time > started
            if reasons:
                return f"{label} 后状态联锁：" + "；".join(reasons)
            if now - started >= settle_sec and has_new_state:
                with self._lock:
                    self._write_log(
                        "precondition_state_ready",
                        json.dumps({
                            "after": label, "settle_sec": settle_sec,
                            "mode": state.get("mode"),
                            "gait_type": state.get("gait_type"),
                            "error_code": state.get("error_code"),
                        }),
                    )
                return ""
            self._activation_cancel.wait(timeout=0.02)
        return f"{label} 后未收到新的无错误状态，拒绝后续切换"

    def _check_motion_mode(self) -> tuple[int, str, str, str]:
        code, data = self._motion_switch_call(
            MOTION_SWITCHER_API_IDS["CHECK_MODE"]
        )
        if code != 0:
            return code, "", "", f"CheckMode 失败，code={code}"
        try:
            form, name = parse_checked_mode(data)
        except ValueError as exc:
            return -2, "", "", str(exc)
        service = motion_service_name(form, name)
        if stair_motion_mode_is_active(form, name):
            with self._lock:
                self.stair_motion_mode_verified_at = time.monotonic()
        return 0, form, name, service

    def _state_callback(self, msg: SportModeState) -> None:
        now = time.monotonic()
        try:
            roll, pitch, yaw = quaternion_wxyz_to_rpy(
                msg.imu_state.quaternion
            )
            position = tuple(float(value) for value in msg.position[:3])
            velocity = tuple(float(value) for value in msg.velocity[:3])
            foot_force = tuple(int(value) for value in msg.foot_force[:4])
            if (
                len(position) != 3
                or len(velocity) != 3
                or len(foot_force) != 4
            ):
                raise ValueError("SportModeState 数组长度不足")
            numeric = position + velocity + (float(msg.yaw_speed),)
            if not all(math.isfinite(value) for value in numeric):
                raise ValueError("SportModeState 包含 NaN 或无穷大")
        except ValueError as exc:
            self._fault_stop(f"机器人状态无效：{exc}")
            return

        state = {
            "error_code": int(msg.error_code),
            "mode": int(msg.mode),
            "gait_type": int(msg.gait_type),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "position": position,
            "velocity": velocity,
            "yaw_speed": float(msg.yaw_speed),
            "foot_force": foot_force,
        }
        should_stop = False
        reasons: list[str] = []
        with self._lock:
            self.last_state_time = now
            self.last_state = state
            if now - self.last_state_log_time >= 0.10:
                self._write_log("state", now=now)
                self.last_state_log_time = now
            reasons = self._state_reasons(now)
            should_stop = bool(reasons) and (
                self.armed or self.active_direction is not None
            )
            if self.ai_step_probe_active and reasons:
                should_stop = True
        if should_stop:
            self._fault_stop("；".join(reasons))

    def _state_reasons(self, now: float) -> list[str]:
        if self.last_state is None or self.last_state_time <= 0.0:
            return ["尚未收到 SportModeState"]
        state = self.last_state
        return self.safety_limits.reasons(
            roll_rad=state["roll"],
            pitch_rad=state["pitch"],
            state_age_sec=now - self.last_state_time,
            error_code=state["error_code"],
            gait_type=state["gait_type"],
        )

    def _publish_move(self, velocity_x: float) -> None:
        request = Request()
        request.header.identity.api_id = api_id("MOVE")
        request.parameter = json.dumps(
            {"x": velocity_x, "y": 0.0, "z": 0.0},
            separators=(",", ":"),
        )
        self.sport_request_pub.publish(request)

    def _publish_stop(self) -> None:
        request = Request()
        request.header.identity.api_id = api_id("STOPMOVE")
        self.sport_request_pub.publish(request)

    def stop_and_disarm(self, reason: str) -> None:
        with self._sport_command_lock:
            with self._lock:
                probe_active = self.ai_step_probe_active
                if probe_active and not self._activation_cancel.is_set():
                    self.probe_cancel_reason = reason
                self._activation_cancel.set()
                was_active = self.active_direction is not None or self.armed
                self.active_direction = None
                self.motion_deadline = 0.0
                self.armed = False
                self.arm_deadline = 0.0
                if was_active or probe_active:
                    self._write_log("stop_and_disarm", reason)
            self._publish_stop()
            if probe_active:
                request = Request()
                request.header.identity.api_id = SWITCH_GAIT_API_ID
                request.parameter = '{"data":0}'
                self.sport_request_pub.publish(request)

    def _fault_stop(self, reason: str) -> None:
        with self._lock:
            self.last_fault = reason
            if self.ai_step_probe_active:
                self.probe_cancel_reason = f"安全联锁：{reason}"
            should_report = (
                self.active_direction is not None or self.armed
                or self.ai_step_probe_active
            )
        self.stop_and_disarm(f"安全联锁：{reason}")
        if should_report:
            self.get_logger().error("已自动停车并锁定：" + reason)

    def _activate_mode_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """只在平地短测官方 AI 持续踏步，结束后恢复锁定站立."""
        if not self._activation_lock.acquire(blocking=False):
            response.success = False
            response.message = "已有 AI 原地踏步短测正在执行"
            return response
        attempted = False
        failure = ""
        try:
            with self._sport_command_lock:
                self.stop_and_disarm("AI 原地踏步短测前停车")
                self._activation_cancel.clear()
                with self._lock:
                    self.activation_results = []
                    self.probe_cancel_reason = ""
                    reasons = self._state_reasons(time.monotonic())
                    if reasons:
                        failure = "状态联锁：" + "；".join(reasons)
                    else:
                        self.ai_step_probe_active = True
                        self.probe_deadline = time.monotonic() + 12.0
            if not failure:
                code, form, name, detail = self._check_motion_mode()
                if code != 0 or not stair_motion_mode_is_active(form, name):
                    failure = f"未验证 ai/ai_sport：{detail}"
            if not failure:
                for label, api, parameter in ai_step_activation_sequence():
                    with self._lock:
                        reasons = self._state_reasons(time.monotonic())
                    if reasons or self._activation_cancel.is_set():
                        failure = (
                            "状态联锁：" + "；".join(reasons)
                            if reasons else self.probe_cancel_reason
                            or "短测被停车取消"
                        )
                        break
                    attempted = True
                    if label == "SwitchGait(2)":
                        with self._lock:
                            self.probe_deadline = (
                                time.monotonic() + self.step_probe_duration
                            )
                    code = self._probe_call(label, api, parameter)
                    if code != 0:
                        failure = f"{label} 失败/超时/取消，code={code}"
                        if self.probe_cancel_reason:
                            failure += f"；{self.probe_cancel_reason}"
                        break
                    if label != "SwitchGait(2)":
                        failure = self._wait_precondition_state(label)
                        if failure:
                            break
            if not failure:
                deadline = self.probe_deadline
                while time.monotonic() < deadline:
                    if self._activation_cancel.wait(timeout=0.02):
                        reason = self.probe_cancel_reason
                        if reason != "原地踏步硬超时":
                            failure = reason or "短测被停车取消"
                        break
        except Exception as exc:
            failure = f"原地踏步短测异常：{exc}"
        finally:
            # 前置步骤失败或超时也可能已有动作生效，始终尝试停车。
            if attempted:
                self.stop_and_disarm("AI 原地踏步短测结束")
                try:
                    stop_code = self._probe_call(
                        "StopMove", api_id("STOPMOVE"), "{}",
                        cancellable=False,
                    )
                    lock_code = self._probe_call(
                        "SwitchGait(0)", SWITCH_GAIT_API_ID, '{"data":0}',
                        cancellable=False,
                    )
                    if stop_code != 0 or lock_code != 0:
                        failure += (
                            f"；停车确认失败 StopMove={stop_code},"
                            f"SwitchGait(0)={lock_code}，需现场人工确认"
                        )
                except Exception as exc:
                    failure += f"；停车异常：{exc}，需人工停车"
            with self._lock:
                if self.probe_cancel_reason.startswith("安全联锁："):
                    failure = failure or self.probe_cancel_reason
                self.ai_step_probe_active = False
                self.probe_deadline = 0.0
                self.armed = False
                self.last_probe_result = failure or "rpc_sequence_completed"
                self._write_log("ai_step_probe_end", self.last_probe_result)
            self._activation_lock.release()
        response.success = not failure
        response.message = json.dumps({
            "control_revision": CONTROL_REVISION,
            "result": self.last_probe_result,
            "rpc_results": self.activation_results,
            "requested_ai_gait": AI_STEP_GAIT,
            "observed_gait_type": (self.last_state or {}).get("gait_type"),
            "stop_reason": self.probe_cancel_reason,
            "last_fault": self.last_fault,
            "step_probe_duration_sec": self.step_probe_duration,
            "armed": False,
            "note": "仅原地踏步短测；接口成功不代表楼梯模式已验证，"
                    "请现场确认踏步和停车",
        }, ensure_ascii=False)
        return response

    def _set_armed_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if not request.data:
            self.stop_and_disarm("电脑端主动锁定")
            response.success = True
            response.message = "已调用 StopMove 并锁定"
            return response

        response.success = False
        response.message = (
            "官方 AI 持续踏步不等于楼梯模式验收；当前版本仅允许 "
            "probe_ai_step 平地短测，上下楼梯运动保持锁定"
        )
        return response

    def _start_motion(
        self, direction: str, response: Trigger.Response
    ) -> Trigger.Response:
        now = time.monotonic()
        with self._sport_command_lock:
            with self._lock:
                if not self.armed or now > self.arm_deadline:
                    self.armed = False
                    response.success = False
                    response.message = "节点未解锁或解锁已超时"
                    return response
                if self.active_direction is not None:
                    response.success = False
                    response.message = "已有楼梯运动正在执行"
                    return response
                reasons = self._state_reasons(now)
                if reasons:
                    self.armed = False
                    response.success = False
                    response.message = (
                        "状态联锁拒绝运动：" + "；".join(reasons)
                    )
                    return response
                mode_age = now - self.stair_motion_mode_verified_at
                if mode_age > self.command_limits.arm_timeout_sec:
                    self.armed = False
                    response.success = False
                    response.message = (
                        "ai/ai_sport 验证已超时，请重新解锁"
                    )
                    return response
                velocity_x = self.command_limits.velocity_for(direction)
                self.active_direction = direction
                self.motion_deadline = (
                    now + self.command_limits.motion_duration_sec
                )
                self._write_log(
                    "motion_start", f"vx={velocity_x:.3f}", now
                )
            self._publish_move(velocity_x)
        response.success = True
        response.message = (
            f"{direction} 命令已在 Jetson 发布："
            f"vx={velocity_x:.2f} m/s，"
            f"最多 {self.command_limits.motion_duration_sec:.2f} 秒；"
            "到时自动 StopMove 并锁定"
        )
        return response

    def _up_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._start_motion("up", response)

    def _down_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._start_motion("down", response)

    def _stop_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.stop_and_disarm("电脑端立即停车")
        response.success = True
        response.message = "已请求停止短测、调用 StopMove 并锁定；请现场确认停车"
        return response

    def _status_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        code, form, name, detail = self._check_motion_mode()
        now = time.monotonic()
        with self._lock:
            state = self.last_state or {}
            payload = {
                "control_revision": CONTROL_REVISION,
                "motion_switch_code": code,
                "motion_form": form,
                "motion_name": name,
                "motion_service": detail if code == 0 else "unknown",
                "stair_motion_mode_active": (
                    code == 0 and stair_motion_mode_is_active(form, name)
                ),
                "stair_gait_active": False,
                "ai_step_probe_active": self.ai_step_probe_active,
                "step_probe_duration_sec": self.step_probe_duration,
                "probe_cancel_reason": self.probe_cancel_reason,
                "requested_ai_step_gait": AI_STEP_GAIT,
                "last_probe_result": self.last_probe_result,
                "activation_results": list(self.activation_results),
                "armed": self.armed,
                "active_direction": self.active_direction,
                "state_age_sec": (
                    None
                    if self.last_state_time <= 0.0
                    else round(now - self.last_state_time, 3)
                ),
                "error_code": state.get("error_code"),
                "mode": state.get("mode"),
                "gait_type": state.get("gait_type"),
                "roll_deg": (
                    None
                    if state.get("roll") is None
                    else round(math.degrees(state["roll"]), 2)
                ),
                "pitch_deg": (
                    None
                    if state.get("pitch") is None
                    else round(math.degrees(state["pitch"]), 2)
                ),
                "last_fault": self.last_fault,
                "log_path": str(self.log_path),
            }
        response.success = code == 0
        response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _watchdog(self) -> None:
        now = time.monotonic()
        with self._sport_command_lock:
            with self._lock:
                if (
                    self.armed
                    and self.active_direction is None
                    and now > self.arm_deadline
                ):
                    expired = True
                else:
                    expired = False
                direction = self.active_direction
                deadline = self.motion_deadline
                probe_active = self.ai_step_probe_active
                probe_deadline = self.probe_deadline
                reasons = (
                    self._state_reasons(now)
                    if direction or probe_active else []
                )

            if expired:
                self.stop_and_disarm("解锁等待超时")
                return
            if reasons:
                self._fault_stop("；".join(reasons))
                return
            if probe_active and now >= probe_deadline:
                self.stop_and_disarm("原地踏步硬超时")
                return
            if direction is None:
                return
            if now >= deadline:
                self.stop_and_disarm("单次楼梯运动定时结束")
                return
            self._publish_move(self.command_limits.velocity_for(direction))

    def close(self) -> None:
        if self.closed:
            return
        self.stop_and_disarm("节点退出")
        with self._lock:
            self._write_log("node_exit")
            self.closed = True
            self.csv_file.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StairRemoteControl()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
