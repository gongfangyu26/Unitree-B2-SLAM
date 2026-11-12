"""B2 电脑端楼梯测试的纯 Python 协议与参数规则."""

from dataclasses import dataclass
import json
import math


MOTION_SWITCHER_API_IDS = {
    "CHECK_MODE": 1001,
    "SELECT_MODE": 1002,
}

SWITCH_GAIT_API_ID = 1011
# AI 运控文档的请求枚举，不是旧通用状态表中的楼梯编号。
AI_STEP_GAIT = 2
AI_GAIT_NAMES = {
    0: "锁定站立", 1: "盲走模式", 2: "持续踏步模式",
    3: "视觉辅助模式", 4: "平面行走模式",
}

STAIR_MOTION_ALIAS = "ai"

MOTION_SERVICE_NAMES = {
    ("0", "normal"): "sport_mode",
    ("0", "ai"): "ai_sport",
    ("0", "advanced"): "advanced_sport",
    ("1", "ai-w"): "wheeled_sport(go2W)",
    ("1", "normal-w"): "wheeled_sport(b2W)",
}


def select_mode_parameter(name_or_alias: str) -> str:
    """生成官方 MotionSwitchClient::SelectMode 的 JSON 参数."""
    if name_or_alias != STAIR_MOTION_ALIAS:
        raise ValueError("楼梯测试只允许选择官方 ai/ai_sport")
    return json.dumps({"name": name_or_alias}, separators=(",", ":"))


def ai_step_activation_sequence() -> tuple[tuple[str, int, str], ...]:
    """官方 AI 文档顺序：解除锁定、经典步态、持续踏步."""
    return (
        ("BalanceStand", 1002, "{}"),
        ("ClassicWalk(true)", 1049, '{"data":true}'),
        ("SwitchGait(2)", SWITCH_GAIT_API_ID, '{"data":2}'),
    )


def validate_step_probe_duration(value: float) -> float:
    """原地踏步可观察 10 秒，异常停车不受该时长限制."""
    duration = float(value)
    if not math.isfinite(duration) or not 0.20 <= duration <= 10.00:
        raise ValueError("原地踏步测试时间必须位于 0.20 到 10.00 秒")
    return duration


def parse_checked_mode(data: str) -> tuple[str, str]:
    """解析官方 MotionSwitchClient::CheckMode 的响应数据."""
    try:
        payload = json.loads(data)
        form = payload["form"]
        name = payload["name"]
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise ValueError("CheckMode 响应不是有效的 form/name JSON") from exc
    if not isinstance(form, str) or not isinstance(name, str):
        raise ValueError("CheckMode 的 form/name 必须是字符串")
    return form, name


def motion_service_name(form: str, name: str) -> str:
    """返回 Unitree B2 官方示例中的运动服务名."""
    return MOTION_SERVICE_NAMES.get((str(form), str(name)), "unknown")


def stair_motion_mode_is_active(form: str, name: str) -> bool:
    """是否已启用本机实测支持的 B2 ai/ai_sport 服务."""
    return str(form) == "0" and str(name) == STAIR_MOTION_ALIAS


@dataclass(frozen=True)
class StairCommandLimits:
    """电脑远程楼梯测试的硬限位."""

    up_velocity_x: float = 0.30
    down_velocity_x: float = -0.30
    motion_duration_sec: float = 0.50
    command_rate_hz: float = 10.0
    arm_timeout_sec: float = 10.0

    def __post_init__(self) -> None:
        values = (
            self.up_velocity_x,
            self.down_velocity_x,
            self.motion_duration_sec,
            self.command_rate_hz,
            self.arm_timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("楼梯控制参数必须是有限数值")
        if not 0.0 < self.up_velocity_x <= 0.30:
            raise ValueError("上楼速度必须大于 0 且不超过 0.30 m/s")
        if not -0.30 <= self.down_velocity_x < 0.0:
            raise ValueError("下楼速度必须小于 0 且绝对值不超过 0.30 m/s")
        if not 0.20 <= self.motion_duration_sec <= 3.00:
            raise ValueError("运动时间必须位于 0.20 到 3.00 秒")
        if not 2.0 <= self.command_rate_hz <= 20.0:
            raise ValueError("命令频率必须位于 2 到 20 Hz")
        if not 3.0 <= self.arm_timeout_sec <= 30.0:
            raise ValueError("解锁有效期必须位于 3 到 30 秒")

    def velocity_for(self, direction: str) -> float:
        if direction == "up":
            return self.up_velocity_x
        if direction == "down":
            return self.down_velocity_x
        raise ValueError(f"未知楼梯方向：{direction}")
