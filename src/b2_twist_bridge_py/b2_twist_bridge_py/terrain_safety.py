"""B2 坡道和楼梯有监督测试的纯 Python 姿态安全规则."""

from dataclasses import dataclass
import math
from typing import Sequence


def quaternion_wxyz_to_rpy(
    quaternion: Sequence[float],
) -> tuple[float, float, float]:
    """将 Unitree w、x、y、z 四元数转换为 roll、pitch、yaw."""
    if len(quaternion) != 4:
        raise ValueError("四元数必须包含 w、x、y、z 四个分量")
    values = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("四元数包含 NaN 或无穷大")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-6:
        raise ValueError("四元数长度接近零")

    w, x, y, z = (value / norm for value in values)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


@dataclass(frozen=True)
class TerrainSafetyLimits:
    """坡道/楼梯点动测试中的硬限位."""

    max_abs_roll_rad: float = math.radians(22.0)
    max_abs_pitch_rad: float = math.radians(35.0)
    state_timeout_sec: float = 0.30
    expected_gait_type: int = -1

    def __post_init__(self) -> None:
        values = (
            self.max_abs_roll_rad,
            self.max_abs_pitch_rad,
            self.state_timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("地形测试安全参数必须是有限数值")
        if not 0.0 < self.max_abs_roll_rad <= math.radians(45.0):
            raise ValueError("横滚角限值必须大于 0 且不超过 45 度")
        if not 0.0 < self.max_abs_pitch_rad <= math.radians(55.0):
            raise ValueError("俯仰角限值必须大于 0 且不超过 55 度")
        if not 0.10 <= self.state_timeout_sec <= 1.0:
            raise ValueError("状态超时必须位于 0.10 到 1.00 秒")
        if not -1 <= int(self.expected_gait_type) <= 255:
            raise ValueError("expected_gait_type 必须位于 -1 到 255")

    def reasons(
        self,
        *,
        roll_rad: float,
        pitch_rad: float,
        state_age_sec: float,
        error_code: int,
        gait_type: int,
    ) -> list[str]:
        """返回必须拒绝或停止运动的原因."""
        values = (roll_rad, pitch_rad, state_age_sec)
        if not all(math.isfinite(float(value)) for value in values):
            return ["姿态或状态延迟包含 NaN/无穷大"]

        result: list[str] = []
        if int(error_code) != 0:
            result.append(f"SportModeState 错误码={int(error_code)}")
        if state_age_sec < 0.0 or state_age_sec > self.state_timeout_sec:
            result.append(
                f"SportModeState 已超时 {state_age_sec:.3f} 秒"
            )
        if abs(roll_rad) > self.max_abs_roll_rad:
            result.append(
                f"横滚角 {math.degrees(roll_rad):.1f} 度超过限值"
            )
        if abs(pitch_rad) > self.max_abs_pitch_rad:
            result.append(
                f"俯仰角 {math.degrees(pitch_rad):.1f} 度超过限值"
            )
        expected = int(self.expected_gait_type)
        if expected >= 0 and int(gait_type) != expected:
            result.append(
                f"实际 gait_type={int(gait_type)}，期望 {expected}"
            )
        return result
