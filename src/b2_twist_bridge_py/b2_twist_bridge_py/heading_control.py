"""与 ROS2 解耦的 B2 航向角计算和比例反馈规则。"""

from dataclasses import dataclass
import math
from typing import Sequence


def wrap_angle(angle: float) -> float:
    """把有限角度归一化到 [-pi, pi]。"""
    value = float(angle)
    if not math.isfinite(value):
        raise ValueError("角度必须是有限数值")
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_wxyz_to_yaw(quaternion: Sequence[float]) -> float:
    """从 Unitree 使用的 w、x、y、z 四元数计算航向角。"""
    if len(quaternion) != 4:
        raise ValueError("四元数必须包含 w、x、y、z 四个分量")
    values = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("四元数包含 NaN 或无穷大")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-6:
        raise ValueError("四元数长度接近零")

    w, x, y, z = (value / norm for value in values)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return wrap_angle(math.atan2(sin_yaw, cos_yaw))


@dataclass(frozen=True)
class HeadingHoldConfig:
    """短脉冲平移使用的保守航向比例控制参数。"""

    kp: float = 0.60
    deadband_rad: float = math.radians(2.0)
    max_correction_rad_s: float = 0.10

    def __post_init__(self) -> None:
        values = (self.kp, self.deadband_rad, self.max_correction_rad_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("航向保持参数必须是有限数值")
        if self.kp <= 0.0:
            raise ValueError("yaw_kp 必须大于零")
        if not 0.0 <= self.deadband_rad <= math.pi:
            raise ValueError("yaw_deadband_rad 必须位于 0 到 pi")
        if not 0.0 < self.max_correction_rad_s <= 0.20:
            raise ValueError("yaw_max_correction 必须大于零且不超过 0.20 rad/s")

    def correction(
        self, target_yaw: float, current_yaw: float
    ) -> tuple[float, float]:
        """返回航向误差和经过死区、限幅后的角速度修正。"""
        error = wrap_angle(float(target_yaw) - float(current_yaw))
        if abs(error) <= self.deadband_rad:
            return error, 0.0
        raw = self.kp * error
        correction = max(
            -self.max_correction_rad_s,
            min(self.max_correction_rad_s, raw),
        )
        return error, correction
