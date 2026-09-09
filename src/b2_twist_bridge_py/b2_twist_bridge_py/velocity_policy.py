"""与 ROS2 解耦的 B2 自主速度整形规则."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VelocityPolicy:
    """根据 2026-09-08/09 实机起步速度限制导航速度."""

    max_linear_x: float = 0.30
    max_linear_y: float = 0.30
    max_angular_z: float = 0.20
    min_effective_linear_x: float = 0.30
    min_effective_linear_y: float = 0.30
    deadband: float = 0.005
    allow_reverse: bool = True
    allow_lateral: bool = True
    # 2026-09-09 只验证了纯前后和纯横移，未验证 x/y 同时非零。
    allow_combined_translation: bool = False

    def __post_init__(self) -> None:
        """拒绝互相冲突或可能绕过限速的参数."""
        positive = (
            self.max_linear_x,
            self.max_linear_y,
            self.max_angular_z,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("最大速度必须是大于零的有限数值")
        if not math.isfinite(self.deadband) or self.deadband < 0.0:
            raise ValueError("速度死区必须是非负有限数值")
        if (
            not math.isfinite(self.min_effective_linear_x)
            or not 0.0 <= self.min_effective_linear_x <= self.max_linear_x
        ):
            raise ValueError("最小有效前进速度必须位于 0 到最大前进速度之间")
        if (
            not math.isfinite(self.min_effective_linear_y)
            or not 0.0 <= self.min_effective_linear_y <= self.max_linear_y
        ):
            raise ValueError("最小有效横移速度必须位于 0 到最大横移速度之间")

    @staticmethod
    def _limit(value: float, maximum: float) -> float:
        return max(-maximum, min(maximum, value))

    def shape(self, x: float, y: float, z: float) -> tuple[float, float, float, list[str]]:
        """返回限幅后的速度和本次发生的安全整形说明."""
        raw = (float(x), float(y), float(z))
        if not all(math.isfinite(value) for value in raw):
            raise ValueError("速度包含 NaN 或无穷大")

        shaped_x = self._limit(raw[0], self.max_linear_x)
        shaped_y = self._limit(raw[1], self.max_linear_y)
        shaped_z = self._limit(raw[2], self.max_angular_z)
        notes: list[str] = []

        shaped_x = 0.0 if abs(shaped_x) < self.deadband else shaped_x
        shaped_y = 0.0 if abs(shaped_y) < self.deadband else shaped_y
        shaped_z = 0.0 if abs(shaped_z) < self.deadband else shaped_z

        if shaped_x < 0.0 and not self.allow_reverse:
            shaped_x = 0.0
            notes.append("后退已被当前参数禁止")
        if shaped_y != 0.0 and not self.allow_lateral:
            shaped_y = 0.0
            notes.append("横移已被当前参数禁止")

        if (
            shaped_x != 0.0
            and shaped_y != 0.0
            and not self.allow_combined_translation
        ):
            shaped_x = 0.0
            shaped_y = 0.0
            notes.append("前后与横移组合运动尚未实机验证，已阻止")

        if 0.0 < abs(shaped_x) < self.min_effective_linear_x:
            shaped_x = math.copysign(self.min_effective_linear_x, shaped_x)
            notes.append(
                f"前进速度提升到实测有效值 {self.min_effective_linear_x:.2f} m/s"
            )
        if 0.0 < abs(shaped_y) < self.min_effective_linear_y:
            shaped_y = math.copysign(self.min_effective_linear_y, shaped_y)
            notes.append(
                f"横移速度提升到实测有效值 {self.min_effective_linear_y:.2f} m/s"
            )

        return shaped_x, shaped_y, shaped_z, notes
