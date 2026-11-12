"""与 ROS2 解耦的人工点击导航安全门状态机."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class NavigationGateConfig:
    """导航安全门使用的数据新鲜度和最长解锁时间."""

    autonomy_timeout_sec: float = 0.80
    map_timeout_sec: float = 3.0
    max_armed_duration_sec: float = 60.0

    def __post_init__(self) -> None:
        """拒绝无效或可能永久解锁的超时配置."""
        values = (
            self.autonomy_timeout_sec,
            self.map_timeout_sec,
            self.max_armed_duration_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("导航安全门的超时参数必须大于零")
        if self.max_armed_duration_sec > 600.0:
            raise ValueError("导航安全门单次解锁时间不得超过 600 秒")


class NavigationGateState:
    """保存地图、自主安全状态和人工解锁状态."""

    def __init__(self, config: NavigationGateConfig) -> None:
        self.config = config
        self.autonomy_ok = False
        self.map_ok = False
        self.last_autonomy_time = 0.0
        self.last_map_time = 0.0
        self.armed = False
        self.armed_time = 0.0

    def update_autonomy(self, ok: bool, now: float) -> None:
        """记录自主安全监视器的最新状态."""
        self.autonomy_ok = bool(ok)
        self.last_autonomy_time = float(now)

    def update_map(self, ok: bool, now: float) -> None:
        """记录二维地图检查的最新状态."""
        self.map_ok = bool(ok)
        self.last_map_time = float(now)

    def blocking_reason(self, now: float) -> str | None:
        """返回当前阻止导航指令通过的首个原因."""
        current = float(now)
        if (
            self.last_autonomy_time <= 0.0
            or current - self.last_autonomy_time
            > self.config.autonomy_timeout_sec
        ):
            return "自主安全状态尚未收到或已超时"
        if not self.autonomy_ok:
            return "自主安全状态未通过"
        if (
            self.last_map_time <= 0.0
            or current - self.last_map_time > self.config.map_timeout_sec
        ):
            return "二维地图状态尚未收到或已超时"
        if not self.map_ok:
            return "二维地图检查未通过"
        return None

    def request_arm(self, now: float) -> tuple[bool, str]:
        """在全部安全条件满足时进入限时解锁状态."""
        reason = self.blocking_reason(now)
        if reason:
            self.armed = False
            self.armed_time = 0.0
            return False, reason
        self.armed = True
        self.armed_time = float(now)
        return True, "导航安全门已限时解锁"

    def disarm(self) -> None:
        """立即锁定安全门."""
        self.armed = False
        self.armed_time = 0.0

    def active_reason(self, now: float) -> str | None:
        """检查已解锁状态是否仍然有效."""
        if not self.armed:
            return "导航安全门未解锁"
        reason = self.blocking_reason(now)
        if reason:
            return reason
        if float(now) - self.armed_time > self.config.max_armed_duration_sec:
            return "导航安全门达到单次最长解锁时间"
        return None
