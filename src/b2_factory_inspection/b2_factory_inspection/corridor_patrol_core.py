"""走廊巡逻的路线、定位证据和超时检查；不依赖 ROS。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PatrolPose:
    """地图中的一个巡逻目标。"""

    name: str
    x: float
    y: float
    yaw: float


def patrol_route(frame: str, waypoints: list[dict], mode: str, laps: int):
    """展开有限圈数；闭环包含末点到首点，终点朝向下一圈首段。"""
    if frame != "map":
        raise ValueError("走廊巡逻只接受 frame_id=map")
    if mode not in ("once", "loop"):
        raise ValueError("route_mode 必须是 once 或 loop")
    if type(laps) is not int or not 1 <= laps <= 10:
        raise ValueError("laps 必须是 1 到 10 的整数")
    if mode == "once" and laps != 1:
        raise ValueError("once 模式只能执行 1 次")
    if not 1 <= len(waypoints) <= 100:
        raise ValueError("巡逻点数量必须位于 1 到 100")
    if mode == "loop" and len(waypoints) < 2:
        raise ValueError("闭环路线至少需要两个不同的点")
    points = []
    for waypoint in waypoints:
        if waypoint["inspect"] or len(waypoint["poses"]) != 1:
            raise ValueError("纯巡逻要求 inspect=false 且每点只有一个位姿")
        pose = waypoint["poses"][0]
        points.append((waypoint["name"], pose["x"], pose["y"]))
    pairs = list(zip(points, points[1:]))
    if mode == "loop":
        pairs.append((points[-1], points[0]))
    if any(math.hypot(a[1] - b[1], a[2] - b[2]) < 0.30 for a, b in pairs):
        raise ValueError("相邻巡逻点（含闭环段）至少相距 0.30 m；不要重复首点")
    expanded = points * laps + [points[0]] if mode == "loop" else points
    if len(expanded) > 500:
        raise ValueError("展开后的目标数量不得超过 500")
    route = []
    for index, (name, x, y) in enumerate(expanded):
        if index + 1 < len(expanded):
            next_point = expanded[index + 1]
            yaw = math.atan2(next_point[2] - y, next_point[1] - x)
        elif mode == "loop":
            yaw = math.atan2(points[1][2] - y, points[1][1] - x)
        elif index:
            previous = expanded[index - 1]
            yaw = math.atan2(y - previous[2], x - previous[1])
        else:
            yaw = math.radians(waypoints[0]["poses"][0]["yaw_deg"])
        route.append(PatrolPose(name, x, y, yaw))
    return route


@dataclass(frozen=True)
class PatrolLimits:
    """约百米走廊实机任务的有界时间与路线限制，均为待验证初值。"""

    goal_timeout_sec: float = 60.0
    mission_timeout_sec: float = 600.0
    planning_timeout_sec: float = 8.0
    validation_ttl_sec: float = 60.0
    max_segment_m: float = 5.0

    def __post_init__(self):
        """拒绝无穷、负数及无界的运动时长。"""
        values = tuple(vars(self).values())
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("巡逻时间和距离限制必须是正有限数值")
        if self.mission_timeout_sec > 600.0:
            raise ValueError("单次任务最长 600 秒，禁止无限巡逻")
        if self.goal_timeout_sec > self.mission_timeout_sec:
            raise ValueError("单目标超时不得超过总任务超时")


def check_path(points, frame, start, goal, max_length):
    """拒绝空路径、错误坐标系、近似终点偏差过大及过长绕行。"""
    if frame != "map" or not points:
        raise ValueError("规划路径为空或不是 map 坐标系")
    if not all(math.isfinite(v) for point in points for v in point):
        raise ValueError("规划路径包含非法坐标")
    if math.dist(points[0], start) > 0.25:
        raise ValueError("规划路径起点偏离请求起点超过 0.25 m")
    if math.dist(points[-1], goal) > 0.25:
        raise ValueError("规划路径终点偏离目标超过 0.25 m")
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    if length > max_length:
        raise ValueError(f"单段路径 {length:.2f} m 超过上限 {max_length:.2f} m")
    return length


def amcl_evidence_ok(frame, position, quaternion, covariance):
    """仅证明收到过有限、归一化且协方差受限的定位消息。"""
    if frame != "map" or len(covariance) != 36:
        return False
    if not all(math.isfinite(v) for v in (*position, *quaternion, *covariance)):
        return False
    norm = sum(v * v for v in quaternion)
    return (
        abs(norm - 1.0) <= 0.05
        and 0 <= covariance[0] <= 0.25
        and 0 <= covariance[7] <= 0.25
        and 0 <= covariance[35] <= math.radians(30) ** 2
    )


class PatrolSession:
    """管理异步操作代次和单调时钟期限，阻止旧回调恢复任务。"""

    def __init__(self, limits: PatrolLimits):
        self.limits = limits
        self.phase = "idle"
        self.generation = 0
        self.index = 0
        self.count = 0
        self.deadline = 0.0
        self.mission_deadline = 0.0
        self.valid_until = 0.0

    def validating(self, count, now):
        """开始一次全新路径检查，使此前检查和回调失效。"""
        self.stop()
        self.phase = "validating"
        self.count = count
        self.deadline = now + self.limits.planning_timeout_sec

    def planned(self, now):
        """推进逐段规划；全部通过后只进入待命。"""
        self.index += 1
        self.deadline = now + self.limits.planning_timeout_sec
        if self.index == self.count:
            self.phase = "ready"
            self.valid_until = now + self.limits.validation_ttl_sec

    def start(self, now):
        """启动必须消费一次尚未过期的规划检查。"""
        if self.phase != "ready" or now >= self.valid_until:
            raise ValueError("请先在锁定状态执行 validate；检查可能已经过期")
        self.generation += 1
        self.phase = "running"
        self.index = 0
        self.deadline = now + self.limits.goal_timeout_sec
        self.mission_deadline = now + self.limits.mission_timeout_sec

    def arrived(self, now):
        """只由当前成功的动作结果推进目标；不重试失败目标。"""
        self.index += 1
        self.deadline = now + self.limits.goal_timeout_sec
        return self.index == self.count

    def timeout_reason(self, now):
        """返回需要停车的超时原因。"""
        if self.phase == "running" and now >= self.mission_deadline:
            return "达到总任务时间上限"
        if self.phase in ("running", "validating") and now >= self.deadline:
            return "导航目标超时" if self.phase == "running" else "路径检查超时"
        return None

    def current(self, generation, index, phase):
        """异步响应必须同时属于当前任务、目标和阶段。"""
        return (self.generation, self.index, self.phase) == (generation, index, phase)

    def stop(self):
        """停车后必须重新检查路线，禁止迟到回调自动继续。"""
        self.generation += 1
        self.phase = "idle"
        self.index = 0
        self.valid_until = 0.0
