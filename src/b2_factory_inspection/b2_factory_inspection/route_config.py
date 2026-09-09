"""加载并严格校验自主巡检路线配置."""

import math
from pathlib import Path
from typing import Any

import yaml


MAX_WAYPOINTS = 500
MAX_POSES_PER_WAYPOINT = 20


def _finite_number(value: Any, field_name: str) -> float:
    """把配置值转换为有限浮点数，并拒绝布尔值等歧义输入."""
    if isinstance(value, bool):
        raise ValueError(f"{field_name} 不能是布尔值")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须是数值") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} 必须是有限数值")
    return number


def load_route(path: str | Path) -> tuple[str, list[dict[str, Any]]]:
    """读取路线 YAML，返回坐标系和规范化后的巡检点列表."""
    route_path = Path(path).expanduser()
    if not route_path.is_file():
        raise ValueError(f"路线文件不存在：{route_path}")
    try:
        data = yaml.safe_load(route_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"无法读取路线文件：{exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("路线文件顶层必须是 YAML 映射")

    frame_id = data.get("frame_id", "map")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id 必须是非空字符串")
    frame_id = frame_id.strip()

    raw_waypoints = data.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("路线文件中必须包含非空 waypoints 列表")
    if len(raw_waypoints) > MAX_WAYPOINTS:
        raise ValueError(f"巡检点数量不能超过 {MAX_WAYPOINTS}")

    names: set[str] = set()
    waypoints: list[dict[str, Any]] = []
    for waypoint_index, raw_waypoint in enumerate(raw_waypoints, start=1):
        prefix = f"第 {waypoint_index} 个巡检点"
        if not isinstance(raw_waypoint, dict):
            raise ValueError(f"{prefix} 必须是 YAML 映射")

        name = raw_waypoint.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{prefix} 的 name 必须是非空字符串")
        name = name.strip()
        if name in names:
            raise ValueError(f"巡检点名称重复：{name}")
        names.add(name)

        inspect = raw_waypoint.get("inspect", True)
        if not isinstance(inspect, bool):
            raise ValueError(f"巡检点 {name} 的 inspect 必须是 true 或 false")

        raw_poses = raw_waypoint.get("poses")
        if not isinstance(raw_poses, list) or not raw_poses:
            raise ValueError(f"巡检点 {name} 必须至少包含一个候选位姿")
        if len(raw_poses) > MAX_POSES_PER_WAYPOINT:
            raise ValueError(
                f"巡检点 {name} 的候选位姿不能超过 {MAX_POSES_PER_WAYPOINT} 个"
            )

        poses: list[dict[str, float]] = []
        pose_keys: set[tuple[float, float, float]] = set()
        for pose_index, raw_pose in enumerate(raw_poses, start=1):
            if not isinstance(raw_pose, dict):
                raise ValueError(f"巡检点 {name} 的第 {pose_index} 个位姿格式错误")
            x = _finite_number(raw_pose.get("x"), f"{name}.poses[{pose_index}].x")
            y = _finite_number(raw_pose.get("y"), f"{name}.poses[{pose_index}].y")
            yaw = _finite_number(
                raw_pose.get("yaw_deg", 0.0),
                f"{name}.poses[{pose_index}].yaw_deg",
            )
            pose_key = (x, y, yaw)
            if pose_key in pose_keys:
                raise ValueError(f"巡检点 {name} 含重复候选位姿：{pose_key}")
            pose_keys.add(pose_key)
            poses.append({"x": x, "y": y, "yaw_deg": yaw})

        waypoints.append({"name": name, "inspect": inspect, "poses": poses})

    return frame_id, waypoints
