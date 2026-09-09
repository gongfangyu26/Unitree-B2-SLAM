"""与 ROS2 解耦的 SLAM 输入数据结构检查函数."""

import math
from typing import Iterable, Sequence


def estimate_rate_hz(arrival_times: Sequence[float]) -> float:
    """根据单调时钟接收记录估计消息频率."""
    if len(arrival_times) < 2:
        return 0.0
    duration = float(arrival_times[-1]) - float(arrival_times[0])
    if not math.isfinite(duration) or duration <= 0.0:
        return 0.0
    return (len(arrival_times) - 1) / duration


def validate_pointcloud_layout(
    *,
    width: int,
    height: int,
    point_step: int,
    row_step: int,
    data_length: int,
    field_names: Iterable[str],
    frame_id: str,
    min_points: int,
) -> list[str]:
    """检查 PointCloud2 是否足以作为三维 SLAM 输入."""
    reasons: list[str] = []
    if not str(frame_id).strip():
        reasons.append("点云 frame_id 为空")
    if width <= 0 or height <= 0:
        reasons.append("点云宽度或高度为零")
    point_count = max(0, int(width)) * max(0, int(height))
    if point_count < int(min_points):
        reasons.append(f"点云点数不足：{point_count} < {int(min_points)}")
    if point_step <= 0:
        reasons.append("point_step 必须大于零")
    if row_step < max(0, int(width)) * max(0, int(point_step)):
        reasons.append("row_step 小于一行点云所需字节数")
    if data_length < max(0, int(row_step)) * max(0, int(height)):
        reasons.append("点云 data 长度不足")
    fields = {str(name).lower() for name in field_names}
    missing = sorted({"x", "y", "z"} - fields)
    if missing:
        reasons.append(f"点云缺少字段：{','.join(missing)}")
    return reasons


def validate_odometry_values(
    *,
    position: Sequence[float],
    quaternion: Sequence[float],
    linear_velocity: Sequence[float],
    angular_velocity: Sequence[float],
    frame_id: str,
    child_frame_id: str,
    expected_frame_id: str,
    expected_child_frame_id: str,
) -> list[str]:
    """检查里程计数值、四元数和坐标系名称."""
    reasons: list[str] = []
    values = tuple(position) + tuple(quaternion) + tuple(linear_velocity) + tuple(
        angular_velocity
    )
    if not all(math.isfinite(float(value)) for value in values):
        reasons.append("里程计包含 NaN 或无穷大")
    if len(quaternion) != 4:
        reasons.append("里程计四元数长度不是 4")
    else:
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if not 0.90 <= norm <= 1.10:
            reasons.append(f"里程计四元数长度异常：{norm:.3f}")
    if str(frame_id) != str(expected_frame_id):
        reasons.append(
            f"里程计父坐标系为 {frame_id or '<空>'}，期望 {expected_frame_id}"
        )
    if str(child_frame_id) != str(expected_child_frame_id):
        reasons.append(
            "里程计子坐标系为 "
            f"{child_frame_id or '<空>'}，期望 {expected_child_frame_id}"
        )
    return reasons
