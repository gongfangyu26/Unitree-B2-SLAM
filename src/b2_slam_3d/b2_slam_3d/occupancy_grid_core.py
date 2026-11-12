"""与 ROS2 解耦的二维占据栅格健康检查逻辑."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class OccupancyGridSummary:
    """记录二维地图中各类栅格的数量和检查结论."""

    ok: bool
    reasons: tuple[str, ...]
    total_cells: int
    unknown_cells: int
    free_cells: int
    occupied_cells: int
    intermediate_cells: int

    @property
    def known_ratio(self) -> float:
        """返回已知栅格占总栅格的比例."""
        if self.total_cells <= 0:
            return 0.0
        return 1.0 - self.unknown_cells / self.total_cells


def analyze_occupancy_grid(
    *,
    width: int,
    height: int,
    resolution: float,
    frame_id: str,
    data: Sequence[int],
    expected_frame: str = "map",
    min_total_cells: int = 100,
    min_known_ratio: float = 0.01,
    free_threshold: int = 25,
    occupied_threshold: int = 65,
) -> OccupancyGridSummary:
    """检查 OccupancyGrid 的结构、取值和基本可导航性."""
    reasons: list[str] = []
    width = int(width)
    height = int(height)
    total_cells = max(0, width) * max(0, height)

    if width <= 0 or height <= 0:
        reasons.append("地图宽度和高度必须大于零")
    if not math.isfinite(float(resolution)) or float(resolution) <= 0.0:
        reasons.append("地图分辨率必须是大于零的有限数值")
    if not str(frame_id).strip():
        reasons.append("地图 frame_id 为空")
    elif expected_frame and str(frame_id) != str(expected_frame):
        reasons.append(
            f"地图坐标系为 {frame_id}，期望 {expected_frame}"
        )
    if total_cells < int(min_total_cells):
        reasons.append(
            f"地图栅格数量过少：{total_cells} < {int(min_total_cells)}"
        )
    if len(data) != total_cells:
        reasons.append(
            f"地图数据长度 {len(data)} 与宽高乘积 {total_cells} 不一致"
        )

    unknown_cells = 0
    free_cells = 0
    occupied_cells = 0
    intermediate_cells = 0
    invalid_values = 0
    for raw_value in data:
        value = int(raw_value)
        if value == -1:
            unknown_cells += 1
        elif 0 <= value <= free_threshold:
            free_cells += 1
        elif occupied_threshold <= value <= 100:
            occupied_cells += 1
        elif free_threshold < value < occupied_threshold:
            intermediate_cells += 1
        else:
            invalid_values += 1

    if invalid_values:
        reasons.append(f"地图含 {invalid_values} 个超出 -1 到 100 的栅格值")

    known_cells = total_cells - unknown_cells if total_cells > 0 else 0
    known_ratio = known_cells / total_cells if total_cells > 0 else 0.0
    if known_ratio < float(min_known_ratio):
        reasons.append(
            f"地图已知区域比例过低：{known_ratio:.3f} < "
            f"{float(min_known_ratio):.3f}"
        )
    if free_cells == 0:
        reasons.append("地图中没有可通行栅格")
    if occupied_cells == 0:
        reasons.append("地图中没有明确障碍栅格")

    return OccupancyGridSummary(
        ok=not reasons,
        reasons=tuple(reasons),
        total_cells=total_cells,
        unknown_cells=unknown_cells,
        free_cells=free_cells,
        occupied_cells=occupied_cells,
        intermediate_cells=intermediate_cells,
    )
