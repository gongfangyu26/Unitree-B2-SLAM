"""纯函数：校验地图点击并生成闭环巡航路线。"""

import math
from typing import Sequence


def occupancy_at(
    *,
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    x: float,
    y: float,
) -> int | None:
    """返回地图坐标对应的栅格值；越界或参数无效时返回 None。"""
    values = (resolution, origin_x, origin_y, x, y)
    if not all(math.isfinite(float(value)) for value in values):
        return None
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    column = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    if column < 0 or row < 0 or column >= width or row >= height:
        return None
    index = row * width + column
    if index >= len(data):
        return None
    return int(data[index])


def route_from_points(
    points: Sequence[tuple[float, float]], *, loop_route: bool
) -> list[dict]:
    """按点击顺序生成路线，并让航向指向下一个巡航点。"""
    if not points:
        raise ValueError("至少需要一个巡航点")
    normalized: list[tuple[float, float]] = []
    for x, y in points:
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            raise ValueError("巡航点坐标必须是有限数值")
        normalized.append((float(x), float(y)))

    route: list[dict] = []
    for index, (x, y) in enumerate(normalized):
        if len(normalized) == 1:
            yaw = 0.0
        elif index + 1 < len(normalized):
            next_x, next_y = normalized[index + 1]
            yaw = math.atan2(next_y - y, next_x - x)
        elif loop_route:
            next_x, next_y = normalized[0]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            previous_x, previous_y = normalized[index - 1]
            yaw = math.atan2(y - previous_y, x - previous_x)
        route.append(
            {
                "name": f"patrol_{index + 1:02d}",
                "inspect": False,
                "poses": [
                    {
                        "x": round(x, 3),
                        "y": round(y, 3),
                        "yaw_deg": round(math.degrees(yaw), 2),
                    }
                ],
            }
        )
    return route
