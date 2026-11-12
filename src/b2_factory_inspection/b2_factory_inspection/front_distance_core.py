"""与 ROS2 解耦的前向点云距离稳健估计。"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class FrontDistanceConfig:
    """前向 ROI、稠密距离簇和有效点数量参数。"""

    x_min: float = 0.30
    x_max: float = 5.00
    y_abs_max: float = 0.50
    z_min: float = -0.20
    z_max: float = 1.50
    cluster_width: float = 0.20
    min_roi_points: int = 20
    min_cluster_points: int = 12

    def __post_init__(self) -> None:
        numeric = (
            self.x_min,
            self.x_max,
            self.y_abs_max,
            self.z_min,
            self.z_max,
            self.cluster_width,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("前向测距参数必须是有限数值")
        if not 0.0 <= self.x_min < self.x_max:
            raise ValueError("x_min 必须非负且小于 x_max")
        if self.y_abs_max <= 0.0 or self.z_min >= self.z_max:
            raise ValueError("横向或高度 ROI 参数无效")
        if self.cluster_width <= 0.0:
            raise ValueError("cluster_width 必须大于零")
        if self.min_roi_points < 1 or self.min_cluster_points < 1:
            raise ValueError("点数阈值必须是正整数")
        if self.min_cluster_points > self.min_roi_points:
            raise ValueError("min_cluster_points 不能大于 min_roi_points")


@dataclass(frozen=True)
class FrontDistanceEstimate:
    """一帧点云中的最近可靠前向表面。"""

    distance_x: float
    lateral_y: float
    height_z: float
    roi_points: int
    cluster_points: int
    spread_x: float


def estimate_front_distance(
    points_xyz: np.ndarray,
    config: FrontDistanceConfig,
) -> FrontDistanceEstimate | None:
    """过滤 ROI，并返回最近一个达到点数要求的稠密距离簇。"""
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.size == 0:
        return None
    if points.ndim == 1 and points.shape[0] == 3:
        points = points.reshape(1, 3)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("点云数组必须是 N×3 的 x、y、z")

    finite = np.isfinite(points).all(axis=1)
    roi_mask = (
        finite
        & (points[:, 0] >= config.x_min)
        & (points[:, 0] <= config.x_max)
        & (np.abs(points[:, 1]) <= config.y_abs_max)
        & (points[:, 2] >= config.z_min)
        & (points[:, 2] <= config.z_max)
    )
    roi = points[roi_mask]
    if len(roi) < config.min_roi_points:
        return None

    sorted_x = np.sort(roi[:, 0])
    cluster_start = None
    cluster_end = None
    for index, start in enumerate(sorted_x):
        end_index = int(
            np.searchsorted(
                sorted_x, start + config.cluster_width, side="right"
            )
        )
        if end_index - index >= config.min_cluster_points:
            cluster_start = float(start)
            cluster_end = float(start + config.cluster_width)
            break
    if cluster_start is None or cluster_end is None:
        return None

    cluster = roi[
        (roi[:, 0] >= cluster_start) & (roi[:, 0] <= cluster_end)
    ]
    distance_x = float(np.median(cluster[:, 0]))
    lateral_y = float(np.median(cluster[:, 1]))
    height_z = float(np.median(cluster[:, 2]))
    spread_x = float(np.median(np.abs(cluster[:, 0] - distance_x)))
    return FrontDistanceEstimate(
        distance_x=distance_x,
        lateral_y=lateral_y,
        height_z=height_z,
        roi_points=int(len(roi)),
        cluster_points=int(len(cluster)),
        spread_x=spread_x,
    )


class MultiFrameDistanceFilter:
    """只在最近窗口中有足够有效帧时输出中位距离。"""

    def __init__(
        self, window_size: int = 5, min_valid_frames: int = 3
    ) -> None:
        if window_size < 1:
            raise ValueError("window_size 必须是正整数")
        if not 1 <= min_valid_frames <= window_size:
            raise ValueError("min_valid_frames 必须位于 1 到 window_size")
        self.window_size = int(window_size)
        self.min_valid_frames = int(min_valid_frames)
        self._values: deque[float | None] = deque(maxlen=self.window_size)

    @property
    def valid_count(self) -> int:
        """返回当前窗口中的有效距离数量。"""
        return sum(value is not None for value in self._values)

    def update(self, distance: float | None) -> float | None:
        """加入一帧结果，并在证据足够时返回中位数。"""
        if distance is None:
            self._values.append(None)
        else:
            value = float(distance)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("距离必须是非负有限数值")
            self._values.append(value)
        valid: Iterable[float] = (
            value for value in self._values if value is not None
        )
        valid_values = list(valid)
        if len(valid_values) < self.min_valid_frames:
            return None
        return float(np.median(valid_values))
