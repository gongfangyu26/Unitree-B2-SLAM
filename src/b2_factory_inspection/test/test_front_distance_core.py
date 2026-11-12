"""前向点云稠密距离簇和多帧滤波测试。"""

import numpy as np
import pytest

from b2_factory_inspection.front_distance_core import (
    FrontDistanceConfig,
    MultiFrameDistanceFilter,
    estimate_front_distance,
)


def test_single_near_outlier_does_not_replace_dense_object():
    """单个近距离噪点不能覆盖约一米处的稠密物体。"""
    rng = np.random.default_rng(7)
    outlier = np.array([[0.36, 0.0, 0.2]])
    obstacle = np.column_stack(
        (
            rng.normal(1.00, 0.02, 40),
            rng.normal(0.0, 0.08, 40),
            rng.normal(0.2, 0.05, 40),
        )
    )
    background = np.column_stack(
        (
            rng.normal(3.0, 0.04, 60),
            rng.normal(0.0, 0.15, 60),
            rng.normal(0.3, 0.08, 60),
        )
    )

    estimate = estimate_front_distance(
        np.vstack((outlier, obstacle, background)), FrontDistanceConfig()
    )

    assert estimate is not None
    assert estimate.distance_x == pytest.approx(1.0, abs=0.04)
    assert estimate.cluster_points >= 12


def test_ground_and_outside_roi_are_filtered():
    """低于地面阈值和侧向 ROI 外的点不能形成前方目标。"""
    ground = np.array([[0.8, 0.0, -0.35]] * 40)
    side = np.array([[0.7, 0.8, 0.2]] * 40)
    obstacle = np.array([[1.5, 0.1, 0.1]] * 20)

    estimate = estimate_front_distance(
        np.vstack((ground, side, obstacle)), FrontDistanceConfig()
    )

    assert estimate is not None
    assert estimate.distance_x == pytest.approx(1.5)
    assert estimate.roi_points == 20


def test_sparse_points_do_not_produce_valid_distance():
    """稀疏噪声不足以达到点数门限时必须返回无效。"""
    points = np.array([[0.5 + index * 0.25, 0.0, 0.2] for index in range(15)])
    assert estimate_front_distance(points, FrontDistanceConfig()) is None


def test_multiframe_median_requires_evidence_and_rejects_one_outlier():
    """多帧输出需满足有效帧数，且单帧异常不能明显拉偏结果。"""
    distance_filter = MultiFrameDistanceFilter(
        window_size=5, min_valid_frames=3
    )

    assert distance_filter.update(1.00) is None
    assert distance_filter.update(None) is None
    assert distance_filter.update(1.02) is None
    assert distance_filter.update(0.40) == pytest.approx(1.00)
    assert distance_filter.update(0.98) == pytest.approx(0.99)


def test_invalid_configuration_is_rejected():
    """相互矛盾的 ROI 或点数参数不能静默运行。"""
    with pytest.raises(ValueError, match="x_min"):
        FrontDistanceConfig(x_min=2.0, x_max=1.0)
    with pytest.raises(ValueError, match="不能大于"):
        FrontDistanceConfig(min_roi_points=5, min_cluster_points=6)
