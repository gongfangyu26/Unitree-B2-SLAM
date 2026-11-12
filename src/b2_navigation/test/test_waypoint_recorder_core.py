"""巡航点采集核心逻辑测试。"""

import pytest

from b2_navigation.waypoint_recorder_core import (
    occupancy_at,
    route_from_points,
)


def test_occupancy_lookup_and_bounds():
    """正确读取栅格并拒绝地图外坐标。"""
    data = [0, 20, 80, -1]
    assert occupancy_at(
        data=data,
        width=2,
        height=2,
        resolution=1.0,
        origin_x=-1.0,
        origin_y=-1.0,
        x=-0.5,
        y=-0.5,
    ) == 0
    assert occupancy_at(
        data=data,
        width=2,
        height=2,
        resolution=1.0,
        origin_x=-1.0,
        origin_y=-1.0,
        x=1.1,
        y=0.0,
    ) is None


def test_loop_route_faces_next_point():
    """闭环路线每个点均朝向下一个点。"""
    route = route_from_points(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], loop_route=True
    )
    assert route[0]["poses"][0]["yaw_deg"] == pytest.approx(0.0)
    assert route[1]["poses"][0]["yaw_deg"] == pytest.approx(90.0)
    assert route[2]["poses"][0]["yaw_deg"] == pytest.approx(-135.0)


def test_empty_route_is_rejected():
    """不允许保存空路线。"""
    with pytest.raises(ValueError, match="至少需要"):
        route_from_points([], loop_route=True)
