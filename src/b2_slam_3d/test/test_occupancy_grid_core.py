"""二维占据栅格健康检查核心的单元测试."""

from b2_slam_3d.occupancy_grid_core import analyze_occupancy_grid


def test_valid_occupancy_grid_passes():
    """同时包含自由、障碍和未知区域的地图应通过."""
    data = [0] * 70 + [100] * 20 + [-1] * 10
    summary = analyze_occupancy_grid(
        width=10,
        height=10,
        resolution=0.05,
        frame_id="map",
        data=data,
    )
    assert summary.ok
    assert summary.free_cells == 70
    assert summary.occupied_cells == 20
    assert summary.known_ratio == 0.9


def test_all_unknown_grid_is_rejected():
    """全未知地图不能作为可导航地图."""
    summary = analyze_occupancy_grid(
        width=10,
        height=10,
        resolution=0.05,
        frame_id="map",
        data=[-1] * 100,
    )
    assert not summary.ok
    assert any("已知区域比例" in reason for reason in summary.reasons)


def test_wrong_data_length_and_frame_are_reported():
    """数据长度和坐标系冲突必须同时报告."""
    summary = analyze_occupancy_grid(
        width=20,
        height=20,
        resolution=0.1,
        frame_id="odom",
        data=[0, 100],
    )
    assert not summary.ok
    assert any("坐标系" in reason for reason in summary.reasons)
    assert any("数据长度" in reason for reason in summary.reasons)
