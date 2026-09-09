"""巡检路线配置单元测试."""

import pytest

from b2_factory_inspection.route_config import load_route


def test_load_valid_route(tmp_path):
    """有效路线应被规范化为浮点位姿."""
    path = tmp_path / "route.yaml"
    path.write_text(
        """
frame_id: map
waypoints:
  - name: target_a
    inspect: true
    poses:
      - {x: 1, y: 2.5, yaw_deg: -10}
""",
        encoding="utf-8",
    )

    frame_id, waypoints = load_route(path)

    assert frame_id == "map"
    assert waypoints[0]["name"] == "target_a"
    assert waypoints[0]["poses"][0] == {
        "x": 1.0,
        "y": 2.5,
        "yaw_deg": -10.0,
    }


def test_reject_duplicate_waypoint_name(tmp_path):
    """重复名称会让目标检测和日志对应错误，必须拒绝."""
    path = tmp_path / "duplicate.yaml"
    path.write_text(
        """
waypoints:
  - {name: same, poses: [{x: 0, y: 0}]}
  - {name: same, poses: [{x: 1, y: 1}]}
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="名称重复"):
        load_route(path)


def test_reject_non_finite_yaw(tmp_path):
    """无穷大偏航角不能进入 Nav2 目标四元数."""
    path = tmp_path / "invalid.yaml"
    path.write_text(
        """
waypoints:
  - name: bad
    poses: [{x: 0, y: 0, yaw_deg: .inf}]
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="有限数值"):
        load_route(path)


def test_reject_duplicate_pose(tmp_path):
    """重复候选位姿只会产生无意义重拍，必须拒绝."""
    path = tmp_path / "duplicate_pose.yaml"
    path.write_text(
        """
waypoints:
  - name: target
    poses:
      - {x: 0, y: 0, yaw_deg: 0}
      - {x: 0, y: 0, yaw_deg: 0}
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="重复候选位姿"):
        load_route(path)
