"""巡逻闭环、失败边界和迟到回调隔离检查。"""

import math

import pytest

from b2_factory_inspection.corridor_patrol_core import (
    PatrolLimits, PatrolSession, amcl_evidence_ok, check_path, patrol_route,
)
from b2_factory_inspection.route_config import load_route


def waypoints():
    """模拟现有 recorder 的 YAML 解析结果。"""
    return [
        {"name": name, "inspect": False, "poses": [{"x": x, "y": y, "yaw_deg": 0.0}]}
        for name, x, y in (("A", 0.0, 0.0), ("B", 1.0, 0.0), ("C", 1.0, 1.0))
    ]


def test_closed_route_returns_to_first_after_requested_laps():
    """两圈应为 A B C A B C A，包含最后一段和最终航向。"""
    route = patrol_route("map", waypoints(), "loop", 2)
    assert [p.name for p in route] == ["A", "B", "C", "A", "B", "C", "A"]
    assert route[0].yaw == route[-1].yaw == 0.0
    assert route[1].yaw == pytest.approx(math.pi / 2)
    assert route[2].yaw == pytest.approx(-3 * math.pi / 4)


def test_once_does_not_turn_terminal_point_back_to_start():
    """短段首测的最终航向取到达方向，不采用 recorder 的闭环末点航向。"""
    route = patrol_route("map", waypoints(), "once", 1)
    assert len(route) == 3
    assert route[-1].yaw == pytest.approx(math.pi / 2)


@pytest.mark.parametrize("mode,laps", [
    ("loop", 0), ("loop", -1), ("loop", True),
    ("loop", 1.5), ("loop", 11), ("once", 2),
])
def test_reject_unbounded_or_ambiguous_repetitions(mode, laps):
    """配置错误不得隐式进入无限循环。"""
    with pytest.raises(ValueError):
        patrol_route("map", waypoints(), mode, laps)


def test_reject_photo_missions_and_duplicate_closure():
    """避免把观测候选或重复首点混入纯巡逻。"""
    points = waypoints()
    points[0]["inspect"] = True
    with pytest.raises(ValueError, match="inspect=false"):
        patrol_route("map", points, "loop", 1)
    points[0]["inspect"] = False
    points.append(points[0])
    with pytest.raises(ValueError, match="不要重复首点"):
        patrol_route("map", points, "loop", 1)


def test_existing_recorder_schema_is_accepted(tmp_path):
    """复用现有文件格式，不需要额外填写演示坐标。"""
    route_file = tmp_path / "route.yaml"
    route_file.write_text(
        "frame_id: map\nwaypoints:\n"
        "- {name: A, inspect: false, poses: [{x: 0, y: 0, yaw_deg: 90}]}\n"
        "- {name: B, inspect: false, poses: [{x: 1, y: 0, yaw_deg: 180}]}\n",
        encoding="utf-8",
    )
    frame, points = load_route(route_file)
    assert [p.name for p in patrol_route(frame, points, "loop", 1)] == ["A", "B", "A"]


@pytest.mark.parametrize("path,frame,maximum", [
    ([], "map", 5),
    ([(0, 0), (1, 0)], "odom", 5),
    ([(0, 0), (math.nan, 0)], "map", 5),
    ([(0.5, 0), (1, 0)], "map", 5),
    ([(0, 0), (0.5, 0)], "map", 5),
    ([(0, 0), (10, 0), (1, 0)], "map", 5),
])
def test_reject_empty_approximate_or_excessively_long_paths(path, frame, maximum):
    """planner 成功状态不足以证明规划路径覆盖目标。"""
    with pytest.raises(ValueError):
        check_path(path, frame, (0, 0), (1, 0), maximum)


def test_measure_real_path_length():
    """使用路径折线长度，不能仅按端点直线距离检查。"""
    assert check_path([(0, 0), (1, 0), (1, 1)], "map", (0, 0), (1, 1), 3) == 2


def test_localization_requires_pose_evidence_and_finite_covariance():
    """仅 TF 存在、零四元数或大协方差不能形成 AMCL 证据。"""
    covariance = [0.0] * 36
    assert amcl_evidence_ok("map", (0, 0, 0), (0, 0, 0, 1), covariance)
    assert not amcl_evidence_ok("odom", (0, 0, 0), (0, 0, 0, 1), covariance)
    assert not amcl_evidence_ok("map", (0, 0, 0), (0, 0, 0, 0), covariance)
    covariance[7] = 0.26
    assert not amcl_evidence_ok("map", (0, 0, 0), (0, 0, 0, 1), covariance)
    covariance[7] = math.nan
    assert not amcl_evidence_ok("map", (0, 0, 0), (0, 0, 0, 1), covariance)


def ready_session():
    """模拟三段检查通过，但尚未发送任何导航目标。"""
    session = PatrolSession(PatrolLimits(goal_timeout_sec=25.0, mission_timeout_sec=60.0))
    session.validating(3, 10)
    for now in (11, 12, 13):
        session.planned(now)
    assert session.phase == "ready"
    return session


def test_full_validation_is_required_and_expires():
    """初始和过期状态均拒绝直接启动。"""
    session = PatrolSession(PatrolLimits())
    with pytest.raises(ValueError):
        session.start(10)
    session = ready_session()
    with pytest.raises(ValueError):
        session.start(74)


def test_stop_and_new_run_reject_old_action_callbacks():
    """暂停后晚到的 goal accepted/result 不能推进任务。"""
    session = ready_session()
    session.start(14)
    old_generation = session.generation
    assert session.current(old_generation, 0, "running")
    session.stop()
    assert not session.current(old_generation, 0, "running")
    with pytest.raises(ValueError):
        session.start(15)
    session.validating(1, 16)
    session.planned(17)
    session.start(18)
    assert not session.current(old_generation, 0, "running")


def test_goal_and_total_deadlines_do_not_reset_with_new_goals():
    """推进到下一目标可更新单段期限，不能延长总任务期限。"""
    session = ready_session()
    session.start(14)
    assert session.timeout_reason(39) == "导航目标超时"
    assert not session.arrived(30)
    assert session.timeout_reason(54) is None
    assert not session.arrived(50)
    assert session.timeout_reason(74) == "达到总任务时间上限"
    assert session.arrived(70)


def test_hundred_metre_mission_can_continue_past_sixty_seconds():
    """默认百米任务应跨越一分钟，且换目标不能延长十分钟总预算。"""
    session = PatrolSession(PatrolLimits())
    assert session.limits.goal_timeout_sec == 60.0
    assert session.limits.mission_timeout_sec == 600.0
    session.validating(21, 10.0)
    for index in range(21):
        session.planned(11.0 + index * 0.1)
    session.start(20.0)
    assert not session.arrived(60.0)
    assert session.timeout_reason(85.0) is None  # 已运行 65 秒
    for now in range(100, 601, 40):
        assert session.timeout_reason(now) is None
        assert not session.arrived(now)
    assert session.timeout_reason(620.0) == "达到总任务时间上限"


@pytest.mark.parametrize("kwargs", [
    {"goal_timeout_sec": 0}, {"mission_timeout_sec": math.inf},
    {"mission_timeout_sec": 601}, {"planning_timeout_sec": math.nan},
    {"goal_timeout_sec": 601}, {"max_segment_m": -1},
])
def test_reject_invalid_limits(kwargs):
    """运动不能依赖非法、无限或互相矛盾的期限。"""
    with pytest.raises(ValueError):
        PatrolLimits(**kwargs)
