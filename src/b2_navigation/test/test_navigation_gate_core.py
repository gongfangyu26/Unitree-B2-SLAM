"""人工点击导航安全门状态机的单元测试."""

from b2_navigation.navigation_gate_core import (
    NavigationGateConfig,
    NavigationGateState,
)


def make_state() -> NavigationGateState:
    """创建使用短超时的测试状态机."""
    return NavigationGateState(
        NavigationGateConfig(
            autonomy_timeout_sec=0.8,
            map_timeout_sec=3.0,
            max_armed_duration_sec=60.0,
        )
    )


def test_gate_starts_locked_and_rejects_missing_safety_state():
    """没有任何安全证据时必须拒绝解锁."""
    state = make_state()
    success, reason = state.request_arm(10.0)
    assert not success
    assert "自主安全状态" in reason
    assert not state.armed


def test_gate_arms_only_after_map_and_autonomy_pass():
    """地图和自主安全状态同时新鲜时才允许解锁."""
    state = make_state()
    state.update_autonomy(True, 10.0)
    state.update_map(True, 10.0)
    success, _ = state.request_arm(10.1)
    assert success
    assert state.active_reason(10.2) is None


def test_gate_detects_stale_autonomy_and_maximum_duration():
    """状态超时和最长解锁时间均应阻止速度继续通过."""
    state = make_state()
    state.update_autonomy(True, 10.0)
    state.update_map(True, 10.0)
    assert state.request_arm(10.1)[0]
    assert "自主安全状态" in state.active_reason(11.0)

    state.update_autonomy(True, 20.0)
    state.update_map(True, 20.0)
    assert state.request_arm(20.0)[0]
    state.update_autonomy(True, 80.1)
    state.update_map(True, 80.1)
    assert "最长解锁时间" in state.active_reason(80.1)


def test_corridor_gate_can_remain_armed_beyond_one_minute():
    """百米走廊使用 600 秒任务限时；安全状态持续刷新时不在 60 秒截断。"""
    state = NavigationGateState(NavigationGateConfig(max_armed_duration_sec=600.0))
    state.update_autonomy(True, 10.0)
    state.update_map(True, 10.0)
    assert state.request_arm(10.0)[0]
    for now in (71.0, 350.0, 609.9):
        state.update_autonomy(True, now)
        state.update_map(True, now)
        assert state.active_reason(now) is None
    state.update_autonomy(True, 610.1)
    state.update_map(True, 610.1)
    assert "最长解锁时间" in state.active_reason(610.1)
