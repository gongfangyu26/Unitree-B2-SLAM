"""无 DDS 网络的 AI 原地踏步失败、取消及响应匹配测试."""

import json
import threading
import time
from types import MethodType, SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("unitree_api")

from std_srvs.srv import SetBool, Trigger  # noqa: E402
from unitree_api.msg import Response  # noqa: E402
from b2_twist_bridge_py.stair_remote_control import (  # noqa: E402
    StairRemoteControl, _PendingRpc,
)


def harness(fail_label=None, cancel_label=None):
    """绑定真实回调，替换发布器；本测试绝不创建 ROS 节点或 DDS 端点."""
    node = SimpleNamespace(
        _lock=threading.RLock(),
        _sport_command_lock=threading.RLock(),
        _activation_lock=threading.Lock(),
        _activation_cancel=threading.Event(),
        ai_step_probe_active=False,
        armed=False,
        activation_results=[],
        last_state={"gait_type": 1},
        step_probe_duration=0.001,
        calls=[],
        stops=[],
        last_probe_result="",
        last_fault="",
        probe_cancel_reason="",
    )
    node._write_log = lambda *args: None
    node._state_reasons = lambda now: []
    node._check_motion_mode = lambda: (0, "0", "ai", "ai_sport")
    node._wait_precondition_state = lambda label: ""

    def stop(reason):
        if (
            node.ai_step_probe_active
            and not node._activation_cancel.is_set()
        ):
            node.probe_cancel_reason = reason
        node._activation_cancel.set()
        node.stops.append(reason)
        node.armed = False

    def call(label, api, parameter, cancellable=True):
        node.calls.append((label, api, json.loads(parameter)))
        if label == cancel_label:
            node._activation_cancel.set()
        code = -1 if label == fail_label else 0
        node.activation_results.append({"function": label, "code": code})
        return code

    node.stop_and_disarm = stop
    node._probe_call = call
    node.activate = MethodType(
        StairRemoteControl._activate_mode_callback, node
    )
    return node


def test_fault_is_reported_even_after_gait_request_succeeds():
    node = harness()
    original_call = node._probe_call

    def call(label, api, parameter, cancellable=True):
        code = original_call(label, api, parameter, cancellable)
        if label == "SwitchGait(2)":
            node.last_fault = "SportModeState 错误码=1"
            node.probe_cancel_reason = "安全联锁：" + node.last_fault
            node._activation_cancel.set()
        return code

    node._probe_call = call
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert "错误码=1" in response.message
    assert node.calls[-1][2] == {"data": 0}


def test_existing_robot_fault_prevents_activation_even_with_ten_seconds():
    node = harness()
    node.step_probe_duration = 10.0
    node._state_reasons = lambda now: ["SportModeState 错误码=1"]
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert node.calls == []
    assert "错误码=1" in response.message


def test_failed_settle_prevents_further_gait_commands():
    node = harness()
    node._wait_precondition_state = (
        lambda label: f"{label} 后状态联锁：错误码=1"
    )
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert [call[0] for call in node.calls] == [
        "BalanceStand", "StopMove", "SwitchGait(0)",
    ]


def test_wait_precondition_rejects_robot_fault():
    node = harness()
    node.last_state_time = time.monotonic()
    node._state_reasons = lambda now: ["SportModeState 错误码=1"]
    detail = StairRemoteControl._wait_precondition_state(
        node, "BalanceStand"
    )
    assert "错误码=1" in detail


def test_wait_precondition_requires_new_telemetry(monkeypatch):
    node = harness()
    node.last_state_time = 0.0
    node._activation_cancel = SimpleNamespace(
        is_set=lambda: False, wait=lambda timeout: None,
    )
    ticks = iter([0.0, 0.5, 0.5, 1.1, 1.1, 4.1])
    monkeypatch.setattr(time, "monotonic", lambda: next(ticks))
    detail = StairRemoteControl._wait_precondition_state(
        node, "ClassicWalk(true)"
    )
    assert "未收到新的" in detail


def test_wait_precondition_accepts_fresh_healthy_telemetry(monkeypatch):
    node = harness()
    node.last_state_time = 1.05
    ticks = iter([0.0, 1.1, 1.1])
    monkeypatch.setattr(time, "monotonic", lambda: next(ticks))
    assert StairRemoteControl._wait_precondition_state(
        node, "BalanceStand"
    ) == ""


def test_classic_walk_waits_full_three_seconds(monkeypatch):
    node = harness()
    node.last_state_time = 3.05
    observations = []
    node._write_log = lambda event, details: observations.append(
        json.loads(details)
    )
    node._activation_cancel = SimpleNamespace(
        is_set=lambda: False, wait=lambda timeout: None,
    )
    ticks = iter([0.0, 1.1, 1.1, 2.5, 2.5, 3.1, 3.1])
    monkeypatch.setattr(time, "monotonic", lambda: next(ticks))
    assert StairRemoteControl._wait_precondition_state(
        node, "ClassicWalk(true)"
    ) == ""
    assert observations[0]["settle_sec"] == 3.0


def test_normal_watchdog_completion_is_not_a_fault():
    node = harness()
    original_call = node._probe_call

    def call(label, api, parameter, cancellable=True):
        code = original_call(label, api, parameter, cancellable)
        if label == "SwitchGait(2)":
            node.probe_deadline = time.monotonic() - 0.01
            node.probe_cancel_reason = "原地踏步硬超时"
            node._activation_cancel.set()
        return code

    node._probe_call = call
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert response.success
    assert "rpc_sequence_completed" in response.message


def test_probe_runs_official_sequence_then_stops_without_arming():
    node = harness()
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert response.success
    assert [call[1] for call in node.calls] == [
        1002, 1049, 1011, 1003, 1011,
    ]
    assert node.calls[2][2] == {"data": 2}
    assert node.calls[-1][2] == {"data": 0}
    assert all(call[1] != 1008 for call in node.calls)
    assert not node.ai_step_probe_active
    assert not node.armed


@pytest.mark.parametrize("failed", [
    "BalanceStand", "ClassicWalk(true)", "SwitchGait(2)",
])
def test_failed_precondition_aborts_remaining_actions_and_stops(failed):
    node = harness(fail_label=failed)
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    labels = [call[0] for call in node.calls]
    expected = ["BalanceStand", "ClassicWalk(true)", "SwitchGait(2)"]
    assert labels == expected[:expected.index(failed) + 1] + [
        "StopMove", "SwitchGait(0)",
    ]
    assert not node.armed


def test_stop_during_classic_walk_prevents_step_request():
    node = harness(cancel_label="ClassicWalk(true)")
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert "SwitchGait(2)" not in [call[0] for call in node.calls]
    assert node.calls[-1][2] == {"data": 0}


def test_stale_state_prevents_all_activation_requests():
    node = harness()
    node._state_reasons = lambda now: ["SportModeState 已超时"]
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert node.calls == []


def test_stop_failure_is_not_reported_as_success():
    node = harness(fail_label="SwitchGait(0)")
    response = node.activate(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert "停车确认失败" in response.message


def test_stair_arming_is_blocked_even_after_probe():
    node = harness()
    response = StairRemoteControl._set_armed_callback(
        node, SetBool.Request(data=True), SetBool.Response()
    )
    assert not response.success
    assert not node.armed


def test_response_requires_matching_id_channel_and_api():
    pending = _PendingRpc(api=1011, channel="sport")
    node = SimpleNamespace(
        _pending_lock=threading.Lock(), _pending={17: pending},
    )
    msg = Response()
    msg.header.identity.id = 17
    msg.header.identity.api_id = 1002
    StairRemoteControl._response_callback(node, "sport", msg)
    assert not pending.event.is_set()
    msg.header.identity.api_id = 1011
    StairRemoteControl._response_callback(node, "motion", msg)
    assert not pending.event.is_set()
    msg.header.identity.id = 18
    StairRemoteControl._response_callback(node, "sport", msg)
    assert not pending.event.is_set()
    msg.header.identity.id = 17
    msg.header.status.code = -1
    StairRemoteControl._response_callback(node, "sport", msg)
    assert pending.event.is_set()
    assert pending.code == -1


def test_cancelled_rpc_never_publishes():
    sent = []
    node = SimpleNamespace(
        _rpc_serial_lock=threading.Lock(),
        _pending_lock=threading.Lock(), _pending={},
        _sport_command_lock=threading.RLock(),
        _activation_cancel=threading.Event(),
        sport_request_pub=SimpleNamespace(publish=sent.append),
    )
    node._activation_cancel.set()
    code, _ = StairRemoteControl._rpc_call(
        node, "sport", 1011, '{"data":2}', cancellable=True
    )
    assert code != 0
    assert sent == []
    assert node._pending == {}


def test_watchdog_stops_probe_at_deadline():
    node = harness()
    node.ai_step_probe_active = True
    node.probe_deadline = time.monotonic() - 0.1
    node.active_direction = None
    node.motion_deadline = 0.0
    StairRemoteControl._watchdog(node)
    assert node._activation_cancel.is_set()
    assert node.stops == ["原地踏步硬超时"]


def test_missing_rpc_response_times_out_and_cleans_pending():
    sent = []
    node = SimpleNamespace(
        _rpc_serial_lock=threading.Lock(),
        _pending_lock=threading.Lock(), _pending={},
        _sport_command_lock=threading.RLock(),
        _activation_cancel=threading.Event(),
        sport_request_pub=SimpleNamespace(publish=sent.append),
    )
    code, detail = StairRemoteControl._rpc_call(
        node, "sport", 1049, '{"data":true}', timeout_sec=0.001,
    )
    assert code != 0
    assert detail == "response_timeout"
    assert len(sent) == 1
    assert sent[0].header.identity.id != 0
    assert node._pending == {}
