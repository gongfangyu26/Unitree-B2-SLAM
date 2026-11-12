"""B2 电脑端楼梯测试的纯逻辑回归测试."""

import json

import pytest

from b2_twist_bridge_py.stair_control import (
    MOTION_SWITCHER_API_IDS,
    STAIR_MOTION_ALIAS,
    AI_STEP_GAIT,
    AI_GAIT_NAMES,
    SWITCH_GAIT_API_ID,
    StairCommandLimits,
    motion_service_name,
    parse_checked_mode,
    select_mode_parameter,
    ai_step_activation_sequence,
    validate_step_probe_duration,
    stair_motion_mode_is_active,
)


def test_official_ai_mode_request_is_exact():
    """Verify the official ai alias and SelectMode API exactly."""
    assert MOTION_SWITCHER_API_IDS["SELECT_MODE"] == 1002
    assert STAIR_MOTION_ALIAS == "ai"
    assert json.loads(select_mode_parameter("ai")) == {"name": "ai"}
    with pytest.raises(ValueError, match="ai/ai_sport"):
        select_mode_parameter("advanced")


def test_ai_step_request_has_required_preconditions():
    """AI 文档的持续踏步请求为 2，必须先解除锁定并调用 ClassicWalk."""
    assert SWITCH_GAIT_API_ID == 1011
    assert AI_STEP_GAIT == 2
    assert "视觉" in AI_GAIT_NAMES[3]
    sequence = ai_step_activation_sequence()
    assert [item[1] for item in sequence] == [1002, 1049, 1011]
    assert [json.loads(item[2]) for item in sequence] == [
        {}, {"data": True}, {"data": 2},
    ]


def test_checked_mode_maps_official_alias_to_service():
    """B2 官方示例的 form/name 映射不得混淆."""
    form, name = parse_checked_mode('{"form":"0","name":"ai"}')
    assert stair_motion_mode_is_active(form, name)
    assert motion_service_name(form, name) == "ai_sport"
    assert motion_service_name("0", "normal") == "sport_mode"
    assert not stair_motion_mode_is_active("0", "advanced")


def test_invalid_check_mode_response_is_rejected():
    with pytest.raises(ValueError, match="form/name"):
        parse_checked_mode('{"name":"normal"}')
    with pytest.raises(ValueError, match="form/name"):
        parse_checked_mode("not-json")


def test_stair_direction_and_hard_limits():
    limits = StairCommandLimits()
    assert limits.velocity_for("up") == pytest.approx(0.30)
    assert limits.velocity_for("down") == pytest.approx(-0.30)
    with pytest.raises(ValueError, match="3.00"):
        StairCommandLimits(motion_duration_sec=3.01)
    with pytest.raises(ValueError, match="0.30"):
        StairCommandLimits(up_velocity_x=0.31)
    with pytest.raises(ValueError, match="未知"):
        limits.velocity_for("side")


@pytest.mark.parametrize("duration", [0.2, 0.5, 1.0, 10.0])
def test_ai_step_duration_accepts_ten_seconds(duration):
    assert validate_step_probe_duration(duration) == duration


@pytest.mark.parametrize("duration", [
    0.19, 10.01, float("nan"), float("inf"),
])
def test_ai_step_duration_rejects_invalid_values(duration):
    with pytest.raises(ValueError, match="10.00"):
        validate_step_probe_duration(duration)
