"""B2 地形点动安全规则的纯 Python 回归测试."""

import math

import pytest

from b2_twist_bridge_py.terrain_safety import (
    TerrainSafetyLimits,
    quaternion_wxyz_to_rpy,
)


def test_quaternion_wxyz_to_rpy_uses_unitree_order():
    """Unitree wxyz 顺序的正俯仰角应被正确解析."""
    half = math.radians(20.0) / 2.0
    roll, pitch, yaw = quaternion_wxyz_to_rpy(
        (math.cos(half), 0.0, math.sin(half), 0.0)
    )
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(math.radians(20.0))
    assert yaw == pytest.approx(0.0)


def test_quaternion_is_normalized_and_invalid_input_is_rejected():
    """非单位四元数可归一化，零四元数必须拒绝."""
    roll, _, _ = quaternion_wxyz_to_rpy((2.0, 0.2, 0.0, 0.0))
    assert roll > 0.0
    with pytest.raises(ValueError, match="接近零"):
        quaternion_wxyz_to_rpy((0.0, 0.0, 0.0, 0.0))


def test_limits_reject_tilt_timeout_error_and_wrong_gait():
    """姿态、超时、状态错误和步态不符均不得解锁."""
    limits = TerrainSafetyLimits(expected_gait_type=5)
    reasons = limits.reasons(
        roll_rad=math.radians(23.0),
        pitch_rad=math.radians(36.0),
        state_age_sec=0.31,
        error_code=7,
        gait_type=3,
    )
    assert len(reasons) == 5
    assert any("横滚角" in reason for reason in reasons)
    assert any("俯仰角" in reason for reason in reasons)
    assert any("gait_type=3" in reason for reason in reasons)


def test_unknown_expected_gait_accepts_observed_gait():
    """-1 表示只记录步态，不对未确认编号作假设."""
    reasons = TerrainSafetyLimits(expected_gait_type=-1).reasons(
        roll_rad=0.0,
        pitch_rad=0.0,
        state_age_sec=0.0,
        error_code=0,
        gait_type=255,
    )
    assert reasons == []


def test_unsafe_limits_are_rejected():
    """用户不能通过启动参数把姿态上限扩大到不受控."""
    with pytest.raises(ValueError, match="45"):
        TerrainSafetyLimits(max_abs_roll_rad=math.radians(46.0))
    with pytest.raises(ValueError, match="55"):
        TerrainSafetyLimits(max_abs_pitch_rad=math.radians(56.0))
