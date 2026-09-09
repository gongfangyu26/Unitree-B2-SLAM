"""B2 实机速度基线的纯 Python 回归测试."""

import math

import pytest

from b2_twist_bridge_py.velocity_policy import VelocityPolicy


def test_small_forward_command_uses_measured_starting_speed():
    """Nav2 小速度必须提升到实测能形成步态的 0.30 m/s."""
    x, y, z, notes = VelocityPolicy().shape(0.08, 0.0, 0.0)
    assert (x, y, z) == (0.30, 0.0, 0.0)
    assert notes


def test_zero_and_rotation_are_not_promoted_to_forward_motion():
    """原地转向和零速度不能意外产生前进速度."""
    policy = VelocityPolicy()
    assert policy.shape(0.0, 0.0, 0.0)[:3] == (0.0, 0.0, 0.0)
    assert policy.shape(0.0, 0.0, 0.10)[:3] == (0.0, 0.0, 0.10)


def test_verified_reverse_and_lateral_use_measured_speed():
    """后退和横移按 2026-09-09 实测最低有效速度整形."""
    reverse = VelocityPolicy().shape(-0.20, 0.0, 0.0)
    lateral = VelocityPolicy().shape(0.0, 0.08, 0.0)
    assert reverse[:3] == (-0.30, 0.0, 0.0)
    assert lateral[:3] == (0.0, 0.30, 0.0)
    assert reverse[3]
    assert lateral[3]


def test_unverified_combined_translation_is_blocked():
    """尚未实机验收的前后加横移组合指令必须停车."""
    x, y, z, notes = VelocityPolicy().shape(0.20, 0.08, 0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert any("组合运动" in note for note in notes)


def test_verified_directions_can_still_be_disabled_explicitly():
    """特殊场地仍可通过参数关闭后退和横移."""
    policy = VelocityPolicy(allow_reverse=False, allow_lateral=False)
    assert policy.shape(-0.10, 0.04, 0.0)[:3] == (0.0, 0.0, 0.0)


def test_non_finite_velocity_is_rejected():
    """非法导航速度不得进入 B2 Sport 请求."""
    with pytest.raises(ValueError, match="NaN"):
        VelocityPolicy().shape(math.nan, 0.0, 0.0)


def test_minimum_speed_cannot_exceed_limit():
    """配置错误不能绕过最大速度限制."""
    with pytest.raises(ValueError, match="最小有效"):
        VelocityPolicy(max_linear_x=0.30, min_effective_linear_x=0.31)


def test_minimum_lateral_speed_cannot_exceed_limit():
    """横移起步速度不能超过横移限速."""
    with pytest.raises(ValueError, match="最小有效横移"):
        VelocityPolicy(max_linear_y=0.30, min_effective_linear_y=0.31)
