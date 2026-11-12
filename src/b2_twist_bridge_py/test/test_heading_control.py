"""B2 航向角换算和反馈限幅的纯 Python 回归测试。"""

import math

import pytest

from b2_twist_bridge_py.heading_control import (
    HeadingHoldConfig,
    quaternion_wxyz_to_yaw,
    wrap_angle,
)


def test_quaternion_wxyz_to_yaw_uses_unitree_order():
    """Unitree 的 w、x、y、z 四元数应正确得到正 90 度航向。"""
    half = math.radians(90.0) / 2.0
    yaw = quaternion_wxyz_to_yaw((math.cos(half), 0.0, 0.0, math.sin(half)))
    assert yaw == pytest.approx(math.pi / 2.0)


def test_quaternion_is_normalized_before_conversion():
    """非单位长度但方向正确的四元数也应得到正确航向。"""
    half = math.radians(-45.0) / 2.0
    quaternion = (2.0 * math.cos(half), 0.0, 0.0, 2.0 * math.sin(half))
    assert quaternion_wxyz_to_yaw(quaternion) == pytest.approx(-math.pi / 4.0)


def test_wrap_angle_uses_shortest_error_across_pi_boundary():
    """179 度到 -179 度应使用约 2 度的最短误差。"""
    error = wrap_angle(math.radians(-179.0) - math.radians(179.0))
    assert error == pytest.approx(math.radians(2.0))


def test_heading_correction_has_deadband_and_limit():
    """小误差不修正，大误差必须受角速度上限约束。"""
    config = HeadingHoldConfig(
        kp=1.0,
        deadband_rad=math.radians(2.0),
        max_correction_rad_s=0.10,
    )
    _, small = config.correction(0.0, math.radians(1.0))
    error, large = config.correction(0.0, math.radians(-30.0))

    assert small == 0.0
    assert error > 0.0
    assert large == pytest.approx(0.10)


def test_invalid_quaternion_and_unsafe_limit_are_rejected():
    """非法姿态和超过当前实测转向上限的修正不得进入控制器。"""
    with pytest.raises(ValueError, match="接近零"):
        quaternion_wxyz_to_yaw((0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="不超过 0.20"):
        HeadingHoldConfig(max_correction_rad_s=0.21)
