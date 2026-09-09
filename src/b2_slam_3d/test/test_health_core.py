"""SLAM 输入结构检查核心的单元测试."""

import pytest

from b2_slam_3d.health_core import (
    estimate_rate_hz,
    validate_odometry_values,
    validate_pointcloud_layout,
)


def test_rate_estimation_uses_intervals():
    """三帧覆盖两个间隔，频率应按间隔数计算."""
    assert estimate_rate_hz((1.0, 1.1, 1.2)) == pytest.approx(10.0)


def test_valid_pointcloud_layout_passes():
    """含 xyz 且字节布局完整的点云应通过."""
    reasons = validate_pointcloud_layout(
        width=2000,
        height=1,
        point_step=16,
        row_step=32000,
        data_length=32000,
        field_names=("x", "y", "z", "intensity"),
        frame_id="rslidar",
        min_points=1000,
    )
    assert reasons == []


def test_malformed_pointcloud_reports_all_key_problems():
    """空坐标系、少点、缺字段和字节不足应被明确报告."""
    reasons = validate_pointcloud_layout(
        width=10,
        height=1,
        point_step=16,
        row_step=10,
        data_length=5,
        field_names=("x",),
        frame_id="",
        min_points=1000,
    )
    assert len(reasons) >= 5


def test_valid_odometry_passes():
    """有限数值、单位四元数和正确坐标系应通过."""
    reasons = validate_odometry_values(
        position=(0.0, 0.0, 0.5),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        linear_velocity=(0.0, 0.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
        frame_id="odom",
        child_frame_id="base_link",
        expected_frame_id="odom",
        expected_child_frame_id="base_link",
    )
    assert reasons == []


def test_wrong_odometry_frames_are_rejected():
    """不能把未知坐标系的板载里程计直接接入当前 TF 树."""
    reasons = validate_odometry_values(
        position=(0.0, 0.0, 0.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        linear_velocity=(0.0, 0.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
        frame_id="camera_init",
        child_frame_id="body",
        expected_frame_id="odom",
        expected_child_frame_id="base_link",
    )
    assert len(reasons) == 2
