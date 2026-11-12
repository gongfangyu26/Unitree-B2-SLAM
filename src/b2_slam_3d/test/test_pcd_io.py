"""PointCloud2 导出 binary PCD 的回归测试."""

import math
from pathlib import Path
import struct

import pytest
from sensor_msgs.msg import PointCloud2, PointField

from b2_slam_3d.pcd_io import write_pointcloud2_pcd


def _cloud_with_one_invalid_point() -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = 2
    msg.fields = [
        PointField(
            name=name,
            offset=index * 4,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for index, name in enumerate(("x", "y", "z", "intensity"))
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 32
    msg.data = struct.pack(
        "<ffffffff",
        1.0,
        2.0,
        3.0,
        4.0,
        math.nan,
        6.0,
        7.0,
        8.0,
    )
    msg.is_dense = False
    return msg


def test_write_binary_pcd_filters_invalid_xyz(tmp_path: Path):
    """导出保留字段，并且不把 NaN 坐标写入实验点云."""
    path, point_count = write_pointcloud2_pcd(
        _cloud_with_one_invalid_point(),
        tmp_path / "map.pcd",
        min_points=1,
    )
    payload = path.read_bytes()
    header, binary = payload.split(b"DATA binary\n", maxsplit=1)
    assert b"FIELDS x y z intensity" in header
    assert b"WIDTH 1" in header
    assert b"POINTS 1" in header
    assert point_count == 1
    assert struct.unpack("<ffff", binary) == pytest.approx(
        (1.0, 2.0, 3.0, 4.0)
    )


def test_existing_file_is_not_overwritten_by_default(tmp_path: Path):
    """已有实验点云默认不得被覆盖."""
    path = tmp_path / "map.pcd"
    path.write_bytes(b"keep")
    with pytest.raises(FileExistsError, match="拒绝覆盖"):
        write_pointcloud2_pcd(_cloud_with_one_invalid_point(), path)
    assert path.read_bytes() == b"keep"


def test_empty_frame_and_too_few_points_are_rejected(tmp_path: Path):
    """空坐标系或有效点不足不应生成伪成功文件."""
    msg = _cloud_with_one_invalid_point()
    msg.header.frame_id = ""
    with pytest.raises(ValueError, match="frame_id"):
        write_pointcloud2_pcd(msg, tmp_path / "empty_frame.pcd")

    msg.header.frame_id = "map"
    with pytest.raises(ValueError, match="有效点数不足"):
        write_pointcloud2_pcd(
            msg,
            tmp_path / "too_small.pcd",
            min_points=2,
        )


def test_export_preserves_nonplanar_heights(tmp_path: Path):
    """PCD 保存应保留来自地图消息的不同高度，不做二维投影。"""
    cloud = _cloud_with_one_invalid_point()
    points = ((1.0, 2.0, -0.5, 4.0), (3.0, 4.0, 2.5, 8.0))
    cloud.data = struct.pack("<8f", *(value for point in points for value in point))
    path, count = write_pointcloud2_pcd(cloud, tmp_path / "corridor.pcd", min_points=2)
    header, data = path.read_bytes().split(b"DATA binary\n", maxsplit=1)
    assert b"POINTS 2" in header and count == 2
    assert struct.unpack("<8f", data) == pytest.approx(tuple(v for p in points for v in p))
