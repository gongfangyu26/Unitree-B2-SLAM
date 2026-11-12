"""将 ROS2 PointCloud2 安全地导出为紧凑的 binary PCD 文件."""

from pathlib import Path
import os
import re

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


_PCD_TYPES = {
    PointField.INT8: ("i1", 1, "I"),
    PointField.UINT8: ("u1", 1, "U"),
    PointField.INT16: ("i2", 2, "I"),
    PointField.UINT16: ("u2", 2, "U"),
    PointField.INT32: ("i4", 4, "I"),
    PointField.UINT32: ("u4", 4, "U"),
    PointField.FLOAT32: ("f4", 4, "F"),
    PointField.FLOAT64: ("f8", 8, "F"),
}
_VALID_FIELD_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _selected_fields(msg: PointCloud2) -> list[PointField]:
    """返回可写入 PCD 的唯一、有效字段，并检查 xyz."""
    selected: list[PointField] = []
    names: set[str] = set()
    for field in sorted(msg.fields, key=lambda item: int(item.offset)):
        name = str(field.name)
        count = max(1, int(field.count))
        if (
            field.datatype not in _PCD_TYPES
            or not _VALID_FIELD_NAME.fullmatch(name)
            or name in names
        ):
            continue
        if name in {"x", "y", "z"} and count != 1:
            raise ValueError(f"PCD 坐标字段 {name} 的 count 必须为 1")
        selected.append(field)
        names.add(name)

    missing = sorted({"x", "y", "z"} - names)
    if missing:
        raise ValueError(f"PointCloud2 缺少坐标字段：{','.join(missing)}")
    return selected


def pointcloud2_to_pcd_bytes(
    msg: PointCloud2,
    *,
    skip_invalid_xyz: bool = True,
    min_points: int = 1,
) -> tuple[bytes, int]:
    """转换为 binary PCD 字节，返回字节串和有效点数."""
    if int(min_points) <= 0:
        raise ValueError("min_points 必须大于零")
    if not str(msg.header.frame_id).strip():
        raise ValueError("PointCloud2 frame_id 为空")
    if int(msg.width) <= 0 or int(msg.height) <= 0:
        raise ValueError("PointCloud2 宽度或高度为零")

    fields = _selected_fields(msg)
    names = [str(field.name) for field in fields]
    points = point_cloud2.read_points(
        msg,
        field_names=names,
        skip_nans=False,
        reshape_organized_cloud=False,
    )
    if skip_invalid_xyz:
        valid = np.ones(len(points), dtype=bool)
        for name in ("x", "y", "z"):
            valid &= np.isfinite(points[name])
        points = points[valid]
    point_count = int(len(points))
    if point_count < int(min_points):
        raise ValueError(
            f"有效点数不足：{point_count} < {int(min_points)}"
        )

    dtype_fields = []
    sizes = []
    types = []
    counts = []
    for field in fields:
        dtype_code, size, pcd_type = _PCD_TYPES[int(field.datatype)]
        count = max(1, int(field.count))
        shape = () if count == 1 else (count,)
        dtype_fields.append((str(field.name), "<" + dtype_code, shape))
        sizes.append(str(size))
        types.append(pcd_type)
        counts.append(str(count))

    packed = np.empty(point_count, dtype=np.dtype(dtype_fields, align=False))
    for name in names:
        try:
            packed[name] = points[name]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"无法转换点云字段 {name}：{exc}") from exc

    header_lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS " + " ".join(names),
        "SIZE " + " ".join(sizes),
        "TYPE " + " ".join(types),
        "COUNT " + " ".join(counts),
        f"WIDTH {point_count}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {point_count}",
        "DATA binary",
        "",
    ]
    header = "\n".join(header_lines).encode("ascii")
    return header + packed.tobytes(order="C"), point_count


def write_pointcloud2_pcd(
    msg: PointCloud2,
    output_path: str | Path,
    *,
    skip_invalid_xyz: bool = True,
    min_points: int = 1,
    overwrite: bool = False,
) -> tuple[Path, int]:
    """原子写入 PCD；默认拒绝覆盖已有实验文件."""
    path = Path(output_path).expanduser()
    if path.suffix.lower() != ".pcd":
        raise ValueError("输出文件必须使用 .pcd 扩展名")
    if path.exists() and not overwrite:
        raise FileExistsError(f"拒绝覆盖已有文件：{path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    payload, point_count = pointcloud2_to_pcd_bytes(
        msg,
        skip_invalid_xyz=skip_invalid_xyz,
        min_points=min_points,
    )
    temporary = path.with_name(f".{path.name}.{os.getpid()}.part")
    try:
        with temporary.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        if overwrite:
            os.replace(temporary, path)
        else:
            # link() 在目标已存在时原子失败，避免并发保存覆盖实验数据。
            os.link(temporary, path)
            temporary.unlink()
    finally:
        if temporary.exists():
            temporary.unlink()
    return path, point_count
