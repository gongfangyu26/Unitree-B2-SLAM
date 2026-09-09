"""加载用于户外验证的 ArUco 目标区域配置."""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


MAX_TARGETS = 500


@dataclass(frozen=True)
class TargetMarkerSpec:
    """描述标记编号及相对于标记的设备目标区域."""

    marker_id: int
    roi_scale_x: float = 1.0
    roi_scale_y: float = 1.0
    roi_offset_x: float = 0.0
    roi_offset_y: float = 0.0


def _finite(value: Any, field: str) -> float:
    """把配置字段转换为有限浮点数."""
    if isinstance(value, bool):
        raise ValueError(f"{field} 不能是布尔值")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 必须是数值") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field} 必须是有限数值")
    return number


def _marker_id(value: Any, field: str) -> int:
    """读取非负整数标记编号，拒绝小数和布尔值."""
    number = _finite(value, field)
    if number < 0 or not number.is_integer():
        raise ValueError(f"{field} 必须是非负整数")
    return int(number)


def load_target_specs(path: str | Path) -> dict[str, TargetMarkerSpec]:
    """读取新式 targets 配置，并兼容旧 target_marker_ids 写法."""
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise ValueError(f"目标配置文件不存在：{config_path}")
    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"无法读取目标配置：{exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("目标配置顶层必须是 YAML 映射")

    raw_targets = data.get("targets")
    legacy_targets = data.get("target_marker_ids")
    if raw_targets is None and legacy_targets is not None:
        if not isinstance(legacy_targets, dict):
            raise ValueError("target_marker_ids 必须是 YAML 映射")
        raw_targets = {
            name: {"marker_id": marker_id}
            for name, marker_id in legacy_targets.items()
        }
    if not isinstance(raw_targets, dict) or not raw_targets:
        raise ValueError("目标配置必须包含非空 targets 映射")
    if len(raw_targets) > MAX_TARGETS:
        raise ValueError(f"验证目标数量不能超过 {MAX_TARGETS}")

    specs: dict[str, TargetMarkerSpec] = {}
    used_marker_ids: set[int] = set()
    for raw_name, raw_spec in raw_targets.items():
        if not isinstance(raw_name, str) or not raw_name.strip():
            raise ValueError("目标名称必须是非空字符串")
        name = raw_name.strip()
        if name in specs:
            raise ValueError(f"目标名称重复：{name}")
        if not isinstance(raw_spec, dict):
            raise ValueError(f"目标 {name} 的配置必须是 YAML 映射")

        marker_id = _marker_id(raw_spec.get("marker_id"), f"{name}.marker_id")
        if marker_id in used_marker_ids:
            raise ValueError(f"ArUco 编号重复：{marker_id}")
        used_marker_ids.add(marker_id)

        scale_x = _finite(raw_spec.get("roi_scale_x", 1.0), f"{name}.roi_scale_x")
        scale_y = _finite(raw_spec.get("roi_scale_y", 1.0), f"{name}.roi_scale_y")
        offset_x = _finite(
            raw_spec.get("roi_offset_x", 0.0), f"{name}.roi_offset_x"
        )
        offset_y = _finite(
            raw_spec.get("roi_offset_y", 0.0), f"{name}.roi_offset_y"
        )
        if not 0.25 <= scale_x <= 20.0 or not 0.25 <= scale_y <= 20.0:
            raise ValueError(f"目标 {name} 的 ROI 缩放必须位于 0.25 到 20.0 之间")
        if abs(offset_x) > 20.0 or abs(offset_y) > 20.0:
            raise ValueError(f"目标 {name} 的 ROI 偏移绝对值不能超过 20.0")

        specs[name] = TargetMarkerSpec(
            marker_id=marker_id,
            roi_scale_x=scale_x,
            roi_scale_y=scale_y,
            roi_offset_x=offset_x,
            roi_offset_y=offset_y,
        )
    return specs
