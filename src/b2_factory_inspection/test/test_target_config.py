"""测试户外验证目标配置的严格校验."""

from pathlib import Path

import pytest

from b2_factory_inspection.target_config import load_target_specs


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "targets.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def test_load_expanded_target(tmp_path):
    """新式配置应正确读出目标区域缩放和偏移."""
    path = _write(
        tmp_path,
        """
targets:
  target_a:
    marker_id: 10
    roi_scale_x: 4.0
    roi_scale_y: 3.0
    roi_offset_x: -0.5
""",
    )
    spec = load_target_specs(path)["target_a"]
    assert spec.marker_id == 10
    assert spec.roi_scale_x == 4.0
    assert spec.roi_offset_x == -0.5


def test_legacy_mapping_is_supported(tmp_path):
    """旧的编号映射仍应按一倍标记框加载."""
    path = _write(tmp_path, "target_marker_ids:\n  target_a: 10\n")
    spec = load_target_specs(path)["target_a"]
    assert spec.marker_id == 10
    assert spec.roi_scale_x == 1.0


@pytest.mark.parametrize(
    "text",
    [
        "targets: {}\n",
        "targets:\n  target_a:\n    marker_id: 1.5\n",
        "targets:\n  a: {marker_id: 10}\n  b: {marker_id: 10}\n",
        "targets:\n  a: {marker_id: 10, roi_scale_x: -1}\n",
    ],
)
def test_invalid_target_config_is_rejected(tmp_path, text):
    """空配置、小数编号、重复编号和负缩放都必须被拒绝."""
    with pytest.raises(ValueError):
        load_target_specs(_write(tmp_path, text))
