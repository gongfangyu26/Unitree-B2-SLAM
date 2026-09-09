"""巡检照片质量计算单元测试."""

import cv2
import numpy as np
import pytest

from b2_factory_inspection.quality_core import (
    QualityThresholds,
    evaluate_image_quality,
    sanitize_target_name,
    validate_bbox,
)


def _checkerboard(size=200):
    """生成亮度正常且边缘丰富的测试图像."""
    rows, columns = np.indices((size, size))
    gray = (((rows // 8 + columns // 8) % 2) * 120 + 70).astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def test_sharp_target_is_accepted():
    """尺寸和曝光正常的清晰目标应通过."""
    thresholds = QualityThresholds(min_blur_score=20.0)
    report = evaluate_image_quality(
        _checkerboard(), (0.5, 0.5, 0.5, 0.5, 0.9), "target_a", thresholds
    )

    assert report["原因"] == []
    assert report["目标"] == "target_a"


def test_uniform_target_is_rejected_as_blurry():
    """无纹理图像的拉普拉斯方差为零，应判为不清晰."""
    image = np.full((160, 160, 3), 120, dtype=np.uint8)
    report = evaluate_image_quality(
        image,
        (0.5, 0.5, 0.5, 0.5, 0.9),
        "target_b",
        QualityThresholds(),
    )

    assert "目标区域清晰度不足" in report["原因"]


def test_invalid_confidence_is_rejected():
    """置信度超出归一化范围时应拒绝目标框."""
    with pytest.raises(ValueError, match="置信度"):
        validate_bbox([0.5, 0.5, 0.2, 0.2, 1.5])


def test_target_name_cannot_create_subdirectory():
    """目标名称中的路径字符必须被替换，避免写到保存目录之外."""
    cleaned = sanitize_target_name("../../设备 A/照片")

    assert "/" not in cleaned
    assert ".." not in cleaned
