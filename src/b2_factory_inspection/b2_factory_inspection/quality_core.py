"""与 ROS2 解耦的巡检图像质量计算核心."""

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np


BBox = tuple[float, float, float, float, float]


@dataclass(frozen=True)
class QualityThresholds:
    """图像质量硬阈值；最终数值必须由真实相机数据标定."""

    require_target_bbox: bool = True
    min_target_confidence: float = 0.60
    min_target_area_ratio: float = 0.03
    max_target_area_ratio: float = 0.65
    min_roi_width_px: int = 24
    min_roi_height_px: int = 24
    min_blur_score: float = 80.0
    min_brightness: float = 45.0
    max_brightness: float = 215.0
    max_dark_ratio: float = 0.35
    max_bright_ratio: float = 0.25

    def __post_init__(self) -> None:
        """拒绝会导致评价失真的阈值组合."""
        numeric_values = (
            self.min_target_confidence,
            self.min_target_area_ratio,
            self.max_target_area_ratio,
            self.min_blur_score,
            self.min_brightness,
            self.max_brightness,
            self.max_dark_ratio,
            self.max_bright_ratio,
        )
        if not all(np.isfinite(value) for value in numeric_values):
            raise ValueError("图像质量阈值必须是有限数值")
        if not 0.0 <= self.min_target_confidence <= 1.0:
            raise ValueError("最小目标置信度必须位于 0 到 1 之间")
        if not (
            0.0 < self.min_target_area_ratio < self.max_target_area_ratio <= 1.0
        ):
            raise ValueError("目标面积比例阈值必须满足 0 < 最小值 < 最大值 <= 1")
        if self.min_roi_width_px <= 0 or self.min_roi_height_px <= 0:
            raise ValueError("目标区域最小像素宽高必须大于零")
        if self.min_blur_score < 0.0:
            raise ValueError("清晰度阈值不能为负数")
        if not 0.0 <= self.min_brightness < self.max_brightness <= 255.0:
            raise ValueError("亮度阈值必须满足 0 <= 最小值 < 最大值 <= 255")
        if not 0.0 <= self.max_dark_ratio <= 1.0:
            raise ValueError("最大暗部比例必须位于 0 到 1 之间")
        if not 0.0 <= self.max_bright_ratio <= 1.0:
            raise ValueError("最大过曝比例必须位于 0 到 1 之间")


def validate_bbox(values: Sequence[float]) -> BBox:
    """校验归一化目标框，格式为中心 x、中心 y、宽、高、置信度."""
    if len(values) != 5:
        raise ValueError("目标框必须包含 5 个数值")
    bbox = tuple(float(value) for value in values)
    if not all(np.isfinite(value) for value in bbox):
        raise ValueError("目标框包含 NaN 或无穷大")
    cx, cy, width, height, confidence = bbox
    if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
        raise ValueError("目标框中心必须位于 0 到 1 之间")
    if not (0.0 < width <= 1.0 and 0.0 < height <= 1.0):
        raise ValueError("目标框宽高必须位于 0 到 1 之间")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("目标置信度必须位于 0 到 1 之间")
    return bbox


def sanitize_target_name(name: str, max_length: int = 80) -> str:
    """生成不含路径分隔符的照片文件名片段."""
    cleaned = "".join(
        character if character.isalnum() or character in "-_" else "_"
        for character in str(name).strip()
    )
    cleaned = cleaned.strip("_")[:max_length]
    return cleaned or "unknown"


def evaluate_image_quality(
    image: np.ndarray,
    bbox: BBox | None,
    target_name: str,
    thresholds: QualityThresholds,
) -> dict:
    """评价目标可见性、目标尺寸、清晰度和曝光，返回中文报告."""
    if not isinstance(image, np.ndarray) or image.size == 0:
        raise ValueError("图像为空")
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("图像必须是 uint8 BGR 三通道格式")

    image_height, image_width = image.shape[:2]
    reasons: list[str] = []
    confidence = 0.0
    area_ratio = 0.0
    roi = image

    if bbox is None:
        if thresholds.require_target_bbox:
            reasons.append("未收到当前目标的有效检测框")
    else:
        cx, cy, box_width, box_height, confidence = bbox
        x1 = max(0, int(round((cx - box_width / 2.0) * image_width)))
        y1 = max(0, int(round((cy - box_height / 2.0) * image_height)))
        x2 = min(image_width, int(round((cx + box_width / 2.0) * image_width)))
        y2 = min(image_height, int(round((cy + box_height / 2.0) * image_height)))
        roi_width = x2 - x1
        roi_height = y2 - y1
        area_ratio = max(
            0.0, roi_width * roi_height / float(image_width * image_height)
        )
        if roi_width <= 0 or roi_height <= 0:
            reasons.append("目标检测框为空")
        else:
            roi = image[y1:y2, x1:x2]
        if roi_width < thresholds.min_roi_width_px:
            reasons.append("目标区域像素宽度不足")
        if roi_height < thresholds.min_roi_height_px:
            reasons.append("目标区域像素高度不足")
        if confidence < thresholds.min_target_confidence:
            reasons.append("目标检测置信度不足")
        if area_ratio < thresholds.min_target_area_ratio:
            reasons.append("目标在图像中太小")
        if area_ratio > thresholds.max_target_area_ratio:
            reasons.append("目标在图像中太大或距离过近")

    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray_all = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_score = float(cv2.Laplacian(gray_roi, cv2.CV_64F).var())
    brightness = float(np.mean(gray_roi))
    dark_ratio = float(np.mean(gray_all < 30))
    bright_ratio = float(np.mean(gray_all > 245))

    if blur_score < thresholds.min_blur_score:
        reasons.append("目标区域清晰度不足")
    if brightness < thresholds.min_brightness:
        reasons.append("目标区域过暗")
    if brightness > thresholds.max_brightness:
        reasons.append("目标区域过亮")
    if dark_ratio > thresholds.max_dark_ratio:
        reasons.append("整幅图像暗部比例过高")
    if bright_ratio > thresholds.max_bright_ratio:
        reasons.append("整幅图像过曝比例过高")

    return {
        "目标": target_name,
        "原因": reasons,
        "清晰度": round(blur_score, 2),
        "亮度": round(brightness, 2),
        "暗部比例": round(dark_ratio, 4),
        "过曝比例": round(bright_ratio, 4),
        "目标面积比例": round(area_ratio, 4),
        "目标置信度": round(confidence, 4),
    }
