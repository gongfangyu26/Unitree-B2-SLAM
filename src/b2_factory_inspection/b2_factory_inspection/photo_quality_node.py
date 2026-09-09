#!/usr/bin/env python3
"""评价巡检照片是否满足后续故障检测的输入质量要求."""

import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, String

from .quality_core import (
    QualityThresholds,
    evaluate_image_quality,
    sanitize_target_name,
    validate_bbox,
)


class PhotoQualityNode(Node):
    """综合评价目标存在性、目标尺寸、清晰度和曝光."""

    def __init__(self) -> None:
        super().__init__("photo_quality_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("compressed_image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("save_dir", "/tmp/b2_inspection_photos")
        self.declare_parameter("require_target_bbox", True)
        self.declare_parameter("bbox_sync_tolerance_sec", 0.20)
        self.declare_parameter("capture_wait_timeout_sec", 2.0)
        self.declare_parameter("min_target_confidence", 0.60)
        self.declare_parameter("min_target_area_ratio", 0.03)
        self.declare_parameter("max_target_area_ratio", 0.65)
        self.declare_parameter("min_roi_width_px", 24)
        self.declare_parameter("min_roi_height_px", 24)
        self.declare_parameter("min_blur_score", 80.0)
        self.declare_parameter("min_brightness", 45.0)
        self.declare_parameter("max_brightness", 215.0)
        self.declare_parameter("max_dark_ratio", 0.35)
        self.declare_parameter("max_bright_ratio", 0.25)

        self.save_dir = Path(str(self.get_parameter("save_dir").value)).expanduser()
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.bbox_sync_tolerance_sec = float(
            self.get_parameter("bbox_sync_tolerance_sec").value
        )
        self.capture_wait_timeout_sec = float(
            self.get_parameter("capture_wait_timeout_sec").value
        )
        if (
            not np.isfinite(self.bbox_sync_tolerance_sec)
            or self.bbox_sync_tolerance_sec < 0.0
        ):
            raise RuntimeError("bbox_sync_tolerance_sec 必须是非负有限数值")
        if (
            not np.isfinite(self.capture_wait_timeout_sec)
            or self.capture_wait_timeout_sec <= 0.0
        ):
            raise RuntimeError("capture_wait_timeout_sec 必须是大于零的有限数值")
        self.thresholds = QualityThresholds(
            require_target_bbox=bool(
                self.get_parameter("require_target_bbox").value
            ),
            min_target_confidence=float(
                self.get_parameter("min_target_confidence").value
            ),
            min_target_area_ratio=float(
                self.get_parameter("min_target_area_ratio").value
            ),
            max_target_area_ratio=float(
                self.get_parameter("max_target_area_ratio").value
            ),
            min_roi_width_px=int(self.get_parameter("min_roi_width_px").value),
            min_roi_height_px=int(self.get_parameter("min_roi_height_px").value),
            min_blur_score=float(self.get_parameter("min_blur_score").value),
            min_brightness=float(self.get_parameter("min_brightness").value),
            max_brightness=float(self.get_parameter("max_brightness").value),
            max_dark_ratio=float(self.get_parameter("max_dark_ratio").value),
            max_bright_ratio=float(self.get_parameter("max_bright_ratio").value),
        )

        self.latest_image = None
        self.latest_image_time = 0.0
        self.latest_bbox = None
        self.latest_bbox_time = 0.0
        self.capture_pending = False
        self.capture_request_time = 0.0
        self.capture_deadline = 0.0
        self.capture_target = "unknown"
        self.capture_index = 0
        self.current_target = "unknown"

        self.result_pub = self.create_publisher(Bool, "/inspection/photo_clear", 10)
        self.report_pub = self.create_publisher(
            String, "/inspection/photo_quality_report", 10
        )
        self.create_subscription(
            Bool, "/inspection/capture_required", self._capture_callback, 10
        )
        target_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/inspection/target_name", self._target_callback, target_qos
        )
        self.create_subscription(
            Float32MultiArray, "/inspection/target_bbox", self._bbox_callback, 10
        )

        if bool(self.get_parameter("use_compressed").value):
            topic = str(self.get_parameter("compressed_image_topic").value)
            self.create_subscription(CompressedImage, topic, self._compressed_callback, 10)
        else:
            topic = str(self.get_parameter("image_topic").value)
            self.create_subscription(Image, topic, self._image_callback, 10)
        self.create_timer(0.05, self._capture_watchdog)
        self.get_logger().info(f"照片质量节点正在监听：{topic}")

    def _target_callback(self, msg: String) -> None:
        self.current_target = msg.data or "unknown"
        # 目标切换后旧检测框不能用于新目标。
        self.latest_bbox = None
        self.latest_bbox_time = 0.0
        if self.capture_pending:
            self.capture_target = self.current_target

    def _bbox_callback(self, msg: Float32MultiArray) -> None:
        try:
            self.latest_bbox = validate_bbox(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"目标框格式错误：{exc}")
            return
        self.latest_bbox_time = time.monotonic()

    def _image_callback(self, msg: Image) -> None:
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
        if channels is None:
            self.get_logger().warn(f"暂不支持图像编码：{msg.encoding}")
            return
        row_bytes = msg.width * channels
        if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
            self.get_logger().warn("图像数据长度或 step 字段不合法")
            return
        expected_bytes = msg.step * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=expected_bytes).reshape(
            msg.height, msg.step
        )
        packed = raw[:, :row_bytes]
        if channels == 1:
            image = cv2.cvtColor(packed.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        else:
            image = packed.reshape(msg.height, msg.width, channels)
            if msg.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self._store_image(image)

    def _compressed_callback(self, msg: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn("压缩图像解码失败")
            return
        self._store_image(image)

    def _store_image(self, image) -> None:
        """保存最新图像及其本机接收时刻."""
        self.latest_image = image.copy()
        self.latest_image_time = time.monotonic()

    def _capture_callback(self, msg: Bool) -> None:
        if not msg.data:
            self.capture_pending = False
            return

        self.capture_pending = True
        self.capture_request_time = time.monotonic()
        self.capture_deadline = (
            self.capture_request_time + self.capture_wait_timeout_sec
        )
        self.capture_target = self.current_target

    def _capture_watchdog(self) -> None:
        """等待拍照请求之后的新图像和同批目标框，避免使用旧帧."""
        if not self.capture_pending:
            return
        now = time.monotonic()
        image_ready = (
            self.latest_image is not None
            and self.latest_image_time >= self.capture_request_time
        )
        bbox_is_new = self.latest_bbox_time >= self.capture_request_time
        bbox_matches_image = (
            bbox_is_new
            and abs(self.latest_image_time - self.latest_bbox_time)
            <= self.bbox_sync_tolerance_sec
        )
        bbox_ready = not self.thresholds.require_target_bbox or bbox_matches_image

        if image_ready and bbox_ready:
            bbox = self.latest_bbox if bbox_matches_image else None
            image = self.latest_image.copy()
            self.capture_pending = False
            try:
                self._evaluate_and_save(image, bbox)
            except (ValueError, cv2.error, OSError) as exc:
                # 单帧异常不应让质量节点退出，任务管理器会
                # 把 false 当作不合格图像并尝试下一个安全候选位姿。
                self._publish_result(
                    False,
                    {
                        "目标": self.capture_target,
                        "原因": [f"图像评价或保存异常：{exc}"],
                    },
                )
            return
        if now <= self.capture_deadline:
            return

        reasons = []
        if not image_ready:
            reasons.append("拍照请求后未收到新相机图像")
        if self.thresholds.require_target_bbox and not bbox_is_new:
            reasons.append("拍照请求后未检测到当前目标")
        elif self.thresholds.require_target_bbox and not bbox_matches_image:
            reasons.append("目标框与相机图像不同步")
        self.capture_pending = False
        self._publish_result(
            False,
            {"目标": self.capture_target, "原因": reasons or ["拍照等待超时"]},
        )

    def _evaluate_and_save(self, image, bbox) -> None:
        """完成质量评价并保存可追溯的原始拍摄结果."""
        report = evaluate_image_quality(
            image, bbox, self.capture_target, self.thresholds
        )
        accepted = not report["原因"]
        self.capture_index += 1
        suffix = "合格" if accepted else "不合格"
        target = sanitize_target_name(self.capture_target)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{timestamp}_{self.capture_index:04d}_{target}_{suffix}.jpg"
        save_path = self.save_dir / filename
        try:
            save_ok = bool(cv2.imwrite(str(save_path), image))
        except cv2.error as exc:
            self.get_logger().error(f"照片保存异常：{exc}")
            save_ok = False
        report["保存路径"] = str(save_path)
        report["保存成功"] = save_ok
        if not save_ok:
            report["原因"].append("照片保存失败")
            accepted = False
        report_path = save_path.with_suffix(".json")
        report["报告路径"] = str(report_path)
        report["是否合格"] = accepted
        try:
            report_path.write_text(
                json.dumps(report, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except (OSError, UnicodeError) as exc:
            report["原因"].append(f"质量报告保存失败：{exc}")
            accepted = False
        self._publish_result(accepted, report)

    def _publish_result(self, accepted: bool, report: dict) -> None:
        self.result_pub.publish(Bool(data=accepted))
        report["是否合格"] = accepted
        text = json.dumps(report, ensure_ascii=False)
        self.report_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoQualityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
