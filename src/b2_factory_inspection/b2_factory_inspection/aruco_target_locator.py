#!/usr/bin/env python3
"""在功能验证阶段用 ArUco 标记定位 A、B 巡检目标."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, String

from .target_config import load_target_specs


class ArucoTargetLocator(Node):
    """根据当前目标名称发布归一化目标框."""

    def __init__(self) -> None:
        super().__init__("aruco_target_locator")
        self.declare_parameter("targets_file", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("compressed_image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("dictionary", "DICT_4X4_50")

        targets_file = str(self.get_parameter("targets_file").value)
        if not targets_file:
            raise RuntimeError("必须设置 targets_file 参数")
        try:
            self.target_specs = load_target_specs(targets_file)
        except ValueError as exc:
            raise RuntimeError(f"目标配置无效：{exc}") from exc

        dictionary_name = str(self.get_parameter("dictionary").value)
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f"OpenCV 不支持 ArUco 字典：{dictionary_name}")
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        marker_count = int(self.dictionary.bytesList.shape[0])
        marker_ids_valid = all(
            spec.marker_id < marker_count for spec in self.target_specs.values()
        )
        if not marker_ids_valid:
            raise RuntimeError(
                f"目标编号必须位于 0 到 {marker_count - 1} 之间"
            )
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.parameters
            )
        else:
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
        self.current_target = ""
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, "/inspection/target_bbox", 10
        )
        target_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/inspection/target_name", self._target_callback, target_qos
        )

        if bool(self.get_parameter("use_compressed").value):
            topic = str(self.get_parameter("compressed_image_topic").value)
            self.create_subscription(CompressedImage, topic, self._compressed_callback, 10)
        else:
            topic = str(self.get_parameter("image_topic").value)
            self.create_subscription(Image, topic, self._image_callback, 10)
        self.get_logger().info(f"ArUco 目标定位器正在监听：{topic}")

    def _target_callback(self, msg: String) -> None:
        self.current_target = msg.data

    def _image_callback(self, msg: Image) -> None:
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
        if channels is None:
            return
        row_bytes = msg.width * channels
        if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
            return
        expected_bytes = msg.step * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=expected_bytes).reshape(
            msg.height, msg.step
        )
        packed = raw[:, :row_bytes]
        if channels == 1:
            image = packed.reshape(msg.height, msg.width)
        else:
            image = packed.reshape(msg.height, msg.width, channels)
            if msg.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self._detect(image)

    def _compressed_callback(self, msg: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            self._detect(image)

    def _detect(self, image) -> None:
        if self.current_target not in self.target_specs:
            return
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image, self.dictionary, parameters=self.parameters
            )
        if ids is None:
            return
        spec = self.target_specs[self.current_target]
        image_height, image_width = image.shape[:2]
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != spec.marker_id:
                continue
            points = marker_corners.reshape(-1, 2)
            marker_x1, marker_y1 = points.min(axis=0)
            marker_x2, marker_y2 = points.max(axis=0)
            marker_width = float(marker_x2 - marker_x1)
            marker_height = float(marker_y2 - marker_y1)
            center_x = float((marker_x1 + marker_x2) / 2.0)
            center_y = float((marker_y1 + marker_y2) / 2.0)
            center_x += spec.roi_offset_x * marker_width
            center_y += spec.roi_offset_y * marker_height
            roi_width = spec.roi_scale_x * marker_width
            roi_height = spec.roi_scale_y * marker_height
            x1 = max(0.0, center_x - roi_width / 2.0)
            y1 = max(0.0, center_y - roi_height / 2.0)
            x2 = min(float(image_width), center_x + roi_width / 2.0)
            y2 = min(float(image_height), center_y + roi_height / 2.0)
            if x2 <= x1 or y2 <= y1:
                return
            message = Float32MultiArray()
            message.data = [
                float((x1 + x2) / 2.0 / image_width),
                float((y1 + y2) / 2.0 / image_height),
                float((x2 - x1) / image_width),
                float((y2 - y1) / image_height),
                1.0,
            ]
            self.bbox_pub.publish(message)
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoTargetLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
