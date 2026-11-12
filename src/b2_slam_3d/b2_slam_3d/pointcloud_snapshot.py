#!/usr/bin/env python3
"""订阅一帧全局点云并导出 PCD，默认不覆盖已有文件."""

from datetime import datetime
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .pcd_io import write_pointcloud2_pcd


class PointCloudSnapshot(Node):
    """等待第一帧合格 PointCloud2，然后一次性写入 PCD."""

    def __init__(self) -> None:
        super().__init__("b2_pointcloud_snapshot")
        self.declare_parameter("cloud_topic", "/rtabmap/cloud_map")
        self.declare_parameter("output_path", "")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("min_points", 100)
        self.declare_parameter("skip_invalid_xyz", True)
        self.declare_parameter("overwrite", False)
        # 2026-09-09 实机确认 RTAB-Map cloud_map 为
        # RELIABLE + TRANSIENT_LOCAL，使后启动的节点也能收到最新地图。
        self.declare_parameter("input_reliability", "reliable")
        self.declare_parameter("input_durability", "transient_local")

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        output_path = str(self.get_parameter("output_path").value).strip()
        if output_path:
            self.output_path = Path(output_path).expanduser()
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.output_path = (
                Path("~/b2_maps").expanduser() / f"b2_cloud_{stamp}.pcd"
            )
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.skip_invalid_xyz = bool(
            self.get_parameter("skip_invalid_xyz").value
        )
        self.overwrite = bool(self.get_parameter("overwrite").value)
        reliability_name = str(
            self.get_parameter("input_reliability").value
        ).strip().lower()
        durability_name = str(
            self.get_parameter("input_durability").value
        ).strip().lower()
        if (
            not math.isfinite(self.timeout_sec)
            or not 1.0 <= self.timeout_sec <= 120.0
        ):
            raise RuntimeError("timeout_sec 必须位于 1 到 120 秒")
        if self.min_points <= 0:
            raise RuntimeError("min_points 必须大于零")
        reliability = {
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
            "reliable": ReliabilityPolicy.RELIABLE,
        }.get(reliability_name)
        if reliability is None:
            raise RuntimeError(
                "input_reliability 只能为 best_effort 或 reliable"
            )
        durability = {
            "volatile": DurabilityPolicy.VOLATILE,
            "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        }.get(durability_name)
        if durability is None:
            raise RuntimeError(
                "input_durability 只能为 volatile 或 transient_local"
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=reliability,
            durability=durability,
        )
        self.done = threading.Event()
        self.error_text = ""
        self.saved_path: Path | None = None
        self.saved_points = 0
        self.status_pub = self.create_publisher(
            String, "/b2_slam/pointcloud_save_status", 10
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_callback, qos
        )
        self.get_logger().info(
            f"等待点云 {self.cloud_topic}，将保存到 {self.output_path}"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if self.done.is_set():
            return
        try:
            path, point_count = write_pointcloud2_pcd(
                msg,
                self.output_path,
                skip_invalid_xyz=self.skip_invalid_xyz,
                min_points=self.min_points,
                overwrite=self.overwrite,
            )
        except (OSError, ValueError) as exc:
            self.error_text = str(exc)
            self.get_logger().error(f"点云保存失败：{exc}")
            self.status_pub.publish(String(data=f"失败：{exc}"))
        else:
            self.saved_path = path
            self.saved_points = point_count
            message = f"成功：{path}，有效点数={point_count}"
            self.get_logger().info(message)
            self.status_pub.publish(String(data=message))
        finally:
            self.done.set()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = PointCloudSnapshot()
    deadline = time.monotonic() + node.timeout_sec
    try:
        while rclpy.ok() and not node.done.is_set():
            if time.monotonic() >= deadline:
                node.error_text = (
                    f"等待 {node.cloud_topic} 超过 {node.timeout_sec:.1f} 秒"
                )
                node.get_logger().error(node.error_text)
                break
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        node.error_text = "操作者取消保存"
        node.get_logger().warn(node.error_text)
    finally:
        succeeded = node.saved_path is not None and not node.error_text
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if succeeded else 1


if __name__ == "__main__":
    main()
