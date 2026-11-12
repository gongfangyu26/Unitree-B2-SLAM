"""从 RoboSense PointCloud2 发布稳定的 B2 前向障碍距离。"""

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String

from .front_distance_core import (
    FrontDistanceConfig,
    MultiFrameDistanceFilter,
    estimate_front_distance,
)


class FrontLidarDistance(Node):
    """只读点云并发布距离，不产生任何机器人运动指令。"""

    def __init__(self) -> None:
        super().__init__("b2_front_lidar_distance")
        self.declare_parameter("cloud_topic", "/rslidar_points")
        self.declare_parameter(
            "distance_topic", "/b2/perception/front_distance"
        )
        self.declare_parameter(
            "valid_topic", "/b2/perception/front_distance_valid"
        )
        self.declare_parameter(
            "report_topic", "/b2/perception/front_distance_report"
        )
        self.declare_parameter("x_min", 0.30)
        self.declare_parameter("x_max", 5.00)
        self.declare_parameter("y_abs_max", 0.50)
        self.declare_parameter("z_min", -0.20)
        self.declare_parameter("z_max", 1.50)
        self.declare_parameter("cluster_width", 0.20)
        self.declare_parameter("min_roi_points", 20)
        self.declare_parameter("min_cluster_points", 12)
        self.declare_parameter("window_size", 5)
        self.declare_parameter("min_valid_frames", 3)
        self.declare_parameter("log_interval_sec", 1.0)

        try:
            self.config = FrontDistanceConfig(
                x_min=float(self.get_parameter("x_min").value),
                x_max=float(self.get_parameter("x_max").value),
                y_abs_max=float(self.get_parameter("y_abs_max").value),
                z_min=float(self.get_parameter("z_min").value),
                z_max=float(self.get_parameter("z_max").value),
                cluster_width=float(self.get_parameter("cluster_width").value),
                min_roi_points=int(self.get_parameter("min_roi_points").value),
                min_cluster_points=int(
                    self.get_parameter("min_cluster_points").value
                ),
            )
            self.distance_filter = MultiFrameDistanceFilter(
                window_size=int(self.get_parameter("window_size").value),
                min_valid_frames=int(
                    self.get_parameter("min_valid_frames").value
                ),
            )
        except ValueError as exc:
            raise RuntimeError(f"前向测距参数无效：{exc}") from exc

        self.log_interval_sec = float(
            self.get_parameter("log_interval_sec").value
        )
        if (
            not np.isfinite(self.log_interval_sec)
            or self.log_interval_sec < 0.0
        ):
            raise RuntimeError("log_interval_sec 必须是非负有限数值")
        self.last_log_time = 0.0

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.distance_pub = self.create_publisher(
            Float32, str(self.get_parameter("distance_topic").value), 10
        )
        self.valid_pub = self.create_publisher(
            Bool, str(self.get_parameter("valid_topic").value), 10
        )
        self.report_pub = self.create_publisher(
            String, str(self.get_parameter("report_topic").value), 10
        )
        self.create_subscription(
            PointCloud2,
            cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"前向测距订阅 {cloud_topic}；当前算法和阈值待 B2 实机验证"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        try:
            points = point_cloud2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
            estimate = estimate_front_distance(points, self.config)
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            self.distance_filter.update(None)
            self._publish_invalid(f"点云解析失败：{exc}", msg.header.frame_id)
            return

        if estimate is None:
            stable = self.distance_filter.update(None)
            self._publish_invalid(
                "ROI 内没有达到点数要求的稠密距离簇",
                msg.header.frame_id,
                stable,
            )
            return

        stable = self.distance_filter.update(estimate.distance_x)
        if stable is None:
            self._publish_invalid(
                "有效帧数量不足，正在建立多帧窗口",
                msg.header.frame_id,
                None,
                estimate,
            )
            return

        self.distance_pub.publish(Float32(data=stable))
        self.valid_pub.publish(Bool(data=True))
        report = {
            "有效": True,
            "稳定前向距离米": stable,
            "单帧前向距离米": estimate.distance_x,
            "横向中位数米": estimate.lateral_y,
            "高度中位数米": estimate.height_z,
            "ROI点数": estimate.roi_points,
            "距离簇点数": estimate.cluster_points,
            "距离簇MAD米": estimate.spread_x,
            "窗口有效帧": self.distance_filter.valid_count,
            "frame_id": msg.header.frame_id,
            "实机状态": "待验证",
        }
        self.report_pub.publish(
            String(data=json.dumps(report, ensure_ascii=False))
        )
        self._log_throttled(
            f"稳定前向距离={stable:.3f} m，单帧={estimate.distance_x:.3f} m，"
            f"距离簇={estimate.cluster_points} 点"
        )

    def _publish_invalid(
        self,
        reason: str,
        frame_id: str,
        stable: float | None = None,
        estimate=None,
    ) -> None:
        self.valid_pub.publish(Bool(data=False))
        report = {
            "有效": False,
            "原因": reason,
            "窗口有效帧": self.distance_filter.valid_count,
            "frame_id": frame_id,
            "实机状态": "待验证",
        }
        if stable is not None:
            report["窗口历史距离米"] = stable
        if estimate is not None:
            report["单帧前向距离米"] = estimate.distance_x
            report["ROI点数"] = estimate.roi_points
            report["距离簇点数"] = estimate.cluster_points
        self.report_pub.publish(
            String(data=json.dumps(report, ensure_ascii=False))
        )
        self._log_throttled(reason, warning=True)

    def _log_throttled(self, message: str, warning: bool = False) -> None:
        now = time.monotonic()
        if (
            self.log_interval_sec > 0.0
            and now - self.last_log_time < self.log_interval_sec
        ):
            return
        if warning:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
        self.last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontLidarDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
