#!/usr/bin/env python3
"""检查 RTAB-Map 或 map_server 发布的二维占据地图."""

import json
import math
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .occupancy_grid_core import analyze_occupancy_grid


class OccupancyGridPreflight(Node):
    """持续发布二维地图是否具备导航所需基本结构的检查结果."""

    def __init__(self) -> None:
        super().__init__("occupancy_grid_preflight")
        self.declare_parameter("map_topic", "/rtabmap/grid_prob_map")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("min_total_cells", 100)
        self.declare_parameter("min_known_ratio", 0.01)
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("stream_timeout_sec", 3.0)
        self.declare_parameter("require_updates", False)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.expected_frame = str(
            self.get_parameter("expected_frame").value
        )
        self.min_total_cells = int(
            self.get_parameter("min_total_cells").value
        )
        self.min_known_ratio = float(
            self.get_parameter("min_known_ratio").value
        )
        self.free_threshold = int(
            self.get_parameter("free_threshold").value
        )
        self.occupied_threshold = int(
            self.get_parameter("occupied_threshold").value
        )
        self.stream_timeout_sec = float(
            self.get_parameter("stream_timeout_sec").value
        )
        self.require_updates = bool(
            self.get_parameter("require_updates").value
        )
        if (
            not math.isfinite(self.min_known_ratio)
            or not 0.0 <= self.min_known_ratio <= 1.0
        ):
            raise RuntimeError("min_known_ratio 必须位于 0 到 1")
        if (
            not math.isfinite(self.stream_timeout_sec)
            or self.stream_timeout_sec <= 0.0
        ):
            raise RuntimeError("stream_timeout_sec 必须大于零")
        if not 0 <= self.free_threshold < self.occupied_threshold <= 100:
            raise RuntimeError("地图空闲和占据阈值设置无效")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos
        )
        self.ok_pub = self.create_publisher(
            Bool, "/b2_navigation/map_ok", 10
        )
        self.report_pub = self.create_publisher(
            String, "/b2_navigation/map_report", 10
        )
        self.last_map_time = 0.0
        self.last_payload = ""
        self.last_ok = False
        self.last_report: dict | None = None
        self.create_timer(1.0, self._timer_callback)
        self.get_logger().info(f"二维地图检查已启动：{self.map_topic}")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.last_map_time = time.monotonic()
        summary = analyze_occupancy_grid(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            frame_id=msg.header.frame_id,
            data=msg.data,
            expected_frame=self.expected_frame,
            min_total_cells=self.min_total_cells,
            min_known_ratio=self.min_known_ratio,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
        )
        payload = {
            "通过": summary.ok,
            "话题": self.map_topic,
            "坐标系": msg.header.frame_id,
            "宽": int(msg.info.width),
            "高": int(msg.info.height),
            "分辨率": round(float(msg.info.resolution), 4),
            "总栅格": summary.total_cells,
            "未知栅格": summary.unknown_cells,
            "可通行栅格": summary.free_cells,
            "障碍栅格": summary.occupied_cells,
            "中间概率栅格": summary.intermediate_cells,
            "已知比例": round(summary.known_ratio, 4),
            "原因": list(summary.reasons),
        }
        self._publish(summary.ok, payload)

    def _timer_callback(self) -> None:
        if self.last_map_time == 0.0:
            self._publish(
                False,
                {
                    "通过": False,
                    "话题": self.map_topic,
                    "原因": ["二维地图尚未收到或已超时"],
                },
            )
        elif (
            self.require_updates
            and time.monotonic() - self.last_map_time
            > self.stream_timeout_sec
        ):
            self._publish(
                False,
                {
                    "通过": False,
                    "话题": self.map_topic,
                    "原因": ["动态二维地图已超过更新时间限制"],
                },
            )
        elif self.last_report is not None:
            # map_server 的静态地图通常只发布一次，定时重发检查状态供安全门使用。
            self._publish(self.last_ok, self.last_report)

    def _publish(self, ok: bool, payload: dict) -> None:
        text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.last_ok = bool(ok)
        self.last_report = payload
        self.ok_pub.publish(Bool(data=bool(ok)))
        self.report_pub.publish(String(data=text))
        if text != self.last_payload:
            if ok:
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            self.last_payload = text


def main(args=None) -> None:
    """运行二维占据地图检查节点."""
    rclpy.init(args=args)
    node = OccupancyGridPreflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ok_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
