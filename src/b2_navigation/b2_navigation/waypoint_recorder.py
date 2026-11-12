"""从 RViz /clicked_point 安全采集并保存巡航点。"""

from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .waypoint_recorder_core import occupancy_at, route_from_points


class WaypointRecorder(Node):
    """只记录地图点击，不发送目标或速度。"""

    def __init__(self) -> None:
        super().__init__("b2_waypoint_recorder")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter(
            "output_file",
            str(Path.home() / "b2_maps" / "b2_patrol_waypoints.yaml"),
        )
        self.declare_parameter("maximum_cost", 0)
        self.declare_parameter("minimum_spacing_m", 0.30)
        self.declare_parameter("loop_route", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.output_file = Path(
            str(self.get_parameter("output_file").value)
        ).expanduser()
        self.maximum_cost = int(self.get_parameter("maximum_cost").value)
        self.minimum_spacing_m = float(
            self.get_parameter("minimum_spacing_m").value
        )
        self.loop_route = bool(self.get_parameter("loop_route").value)
        if not 0 <= self.maximum_cost <= 100:
            raise RuntimeError("maximum_cost 必须位于 0 到 100")
        if self.minimum_spacing_m <= 0.0:
            raise RuntimeError("minimum_spacing_m 必须大于零")

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/b2_navigation/waypoint_markers",
            transient_qos,
        )
        self.status_pub = self.create_publisher(
            String, "/b2_navigation/waypoints/status", 10
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._costmap_callback,
            transient_qos,
        )
        self.create_subscription(
            PointStamped, "/clicked_point", self._point_callback, 10
        )
        self.create_service(
            Trigger, "/b2_navigation/waypoints/undo", self._undo_callback
        )
        self.create_service(
            Trigger, "/b2_navigation/waypoints/clear", self._clear_callback
        )
        self.create_service(
            Trigger, "/b2_navigation/waypoints/save", self._save_callback
        )
        self.costmap: OccupancyGrid | None = None
        self.points: list[tuple[float, float]] = []
        self._publish_status(
            "巡航点记录器已启动；使用 Publish Point 按顺序点击空闲区域"
        )

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self.costmap = msg

    def _point_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self._publish_status(
                f"拒绝点击：坐标系 {msg.header.frame_id} 不是 {self.frame_id}",
                warning=True,
            )
            return
        if self.costmap is None:
            self._publish_status("拒绝点击：尚未收到全局代价地图", warning=True)
            return
        info = self.costmap.info
        cost = occupancy_at(
            data=self.costmap.data,
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            x=msg.point.x,
            y=msg.point.y,
        )
        if cost is None:
            self._publish_status("拒绝点击：巡航点位于地图外", warning=True)
            return
        if cost < 0 or cost > self.maximum_cost:
            self._publish_status(
                f"拒绝点击：该位置不是安全空闲区，代价值={cost}",
                warning=True,
            )
            return
        if self.points:
            previous_x, previous_y = self.points[-1]
            distance = (
                (msg.point.x - previous_x) ** 2
                + (msg.point.y - previous_y) ** 2
            ) ** 0.5
            if distance < self.minimum_spacing_m:
                self._publish_status(
                    f"拒绝点击：与上一个点仅相距 {distance:.2f} m",
                    warning=True,
                )
                return
        self.points.append((float(msg.point.x), float(msg.point.y)))
        self._publish_markers()
        self._publish_status(
            f"已记录巡航点 {len(self.points)}："
            f"({msg.point.x:.3f}, {msg.point.y:.3f})"
        )

    def _undo_callback(self, request, response):
        del request
        if not self.points:
            response.success = False
            response.message = "没有可撤销的巡航点"
            return response
        self.points.pop()
        self._publish_markers()
        response.success = True
        response.message = f"已撤销，当前剩余 {len(self.points)} 个点"
        self._publish_status(response.message)
        return response

    def _clear_callback(self, request, response):
        del request
        self.points.clear()
        self._publish_markers()
        response.success = True
        response.message = "巡航点已清空"
        self._publish_status(response.message)
        return response

    def _save_callback(self, request, response):
        del request
        if not self.points:
            response.success = False
            response.message = "没有巡航点，未保存"
            return response
        payload = {
            "frame_id": self.frame_id,
            "waypoints": route_from_points(
                self.points, loop_route=self.loop_route
            ),
        }
        try:
            self.output_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output_file.with_suffix(
                self.output_file.suffix + ".tmp"
            )
            temporary.write_text(
                yaml.safe_dump(
                    payload, allow_unicode=True, sort_keys=False
                ),
                encoding="utf-8",
            )
            temporary.replace(self.output_file)
        except OSError as exc:
            response.success = False
            response.message = f"保存巡航点失败：{exc}"
            self._publish_status(response.message, warning=True)
            return response
        response.success = True
        response.message = (
            f"已保存 {len(self.points)} 个巡航点：{self.output_file}"
        )
        self._publish_status(response.message)
        return response

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        stamp = self.get_clock().now().to_msg()
        for index, (x, y) in enumerate(self.points):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "b2_patrol_points"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.12
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.24
            sphere.scale.y = 0.24
            sphere.scale.z = 0.24
            sphere.color.r = 1.0
            sphere.color.g = 0.55
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "b2_patrol_labels"
            label.id = 1000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.28
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = chr(ord("A") + index) if index < 26 else str(index + 1)
            marker_array.markers.append(label)

        if len(self.points) >= 2:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = "b2_patrol_route"
            line.id = 2000
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.06
            line.color.r = 1.0
            line.color.g = 0.65
            line.color.b = 0.0
            line.color.a = 1.0
            for x, y in self.points:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.08
                line.points.append(point)
            if self.loop_route and len(self.points) >= 3:
                point = Point()
                point.x = self.points[0][0]
                point.y = self.points[0][1]
                point.z = 0.08
                line.points.append(point)
            marker_array.markers.append(line)
        self.marker_pub.publish(marker_array)

    def _publish_status(self, text: str, *, warning: bool = False) -> None:
        self.status_pub.publish(String(data=text))
        if warning:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)


def main(args=None) -> None:
    """运行只读巡航点记录器。"""
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
