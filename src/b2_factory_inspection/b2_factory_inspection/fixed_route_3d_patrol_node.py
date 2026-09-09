#!/usr/bin/env python3
"""
旧版宇树 B2 固定路线巡检控制器，仅保留用于代码对照.

该节点直接发布速度并用短距离激光规则绕障，未接入当前完整安全联锁，安装入口已移除。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import yaml

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    inspect: bool
    approach_step: float
    max_approaches: int


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_waypoints(path: str) -> tuple[str, List[Waypoint]]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    frame_id = str(data.get("frame_id", "map"))
    waypoints: List[Waypoint] = []

    for item in data.get("waypoints", []):
        yaw = item.get("yaw_rad")
        if yaw is None:
            yaw = math.radians(float(item.get("yaw_deg", 0.0)))

        waypoints.append(
            Waypoint(
                name=str(item["name"]),
                x=float(item["x"]),
                y=float(item["y"]),
                yaw=float(yaw),
                inspect=bool(item.get("inspect", True)),
                approach_step=float(item.get("approach_step", 0.25)),
                max_approaches=int(item.get("max_approaches", 2)),
            )
        )

    if not waypoints:
        raise RuntimeError(f"No waypoints found in {path}")

    return frame_id, waypoints


class FixedRoute3DPatrol(Node):
    def __init__(self) -> None:
        super().__init__("fixed_route_3d_patrol")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("control_frequency", 15.0)
        self.declare_parameter("xy_tolerance", 0.18)
        self.declare_parameter("yaw_tolerance", 0.25)
        self.declare_parameter("max_linear_speed", 0.28)
        self.declare_parameter("max_lateral_speed", 0.22)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter("slowdown_radius", 0.8)
        self.declare_parameter("front_stop_distance", 0.85)
        self.declare_parameter("front_clear_distance", 1.25)
        self.declare_parameter("side_clear_distance", 0.75)
        self.declare_parameter("avoid_lateral_speed", 0.18)
        self.declare_parameter("avoid_turn_speed", 0.25)
        self.declare_parameter("loop", True)
        self.declare_parameter("dwell_sec", 1.0)
        self.declare_parameter("auto_continue_without_inspection", False)

        waypoints_file = str(self.get_parameter("waypoints_file").value)
        if not waypoints_file:
            raise RuntimeError("Parameter waypoints_file is required")

        file_frame, self.waypoints = load_waypoints(waypoints_file)
        self.global_frame = str(self.get_parameter("global_frame").value) or file_frame
        if self.global_frame == "map":
            self.global_frame = file_frame

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.xy_tolerance = float(self.get_parameter("xy_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_lateral_speed = float(self.get_parameter("max_lateral_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.slowdown_radius = float(self.get_parameter("slowdown_radius").value)
        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)
        self.front_clear_distance = float(self.get_parameter("front_clear_distance").value)
        self.side_clear_distance = float(self.get_parameter("side_clear_distance").value)
        self.avoid_lateral_speed = float(self.get_parameter("avoid_lateral_speed").value)
        self.avoid_turn_speed = float(self.get_parameter("avoid_turn_speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.dwell_sec = float(self.get_parameter("dwell_sec").value)
        self.auto_continue_without_inspection = bool(
            self.get_parameter("auto_continue_without_inspection").value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.capture_pub = self.create_publisher(Bool, "/inspection/capture_required", 10)
        self.state_pub = self.create_publisher(String, "/inspection/state", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._scan_cb, 10
        )
        self.photo_sub = self.create_subscription(
            Bool, "/inspection/photo_clear", self._photo_clear_cb, 10
        )
        self.manual_result_srv = self.create_service(
            SetBool, "/inspection/set_photo_clear", self._manual_result_cb
        )
        self.skip_srv = self.create_service(
            Trigger, "/inspection/skip_current", self._skip_current_cb
        )

        self.index = 0
        self.approach_count = 0
        self.waiting_for_photo = False
        self.scan: Optional[LaserScan] = None
        self.dwell_until: Optional[Time] = None
        self.last_tf_warn_ns = 0

        period = 1.0 / float(self.get_parameter("control_frequency").value)
        self.timer = self.create_timer(period, self._control_loop)
        self._publish_state("started")
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} fixed-route waypoints in {self.global_frame}"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _current_waypoint(self) -> Waypoint:
        return self.waypoints[self.index]

    def _target_xy(self, waypoint: Waypoint) -> tuple[float, float]:
        x = waypoint.x
        y = waypoint.y
        if self.approach_count > 0:
            x += math.cos(waypoint.yaw) * waypoint.approach_step * self.approach_count
            y += math.sin(waypoint.yaw) * waypoint.approach_step * self.approach_count
        return x, y

    def _robot_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time()
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns > 2_000_000_000:
                self.last_tf_warn_ns = now_ns
                self.get_logger().warn(
                    f"Waiting for TF {self.global_frame}->{self.base_frame}: {exc}"
                )
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return t.x, t.y, yaw

    def _control_loop(self) -> None:
        if self.waiting_for_photo:
            self._publish_stop()
            return

        if self.dwell_until is not None:
            if self.get_clock().now() < self.dwell_until:
                self._publish_stop()
                return
            self.dwell_until = None
            self._advance_route()
            return

        pose = self._robot_pose()
        if pose is None:
            self._publish_stop()
            return

        waypoint = self._current_waypoint()
        target_x, target_y = self._target_xy(waypoint)
        robot_x, robot_y, robot_yaw = pose
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        yaw_error = normalize_angle(waypoint.yaw - robot_yaw)

        if distance <= self.xy_tolerance and abs(yaw_error) <= self.yaw_tolerance:
            self._arrive_at_waypoint()
            return

        obstacle_cmd = self._obstacle_avoidance_cmd()
        if obstacle_cmd is not None:
            self.cmd_pub.publish(obstacle_cmd)
            self._publish_state(f"avoiding:{waypoint.name}")
            return

        path_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(path_heading - robot_yaw)
        speed_scale = min(1.0, max(0.25, distance / self.slowdown_radius))

        cmd = Twist()
        cmd.linear.x = self._clamp(
            math.cos(heading_error) * self.max_linear_speed * speed_scale,
            -self.max_linear_speed,
            self.max_linear_speed,
        )
        cmd.linear.y = self._clamp(
            math.sin(heading_error) * self.max_lateral_speed * speed_scale,
            -self.max_lateral_speed,
            self.max_lateral_speed,
        )
        cmd.angular.z = self._clamp(
            yaw_error * 1.2,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.cmd_pub.publish(cmd)
        self._publish_state(f"moving:{waypoint.name}:distance={distance:.2f}")

    def _obstacle_avoidance_cmd(self) -> Optional[Twist]:
        if self.scan is None:
            return None

        front = self._sector_min(-25.0, 25.0)
        left = self._sector_min(35.0, 100.0)
        right = self._sector_min(-100.0, -35.0)

        if front >= self.front_clear_distance:
            return None

        cmd = Twist()
        if front < self.front_stop_distance:
            if left > right and left > self.side_clear_distance:
                cmd.linear.y = self.avoid_lateral_speed
                cmd.angular.z = self.avoid_turn_speed
            elif right > self.side_clear_distance:
                cmd.linear.y = -self.avoid_lateral_speed
                cmd.angular.z = -self.avoid_turn_speed
            else:
                cmd.angular.z = self.avoid_turn_speed
            return cmd

        if left > right:
            cmd.linear.x = 0.08
            cmd.linear.y = self.avoid_lateral_speed * 0.5
            cmd.angular.z = self.avoid_turn_speed * 0.5
        else:
            cmd.linear.x = 0.08
            cmd.linear.y = -self.avoid_lateral_speed * 0.5
            cmd.angular.z = -self.avoid_turn_speed * 0.5
        return cmd

    def _sector_min(self, start_deg: float, end_deg: float) -> float:
        if self.scan is None:
            return float("inf")

        start = math.radians(start_deg)
        end = math.radians(end_deg)
        values = []

        for i, value in enumerate(self.scan.ranges):
            if not math.isfinite(value):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if start <= angle <= end and self.scan.range_min <= value <= self.scan.range_max:
                values.append(value)

        return min(values) if values else float("inf")

    def _arrive_at_waypoint(self) -> None:
        waypoint = self._current_waypoint()
        self._publish_stop()
        self.get_logger().info(f"Arrived at {waypoint.name}")

        if not waypoint.inspect:
            self.dwell_until = self.get_clock().now() + Duration(seconds=self.dwell_sec)
            return

        self.waiting_for_photo = True
        self.capture_pub.publish(Bool(data=True))
        self._publish_state(f"capture_required:{waypoint.name}")

        if self.auto_continue_without_inspection:
            self._accept_photo_and_advance()

    def _photo_clear_cb(self, msg: Bool) -> None:
        if not self.waiting_for_photo:
            return
        if msg.data:
            self._accept_photo_and_advance()
        else:
            self._move_closer_or_advance()

    def _manual_result_cb(self, request: SetBool.Request, response: SetBool.Response):
        if not self.waiting_for_photo:
            response.success = False
            response.message = "Robot is not waiting for photo quality result"
            return response
        if request.data:
            self._accept_photo_and_advance()
            response.success = True
            response.message = "Photo accepted"
            return response
        self._move_closer_or_advance()
        response.success = True
        response.message = "Photo rejected; moving closer if configured"
        return response

    def _skip_current_cb(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().warn("Skipped current inspection point")
        self.waiting_for_photo = False
        self.approach_count = 0
        self.capture_pub.publish(Bool(data=False))
        self.dwell_until = self.get_clock().now() + Duration(seconds=self.dwell_sec)
        response.success = True
        response.message = "Skipped"
        return response

    def _accept_photo_and_advance(self) -> None:
        waypoint = self._current_waypoint()
        self.waiting_for_photo = False
        self.approach_count = 0
        self.capture_pub.publish(Bool(data=False))
        self._publish_state(f"photo_clear:{waypoint.name}")
        self.dwell_until = self.get_clock().now() + Duration(seconds=self.dwell_sec)

    def _move_closer_or_advance(self) -> None:
        waypoint = self._current_waypoint()
        self.waiting_for_photo = False
        self.capture_pub.publish(Bool(data=False))

        if self.approach_count >= waypoint.max_approaches:
            self.get_logger().error(
                f"Photo still unclear at {waypoint.name}; max approach count reached"
            )
            self.approach_count = 0
            self.dwell_until = self.get_clock().now() + Duration(seconds=self.dwell_sec)
            return

        self.approach_count += 1
        self._publish_state(f"photo_unclear:{waypoint.name}:approach={self.approach_count}")

    def _advance_route(self) -> None:
        if self.index + 1 < len(self.waypoints):
            self.index += 1
        elif self.loop:
            self.index = 0
        else:
            self._publish_stop()
            self._publish_state("finished")
            self.timer.cancel()
            return

        self._publish_state(f"next:{self._current_waypoint().name}")

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_state(self, text: str) -> None:
        self.state_pub.publish(String(data=text))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = FixedRoute3DPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
