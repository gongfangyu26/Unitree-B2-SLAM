"""人工启动、有限圈数的走廊巡逻；只发送 Nav2 目标，不自动解锁。"""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .corridor_patrol_core import (
    PatrolLimits, PatrolSession, amcl_evidence_ok, check_path, patrol_route,
)
from .route_config import load_route


class CorridorPatrol(Node):
    """锁定规划、人工确认定位、人工解锁后开始的巡逻状态机。"""

    def __init__(self):
        super().__init__("b2_corridor_patrol")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("route_mode", "loop")
        self.declare_parameter("laps", 1)
        for name, value in vars(PatrolLimits()).items():
            self.declare_parameter(name, value)
        self.declare_parameter("code_revision", "unrecorded")
        self.declare_parameter("report_dir", str(Path.home() / "b2_patrol_reports"))
        self.limits = PatrolLimits(**{
            name: float(self.get_parameter(name).value)
            for name in vars(PatrolLimits())
        })
        self.session = PatrolSession(self.limits)
        self.route_file = Path(str(self.get_parameter("waypoints_file").value)).expanduser()
        self.mode = str(self.get_parameter("route_mode").value)
        self.laps = self.get_parameter("laps").value
        self.route = []
        self.route_hash = ""
        self.origin = None
        self.health = {}
        self.amcl_ok = False
        self.amcl_time = 0.0
        self.goal_handle = None
        self.stop_pending = False
        self.stop_requested_at = 0.0
        self.last_stop_request = -math.inf
        self.stop_futures = {}
        self.nomotion_future = None
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.control_client = self.create_client(SetBool, "/b2/control/set_enabled")
        self.gate_client = self.create_client(SetBool, "/b2/navigation/set_armed")
        self.nomotion_client = self.create_client(Empty, "/request_nomotion_update")
        self.zero_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/patrol/state", 10)
        self.alive_pub = self.create_publisher(Bool, "/patrol/supervisor_alive", 10)
        self.path_pub = self.create_publisher(
            NavPath, "/patrol/checked_path", 10
        )
        for key, topic in (
            ("autonomy", "/b2/autonomy_ok"),
            ("map", "/b2_navigation/map_ok"),
            ("gate", "/b2_navigation/armed"),
            ("bridge", "/b2/control/enabled"),
        ):
            self.create_subscription(
                Bool, topic, lambda msg, key=key: self._health_msg(key, msg), 10
            )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_msg, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, "/patrol/validate", self._validate)
        self.create_service(Trigger, "/patrol/start", self._start)
        self.create_service(Trigger, "/patrol/stop", self._stop_service)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(1.0, self._nomotion_update)
        report_dir = Path(str(self.get_parameter("report_dir").value)).expanduser()
        report_dir.mkdir(parents=True, exist_ok=True)
        self.report_path = report_dir / (
            datetime.now(timezone.utc).strftime("corridor_%Y%m%dT%H%M%S_%fZ.jsonl")
        )
        self.report = self.report_path.open("x", encoding="utf-8")
        self._event("startup", limits=vars(self.limits), mode=self.mode, laps=self.laps,
                    revision=str(self.get_parameter("code_revision").value),
                    route_file=str(self.route_file), report=str(self.report_path))
        self._status("待命：两级锁关闭时确认定位与激光对齐，然后调用 /patrol/validate")

    def _event(self, event, **details):
        record = {
            "time_utc": datetime.now(timezone.utc).isoformat(),
            "monotonic": time.monotonic(), "event": event,
            "phase": self.session.phase, "index": self.session.index, **details,
        }
        self.report.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.report.flush()

    def _status(self, message):
        self.state_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _health_msg(self, key, msg):
        self.health[key] = (bool(msg.data), time.monotonic())
        # A false edge while moving must invalidate the run immediately, even
        # if a later true sample arrives before the watchdog timer.
        if not msg.data and self.session.phase == "running":
            self._halt(f"安全输入 {key} 失效")
        elif self.session.phase in ("validating", "ready") and (
            (key in ("autonomy", "map") and not msg.data)
            or (key in ("gate", "bridge") and msg.data
                and self.session.phase == "validating")
        ):
            self._halt("路径检查期间安全条件或控制锁状态改变")

    def _amcl_msg(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        age = (self.get_clock().now().nanoseconds
               - Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        self.amcl_ok = -0.5 <= age <= 5.0 and amcl_evidence_ok(
            msg.header.frame_id, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
            msg.pose.covariance,
        )
        self.amcl_time = time.monotonic()
        if not self.amcl_ok and self.session.phase in ("validating", "ready", "running"):
            self._halt("AMCL 消息无效、过期或定位协方差过大")

    def _nomotion_update(self):
        # AMCL may not publish new poses when stationary. Ask for a sensor
        # update, never for a robot movement; keep at most one RPC in flight.
        if self.nomotion_client.service_is_ready() and (
            self.nomotion_future is None or self.nomotion_future.done()
        ):
            self.nomotion_future = self.nomotion_client.call_async(Empty.Request())

    def _robot_pose(self):
        transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        age = (self.get_clock().now().nanoseconds
               - Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
        p, q = transform.transform.translation, transform.transform.rotation
        if not -0.6 <= age <= 0.8:
            raise ValueError("map -> base_link 变换过期")
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError("定位 TF 包含非法值")
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 0.05:
            raise ValueError("定位 TF 四元数未归一化")
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    def _require_health(self, locks=None):
        now = time.monotonic()
        for key, timeout in (("autonomy", 0.8), ("map", 3.0),
                             ("gate", 0.8), ("bridge", 1.5)):
            value, stamp = self.health.get(key, (False, -math.inf))
            if now - stamp > timeout:
                raise ValueError(f"安全输入 {key} 尚未收到或超时")
            expected = True if key in ("autonomy", "map") else locks
            if expected is not None and value != expected:
                raise ValueError(f"安全输入 {key} 不满足要求（应为 {expected}）")
        if not self.amcl_ok or now - self.amcl_time > 5.0:
            raise ValueError("尚无有效且新鲜的 /amcl_pose，不能仅凭 TF 许可巡逻")
        return self._robot_pose()

    @staticmethod
    def _pose(x, y, yaw, stamp):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def _validate(self, request, response):
        del request
        if self.session.phase in ("validating", "running") or self.stop_pending:
            response.success, response.message = False, "正在执行任务或等待停车确认"
            return response
        try:
            self.origin = self._require_health(locks=False)
            if not self.plan_client.server_is_ready():
                raise ValueError("路径规划 action 未就绪")
            content_hash = hashlib.sha256(self.route_file.read_bytes()).hexdigest()
            frame, waypoints = load_route(self.route_file)
            self.route = patrol_route(frame, waypoints, self.mode, self.laps)
            if hashlib.sha256(self.route_file.read_bytes()).hexdigest() != content_hash:
                raise ValueError("读取期间路线文件发生变化，请重试")
            self.route_hash = content_hash
            self.session.validating(len(self.route), time.monotonic())
            self._event("validation_started", sha256=self.route_hash,
                        route=[vars(pose) for pose in self.route], origin=self.origin)
            self._dispatch(planning=True)
            response.success = True
            response.message = "逐段只规划检查已开始；观察 /patrol/state，直到 READY"
        except (OSError, ValueError, TransformException) as exc:
            self._halt(f"拒绝路径检查：{exc}")
            response.success, response.message = False, str(exc)
        return response

    def _start(self, request, response):
        del request
        try:
            if self.stop_pending:
                raise ValueError("尚未收到两级锁关闭确认")
            current = self._require_health(locks=True)
            if self.origin is None or math.dist(current[:2], self.origin[:2]) > 0.20:
                raise ValueError("位置偏离检查起点；锁定并重新 validate")
            if abs(math.remainder(current[2] - self.origin[2], 2 * math.pi)) > 0.20:
                raise ValueError("航向偏离检查起点；锁定并重新 validate")
            if hashlib.sha256(self.route_file.read_bytes()).hexdigest() != self.route_hash:
                raise ValueError("路线文件已改变；锁定并重新 validate")
            if not self.nav_client.server_is_ready():
                raise ValueError("导航 action 未就绪")
            self.session.start(time.monotonic())
            self._event("mission_started")
            self._dispatch(planning=False)
            response.success, response.message = True, "巡逻已启动；/patrol/stop 可立即中止"
        except (OSError, ValueError, TransformException) as exc:
            # A rejected start while armed must not leave an enabled robot.
            self._halt(f"拒绝启动：{exc}")
            response.success, response.message = False, str(exc)
        return response

    def _dispatch(self, *, planning):
        index, generation = self.session.index, self.session.generation
        target = self.route[index]
        stamp = self.get_clock().now().to_msg()
        target_pose = self._pose(target.x, target.y, target.yaw, stamp)
        if planning:
            goal = ComputePathToPose.Goal()
            goal.goal = target_pose
            start = self.origin if index == 0 else (
                self.route[index - 1].x, self.route[index - 1].y,
                self.route[index - 1].yaw,
            )
            goal.start = self._pose(*start, stamp)
            goal.use_start, goal.planner_id = True, "GridBased"
            client = self.plan_client
        else:
            goal = NavigateToPose.Goal()
            goal.pose = target_pose
            goal.behavior_tree = str(
                Path(get_package_share_directory("b2_factory_inspection"))
                / "config" / "corridor_no_recovery.xml"
            )
            client = self.nav_client
        self._status(f"{'检查' if planning else '导航'} {index + 1}/{len(self.route)} {target.name}")
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._accepted(f, generation, index, planning)
        )

    def _accepted(self, future, generation, index, planning):
        phase = "validating" if planning else "running"
        try:
            handle = future.result()
            if not self.session.current(generation, index, phase):
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                raise ValueError("Nav2 拒绝目标")
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f: self._result(f, generation, index, planning)
            )
        except Exception as exc:
            if self.session.current(generation, index, phase):
                self._halt(f"目标发送异常：{exc}")

    def _result(self, future, generation, index, planning):
        phase = "validating" if planning else "running"
        if not self.session.current(generation, index, phase):
            return
        self.goal_handle = None
        try:
            self._require_health(locks=not planning)
            if self.session.timeout_reason(time.monotonic()):
                raise ValueError(self.session.timeout_reason(time.monotonic()))
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise ValueError(f"Nav2 目标未成功，状态={result.status}；本次不重试")
            target = self.route[index]
            if planning:
                path = result.result.path
                start = self.origin[:2] if index == 0 else (
                    self.route[index - 1].x, self.route[index - 1].y
                )
                length = check_path(
                    [(p.pose.position.x, p.pose.position.y) for p in path.poses],
                    path.header.frame_id, start, (target.x, target.y),
                    self.limits.max_segment_m,
                )
                self.path_pub.publish(path)
                self._event("path_checked", length_m=length, target=target.name)
                self.session.planned(time.monotonic())
                if self.session.phase == "ready":
                    self.alive_pub.publish(Bool(data=True))
                    self._event("ready")
                    self._status("READY：检查通过；人工核对路径后限时解锁，调用 /patrol/start")
                    return
            else:
                pose = self._robot_pose()
                if math.dist(pose[:2], (target.x, target.y)) > 0.30:
                    raise ValueError("Nav2 报告成功但定位距目标超过 0.30 m")
                if abs(math.remainder(pose[2] - target.yaw, 2 * math.pi)) > 0.30:
                    raise ValueError("Nav2 报告成功但定位航向偏差超过 0.30 rad")
                self._event("arrived", target=target.name, pose=pose)
                if self.session.arrived(time.monotonic()):
                    self._halt("全部巡逻目标完成，已请求停车", completed=True)
                    return
            self._dispatch(planning=planning)
        except Exception as exc:
            self._halt(f"任务中止：{exc}")

    def _stop_service(self, request, response):
        del request
        self._halt("人工停止")
        response.success, response.message = True, "已发送零速度并请求关闭两级锁；等待确认"
        return response

    def _halt(self, reason, *, completed=False):
        handle, self.goal_handle = self.goal_handle, None
        self.session.stop()
        self.stop_pending = True
        self.stop_requested_at = time.monotonic()
        self.alive_pub.publish(Bool(data=False))
        self.zero_pub.publish(Twist())
        if handle is not None:
            handle.cancel_goal_async()
        self._request_locks_closed()
        self._event("completed" if completed else "stopped", reason=reason)
        self._status(reason + "；等待两级锁关闭确认")

    def _request_locks_closed(self):
        if time.monotonic() - self.last_stop_request < 0.5:
            return
        self.last_stop_request = time.monotonic()
        # Request both locks independently; a stalled RPC must not prevent
        # the other lock from receiving its stop request.
        for client in (self.control_client, self.gate_client):
            pending = self.stop_futures.get(client)
            if client.service_is_ready() and (pending is None or pending.done()):
                self.stop_futures[client] = client.call_async(SetBool.Request(data=False))

    def _watchdog(self):
        self.alive_pub.publish(Bool(
            data=self.session.phase in ("ready", "running") and not self.stop_pending
        ))
        if self.stop_pending:
            self.zero_pub.publish(Twist())
            self._request_locks_closed()
            now = time.monotonic()
            if all(key in self.health and not self.health[key][0]
                   and self.health[key][1] >= self.stop_requested_at
                   and now - self.health[key][1] < 1.5 for key in ("gate", "bridge")):
                self.stop_pending = False
                self._event("locks_closed")
                self._status("STOPPED：已确认两级锁关闭；再次启动前需要重新 validate")
            return
        if self.session.phase in ("validating", "ready", "running"):
            try:
                expected = {"validating": False, "ready": None, "running": True}
                self._require_health(locks=expected[self.session.phase])
                reason = self.session.timeout_reason(time.monotonic())
                if reason:
                    raise ValueError(reason)
                if self.session.phase == "ready" and time.monotonic() >= self.session.valid_until:
                    raise ValueError("路线检查已过期；请重新 validate")
            except (ValueError, TransformException) as exc:
                self._halt(str(exc))


def main(args=None):
    """保持 DDS 上下文可用直到退出停车请求发出并短暂等待确认。"""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = CorridorPatrol()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.10)
    finally:
        if node is not None and rclpy.ok():
            node._halt("节点退出")
            deadline = time.monotonic() + 1.5
            while node.stop_pending and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.report.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
