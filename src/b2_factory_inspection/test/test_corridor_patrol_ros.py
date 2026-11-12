"""本机隔离 ROS 测试：假定位/规划/导航服务，不启动机器人驱动。"""
# flake8: noqa: E402

import json
import math
from pathlib import Path
import time

import pytest

pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from b2_factory_inspection import corridor_patrol
from b2_navigation.navigation_gate import NavigationCommandGate


class FakeHallway(Node):
    """只在隔离 ROS 域中提供可控制的传感器证据和动作响应。"""

    def __init__(self):
        super().__init__("fake_hallway")
        self.pose = (-0.5, 0.0, 0.0)
        self.bridge_enabled = False
        self.nav_goals = []
        self.plan_goals = []
        self.navigation_delay = 0.12
        self.acceptance_delay = 0.0
        self.goal_requests = 0
        self.covariance_xy = 0.0
        self.rejected = False
        self.cancelled = False
        self.zero_commands = 0
        group = ReentrantCallbackGroup()
        self.tf = TransformBroadcaster(self)
        self.amcl = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.autonomy = self.create_publisher(Bool, "/b2/autonomy_ok", 10)
        self.map_ok = self.create_publisher(Bool, "/b2_navigation/map_ok", 10)
        self.bridge = self.create_publisher(Bool, "/b2/control/enabled", 10)
        self.create_subscription(Twist, "/cmd_vel", self._velocity, 10)
        self.create_service(SetBool, "/b2/control/set_enabled", self._enable,
                            callback_group=group)
        self.planner = ActionServer(
            self, ComputePathToPose, "/compute_path_to_pose", self._plan,
            callback_group=group,
        )
        self.navigator = ActionServer(
            self, NavigateToPose, "/navigate_to_pose", self._navigate,
            goal_callback=self._accept_goal,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.create_timer(0.05, self._publish, callback_group=group)

    def _enable(self, request, response):
        self.bridge_enabled = request.data
        self.bridge.publish(Bool(data=self.bridge_enabled))
        response.success = True
        return response

    def _accept_goal(self, request):
        self.goal_requests += 1
        time.sleep(self.acceptance_delay)
        return GoalResponse.REJECT if self.rejected else GoalResponse.ACCEPT

    def _velocity(self, msg):
        if msg.linear.x == msg.linear.y == msg.angular.z == 0:
            self.zero_commands += 1

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self.autonomy.publish(Bool(data=True))
        self.map_ok.publish(Bool(data=True))
        self.bridge.publish(Bool(data=self.bridge_enabled))
        x, y, yaw = self.pose
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_link"
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.tf.sendTransform(transform)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = "map", stamp
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation = transform.transform.rotation
        pose.pose.covariance[0] = pose.pose.covariance[7] = self.covariance_xy
        self.amcl.publish(pose)

    def _plan(self, handle):
        request = handle.request
        self.plan_goals.append(request)
        assert request.use_start
        result = ComputePathToPose.Result()
        result.path.header.frame_id = "map"
        result.path.poses = [request.start, request.goal]
        handle.succeed()
        return result

    def _navigate(self, handle):
        self.nav_goals.append(handle.request)
        deadline = time.monotonic() + self.navigation_delay
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.cancelled = True
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        p, q = handle.request.pose.pose.position, handle.request.pose.pose.orientation
        self.pose = (p.x, p.y, 2 * math.atan2(q.z, q.w))
        # Allow the test TF publisher/listener to observe the reached pose.
        time.sleep(0.12)
        if handle.is_cancel_requested:
            self.cancelled = True
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()


def spin_until(executor, predicate, timeout=6.0):
    """等待实际 ROS 回调，不以固定睡眠假定动作已完成。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError("ROS scenario timed out")


def call(executor, node, kind, service, request):
    """异步调用实测服务并检查响应。"""
    client = node.create_client(kind, service)
    spin_until(executor, client.service_is_ready)
    future = client.call_async(request)
    spin_until(executor, future.done)
    result = future.result()
    node.destroy_client(client)
    return result


@pytest.fixture
def hallway(tmp_path, monkeypatch):
    """所有测试必须显式使用本机 ROS 域，防止接入真实设备。"""
    import os
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "218":
        pytest.skip("需 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=218 才能运行隔离 ROS 测试")
    route_file = tmp_path / "route.yaml"
    route_file.write_text(
        "frame_id: map\nwaypoints:\n"
        "- {name: A, inspect: false, poses: [{x: 0, y: 0}]}\n"
        "- {name: B, inspect: false, poses: [{x: 1, y: 0}]}\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(corridor_patrol, "get_package_share_directory",
                        lambda name: str(Path(__file__).resolve().parents[1]))
    rclpy.init(args=[
        "--ros-args", "-p", f"b2_corridor_patrol:waypoints_file:={route_file}",
        "-p", f"b2_corridor_patrol:report_dir:={tmp_path / 'reports'}",
        "-p", "b2_navigation_command_gate:supervisor_heartbeat_topic:=/patrol/supervisor_alive",
    ])
    fake = FakeHallway()
    gate = NavigationCommandGate()
    patrol = corridor_patrol.CorridorPatrol()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (fake, gate, patrol):
        executor.add_node(node)
    spin_until(executor, lambda: patrol.amcl_ok and len(patrol.health) == 4)
    spin_until(executor, lambda: patrol.tf_buffer.can_transform(
        "map", "base_link", rclpy.time.Time()
    ))
    try:
        yield executor, fake, gate, patrol
    finally:
        patrol._halt("测试退出")
        spin_until(executor, lambda: not patrol.stop_pending)
        executor.shutdown(timeout_sec=2.0)
        patrol.report.close()
        for node in (fake, gate, patrol):
            node.destroy_node()
        rclpy.shutdown()


def validate_and_arm(executor, fake, gate, patrol):
    """先只规划，确认没有导航目标，再模拟人工解锁。"""
    result = call(executor, fake, Trigger, "/patrol/validate", Trigger.Request())
    assert result.success, result.message
    spin_until(executor, lambda: patrol.session.phase == "ready")
    assert len(fake.plan_goals) == 3  # robot -> A -> B -> A
    assert not fake.nav_goals
    assert not fake.bridge_enabled
    spin_until(executor, lambda: gate._supervisor_reason(time.monotonic()) is None)
    for service in ("/b2/navigation/set_armed", "/b2/control/set_enabled"):
        result = call(executor, fake, SetBool, service, SetBool.Request(data=True))
        assert result.success, result.message
    spin_until(executor, lambda: all(patrol.health[k][0] for k in ("gate", "bridge")))


def test_real_actions_close_one_lap_and_lock(hallway):
    """实际 ROS action 的完整一圈必须 A B A，最后确认两级锁关闭。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    result = call(executor, fake, Trigger, "/patrol/start", Trigger.Request())
    assert result.success, result.message
    spin_until(executor, lambda: patrol.session.phase == "idle" and not patrol.stop_pending)
    assert [g.pose.pose.position.x for g in fake.nav_goals] == [0.0, 1.0, 0.0]
    assert all(Path(g.behavior_tree).is_file() for g in fake.nav_goals)
    assert not gate.state.armed and not fake.bridge_enabled
    events = [json.loads(line)["event"] for line in patrol.report_path.read_text().splitlines()]
    assert events.count("arrived") == 3
    assert "completed" in events and "locks_closed" in events


def test_navigation_rejection_stops_without_retry(hallway):
    """目标被拒绝后应停车，不能静默跳点或自行重试。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    fake.rejected = True
    assert call(executor, fake, Trigger, "/patrol/start", Trigger.Request()).success
    spin_until(executor, lambda: patrol.session.phase == "idle" and not patrol.stop_pending)
    assert not fake.nav_goals and not gate.state.armed and not fake.bridge_enabled
    assert '"completed"' not in patrol.report_path.read_text()


def test_stop_cancels_goal_and_prevents_later_targets(hallway):
    """显式停止期间正在执行的 action 必须被取消且不会续发。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    fake.navigation_delay = 1.5
    assert call(executor, fake, Trigger, "/patrol/start", Trigger.Request()).success
    spin_until(executor, lambda: len(fake.nav_goals) == 1 and patrol.goal_handle is not None)
    assert call(executor, fake, Trigger, "/patrol/stop", Trigger.Request()).success
    spin_until(executor, lambda: fake.cancelled and not patrol.stop_pending)
    assert len(fake.nav_goals) == 1
    assert not gate.state.armed and not fake.bridge_enabled


def test_suspended_patrol_causes_gate_heartbeat_timeout(hallway):
    """暂停调度巡逻节点，其他节点继续；验证 gate 独立锁定并发布零速。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    previous_zeros = fake.zero_commands
    executor.remove_node(patrol)
    try:
        spin_until(executor, lambda: not gate.state.armed, timeout=2)
        spin_until(executor, lambda: fake.zero_commands > previous_zeros)
        assert "心跳" in gate.last_status
    finally:
        executor.add_node(patrol)


def test_late_goal_acceptance_after_stop_is_cancelled(hallway):
    """停止发生在 accepted 响应之前时，迟到目标仍必须取消。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    fake.acceptance_delay, fake.navigation_delay = 0.3, 1.5
    assert call(executor, fake, Trigger, "/patrol/start", Trigger.Request()).success
    spin_until(executor, lambda: fake.goal_requests == 1)
    assert call(executor, fake, Trigger, "/patrol/stop", Trigger.Request()).success
    spin_until(executor, lambda: fake.cancelled and not patrol.stop_pending)
    assert fake.goal_requests == 1 and len(fake.nav_goals) == 1
    assert not gate.state.armed and not fake.bridge_enabled


def test_localization_degradation_cancels_running_patrol(hallway):
    """即使旧 TF 仍存在，坏的 AMCL 新消息也必须中止巡逻。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    fake.navigation_delay = 1.5
    assert call(executor, fake, Trigger, "/patrol/start", Trigger.Request()).success
    spin_until(executor, lambda: len(fake.nav_goals) == 1)
    fake.covariance_xy = 0.26
    spin_until(executor, lambda: fake.cancelled and not patrol.stop_pending)
    assert len(fake.nav_goals) == 1 and not gate.state.armed and not fake.bridge_enabled


def test_changed_route_file_invalidates_start_and_closes_locks(hallway):
    """已检查路线被修改后不能以旧检查结果发起运动。"""
    executor, fake, gate, patrol = hallway
    validate_and_arm(executor, fake, gate, patrol)
    patrol.route_file.write_text(patrol.route_file.read_text() + "\n# changed\n")
    result = call(executor, fake, Trigger, "/patrol/start", Trigger.Request())
    assert not result.success
    spin_until(executor, lambda: not patrol.stop_pending)
    assert not fake.nav_goals and not gate.state.armed and not fake.bridge_enabled


def test_gate_refuses_arm_before_route_is_validated(hallway):
    """启动后任务未就绪时，存活联锁应拒绝人工解锁导航门。"""
    executor, fake, gate, patrol = hallway
    result = call(executor, fake, SetBool, "/b2/navigation/set_armed",
                  SetBool.Request(data=True))
    assert not result.success and not gate.state.armed
    assert not fake.nav_goals and patrol.session.phase == "idle"
