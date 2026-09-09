#!/usr/bin/env python3
"""基于 Nav2 和候选观测位姿的 B2 自主巡检任务管理器."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .route_config import load_route


class MissionManager(Node):
    """按路线导航，并在图像不合格时选择下一个安全候选观测位姿."""

    def __init__(self) -> None:
        super().__init__("b2_inspection_mission_manager")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("navigation_action", "/navigate_to_pose")
        self.declare_parameter("quality_timeout_sec", 8.0)
        self.declare_parameter("safety_timeout_sec", 0.80)
        self.declare_parameter("settle_time_sec", 1.50)
        self.declare_parameter("control_enable_delay_sec", 0.30)
        self.declare_parameter("continue_after_inspection_failure", False)
        self.declare_parameter("autostart", False)
        self.declare_parameter("autostart_delay_sec", 10.0)

        route_file = str(self.get_parameter("waypoints_file").value)
        if not route_file:
            raise RuntimeError("必须设置 waypoints_file 参数")
        try:
            self.frame_id, self.waypoints = load_route(route_file)
        except ValueError as exc:
            raise RuntimeError(f"巡检路线无效：{exc}") from exc

        self.quality_timeout_sec = float(self.get_parameter("quality_timeout_sec").value)
        self.safety_timeout_sec = float(self.get_parameter("safety_timeout_sec").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.control_enable_delay_sec = float(
            self.get_parameter("control_enable_delay_sec").value
        )
        if not math.isfinite(self.quality_timeout_sec) or self.quality_timeout_sec <= 0.0:
            raise RuntimeError("quality_timeout_sec 必须是大于零的有限数值")
        if not math.isfinite(self.safety_timeout_sec) or self.safety_timeout_sec <= 0.0:
            raise RuntimeError("safety_timeout_sec 必须是大于零的有限数值")
        if not math.isfinite(self.settle_time_sec) or self.settle_time_sec < 0.0:
            raise RuntimeError("settle_time_sec 必须是非负有限数值")
        if (
            not math.isfinite(self.control_enable_delay_sec)
            or not 0.10 <= self.control_enable_delay_sec <= 2.0
        ):
            raise RuntimeError("control_enable_delay_sec 必须位于 0.10 到 2.00 秒之间")
        self.continue_after_failure = bool(
            self.get_parameter("continue_after_inspection_failure").value
        )
        self.autostart_enabled = bool(self.get_parameter("autostart").value)
        self.autostart_delay_sec = float(
            self.get_parameter("autostart_delay_sec").value
        )
        if (
            not math.isfinite(self.autostart_delay_sec)
            or not 0.0 <= self.autostart_delay_sec <= 300.0
        ):
            raise RuntimeError("autostart_delay_sec 必须位于 0 到 300 秒之间")

        action_name = str(self.get_parameter("navigation_action").value)
        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.control_client = self.create_client(SetBool, "/b2/control/set_enabled")
        self.state_pub = self.create_publisher(String, "/inspection/state", 10)
        target_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_pub = self.create_publisher(
            String, "/inspection/target_name", target_qos
        )
        self.capture_pub = self.create_publisher(Bool, "/inspection/capture_required", 10)
        self.heartbeat_pub = self.create_publisher(
            Bool, "/inspection/control_heartbeat", 10
        )
        self.create_subscription(Bool, "/inspection/photo_clear", self._quality_callback, 10)
        self.create_subscription(Bool, "/b2/autonomy_ok", self._safety_callback, 10)
        self.create_service(Trigger, "/inspection/start", self._start_callback)
        self.create_service(Trigger, "/inspection/pause", self._pause_callback)
        self.create_service(Trigger, "/inspection/skip_current", self._skip_callback)
        self.create_timer(0.10, self._safety_watchdog)
        self.create_timer(0.20, self._publish_heartbeat)

        self.running = False
        self.waiting_for_settle = False
        self.waiting_for_quality = False
        self.waiting_for_control_enable = False
        self.safety_ok = False
        self.last_safety_time = 0.0
        self.settle_deadline = 0.0
        self.quality_deadline = 0.0
        self.control_enable_deadline = 0.0
        self.waypoint_index = 0
        self.pose_index = 0
        self.goal_handle = None
        self.run_generation = 0
        self.autostart_pending = self.autostart_enabled
        self.autostart_ready_time = time.monotonic() + self.autostart_delay_sec
        if self.autostart_pending:
            self._publish_state(
                f"自动启动待命：至少等待 {self.autostart_delay_sec:.1f} 秒，"
                "并等待安全状态与 Nav2 就绪"
            )
        else:
            self._publish_state("空闲：等待 /inspection/start")

    def _publish_state(self, text: str) -> None:
        self.state_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _set_control_enabled(self, enabled: bool):
        if not self.control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("安全速度桥服务不可用")
            return None
        request = SetBool.Request()
        request.data = enabled
        return self.control_client.call_async(request)

    def _start_callback(self, request, response):
        del request
        success, message = self._begin_mission(wait_for_nav=True)
        response.success = success
        response.message = message
        return response

    def _begin_mission(self, wait_for_nav: bool) -> tuple[bool, str]:
        """在安全条件齐备时开始一轮巡检."""
        if self.running:
            return False, "巡检已经在运行"
        safety_fresh = (
            time.monotonic() - self.last_safety_time <= self.safety_timeout_sec
        )
        if not self.safety_ok or not safety_fresh:
            return False, "机器人、雷达或定位安全状态未就绪，拒绝启动"
        nav_ready = (
            self.nav_client.wait_for_server(timeout_sec=2.0)
            if wait_for_nav
            else self.nav_client.server_is_ready()
        )
        if not nav_ready:
            return False, "Nav2 导航服务不可用"
        self.running = True
        self.autostart_pending = False
        self.run_generation += 1
        self.waiting_for_settle = False
        self.waiting_for_quality = False
        self.waiting_for_control_enable = True
        self.capture_pub.publish(Bool(data=False))
        self._publish_heartbeat()
        self.waypoint_index = 0
        self.pose_index = 0
        self.control_enable_deadline = (
            time.monotonic() + self.control_enable_delay_sec
        )
        return True, (
            "启动请求已接收，正在预发送安全心跳并等待控制锁确认"
        )

    def _request_control_enable(self) -> None:
        """心跳稳定发布一小段时间后，再请求速度桥解锁."""
        self.waiting_for_control_enable = False
        generation = self.run_generation
        enable_future = self._set_control_enabled(True)
        if enable_future is None:
            self._enter_safe_pause("安全速度桥不可用")
            return
        enable_future.add_done_callback(
            lambda future: self._control_enable_result_callback(future, generation)
        )

    def _control_enable_result_callback(self, future, generation: int) -> None:
        if not self.running or generation != self.run_generation:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._enter_safe_pause(f"启用安全控制锁异常：{exc}")
            return
        if not result.success:
            self._enter_safe_pause(f"安全控制锁拒绝启用：{result.message}")
            return
        self._dispatch_current_pose()

    def _pause_callback(self, request, response):
        del request
        # 手工暂停后禁止本轮启动过程再次自动启动。
        self.autostart_pending = False
        self._enter_safe_pause("收到暂停命令")
        response.success = True
        response.message = "任务已暂停，控制锁已关闭"
        return response

    def _skip_callback(self, request, response):
        del request
        if not self.running:
            response.success = False
            response.message = "任务未运行"
            return response
        if not self.waiting_for_quality:
            response.success = False
            response.message = "只有等待照片质量结果时才能跳过当前点"
            return response
        self.waiting_for_quality = False
        self.capture_pub.publish(Bool(data=False))
        self._advance_waypoint("人工跳过当前巡检点")
        response.success = True
        response.message = "已跳过当前巡检点"
        return response

    def _dispatch_current_pose(self) -> None:
        if not self.running:
            return
        waypoint = self.waypoints[self.waypoint_index]
        pose_data = waypoint["poses"][self.pose_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose_data["x"])
        goal.pose.pose.position.y = float(pose_data["y"])
        yaw = math.radians(float(pose_data.get("yaw_deg", 0.0)))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._publish_state(
            f"导航：{waypoint['name']}，候选位姿 {self.pose_index + 1}/"
            f"{len(waypoint['poses'])}"
        )
        future = self.nav_client.send_goal_async(goal)
        generation = self.run_generation
        waypoint_index = self.waypoint_index
        pose_index = self.pose_index
        future.add_done_callback(
            lambda result_future: self._goal_response_callback(
                result_future, generation, waypoint_index, pose_index
            )
        )

    def _goal_response_callback(
        self, future, generation: int, waypoint_index: int, pose_index: int
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._goal_is_current(generation, waypoint_index, pose_index):
                self._enter_safe_pause(f"发送导航目标异常：{exc}")
            return
        if not self._goal_is_current(generation, waypoint_index, pose_index):
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self.goal_handle = goal_handle
        if not goal_handle.accepted:
            self._enter_safe_pause("Nav2 拒绝导航目标")
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._navigation_result_callback(
                result_future, generation, waypoint_index, pose_index
            )
        )

    def _navigation_result_callback(
        self, future, generation: int, waypoint_index: int, pose_index: int
    ) -> None:
        if not self._goal_is_current(generation, waypoint_index, pose_index):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._enter_safe_pause(f"导航结果异常：{exc}")
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._enter_safe_pause(f"导航失败，状态码={result.status}")
            return
        self.goal_handle = None

        waypoint = self.waypoints[self.waypoint_index]
        if not bool(waypoint.get("inspect", True)):
            self._advance_waypoint(f"通过路线点：{waypoint['name']}")
            return

        self.target_pub.publish(String(data=str(waypoint["name"])))
        self.waiting_for_settle = True
        self.settle_deadline = time.monotonic() + self.settle_time_sec
        self._publish_state(
            f"已到达 {waypoint['name']}，静止 {self.settle_time_sec:.1f} 秒后拍照"
        )

    def _goal_is_current(
        self, generation: int, waypoint_index: int, pose_index: int
    ) -> bool:
        """判断异步回调是否仍属于当前任务和当前候选位姿."""
        return (
            self.running
            and generation == self.run_generation
            and waypoint_index == self.waypoint_index
            and pose_index == self.pose_index
        )

    def _request_capture(self) -> None:
        """在机器人稳定后请求一组新图像和新目标框."""
        self.waiting_for_settle = False
        self.waiting_for_quality = True
        self.quality_deadline = time.monotonic() + self.quality_timeout_sec
        waypoint = self.waypoints[self.waypoint_index]
        self.capture_pub.publish(Bool(data=True))
        self._publish_state(f"拍照并评价：{waypoint['name']}")

    def _quality_callback(self, msg: Bool) -> None:
        if not self.running or not self.waiting_for_quality:
            return
        self.waiting_for_quality = False
        self.capture_pub.publish(Bool(data=False))
        waypoint = self.waypoints[self.waypoint_index]
        if msg.data:
            self._advance_waypoint(f"图像合格：{waypoint['name']}")
            return
        if self.pose_index + 1 < len(waypoint["poses"]):
            self.pose_index += 1
            self._publish_state(f"图像不合格，选择下一候选位姿：{waypoint['name']}")
            self._dispatch_current_pose()
            return

        message = f"所有候选位姿均未获得合格图像：{waypoint['name']}"
        if self.continue_after_failure:
            self._advance_waypoint(message)
        else:
            self._enter_safe_pause(message)

    def _advance_waypoint(self, reason: str) -> None:
        self._publish_state(reason)
        self.waiting_for_settle = False
        self.pose_index = 0
        if self.waypoint_index + 1 >= len(self.waypoints):
            self.running = False
            self.run_generation += 1
            self._publish_heartbeat()
            self._set_control_enabled(False)
            self._publish_state("巡检完成，控制锁已关闭")
            return
        self.waypoint_index += 1
        self._dispatch_current_pose()

    def _safety_callback(self, msg: Bool) -> None:
        self.safety_ok = bool(msg.data)
        self.last_safety_time = time.monotonic()

    def _publish_heartbeat(self) -> None:
        self.heartbeat_pub.publish(Bool(data=self.running))

    def _safety_watchdog(self) -> None:
        now = time.monotonic()
        if not self.running:
            if self.autostart_pending and now >= self.autostart_ready_time:
                safety_fresh = (
                    now - self.last_safety_time <= self.safety_timeout_sec
                )
                if (
                    self.safety_ok
                    and safety_fresh
                    and self.nav_client.server_is_ready()
                ):
                    success, message = self._begin_mission(wait_for_nav=False)
                    if success:
                        self._publish_state("安全条件已满足，Jetson 自动启动巡检")
                    else:
                        self._publish_state(f"自动启动失败：{message}")
                        self.autostart_pending = False
            return
        safety_stale = now - self.last_safety_time > self.safety_timeout_sec
        if not self.safety_ok or safety_stale:
            self._enter_safe_pause("机器人、雷达或定位安全状态丢失")
            return
        if (
            self.waiting_for_control_enable
            and now >= self.control_enable_deadline
        ):
            self._request_control_enable()
            return
        if self.waiting_for_settle and now >= self.settle_deadline:
            self._request_capture()
            return
        if self.waiting_for_quality and now > self.quality_deadline:
            self.waiting_for_quality = False
            self.capture_pub.publish(Bool(data=False))
            self._enter_safe_pause("图像质量评价超时")

    def _enter_safe_pause(self, reason: str) -> None:
        was_running = self.running
        goal_handle = self.goal_handle
        self.running = False
        self.run_generation += 1
        self.waiting_for_settle = False
        self.waiting_for_quality = False
        self.waiting_for_control_enable = False
        self.goal_handle = None
        self.capture_pub.publish(Bool(data=False))
        self._publish_heartbeat()
        self._set_control_enabled(False)
        if was_running and goal_handle is not None:
            goal_handle.cancel_goal_async()
        self._publish_state(f"安全暂停：{reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._enter_safe_pause("节点退出")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
