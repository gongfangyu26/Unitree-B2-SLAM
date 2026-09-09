"""把 B2 状态转换为 ROS2 里程计、坐标变换和关节状态."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from unitree_go.msg import LowState, SportModeState


class Driver(Node):
    def __init__(self):
        super().__init__("b2_driver_py")
        # 声明参数
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("sport_state_topic", "/lf/sportmodestate")
        self.declare_parameter("low_state_topic", "/lf/lowstate")
        self.declare_parameter("state_timeout_sec", 0.50)
        self.declare_parameter("position_variance", 0.0025)
        self.declare_parameter("orientation_variance", 0.01)
        self.declare_parameter("linear_velocity_variance", 0.01)
        self.declare_parameter("angular_velocity_variance", 0.04)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.position_variance = float(
            self.get_parameter("position_variance").value
        )
        self.orientation_variance = float(
            self.get_parameter("orientation_variance").value
        )
        self.linear_velocity_variance = float(
            self.get_parameter("linear_velocity_variance").value
        )
        self.angular_velocity_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )
        variances = (
            self.position_variance,
            self.orientation_variance,
            self.linear_velocity_variance,
            self.angular_velocity_variance,
        )
        if not math.isfinite(self.state_timeout_sec) or self.state_timeout_sec <= 0.0:
            raise RuntimeError("state_timeout_sec 必须是大于零的有限数值")
        if not all(math.isfinite(value) and value >= 0.0 for value in variances):
            raise RuntimeError("里程计协方差参数必须是非负有限数值")
        self.last_sport_state_time = 0.0
        self.sport_state_valid = False
        self.last_error_text = ""
        self.last_error_time = 0.0
        # 创建里程计发布对象
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.state_ok_pub = self.create_publisher(Bool, "/b2/state_ok", 10)
        # 创建机器人运动状态订阅对象
        self.mode_sub = self.create_subscription(
            SportModeState,
            str(self.get_parameter("sport_state_topic").value),
            self.mode_cb,
            qos_profile_sensor_data,
        )
        # 创建坐标变换广播器
        self.tf_bro = TransformBroadcaster(self)
        # 创建关节状态发布对象
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        # 创建低层状态获取对象，回调函数中实现数据转换
        self.state_sub = self.create_subscription(
            LowState,
            str(self.get_parameter("low_state_topic").value),
            self.state_cb,
            qos_profile_sensor_data,
        )
        self.create_timer(0.10, self._state_watchdog)

    def state_cb(self, state: LowState):
        # 获取并发布关节状态
        joint_state = JointState()
        # 数据组织
        joint_state.header.stamp = self.get_clock().now().to_msg()
        # 设置关节名称
        joint_state.name = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        ]
        # 设置角度
        if len(state.motor_state) < 12:
            self.get_logger().error("LowState 电机数量少于 12，拒绝发布关节状态")
            return
        positions = [float(state.motor_state[i].q) for i in range(12)]
        if not all(math.isfinite(value) for value in positions):
            self.get_logger().error("LowState 含非法关节角，拒绝发布关节状态")
            return
        joint_state.position = positions
        # 发布
        self.joint_pub.publish(joint_state)

    def mode_cb(self, mode: SportModeState):
        if int(mode.error_code) != 0:
            self._invalidate_state(f"SportModeState 错误码={int(mode.error_code)}")
            return
        if len(mode.position) < 3 or len(mode.velocity) < 3:
            self._invalidate_state("SportModeState 位置或速度数组长度不足")
            return
        if len(mode.imu_state.quaternion) < 4:
            self._invalidate_state("SportModeState 四元数长度不足")
            return
        position = [float(value) for value in mode.position[:3]]
        velocity = [float(value) for value in mode.velocity[:3]]
        quaternion = [float(value) for value in mode.imu_state.quaternion[:4]]
        yaw_speed = float(mode.yaw_speed)
        values = position + velocity + quaternion + [yaw_speed]
        if not all(math.isfinite(value) for value in values):
            self._invalidate_state("SportModeState 含 NaN 或无穷大")
            return
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
        if quaternion_norm < 1.0e-6:
            self._invalidate_state("收到零长度四元数")
            return
        quaternion = [value / quaternion_norm for value in quaternion]
        self.last_sport_state_time = time.monotonic()
        self.sport_state_valid = True
        # 解析生成里程计对象
        odom = Odometry()
        # 组织数据
        stamp = self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        # 位姿
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        # 宇树消息中的四元数顺序为 w、x、y、z。
        odom.pose.pose.orientation.w = quaternion[0]
        odom.pose.pose.orientation.x = quaternion[1]
        odom.pose.pose.orientation.y = quaternion[2]
        odom.pose.pose.orientation.z = quaternion[3]
        odom.twist.twist.linear.x = velocity[0]
        odom.twist.twist.linear.y = velocity[1]
        odom.twist.twist.linear.z = velocity[2]
        odom.twist.twist.angular.z = yaw_speed
        self._set_covariances(odom)
        # 发布
        self.odom_pub.publish(odom)

        # 广播坐标变化
        if self.publish_tf:
            # 生成坐标变换数据
            trans_form = TransformStamped()
            trans_form.header.stamp = stamp
            trans_form.header.frame_id = self.odom_frame
            trans_form.child_frame_id = self.base_frame
            # 设置偏移量
            trans_form.transform.translation.x = odom.pose.pose.position.x
            trans_form.transform.translation.y = odom.pose.pose.position.y
            trans_form.transform.translation.z = odom.pose.pose.position.z
            # 设置旋转角度
            trans_form.transform.rotation = odom.pose.pose.orientation
            # 发布
            self.tf_bro.sendTransform(trans_form)

    def _state_watchdog(self):
        """状态超过规定时间未更新时，向安全层报告失联."""
        if self.last_sport_state_time == 0.0:
            self.state_ok_pub.publish(Bool(data=False))
            return
        elapsed = time.monotonic() - self.last_sport_state_time
        state_ok = self.sport_state_valid and elapsed <= self.state_timeout_sec
        self.state_ok_pub.publish(Bool(data=state_ok))

    def _invalidate_state(self, reason: str) -> None:
        """立即把非法状态传递到安全层，并限制相同错误的日志频率."""
        self.sport_state_valid = False
        self.state_ok_pub.publish(Bool(data=False))
        now = time.monotonic()
        if reason != self.last_error_text or now - self.last_error_time >= 2.0:
            self.get_logger().error(f"{reason}，拒绝发布里程计")
            self.last_error_text = reason
            self.last_error_time = now

    def _set_covariances(self, odom: Odometry) -> None:
        """写入可配置的保守对角协方差，避免用全零表示完全确定."""
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        for index in (0, 7, 14):
            odom.pose.covariance[index] = self.position_variance
            odom.twist.covariance[index] = self.linear_velocity_variance
        for index in (21, 28, 35):
            odom.pose.covariance[index] = self.orientation_variance
            odom.twist.covariance[index] = self.angular_velocity_variance


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
