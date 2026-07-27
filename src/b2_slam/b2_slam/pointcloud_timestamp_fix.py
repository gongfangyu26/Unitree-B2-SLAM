"""
点云时间戳修复节点
解决问题: 传感器时间与系统时间不同步导致 TF 坐标变换失败
原理: 复制整帧点云，仅将时间戳替换为当前系统时间

Go2 默认输入: /utlidar/cloud_deskewed
B2  默认输入: /rslidar_points (RoboSense 雷达)

参考:
  https://ztl3106742440-hub.github.io/go2-tutorial/04-perception/10-lidar/
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration                # 用来构造"回拨 0.05 秒"的时间差
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PointCloudTimestampFix(Node):
    def __init__(self):
        super().__init__("pointcloud_timestamp_fix")

        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("output_topic", "/utlidar/cloud_fixed")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        # B2 的 /rslidar_points 使用 RELIABLE QoS
        self.backdate = Duration(seconds=0.05)
        # sub_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10,
        # )
        # pub_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10,
        # )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.pointcloud_callback, sub_qos
        )
        self.publisher = self.create_publisher(PointCloud2, output_topic, pub_qos)

        # 调试: 每5秒报告接收状态
        self._msg_count = 0
        self._last_log = self.get_clock().now()
        self._log_timer = self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f"TimestampFix started: {input_topic} -> {output_topic}"
        )

    def _log_status(self):
        elapsed = (self.get_clock().now() - self._last_log).nanoseconds / 1e9
        rate = self._msg_count / elapsed if elapsed > 0 else 0
        if self._msg_count == 0:
            self.get_logger().warn(
                f"WARNING: 5s no pointcloud! Check:\n"
                f"  1. Radar driver running?\n"
                f"  2. Topic exists? → ros2 topic info /rslidar_points\n"
                f"  3. QoS compatible? (needs RELIABLE)"
            )
        else:
            self.get_logger().info(
                f"OK: {self._msg_count} msgs in {elapsed:.1f}s ({rate:.1f} Hz)"
            )
        self._msg_count = 0
        self._last_log = self.get_clock().now()

    def pointcloud_callback(self, msg: PointCloud2):
        """复制点云数据，仅修正时间戳为当前系统时间"""
        self._msg_count += 1

        fixed_msg = PointCloud2()
        fixed_msg.header = msg.header
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.fields = msg.fields
        fixed_msg.is_bigendian = msg.is_bigendian
        fixed_msg.point_step = msg.point_step
        fixed_msg.row_step = msg.row_step
        fixed_msg.data = msg.data
        fixed_msg.is_dense = msg.is_dense

        # 核心: 将时间戳改为当前系统时间
        # fixed_msg.header.stamp = self.get_clock().now().to_msg()
        fixed_msg.header.stamp = (self.get_clock().now() - self.backdate).to_msg()
        self.publisher.publish(fixed_msg)


def main():
    rclpy.init()
    rclpy.spin(PointCloudTimestampFix())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
