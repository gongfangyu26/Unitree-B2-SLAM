"""
    需求：订阅twist消息，并将之转换为go2所需的request消息格式，以控制机器狗运动。
    实现：
        1.创建一个Request的发布对象
        2.创建一个Twist的订阅对象
        3.在回调函数中，将Twist消息转换为Request消息，并发布出去
"""
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from geometry_msgs.msg import Twist
from .sport_model import ROBOT_SPORT_API_IDS
import json

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        self.request_publisher = self.create_publisher(Request, '/api/sport/request', 10)
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

    def twist_callback(self, twist):
        # 将Twist消息转换为Request消息
        request = Request()
        # 获取速度
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z
        # 设置api_id
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"] 
        if x != 0 or y != 0 or z != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            # 设置参数
            js = {"x": x, "y": y, "z": z}
            request.parameter = json.dumps(js)
        request.header.identity.api_id = api_id
        self.request_publisher.publish(request)

def main(args=None):
    rclpy.init(args=args)
    twist_bridge = TwistBridge()
    rclpy.spin(twist_bridge)
    rclpy.shutdown()

if __name__ == '__main__':
    main()