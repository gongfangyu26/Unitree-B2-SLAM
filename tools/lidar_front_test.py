#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math


class FrontLidarTest(Node):
    def __init__(self):
        super().__init__('front_lidar_test')

        self.sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.callback,
            10
        )

        print("Waiting for one LiDAR frame...")

    def callback(self, msg):
        front_points = []
        total_valid = 0

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if not (math.isfinite(x) and
                    math.isfinite(y) and
                    math.isfinite(z)):
                continue

            total_valid += 1

            # 暂定机器人正前方检测区域
            # 前方：0.30 ~ 5.00 m
            # 左右：±0.50 m
            # 高度：-0.30 ~ 1.50 m
            if (
                0.30 < x < 5.00 and
                abs(y) < 0.50 and
                -0.30 < z < 1.50
            ):
                front_points.append((x, y, z))

        print("\n===== Front LiDAR Test =====")
        print(f"frame_id: {msg.header.frame_id}")
        print(f"全部有效点数: {total_valid}")
        print(f"正前方区域点数: {len(front_points)}")

        if front_points:
            nearest = min(
                front_points,
                key=lambda p: math.sqrt(
                    p[0]**2 + p[1]**2 + p[2]**2
                )
            )

            x, y, z = nearest
            distance = math.sqrt(x*x + y*y + z*z)

            print(
                f"正前方最近点: "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f} m"
            )
            print(f"3D距离: {distance:.3f} m")
            print(f"前向距离 X: {x:.3f} m")
        else:
            print("当前筛选区域内没有检测到点")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = FrontLidarTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
