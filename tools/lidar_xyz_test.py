#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math


class LidarXYZTest(Node):
    def __init__(self):
        super().__init__('lidar_xyz_test')

        self.sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.callback,
            10
        )

        print("Waiting for one /rslidar_points frame...")

    def callback(self, msg):
        points = []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))

        print("\n===== LiDAR XYZ Test =====")
        print(f"frame_id: {msg.header.frame_id}")
        print(f"width x height: {msg.width} x {msg.height}")
        print(f"Valid XYZ points: {len(points)}")

        if points:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            zs = [p[2] for p in points]

            print(f"X range: {min(xs):.3f} ~ {max(xs):.3f} m")
            print(f"Y range: {min(ys):.3f} ~ {max(ys):.3f} m")
            print(f"Z range: {min(zs):.3f} ~ {max(zs):.3f} m")

            nearest = min(
                points,
                key=lambda p: math.sqrt(
                    p[0]**2 + p[1]**2 + p[2]**2
                )
            )

            distance = math.sqrt(
                nearest[0]**2 +
                nearest[1]**2 +
                nearest[2]**2
            )

            print(
                f"Nearest point: "
                f"x={nearest[0]:.3f}, "
                f"y={nearest[1]:.3f}, "
                f"z={nearest[2]:.3f} m"
            )
            print(f"Nearest 3D distance: {distance:.3f} m")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = LidarXYZTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
