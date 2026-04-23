"""
lidar_to_camera.py — LiDAR timestamp synchronization diagnostic.

Subscribes to /rslidar_points and prints the latency between each LiDAR
message timestamp and the system wall clock. Use this to verify that the
Jetson's clock and the LiDAR's internal clock are reasonably in sync.

Usage:
    python3 lidar_to_camera.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image
import time


class PointCloudCameraNode(Node):
    """Prints the time difference between LiDAR message stamps and wall clock."""

    def __init__(self):
        super().__init__("PointCloudCameraNode")
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.callback,
            qos_profile=qos_profile_sensor_data
        )

    def callback(self, msg):
        """Compute and print wall-clock latency for each LiDAR scan."""
        time_lidar = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        now = time.time()
        print(now - time_lidar)
        print("--------")


def main():
    rclpy.init()
    node = PointCloudCameraNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()

