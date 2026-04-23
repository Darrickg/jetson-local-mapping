"""
capture_data.py — Synchronized LiDAR + camera data capture for calibration.

ROS 2 node that subscribes to the ZED camera image and Robosense LiDAR point
cloud, time-synchronizes them, and saves matched PNG/PCD pairs to disk on
each keypress. The saved pairs are used by the MATLAB calibration pipeline.

Usage:
    python3 capture_data.py

Controls:
    Enter — save the latest synchronized pair
    q     — quit
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import message_filters
import cv2
import os
import struct
import threading


class CaptureSubscriberNode(Node):
    """Captures time-synchronized camera/LiDAR pairs on demand."""

    def __init__(self):
        super().__init__("capture_subscriber")
        self.bridge = CvBridge()
        self.count = 0
        self.latest_img = None
        self.latest_lidar = None

        # Subscribe to ZED raw image and Robosense LiDAR point cloud
        self.img_subscriber = message_filters.Subscriber(
            self,
            Image,
            '/zed/zed_node/rgb/color/raw/image',
            qos_profile=qos_profile_sensor_data
        )

        self.lidar_subscriber = message_filters.Subscriber(
            self,
            PointCloud2,
            '/rslidar_points',
            qos_profile=qos_profile_sensor_data
        )

        # Create output directories for calibration data
        os.makedirs('calibration_data', exist_ok=True)
        os.makedirs('calibration_data/images', exist_ok=True)
        os.makedirs('calibration_data/pointclouds', exist_ok=True)

        # Pair messages by timestamp (10 s slop to handle clock offsets)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.img_subscriber, self.lidar_subscriber],
            queue_size=30,
            slop= 10)
        self.sync.registerCallback(self.callback)

        # Keyboard listener runs in a background thread
        thread = threading.Thread(target=self.keyboard, daemon=True)
        thread.start()
        self.get_logger().info('Press Enter to capture, q to quit.')

    def callback(self, img_msg, lidar_msg):
        """Store the latest synchronized image/LiDAR pair and log the time diff."""
        self.latest_img = img_msg
        self.latest_lidar = lidar_msg
        self.get_logger().info('Synced pair received')
        img_t = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        lid_t = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f'Sync diff: {abs(img_t - lid_t)*1000:.1f} ms')
        self.latest_img = img_msg
        self.latest_lidar = lidar_msg

    def save_pcd(self, msg, filename):
        """Convert a PointCloud2 message to ASCII PCD format and write to disk."""
        points = []
        point_step = msg.point_step
        # Unpack each point: x, y, z, intensity (4 floats at known offsets)
        for i in range(msg.width * msg.height):
            offset = i * point_step
            x = struct.unpack_from('f', msg.data, offset)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            intensity = struct.unpack_from('f', msg.data, offset + 12)[0]

            # Skip zero-points (invalid returns)
            if not (x == 0 and y == 0 and z == 0):
                points.append((x, y, z, intensity))

        # Write PCD v0.7 ASCII header + point data
        with open(filename, 'w') as f:
            f.write('VERSION .7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for p in points:
                f.write(f'{p[0]} {p[1]} {p[2]} {p[3]}\n')

    def keyboard(self):
        """Block on stdin; save a pair on Enter, exit on 'q'."""
        while True:
            key = input()
            if key == 'q':
                raise SystemExit

            if self.latest_img is None or self.latest_lidar is None:
                self.get_logger().warn(f'latest_img = {self.latest_img}')
                self.get_logger().warn(f'latest_lidar = {self.latest_lidar}')
                self.get_logger().warn('No data yet, wait...')
                continue

            self.count += 1
            label = f'{self.count:03d}'

            # Save image as PNG
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
            cv2.imwrite(f'calibration_data/images/img_{label}.png', cv_img)

            # Save point cloud as PCD
            self.save_pcd(self.latest_lidar, f'calibration_data/pointclouds/pc_{label}.pcd')

            self.get_logger().info(f'Captured {self.count}')


def main():
    rclpy.init()
    node = CaptureSubscriberNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
