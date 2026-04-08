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

    def __init__(self):
        super().__init__("capture_subscriber")
        self.bridge = CvBridge()
        self.count = 0
        self.latest_img = None
        self.latest_lidar = None

        self.img_subscriber = message_filters.Subscriber(
            self,
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            qos_profile=qos_profile_sensor_data
        )

        self.lidar_subscriber = message_filters.Subscriber(
            self,
            PointCloud2,
            '/rslidar_points',
            qos_profile=qos_profile_sensor_data
        )

        os.makedirs('calibration_data', exist_ok=True)
        os.makedirs('calibration_data/images', exist_ok=True)
        os.makedirs('calibration_data/pointclouds', exist_ok=True)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.img_subscriber, self.lidar_subscriber],
            queue_size=30,
            slop=1
        )
        self.sync.registerCallback(self.callback)

        thread = threading.Thread(target=self.keyboard, daemon=True)
        thread.start()
        self.get_logger().info('Press Enter to capture, q to quit.')

    def callback(self, img_msg, lidar_msg):
        self.latest_img = img_msg
        self.latest_lidar = lidar_msg
        self.get_logger().info('Synced pair received')

    def save_pcd(self, msg, filename):
        points = []
        point_step = msg.point_step
        for i in range(msg.width * msg.height):
            offset = i * point_step
            x = struct.unpack_from('f', msg.data, offset)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            intensity = struct.unpack_from('f', msg.data, offset + 12)[0]

            if not (x == 0 and y == 0 and z == 0):
                points.append((x, y, z, intensity))

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

            cv_img = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
            cv2.imwrite(f'calibration_data/images/img_{label}.png', cv_img)

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