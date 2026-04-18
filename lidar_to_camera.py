import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image
import time

class PointCloudCameraNode(Node):
    def __init__(self):
        super().__init__("PointCloudCameraNode")
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.callback,
            qos_profile=qos_profile_sensor_data
        )
    def callback(self, msg):
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

