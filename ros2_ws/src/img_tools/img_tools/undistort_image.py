import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ZedUndistortCropNode(Node):
    def __init__(self):
        super().__init__('zed_undistort_crop_node')

        # Topics
        self.declare_parameter('input_topic', '/zed/zed_node/rgb/color/raw/image')
        self.declare_parameter('output_image_topic', '/zed/zed_node/rgb/color/image_rect_color')
        self.declare_parameter('output_camera_info_topic', '/zed/zed_node/rgb/color/camera_info_rect')

        # Optional extra trim after ROI crop, in pixels
        self.declare_parameter('extra_crop_pixels', 0)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        output_camera_info_topic = self.get_parameter('output_camera_info_topic').get_parameter_value().string_value
        self.extra_crop_pixels = self.get_parameter('extra_crop_pixels').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Original calibration
        self.camera_matrix = np.array([
            [1416.95644,    0.0,       942.508882],
            [   0.0,     1420.40169,   536.529123],
            [   0.0,        0.0,         1.0]
        ], dtype=np.float64)

        self.dist_coeffs = np.array(
            [-0.02675044, 0.15411475, -0.00074599, -0.00087969, -0.36664865],
            dtype=np.float64
        )

        # Cached undistortion state
        self.map1 = None
        self.map2 = None
        self.cached_size = None
        self.new_camera_matrix = None
        self.valid_roi = None

        # ROS interfaces
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            output_image_topic,
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            output_camera_info_topic,
            10
        )

        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing image to: {output_image_topic}')
        self.get_logger().info(f'Publishing camera_info to: {output_camera_info_topic}')
        self.get_logger().info(f'Extra crop pixels: {self.extra_crop_pixels}')
        self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
        self.get_logger().info(f'Dist coeffs: {self.dist_coeffs.ravel()}')

    def build_undistort_maps(self, width, height):
        image_size = (width, height)

        # alpha=0 => minimize black/invalid border pixels
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            image_size,
            0,
            image_size
        )

        map1, map2 = cv.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            new_camera_matrix,
            image_size,
            cv.CV_16SC2
        )

        self.map1 = map1
        self.map2 = map2
        self.cached_size = image_size
        self.new_camera_matrix = new_camera_matrix
        self.valid_roi = roi

        self.get_logger().info(
            f'Updated undistort maps for size {width}x{height}, ROI={roi}'
        )

    def undistort_image(self, img):
        height, width = img.shape[:2]

        if self.cached_size != (width, height) or self.map1 is None or self.map2 is None:
            self.build_undistort_maps(width, height)

        undistorted = cv.remap(img, self.map1, self.map2, interpolation=cv.INTER_LINEAR)
        return undistorted

    def crop_to_valid_roi(self, img):
        if self.valid_roi is None:
            return img, 0, 0

        x, y, w, h = self.valid_roi
        if w <= 0 or h <= 0:
            return img, 0, 0

        # Optional extra trim for any tiny interpolation slivers
        pad = max(0, int(self.extra_crop_pixels))

        x1 = x + pad
        y1 = y + pad
        x2 = x + w - pad
        y2 = y + h - pad

        img_h, img_w = img.shape[:2]
        x1 = max(0, min(x1, img_w - 1))
        y1 = max(0, min(y1, img_h - 1))
        x2 = max(x1 + 1, min(x2, img_w))
        y2 = max(y1 + 1, min(y2, img_h))

        cropped = img[y1:y2, x1:x2]
        return cropped, x1, y1

    def make_camera_info_msg(self, header, out_width, out_height, crop_x, crop_y):
        cam_info = CameraInfo()
        cam_info.header = header
        cam_info.width = int(out_width)
        cam_info.height = int(out_height)

        # Rectified / undistorted output
        cam_info.distortion_model = 'plumb_bob'
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = float(self.new_camera_matrix[0, 0])
        fy = float(self.new_camera_matrix[1, 1])
        cx = float(self.new_camera_matrix[0, 2])
        cy = float(self.new_camera_matrix[1, 2])

        # Cropping shifts the principal point
        cx_cropped = cx - float(crop_x)
        cy_cropped = cy - float(crop_y)

        cam_info.k = [
            fx, 0.0, cx_cropped,
            0.0, fy, cy_cropped,
            0.0, 0.0, 1.0
        ]

        cam_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        cam_info.p = [
            fx, 0.0, cx_cropped, 0.0,
            0.0, fy, cy_cropped, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return cam_info

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV color image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort
            undistorted = self.undistort_image(cv_image)

            # Auto-crop using the valid ROI from OpenCV
            cropped, crop_x, crop_y = self.crop_to_valid_roi(undistorted)

            # Convert back to ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
            out_msg.header = msg.header

            # Matching camera info for RViz Camera display
            cam_info_msg = self.make_camera_info_msg(
                header=msg.header,
                out_width=cropped.shape[1],
                out_height=cropped.shape[0],
                crop_x=crop_x,
                crop_y=crop_y
            )

            self.image_pub.publish(out_msg)
            self.camera_info_pub.publish(cam_info_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ZedUndistortCropNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


