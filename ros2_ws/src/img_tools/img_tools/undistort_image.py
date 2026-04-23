import os
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ZedUndistortCropNode(Node):
    def __init__(self):
        super().__init__('zed_undistort_crop_node')

        self.declare_parameter('input_topic', '/zed/zed_node/rgb/color/raw/image')
        self.declare_parameter('output_topic', '/zed/zed_node/rgb/color/undistorted_cropped/image')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/color/undistorted_cropped/camera_info')

        self.declare_parameter('package_name', 'img_tools')
        self.declare_parameter('calibration_filename', 'calibration_parameters.npz')

        self.declare_parameter('crop_x', 0)
        self.declare_parameter('crop_y', 0)
        self.declare_parameter('crop_width', -1)
        self.declare_parameter('crop_height', -1)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        package_name = self.get_parameter('package_name').get_parameter_value().string_value
        calibration_filename = self.get_parameter('calibration_filename').get_parameter_value().string_value

        self.crop_x = self.get_parameter('crop_x').get_parameter_value().integer_value
        self.crop_y = self.get_parameter('crop_y').get_parameter_value().integer_value
        self.crop_width = self.get_parameter('crop_width').get_parameter_value().integer_value
        self.crop_height = self.get_parameter('crop_height').get_parameter_value().integer_value

        self.bridge = CvBridge()

        self.camera_matrix, self.dist_coeffs = self.load_calibration(package_name, calibration_filename)

        self.map1 = None
        self.map2 = None
        self.cached_size = None

        self.new_camera_matrix = None
        self.undistort_roi = None

        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, output_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)

    def load_calibration(self, package_name, calibration_filename):
        default_camera_matrix = np.array([
            [1416.95644, 0.0, 942.508882],
            [0.0, 1420.40169, 536.529123],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        default_dist_coeffs = np.array(
            [-0.02675044, 0.15411475, -0.00074599, -0.00087969, -0.36664865],
            dtype=np.float64
        )

        try:
            pkg_share = get_package_share_directory(package_name)
            calib_path = os.path.join(pkg_share, 'calibration', calibration_filename)

            data = np.load(calib_path)

            if 'cameraMatrix' in data:
                camera_matrix = data['cameraMatrix']
            elif 'camera_matrix' in data:
                camera_matrix = data['camera_matrix']
            else:
                raise KeyError('cameraMatrix not found in npz')

            if 'distCoeffs' in data:
                dist_coeffs = data['distCoeffs']
            elif 'dist_coeffs' in data:
                dist_coeffs = data['dist_coeffs']
            else:
                raise KeyError('distCoeffs not found in npz')

            camera_matrix = np.array(camera_matrix, dtype=np.float64)
            dist_coeffs = np.array(dist_coeffs, dtype=np.float64).reshape(-1)

            self.get_logger().info(f'Loaded calibration from: {calib_path}')
            return camera_matrix, dist_coeffs

        except Exception as e:
            self.get_logger().warning(
                f'Failed to load calibration file, using defaults: {e}'
            )
            return default_camera_matrix, default_dist_coeffs

    def build_undistort_maps(self, width, height):
        image_size = (width, height)

        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            image_size,
            0,   # alpha=0 removes border area as much as possible
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
        self.undistort_roi = roi

    def undistort_image(self, img):
        height, width = img.shape[:2]

        if self.cached_size != (width, height) or self.map1 is None:
            self.build_undistort_maps(width, height)

        return cv.remap(img, self.map1, self.map2, interpolation=cv.INTER_LINEAR)

    def crop_valid_roi(self, img):
        if self.undistort_roi is None:
            return img, 0, 0

        x, y, w, h = self.undistort_roi

        if w <= 0 or h <= 0:
            return img, 0, 0

        return img[y:y + h, x:x + w], x, y

    def crop_image(self, img):
        h, w = img.shape[:2]

        x = self.crop_x
        y = self.crop_y
        crop_w = self.crop_width
        crop_h = self.crop_height

        if crop_w == -1:
            crop_w = w - x
        if crop_h == -1:
            crop_h = h - y

        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        crop_w = max(1, min(crop_w, w - x))
        crop_h = max(1, min(crop_h, h - y))

        return img[y:y + crop_h, x:x + crop_w], x, y

    def make_camera_info_msg(self, header, out_width, out_height, crop_x, crop_y):
        cam_info = CameraInfo()
        cam_info.header = header
        cam_info.width = out_width
        cam_info.height = out_height

        cam_info.distortion_model = 'plumb_bob'
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = float(self.new_camera_matrix[0, 0])
        fy = float(self.new_camera_matrix[1, 1])
        cx = float(self.new_camera_matrix[0, 2])
        cy = float(self.new_camera_matrix[1, 2])

        cx_cropped = cx - crop_x
        cy_cropped = cy - crop_y

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
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            undistorted = self.undistort_image(cv_image)

            roi_cropped, roi_x, roi_y = self.crop_valid_roi(undistorted)
            cropped, user_x, user_y = self.crop_image(roi_cropped)

            total_crop_x = roi_x + user_x
            total_crop_y = roi_y + user_y

            out_msg = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
            out_msg.header = msg.header

            cam_info_msg = self.make_camera_info_msg(
                header=msg.header,
                out_width=cropped.shape[1],
                out_height=cropped.shape[0],
                crop_x=total_crop_x,
                crop_y=total_crop_y
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
