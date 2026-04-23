# img_tools — Image Undistortion and Cropping Node

A ROS 2 Python package that provides a real-time image undistortion and cropping node for the ZED camera. This node corrects lens distortion, auto-crops to the valid image region, and republishes both the corrected image and a matching `CameraInfo` message for use with RViz and downstream processing.

---

## What It Does

1. **Subscribes** to the raw (distorted) ZED camera image
2. **Undistorts** using a hardcoded camera matrix and distortion coefficients
3. **Crops** to the valid region of interest (ROI) produced by `cv2.getOptimalNewCameraMatrix`
4. **Publishes** the corrected image and a corresponding `CameraInfo` message with updated intrinsics that reflect the crop offset

---

## Prerequisites

This package requires the following ROS 2 dependencies (declared in `package.xml`):

- `rclpy`
- `sensor_msgs`
- `cv_bridge`

And the following Python packages:

- `numpy`
- `opencv-python` (`cv2`)

---

## Building

From the ROS 2 workspace root:

```bash
cd ~/jetson-local-mapping/ros2_ws
colcon build --packages-select img_tools --symlink-install
source install/setup.bash
```

---

## Running

```bash
ros2 run img_tools undistort_image
```

### Parameters

All parameters can be set via the command line or a launch file:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `input_topic` | string | `/zed/zed_node/rgb/color/raw/image` | Topic to subscribe to for raw images |
| `output_image_topic` | string | `/zed/zed_node/rgb/color/image_rect_color` | Topic for the undistorted image |
| `output_camera_info_topic` | string | `/zed/zed_node/rgb/color/camera_info_rect` | Topic for the matching CameraInfo |
| `extra_crop_pixels` | int | `0` | Additional border pixels to trim after ROI crop |

Example with custom parameters:

```bash
ros2 run img_tools undistort_image \
  --ros-args \
  -p input_topic:=/zed/zed_node/rgb/color/raw/image \
  -p output_image_topic:=/camera/image_rect \
  -p extra_crop_pixels:=5
```

---

## Camera Calibration Values

The node uses the following hardcoded intrinsics (ZED camera, FHD resolution):

```
Camera Matrix:
  fx = 1416.96    cx = 942.51
  fy = 1420.40    cy = 536.53

Distortion Coefficients (plumb_bob):
  k1 = -0.02675    k2 = 0.15411    p1 = -0.00075    p2 = -0.00088    k3 = -0.36665
```

> **Note:** If you are using a different ZED model or resolution, update the `camera_matrix` and `dist_coeffs` arrays in `undistort_image.py`.

The output `CameraInfo` message reports zero distortion (since the image is already undistorted) and adjusts the principal point to account for any cropping.

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| (configurable) | `sensor_msgs/Image` | Undistorted, cropped RGB image |
| (configurable) | `sensor_msgs/CameraInfo` | Intrinsics matching the output image (zero distortion, crop-adjusted principal point) |

---

## How It Works

1. On the first received frame, the node computes undistortion remap tables using `cv2.initUndistortRectifyMap` and caches them for subsequent frames.
2. Each incoming frame is remapped (undistorted), then cropped to the valid ROI returned by `cv2.getOptimalNewCameraMatrix` (with `alpha=0` to minimize black borders).
3. The `CameraInfo` message is constructed with the new camera matrix, with `cx` and `cy` shifted by the crop offset.
4. If the input resolution changes mid-stream, the remap tables are automatically recomputed.