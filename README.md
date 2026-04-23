# Jetson Local Mapping

## Abstract

This project aims to build a calibrated, synchronized data-collection rig on a mobile robot using a Robosense AIRY 3D LiDAR and an RGB camera mounted on a Turtlebot2. The robot will be teleoperated to collect time synced LiDAR point clouds and camera images for indoor mapping and perception research. The baseline starting point will be off-the-shelf ROS2 sensor drivers and standard ROS2 tooling, then extended with a complete calibration pipeline: camera intrinsic calibration, LiDAR–camera extrinsic 6D calibration, and a repeatable launch/data-recording workflow. The end result will be a ROS2 system that publishes correctly framed topics, supports visualization, and can reliably record a small calibrated dataset. This will also be accompanied by a visual interface allowing for raw sensor information to be observed, alongside some fused data representations.

---

## Hardware

- **Robot platform:** Turtlebot2
- **LiDAR:** Robosense AIRY 3D
- **Camera:** ZED (stereo RGB camera)
- **Compute:** NVIDIA Jetson (onboard)

---

## Prerequisites

- **ROS2** (tested with ROS2 Humble)
- **ZED SDK** and the [`zed_wrapper`](https://github.com/stereolabs/zed-ros2-wrapper) ROS2 package
- **Robosense LiDAR SDK** ([`rslidar_sdk`](https://github.com/RoboSense-LiDAR/rslidar_sdk)) built as a ROS2 node
- **Python dependencies:** `pyzed`, `opencv-python`, `cv_bridge`, `message_filters`
- **MATLAB** (for extrinsic calibration scripts in `matlab-calibration/`)
- A ROS2 workspace (referred to below as `~/ros2_ws`)

---

## Setup

### 1. Clone and build the workspace

```bash
cd ~/ros2_ws/src
# Clone zed_wrapper and rslidar_sdk into src, then:
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Copy the RViz configuration

The launch file expects the RViz config at `~/sensor_fusion.rviz`. Copy it from the repository root:

```bash
cp sensor_fusion.rviz ~/sensor_fusion.rviz
```

---

## Running the Full Sensor Fusion Stack

The `sensor_fusion.launch.py` file starts all components in one command:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sensor_fusion.launch.py
```

This launches:
1. A static TF transform from `rslidar` → `zed_left_camera_frame_optical` using the MATLAB-calibrated extrinsics.
2. The ZED camera wrapper (publishes RGB images and point cloud).
3. The Robosense LiDAR node (publishes `/rslidar_points`).
4. RViz2 with the pre-configured `sensor_fusion.rviz` view.

---

## Calibration Workflow

### Step 1 — Collect synchronized calibration data

Start both sensor drivers (ZED + LiDAR) and then run the calibration capture node:

```bash
bash data_start.sh
```

Or manually:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed &
ros2 launch rslidar_sdk start.py &
python capture_calibration.py
```

While `capture_calibration.py` is running, press **Enter** to save a synchronized image/point-cloud pair, or **q** to quit. Pairs are written to `calibration_data/images/` and `calibration_data/pointclouds/`.

Aim for **20–30 pairs** with the calibration target in varying positions across the shared sensor field of view.

### Step 2 — Camera intrinsics (MATLAB)

Open MATLAB and run `matlab-calibration/lidarcam.m`. This script hard-codes the ZED FHD intrinsic parameters (focal length, principal point, image size) and saves them to `matlab-calibration/zed2_fhd_intrinsics.mat`.

If you need to re-derive intrinsics from scratch, use the MATLAB Camera Calibrator app with a checkerboard target.

### Step 3 — LiDAR–camera extrinsic calibration (MATLAB)

1. Edit `matlab-calibration/initialtransform.m` to set a reasonable initial guess for the 6DoF transform (rotation in degrees + translation in metres).
2. Use the MATLAB **Lidar Camera Calibrator** app (or a custom script) with the collected `calibration_data/` pairs and `zed2_fhd_intrinsics.mat`.
3. The refined transform is saved to `matlab-calibration/results.mat` and the quaternion/translation values should be copied into `sensor_fusion.launch.py` under `static_tf_node`.

---

## Recording a Dataset (rosbag)

To record all relevant topics for offline processing:

```bash
ros2 bag record -o my_dataset \
  /rslidar_points \
  /zed/zed_node/rgb/color/rect/image \
  /zed/zed_node/rgb/camera_info \
  /zed/zed_node/point_cloud/cloud_registered \
  /tf_static
```

Play back a recording:

```bash
ros2 bag play my_dataset
```

---

## Visualization

### RViz2

The included `sensor_fusion.rviz` configuration displays:
- **LiDAR point cloud** (`/rslidar_points`) coloured by intensity.
- **ZED point cloud** (`/zed/zed_node/point_cloud/cloud_registered`) coloured by RGB.
- **ZED camera feed** (`/zed/zed_node/rgb/color/rect/image`) as a camera overlay.

Launch RViz2 standalone:

```bash
rviz2 -d ~/sensor_fusion.rviz
```

### rqt image view

To inspect the raw camera stream:

```bash
ros2 run rqt_image_view rqt_image_view
```

Subscribe to `/zed/zed_node/rgb/color/rect/image` (or its compressed variant).

---

## Project Structure

```
jetson-local-mapping/
├── camera_capture.py          # Standalone ZED image capture using the pyzed SDK (no ROS)
├── capture_calibration.py     # ROS2 node: time-syncs camera + LiDAR and saves pairs for calibration
├── lidar_to_camera.py         # ROS2 node: prints LiDAR timestamp latency for sync diagnostics
├── sensor_fusion.launch.py    # ROS2 launch file: starts TF, ZED, LiDAR, and RViz2 together
├── sensor_fusion.rviz         # RViz2 configuration for visualizing fused sensor data
├── data_start.sh              # Shell script shortcut to launch sensors and start calibration capture
└── matlab-calibration/
    ├── initialtransform.m     # Defines the initial guess for the LiDAR-to-camera extrinsic transform
    ├── lidarcam.m             # Encodes ZED camera intrinsics and saves them as a .mat file
    ├── zed2_fhd_intrinsics.mat # Saved camera intrinsic parameters (focal length, principal point)
    ├── results.mat            # Output from MATLAB LiDAR–camera calibration (final extrinsics)
    └── calibration_data/
        ├── images/            # Captured PNG images used for calibration (img_001.png … img_030.png)
        └── pointclouds/       # Captured PCD point clouds paired with the images (pc_001.pcd … pc_030.pcd)
```

### File descriptions

| File | Purpose |
|---|---|
| `camera_capture.py` | Directly accesses the ZED camera via the `pyzed` SDK (no ROS required). Grabs 1000 frames and saves each as a timestamped PNG in `camera_test/`. Useful for verifying the camera works in isolation. |
| `capture_calibration.py` | A ROS2 node that subscribes to `/zed/zed_node/rgb/color/raw/image` and `/rslidar_points`, uses `ApproximateTimeSynchronizer` to pair frames within 60 ms, and saves matched PNG + PCD pairs to `calibration_data/` on each Enter keypress. This is the primary data-collection tool for the calibration pipeline. |
| `lidar_to_camera.py` | A lightweight ROS2 diagnostic node that subscribes to `/rslidar_points` and prints the difference between the LiDAR message timestamp and the current system clock. Use this to verify time synchronisation between the Jetson and the LiDAR. |
| `sensor_fusion.launch.py` | The main ROS2 launch file. Publishes a static TF transform (LiDAR → camera frame) using the calibrated quaternion/translation from MATLAB, then starts the ZED wrapper, Robosense LiDAR node, and RViz2 all together. Edit the `--qx/--qy/--qz/--qw` and `--x/--y/--z` arguments after re-calibration. |
| `sensor_fusion.rviz` | RViz2 configuration file. Displays the LiDAR intensity point cloud, the ZED coloured point cloud, and a camera overlay in a single view. The fixed frame is `zed_left_camera_frame_optical`. |
| `data_start.sh` | Convenience shell script that launches the ZED wrapper and LiDAR in the background, sources the ROS2 workspace, and immediately starts `capture_calibration.py`. |
| `matlab-calibration/initialtransform.m` | Constructs a `rigidtform3d` initial-guess transform from user-specified Euler angles and translation. Feed this into the MATLAB Lidar Camera Calibrator as the starting point. |
| `matlab-calibration/lidarcam.m` | Hard-codes the ZED FHD intrinsic parameters and saves a `cameraIntrinsics` object to `zed2_fhd_intrinsics.mat` for use in the MATLAB calibration toolbox. |
| `matlab-calibration/zed2_fhd_intrinsics.mat` | Pre-saved MATLAB camera intrinsics file (1920×1080, f≈1401.65 px). |
| `matlab-calibration/results.mat` | MATLAB workspace file containing the final extrinsic calibration result. The quaternion and translation values in this file are what get copied into `sensor_fusion.launch.py`. |

---

## Key ROS2 Topics

| Topic | Type | Source |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Robosense LiDAR node |
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | ZED wrapper |
| `/zed/zed_node/rgb/color/raw/image` | `sensor_msgs/Image` | ZED wrapper |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | ZED wrapper |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | ZED wrapper |


