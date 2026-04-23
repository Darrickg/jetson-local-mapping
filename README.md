# Jetson Local Mapping

A calibrated, synchronized sensor-fusion rig for indoor mapping and perception research.  
Uses a **Robosense AIRY 3D LiDAR** and a **ZED stereo camera** mounted on a **Turtlebot2**, with all processing running onboard an **NVIDIA Jetson**.

---

## Table of Contents

- [Abstract](#abstract)
- [Hardware](#hardware)
- [Prerequisites](#prerequisites)
- [Repository Setup](#repository-setup)
- [Calibration Workflow](#calibration-workflow)
- [Running the Sensor Fusion Stack](#running-the-sensor-fusion-stack)
- [Recording and Playing Back Datasets](#recording-and-playing-back-datasets)
- [Visualization](#visualization)
- [Project Structure](#project-structure)
- [ROS 2 Topics Reference](#ros-2-topics-reference)
- [Sample Data](#sample-data)
- [Troubleshooting](#troubleshooting)

---

## Abstract

This project builds a calibrated, synchronized data-collection rig on a mobile robot using a Robosense AIRY 3D LiDAR and an RGB camera mounted on a Turtlebot2. The robot is teleoperated to collect time-synced LiDAR point clouds and camera images for indoor mapping and perception research. The system extends off-the-shelf ROS 2 sensor drivers with a complete calibration pipeline: camera intrinsic calibration, LiDAR–camera extrinsic 6-DOF calibration, and a repeatable launch/data-recording workflow. The end result is a ROS 2 system that publishes correctly framed topics, supports visualization in RViz2, and can reliably record calibrated datasets. A visual interface allows raw sensor information to be observed alongside fused data representations.

---

## Hardware

| Component | Model |
|---|---|
| Robot platform | Turtlebot2 |
| LiDAR | Robosense AIRY 3D |
| Camera | ZED (stereo RGB, 1920×1080 FHD) |
| Compute | NVIDIA Jetson (onboard) |

> **Note:** The LiDAR and camera are rigidly mounted to the Turtlebot2 chassis. The physical offset between them is resolved during extrinsic calibration (see [Calibration Workflow](#calibration-workflow)).

---

## Prerequisites

### Software

| Dependency | Tested Version | Notes |
|---|---|---|
| **ROS 2** | Humble Hawksbill | Full desktop install recommended |
| **ZED SDK** | ≥ 4.0 | Must match your Jetson's JetPack version |
| **zed-ros2-wrapper** | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) | Clone into `ros2_ws/src/` |
| **rslidar_sdk** | [RoboSense-LiDAR/rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk) | Build as a ROS 2 package |
| **MATLAB** | R2023a or later | Requires Lidar Toolbox + Computer Vision Toolbox |
| **Python 3** | 3.10+ | Ships with ROS 2 Humble |

### Python Packages

```
pyzed          # ZED Python API (installed with ZED SDK)
opencv-python  # cv2
cv_bridge      # ROS 2 ↔ OpenCV bridge
message_filters # Time-synchronized subscriptions
numpy
```

These are available through `apt` (ROS packages) or `pip`:

```bash
# ROS-packaged dependencies
sudo apt install ros-humble-cv-bridge ros-humble-message-filters

# pip dependencies (if not already installed)
pip install opencv-python numpy
```

### Hardware Checklist

Before first use, confirm:

- [ ] ZED camera is connected via USB 3.0 and recognized (`lsusb`)
- [ ] Robosense AIRY LiDAR is powered and Ethernet-connected to the Jetson
- [ ] LiDAR's IP/port settings in `rslidar_sdk` config match your network
- [ ] Turtlebot2 base is powered on (if teleoperation is needed)

---

## Repository Setup

### 1. Clone the repository

```bash
git clone https://github.com/Darrickg/jetson-local-mapping.git
cd jetson-local-mapping
```

### 2. Set up the ROS 2 workspace

The ROS 2 workspace lives at `ros2_ws/`. You need to clone the sensor driver packages into `ros2_ws/src/` before building:

```bash
cd ros2_ws/src

# Clone the ZED ROS 2 wrapper
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

# Clone the Robosense LiDAR SDK (ensure you build as ROS 2)
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule update --init --recursive
```

> **Important:** Follow the `rslidar_sdk` README to enable the ROS 2 build by editing its `CMakeLists.txt` (set `COMPILE_METHOD` to `COLCON`).

### 3. Build the workspace

```bash
cd ~/jetson-local-mapping/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Add the source command to your shell profile for convenience:

```bash
echo "source ~/jetson-local-mapping/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4. Copy the RViz configuration

The launch file expects the RViz config at `~/sensor_fusion.rviz`:

```bash
cp ~/jetson-local-mapping/sensor_fusion.rviz ~/sensor_fusion.rviz
```

---

## Calibration Workflow

Calibration must be completed before fused visualization will be correct. There are three stages: data collection, camera intrinsics, and LiDAR–camera extrinsics. For detailed MATLAB steps, see [`matlab-calibration/README.md`](matlab-calibration/README.md).

### Step 1 — Collect synchronized calibration pairs

Start both sensors and the capture tool:

```bash
# Option A: Use the convenience script
bash data_start.sh

# Option B: Start manually
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed &
source ros2_ws/install/setup.bash
ros2 launch rslidar_sdk start.py &
python3 capture_data.py
```

While `capture_data.py` is running:

- Press **Enter** to save a synchronized image + point cloud pair
- Press **q** then Enter to quit
- Pairs are saved to `calibration_data/images/` and `calibration_data/pointclouds/`

**Recommendations:**

- Capture **20–30 pairs** minimum
- Place the checkerboard at varying distances, angles, and positions across the shared sensor field of view
- Ensure the checkerboard is fully visible in both the camera image and point cloud
- Avoid motion blur — hold the board still for each capture

The node uses `ApproximateTimeSynchronizer` with a 10-second slop window to pair camera frames and LiDAR scans by timestamp.

### Step 2 — Camera intrinsic calibration

Two methods are available:

**Method A — MATLAB (recommended for this pipeline):**

```matlab
% In MATLAB, from the matlab-calibration/ directory:
run("setCameraIntrinsics.m")
```

This saves pre-measured ZED FHD intrinsics to `zed2_fhd_intrinsics.mat`. If your camera parameters differ (e.g., different ZED model or resolution), update `setCameraIntrinsics.m` first.

**Method B — OpenCV (Python alternative):**

```bash
python3 calibrate_camera.py
```

This uses a chessboard pattern (9×6 inner corners) detected in images from `calibration_data/images/`, and saves the result to `calibration_data/calibration_parameters.npz`.

### Step 3 — LiDAR–camera extrinsic calibration (MATLAB)

1. **Set an initial transform guess:**

   ```matlab
   run("setInitialTransform.m")
   ```

   Edit the `theta` (rotation in degrees) and `translation` (offset in meters) variables to reflect the approximate physical arrangement of the LiDAR relative to the camera.

2. **Run the MATLAB Lidar Camera Calibrator app** with the collected image/point-cloud pairs and the saved intrinsics. See [`matlab-calibration/README.md`](matlab-calibration/README.md) for the full step-by-step procedure.

3. **Export and convert the result:**

   ```matlab
   run("quaternionConversion.m")
   ```

   This prints the translation `[X Y Z]` and quaternion `[X Y Z W]` in ROS 2 format.

4. **Update the launch file** — Copy the printed values into `sensor_fusion.launch.py`:

   ```python
   # In sensor_fusion.launch.py, update these arguments:
   '--x', '<X>', '--y', '<Y>', '--z', '<Z>',
   '--qx', '<QX>', '--qy', '<QY>', '--qz', '<QZ>', '--qw', '<QW>',
   ```

---

## Running the Sensor Fusion Stack

Once calibration is complete, launch everything with a single command:

```bash
source ~/jetson-local-mapping/ros2_ws/install/setup.bash
ros2 launch sensor_fusion.launch.py
```

This starts four components:

| # | Component | What it does |
|---|---|---|
| 1 | Static TF publisher | Broadcasts the calibrated LiDAR → camera transform |
| 2 | ZED camera wrapper | Publishes RGB images and camera point cloud |
| 3 | Robosense LiDAR node | Publishes `/rslidar_points` |
| 4 | RViz2 | Opens the pre-configured visualization |

After launch, you should see the LiDAR point cloud and camera data aligned in RViz2.

### Running the undistort node (optional)

The `img_tools` package provides a ROS 2 node that undistorts and crops the raw camera feed in real time:

```bash
ros2 run img_tools undistort_image
```

This subscribes to the raw image topic, applies undistortion with the hardcoded camera matrix, auto-crops to the valid region, and republishes both the corrected image and a matching `CameraInfo` message.

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `/zed/zed_node/rgb/color/raw/image` | Raw image source |
| `output_image_topic` | `/zed/zed_node/rgb/color/image_rect_color` | Undistorted output |
| `output_camera_info_topic` | `/zed/zed_node/rgb/color/camera_info_rect` | Matching CameraInfo |
| `extra_crop_pixels` | `0` | Additional border trim (px) |

---

## Recording and Playing Back Datasets

### Recording

To record all relevant topics for offline processing:

```bash
ros2 bag record -o my_dataset \
  /rslidar_points \
  /zed/zed_node/rgb/color/rect/image \
  /zed/zed_node/rgb/camera_info \
  /zed/zed_node/point_cloud/cloud_registered \
  /tf_static
```

### Playback

```bash
ros2 bag play my_dataset
```

To replay while observing in RViz2, open RViz in a separate terminal:

```bash
rviz2 -d ~/sensor_fusion.rviz
```

---

## Visualization

### RViz2

The included `sensor_fusion.rviz` configuration displays:

- **LiDAR point cloud** (`/rslidar_points`) — coloured by intensity
- **ZED point cloud** (`/zed/zed_node/point_cloud/cloud_registered`) — coloured by RGB
- **ZED camera feed** (`/zed/zed_node/rgb/color/rect/image`) — as a camera overlay

The fixed frame is `zed_left_camera_frame_optical`.

Launch RViz2 standalone:

```bash
rviz2 -d ~/sensor_fusion.rviz
```

### rqt_image_view

To inspect the raw or rectified camera stream:

```bash
ros2 run rqt_image_view rqt_image_view
```

Subscribe to any image topic, for example:

- `/zed/zed_node/rgb/color/rect/image` (rectified)
- `/zed/zed_node/rgb/color/raw/image` (raw)

---

## Project Structure

```
jetson-local-mapping/
├── README.md                          # This file
├── sensor_fusion.launch.py            # Main ROS 2 launch: TF + ZED + LiDAR + RViz2
├── sensor_fusion.rviz                 # RViz2 visualization config
├── data_start.sh                      # Convenience script to start sensors + capture
├── capture_data.py                    # Time-synced image/pointcloud pair capture
├── calibrate_camera.py                # OpenCV chessboard intrinsic calibration
├── lidar_to_camera.py                 # LiDAR timestamp sync diagnostic tool
│
├── matlab-calibration/                # MATLAB extrinsic calibration pipeline
│   ├── README.md                      #   ↳ Full calibration instructions
│   ├── setCameraIntrinsics.m          #   ↳ ZED FHD intrinsics → .mat
│   ├── setInitialTransform.m          #   ↳ Initial 6-DOF guess
│   ├── quaternionConversion.m         #   ↳ Export result as ROS 2 quaternion
│   ├── graphErrors.m                  #   ↳ Visualize calibration errors
│   └── sample_calibration_data/       #   ↳ Example image/pointcloud pairs
│       ├── images/
│       └── pointclouds/
│
├── ros2_ws/                           # ROS 2 workspace
│   └── src/
│       ├── img_tools/                 #   ↳ Custom undistort/crop node
│       │   ├── README.md              #       ↳ Package documentation
│       │   └── img_tools/
│       │       └── undistort_image.py
│       └── zed-ros2-wrapper/          #   ↳ ZED camera driver (cloned)
│
└── sample_rosbag/                     # Pre-recorded dataset (~750 MB)
    ├── metadata.yaml
    └── sample_rosbag.db3
```

### File Reference

| File | Purpose |
|---|---|
| `sensor_fusion.launch.py` | Main launch file. Publishes a static TF (LiDAR → camera) using calibrated quaternion/translation, then starts ZED wrapper, Robosense LiDAR node, and RViz2. Edit the `--qx/qy/qz/qw` and `--x/y/z` arguments after re-calibrating. |
| `sensor_fusion.rviz` | RViz2 config. Displays the LiDAR intensity cloud, ZED RGB cloud, and camera overlay. Fixed frame: `zed_left_camera_frame_optical`. |
| `capture_data.py` | ROS 2 node. Subscribes to `/zed/zed_node/rgb/color/raw/image` and `/rslidar_points`, uses `ApproximateTimeSynchronizer` to pair frames, and saves matched PNG + PCD pairs on keypress. |
| `calibrate_camera.py` | OpenCV chessboard calibration. Detects a 9×6 corner pattern in `calibration_data/images/`, computes intrinsics, and saves to `calibration_data/calibration_parameters.npz`. |
| `lidar_to_camera.py` | Diagnostic node. Subscribes to `/rslidar_points` and prints the difference between LiDAR message timestamp and system clock. Use to verify time sync. |
| `data_start.sh` | Convenience script. Launches ZED + LiDAR in the background, then starts `capture_data.py`. |

---

## ROS 2 Topics Reference

| Topic | Message Type | Source | Description |
|---|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Robosense LiDAR | 3D point cloud with intensity |
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | ZED wrapper | Rectified RGB image |
| `/zed/zed_node/rgb/color/raw/image` | `sensor_msgs/Image` | ZED wrapper | Raw (distorted) RGB image |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | ZED wrapper | Camera intrinsic parameters |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | ZED wrapper | RGB-coloured stereo point cloud |
| `/tf_static` | `tf2_msgs/TFMessage` | Static TF publisher | LiDAR ↔ camera transform |

---

## Sample Data

A pre-recorded rosbag is included at `sample_rosbag/` (~750 MB, ~6.4 seconds of data):

- **148 messages** total: 84 camera images + 64 LiDAR scans
- Topics: `/zed/zed_node/rgb/color/rect/image`, `/rslidar_points`

To replay:

```bash
ros2 bag play sample_rosbag/
# In another terminal:
rviz2 -d ~/sensor_fusion.rviz
```

Sample calibration image/pointcloud pairs are also provided in `matlab-calibration/sample_calibration_data/` for testing the MATLAB calibration pipeline without access to the physical hardware.

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| No LiDAR data in RViz | LiDAR not publishing | Check `ros2 topic list` for `/rslidar_points`. Verify Ethernet connection and `rslidar_sdk` config (IP, port, LiDAR model). |
| No camera image | ZED not detected | Run `lsusb` to confirm USB connection. Ensure ZED SDK version matches JetPack. |
| Point clouds misaligned | Stale or bad calibration | Re-run the calibration workflow. Verify quaternion/translation values in `sensor_fusion.launch.py`. |
| `capture_data.py` says "No data yet" | Sensors not started | Ensure both `zed_wrapper` and `rslidar_sdk` are running before starting capture. |
| MATLAB calibrator fails to detect checkerboard | Poor data quality | Ensure board is fully visible, well-lit, sharp (no motion blur), and covers diverse poses. |
| `colcon build` fails | Missing dependencies | Install: `sudo apt install ros-humble-cv-bridge ros-humble-message-filters ros-humble-tf2-ros` |
| RViz config not found | Wrong path | Confirm `~/sensor_fusion.rviz` exists. Re-copy from the repo root if needed. |
| Large time-sync diff in `lidar_to_camera.py` | Clock mismatch | Ensure the Jetson system clock is synced (NTP). The LiDAR may use its own internal clock. |
