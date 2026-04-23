# MATLAB LiDAR–Camera Calibration

Step-by-step workflow for calibrating a ZED camera to a Robosense LiDAR using the MATLAB **Lidar Camera Calibrator** app, then exporting the transform for use in ROS 2.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Folder Contents](#folder-contents)
- [Quick-Start Summary](#quick-start-summary)
- [Detailed Procedure](#detailed-procedure)
  - [1. Prepare Camera Intrinsics](#1-prepare-camera-intrinsics)
  - [2. Set an Initial Transform Guess](#2-set-an-initial-transform-guess)
  - [3. Open the Lidar Camera Calibrator](#3-open-the-lidar-camera-calibrator)
  - [4. Load Data and Intrinsics](#4-load-data-and-intrinsics)
  - [5. Detect Checkerboards](#5-detect-checkerboards)
  - [6. Calibrate](#6-calibrate)
  - [7. Export Results](#7-export-results)
  - [8. Convert to ROS 2 Format](#8-convert-to-ros-2-format)
  - [9. Visualize Errors (Optional)](#9-visualize-errors-optional)
- [Applying Results to the Launch File](#applying-results-to-the-launch-file)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

| Requirement | Details |
|---|---|
| MATLAB | R2023a or later |
| Lidar Toolbox | Required for the Lidar Camera Calibrator app |
| Computer Vision Toolbox | Required for `cameraIntrinsics` and `rotm2quat` |
| Calibration data | Paired images and point clouds (same scene, same index) |
| Checkerboard | Known dimensions (row count, column count, square size) |

---

## Folder Contents

```
matlab-calibration/
├── README.md                      # This file
├── setCameraIntrinsics.m          # Creates cameraIntrinsics → saves to .mat
├── setInitialTransform.m          # Defines initial 6-DOF transform guess
├── quaternionConversion.m         # Converts final result to ROS 2 quaternion format
├── graphErrors.m                  # Plots per-pose translation/rotation errors
└── sample_calibration_data/       # Example paired data for testing
    ├── images/                    #   img_001.png … img_NNN.png
    └── pointclouds/               #   pc_001.pcd … pc_NNN.pcd
```

---

## Quick-Start Summary

If you are already familiar with the MATLAB Lidar Camera Calibrator, here is the minimal workflow:

```matlab
% 1. Save camera intrinsics
run("setCameraIntrinsics.m")

% 2. Create initial transform guess
run("setInitialTransform.m")

% 3. Open the calibrator app, load data + intrinsics, detect, calibrate

% 4. Export 'lidarCameraTform' and 'errors' to workspace

% 5. Convert and print ROS 2 quaternion
run("quaternionConversion.m")

% 6. (Optional) Plot errors
run("graphErrors.m")
```

Then copy the printed translation/quaternion values into `sensor_fusion.launch.py`.

---

## Detailed Procedure

### 1. Prepare Camera Intrinsics

Run from the MATLAB command window (working directory: `matlab-calibration/`):

```matlab
run("setCameraIntrinsics.m")
```

**What it does:**

- Creates a `cameraIntrinsics` object using the ZED FHD parameters:
  - Focal length: `[1392.92, 1393.02]` px
  - Principal point: `[970.64, 520.52]` px
  - Image size: `1080 × 1920`
  - Radial distortion: `[-0.1731, 0.1884, -0.5804]`
  - Tangential distortion: `[-0.00107, -0.000479]`
- Saves the result to `zed2_fhd_intrinsics.mat`

**If your camera differs:** Edit `setCameraIntrinsics.m` with your own focal length, principal point, image size, and distortion coefficients before running.

### 2. Set an Initial Transform Guess

```matlab
run("setInitialTransform.m")
```

**What it does:**

- Defines `theta = [Rx Ry Rz]` in degrees (rotation of LiDAR relative to camera)
- Defines `translation = [X Y Z]` in meters (physical offset)
- Creates a `rigidtform3d` object named `initialGuess`

**Tips for a good initial guess:**

- Measure the physical offset between the LiDAR and camera with a ruler
- Estimate rotation angles from the mounting geometry
- A reasonable guess improves convergence and prevents the optimizer from settling in a local minimum

### 3. Open the Lidar Camera Calibrator

```matlab
lidarCameraCalibrator
```

Or launch from the MATLAB **Apps** tab → **Lidar Camera Calibrator**.

### 4. Load Data and Intrinsics

In the calibrator app:

1. **Load images** from `sample_calibration_data/images/` (or your own `calibration_data/images/`)
2. **Load point clouds** from `sample_calibration_data/pointclouds/` (or your own `calibration_data/pointclouds/`)
3. **Load camera intrinsics** from `zed2_fhd_intrinsics.mat` — select the `intrinsics` variable
4. **Set checkerboard parameters**: enter the number of rows, columns, and the physical square size

> **Important:** Image and point cloud files must be ordered so that index `N` of each set corresponds to the same physical scene and pose. The capture script (`capture_calibration.py`) ensures this automatically.

### 5. Detect Checkerboards

For each image/point-cloud pair:

1. Select the checkerboard region in the **image** view
2. Select the checkerboard plane in the **point cloud** view
3. Run detection and verify that corners are correctly identified

**Best practices:**

- Use diverse poses: vary distance (0.5–3 m), angle, and lateral position
- Remove frames where the board is partially occluded, blurred, or has poor point-cloud density
- Re-run detection on failed pairs rather than forcing incorrect detections
- Aim for ≥15 accepted pairs for a stable calibration

### 6. Calibrate

1. In the app, provide `initialGuess` from the MATLAB workspace as the initial transform
2. Click **Calibrate**
3. Inspect residual errors — look for outlier poses with disproportionately large errors
4. Remove problematic pairs and re-run if necessary

### 7. Export Results

When calibration quality is acceptable, export results to the MATLAB workspace. Ensure the exported variables include:

- `lidarCameraTform` — the final `rigidtform3d` object
- `errors` — the calibration error structure

If the app exports a different variable name, rename it:

```matlab
lidarCameraTform = <yourExportedTransformVariable>;
```

### 8. Convert to ROS 2 Format

```matlab
run("quaternionConversion.m")
```

**What it does:**

1. Reads `lidarCameraTform.Translation` → `[X, Y, Z]`
2. Converts the rotation matrix to a quaternion via `rotm2quat`
3. Applies quaternion conjugate (inverts the rotation for the TF convention)
4. Prints both values in ROS 2 format:

```
Translation (X Y Z): 0.050004 -0.204872 -0.065184
Quaternion (X Y Z W): 0.330842 -0.317747 0.634132 0.622461
```

### 9. Visualize Errors (Optional)

```matlab
run("graphErrors.m")
```

Generates a two-panel bar chart:

- **Top:** Per-pose translation error (X, Y, Z)
- **Bottom:** Per-pose rotation error (Roll, Pitch, Yaw in degrees)

Also prints the maximum absolute errors to the command window.

> **Prerequisite:** The `errors` variable must exist in the workspace (exported from the calibrator app in Step 7).

---

## Applying Results to the Launch File

After running `quaternionConversion.m`, copy the printed values into `sensor_fusion.launch.py`:

```python
static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='rslidar_to_zed_tf',
    arguments=[
        '--x', '<X>', '--y', '<Y>', '--z', '<Z>',
        '--qx', '<QX>', '--qy', '<QY>', '--qz', '<QZ>', '--qw', '<QW>',
        '--frame-id', 'zed_left_camera_frame_optical',
        '--child-frame-id', 'rslidar',
    ]
)
```

Replace the placeholder values with the exact numbers from the MATLAB output.

---

## Troubleshooting

| Problem | Cause | Solution |
|---|---|---|
| `errors not found in the workspace` | `errors` was not exported from the calibrator app | Export both `lidarCameraTform` and `errors` before running `graphErrors.m` |
| `Undefined function or variable 'lidarCameraTform'` | Transform was not exported or was saved with a different name | Export the final transform and assign it to `lidarCameraTform` |
| Poor calibration quality (large residuals) | Insufficient or low-quality data | Increase pose diversity, ensure checkerboard is fully visible, reject blurred/occluded frames, and refine `initialGuess` |
| Checkerboard not detected in point cloud | Too few LiDAR points on the board | Move the board closer to the LiDAR, or use a larger checkerboard |
| `rigidtform3d` not recognized | Older MATLAB version | Use `rigid3d(theta, translation)` instead |