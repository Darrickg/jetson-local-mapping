# MATLAB LiDAR-Camera Calibration

This folder contains a MATLAB-first workflow for calibrating a ZED camera to a LiDAR using the Lidar Camera Calibrator app, then exporting the transform in a format usable by ROS 2.

## Folder Contents

- `sample_calibration_data/images/`: Sample camera images used for calibration
- `sample_calibration_data/pointclouds/`: Sample LiDAR point clouds paired with the images
- `setCameraIntrinsics.m`: Creates and saves camera intrinsics to `zed2_fhd_intrinsics.mat`
- `setInitialTransform.m`: Creates an initial transform guess as `initialGuess`
- `quaternionConversion.m`: Converts final calibration to ROS-friendly translation/quaternion output
- `graphErrors.m`: Visualizes calibration translation/rotation errors from exported `errors`

## Prerequisites

- MATLAB with:
  - Lidar Toolbox
  - Computer Vision Toolbox
- A paired dataset of images and point clouds (same scene/pose index alignment)
- Checkerboard dimensions and physical square size used during data capture

## 1. Prepare Camera Intrinsics

From MATLAB, run:

```matlab
run("setCameraIntrinsics.m")
```

This script creates a `cameraIntrinsics` object named `intrinsics` and saves it to `zed2_fhd_intrinsics.mat`.

If you have different camera parameters, update `setCameraIntrinsics.m` first.

## 2. Set an Initial Transform Guess

Run:

```matlab
run("setInitialTransform.m")
```

This creates `initialGuess` as a `rigidtform3d` object.

- `theta`: Initial rotation guess in degrees `[Rx Ry Rz]`
- `translation`: Initial offset `[X Y Z]` (currently configured in meters)

A reasonable initial guess usually improves convergence and checkerboard pose consistency.

## 3. Open the Lidar Camera Calibrator App

Launch the app either from MATLAB Apps or command line:

```matlab
lidarCameraCalibrator
```

## 4. Load Data and Camera Intrinsics

In the app:

1. Load the image set from `sample_calibration_data/images/`.
2. Load the point cloud set from `sample_calibration_data/pointclouds/`.
3. Load camera intrinsics from `zed2_fhd_intrinsics.mat` (the `intrinsics` variable).
4. Set checkerboard settings (rows, columns, and square size) to match your physical board.

Notes:
- Keep image and point cloud ordering aligned by pose.
- Square-size units control the scale of translation/error outputs. Keep units consistent with your workflow.

## 5. Select and Detect Checkerboards

For each image/point cloud pair in the app:

1. Select the checkerboard region in the image.
2. Select the checkerboard region/plane in the point cloud.
3. Run checkerboard detection.
4. Verify corners are correctly detected before accepting the pose.

Recommendations:
- Use diverse poses (distance, angle, and position variation).
- Remove frames with partial board visibility, blur, heavy occlusion, or poor point density.
- Re-run detection on failed pairs instead of forcing bad detections.

## 6. Apply Initial Transform and Calibrate

Before final solve:

1. Provide `initialGuess` from the MATLAB workspace as the app's initial transform.
2. Run calibration/optimization.
3. Inspect residuals and outliers; remove problematic pairs and re-run if needed.

## 7. Export Calibration for Scripts

When calibration quality is acceptable, export results to the MATLAB workspace.

For compatibility with this folder's scripts, ensure the exported variables include:

- `lidarCameraTform`: final rigid transform object
- `errors`: calibration error structure

If MATLAB exports a different transform variable name, rename it in the workspace:

```matlab
lidarCameraTform = <yourExportedTransformVariable>;
```

## 8. Convert to ROS-Oriented Quaternion Output

Run:

```matlab
run("quaternionConversion.m")
```

This script:
- Reads `lidarCameraTform.Translation`
- Converts rotation matrix to quaternion
- Applies quaternion conjugate for inverse rotation
- Prints quaternion in ROS order `[X Y Z W]`

## 9. Plot Calibration Errors (Optional)

Run:

```matlab
run("graphErrors.m")
```

This visualizes per-pose translation/rotation errors and prints max absolute error values.

## Troubleshooting

- `errors not found in the workspace`:
  - Export `errors` from the app before running `graphErrors.m`.
- `Undefined function or variable 'lidarCameraTform'`:
  - Export the final transform and assign it to `lidarCameraTform`.
- Poor calibration quality:
  - Improve checkerboard visibility, increase pose diversity, reject bad pairs, and refine `initialGuess`.
