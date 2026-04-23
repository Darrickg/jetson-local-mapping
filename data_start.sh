#!/bin/bash
# Start both sensors and launch the calibration data capture tool.

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed &
source ros2_ws/install/setup.bash
ros2 launch rslidar_sdk start.py &
python capture_data.py
