"""
sensor_fusion.launch.py — Main ROS 2 launch file for the sensor fusion stack.

Starts the static TF publisher (calibrated LiDAR→camera transform), the ZED
camera wrapper, the Robosense LiDAR node, and RViz2 with a pre-configured view.

Usage:
    ros2 launch sensor_fusion.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. The Static Transform (Using your exact MATLAB calibration)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_to_zed_tf',
        arguments=[
            '--x', '0.050004', '--y', '-0.204872', '--z', '-0.065184',
            '--qx', '0.330842', '--qy', '-0.317747', '--qz', '0.634132', '--qw', '0.622461',
            '--frame-id', 'zed_left_camera_frame_optical',
            '--child-frame-id', 'rslidar',
        ]
    )


    # 2. ZED Camera Node
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch'),
            '/zed_camera.launch.py']),
        launch_arguments={'camera_model': 'zed'}.items(), # Change 'zed' if using zed2 or zed2i
    )

    # 3. Robosense Lidar Node
    # Note: Using ExecuteProcess here as start.py is sometimes run directly in the rslidar_sdk
    # rslidar_launch = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'],
    #     output='screen'
    # )
    rslidar_launch = Node(
	package='rslidar_sdk',
	executable='rslidar_sdk_node',
        output='screen'
    )

    # 4. RViz2 (Loading your saved configuration)
    # Ensure this path matches where you saved the .rviz file in Step 1
    rviz_config_dir = os.path.expanduser('~/sensor_fusion.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # Return the launch description to execute all nodes simultaneously
    return LaunchDescription([
        static_tf_node,
        zed_launch,
        rslidar_launch,
        rviz_node
    ])
