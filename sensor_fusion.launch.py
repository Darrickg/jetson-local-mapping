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
            '--x', '0.058607', '--y', '-0.058351', '--z', '-0.081853',
            '--qx', '0.366536', '--qy', '-0.374354', '--qz', '0.601807', '--qw', '0.602776',
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
