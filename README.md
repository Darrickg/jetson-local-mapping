Initial README for Jetson project

### Helpful Commands for running Zed Camera Wrapper and Rviz Interface

####Cd into ros2_ws

Source: ``source install/setup.bash``

#### Run the wrapper that publishes to topics
``ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed``

####Check topic list
``ros2 topic list``

#### opening rviz
``rviz2``

- Click Add in bottom left and subscribe to the appropriate topic
- This is the topic we currently have working in rviz2 (looking into if we need compression or not)
``/zed/zed_node/rgb/color/rect/image``

#### Check compressed Image feed:
``/zed/zed_node/rgb/color/rect/image``
``ros2 run rqt_image_view rqt_image_view``

#### Rosbag to save topic log/data
``ros2 bag record -o zed_rgb_compressed /zed/zed_node/rgb/color/rect/image/compressed``

#### May want to save Camera Info, could have with compressed image playback
``/zed/zed_node/rgb/camera_info``

#### Play the bag data
``ros2 bag play zed_rgb_compressed``


