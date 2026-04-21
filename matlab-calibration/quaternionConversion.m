% Assuming your final calibration object is named 'lidarCameraTform'
trans = lidarCameraTform.Translation; 
quat = rotm2quat(lidarCameraTform.Rotation); 

% Invert the rotation by taking the quaternion conjugate.
% A standard MATLAB quaternion is [W, X, Y, Z]. The conjugate is [W, -X, -Y, -Z].
quat_inv = [quat(1), -quat(2), -quat(3), -quat(4)];

% Display the translation values formatted for ROS 2 (Non-inverted)
fprintf('Translation (X Y Z): %f %f %f\n', trans(1), trans(2), trans(3));

% Display the inverted quaternion in ROS 2 format: [X, Y, Z, W]
fprintf('Quaternion (X Y Z W): %f %f %f %f\n', quat_inv(2), quat_inv(3), quat_inv(4), quat_inv(1));