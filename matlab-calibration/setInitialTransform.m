% 1. Define the rotation in degrees [X-axis, Y-axis, Z-axis]
% Think of this as how the lidar is rotated relative to the camera's forward view
theta = [0, 30, 0]; 

% 2. Define the translation in meters [X, Y, Z]
% How far offset is the lidar from the camera lens?
translation = [0.1084, 0.5225, 0]; 

% 3. Create the rigid transformation object
% Note: Use rigid3d(theta, translation) if you are on an older MATLAB version
initialGuess = rigidtform3d(theta, translation);