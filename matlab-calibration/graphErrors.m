% Check if the data exists in the workspace
if ~exist('errors', 'var')
    error('errors not found in the workspace. Please load your saved .mat file first.');
end

% Get the number of poses (image/point-cloud pairs)
numPoses = size(errors.TranslationError, 1);
poseIndices = 1:numPoses;

% Create a new, large figure window
figure('Name', 'Lidar-Camera Calibration Errors', 'Position', [100, 100, 900, 700]);

% -----------------------------------------------------------
% Subplot 1: Translation Errors (X, Y, Z)
% -----------------------------------------------------------
subplot(2, 1, 1);
bar(poseIndices, errors.TranslationError);
title('Translation Errors per Pose', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Image/Pose Index', 'FontSize', 12);
ylabel('Translation Error', 'FontSize', 12); % Units match your checkerboard square size unit (e.g., mm)
legend('X Error', 'Y Error', 'Z Error', 'Location', 'northeast');
grid on;
set(gca, 'TickDir', 'out', 'Box', 'off');

% -----------------------------------------------------------
% Subplot 2: Rotation Errors (Roll, Pitch, Yaw)
% -----------------------------------------------------------
subplot(2, 1, 2);
bar(poseIndices, errors.RotationError);
title('Rotation Errors per Pose', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Image/Pose Index', 'FontSize', 12);
ylabel('Rotation Error (Degrees)', 'FontSize', 12);
legend('X (Roll)', 'Y (Pitch)', 'Z (Yaw)', 'Location', 'northeast');
grid on;
set(gca, 'TickDir', 'out', 'Box', 'off');

% Optional: Print a summary to the command window
fprintf('\n--- Error Summary ---\n');
fprintf('Max Translation Error: %.4f\n', max(abs(errors.TranslationError(:))));
fprintf('Max Rotation Error:    %.4f degrees\n\n', max(abs(errors.RotationError(:))));