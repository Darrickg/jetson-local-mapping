focalLength = [1401.65 1401.65];
principalPoint = [963.17 508.466];
imageSize = [1080 1920];

intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

save('zed2_fhd_intrinsics.mat', "intrinsics")