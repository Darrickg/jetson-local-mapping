focalLength = [1392.92185 1393.02317];
principalPoint = [970.64089 520.523935];
imageSize = [1080 1920];
radialDistortion = [-0.173077983, 0.188432316, -0.580367151];
tangentialDistortion = [-0.00107256632, -0.000478540709];

intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize, RadialDistortion=radialDistortion, TangentialDistortion = tangentialDistortion);

save('zed2_fhd_intrinsics.mat', "intrinsics")