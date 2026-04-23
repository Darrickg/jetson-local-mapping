#!/usr/bin/env python3
"""
calibrate_camera.py — OpenCV chessboard camera intrinsic calibration.

Detects a 6×9 inner-corner chessboard pattern in images from
calibration_data/images/, computes the camera matrix and distortion
coefficients, and saves everything to calibration_data/calibration_parameters.npz.

Usage:
    python3 calibrate_camera.py
"""

import numpy as np
import cv2 as cv
import glob
import os
import matplotlib.pyplot as plt


def calibrate(showPics=True):
  """Detect chessboard corners, run cv.calibrateCamera, and save results."""
  root = os.getcwd()
  calibrationDir = os.path.join(root, 'calibration_data/images')
  imgPathList = glob.glob(os.path.join(calibrationDir, '*.png'))

  # Chessboard inner-corner dimensions
  Rows = 9
  Cols = 6
  termination = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  # Prepare 3D world coordinates for the chessboard (z=0 plane)
  worldPtsCur = np.zeros((Rows*Cols, 3), np.float32)
  worldPtsCur[:, :2] = np.mgrid[0:Cols, 0:Rows].T.reshape(-1, 2)
  worldPtsList = []
  imgPtsList = []

  # Detect corners in each image
  for imgPath in imgPathList:
    imgBGR = cv.imread(imgPath)
    imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(imgGray, (Cols, Rows), None)

    if ret:
      worldPtsList.append(worldPtsCur)
      # Refine corner locations to sub-pixel accuracy
      cornersRefined = cv.cornerSubPix(imgGray, corners, (11, 11), (-1, -1), termination)
      imgPtsList.append(cornersRefined)

      if showPics:
        cv.drawChessboardCorners(imgBGR, (Cols, Rows), cornersRefined, ret)
        cv.imshow('Chessboard', imgBGR)
        cv.waitKey(500)
  cv.destroyAllWindows()

  # Run calibration to get intrinsic matrix and distortion coefficients
  repError, cameraMatrix, distCoeffs, rvecs, tvecs = cv.calibrateCamera(worldPtsList, imgPtsList, imgGray.shape[::-1], None, None)
  print("Camera Matrix:\n", cameraMatrix)
  print("Distortion Coefficients:\n", distCoeffs)
  print("Reprojection Error (pixels): {:.4f}".format(repError))

  # Save all calibration parameters to a single .npz file
  curFolder = os.path.dirname(os.path.abspath(__file__))
  paramPath = os.path.join(curFolder, 'calibration_data/calibration_parameters.npz')
  np.savez(paramPath,
           repError=repError,
           cameraMatrix=cameraMatrix,
           distCoeffs=distCoeffs,
           rvecs=rvecs,
           tvecs=tvecs)

  return cameraMatrix, distCoeffs


def removeDistortion(cameraMatrix, distCoeffs):
  """Undistort a sample image and display it (visual sanity check)."""
  root = os.getcwd()
  imgPath = os.path.join(root, 'calibration_data/images/img_048.png')
  img = cv.imread(imgPath)
  height, width = img.shape[:2]
  camMatrixNew, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (width, height), 1, (width, height))
  undistortedImg = cv.undistort(img, cameraMatrix, distCoeffs, None, camMatrixNew)

  plt.imshow(undistortedImg)
  plt.show()


def runCalibration():
  """Run calibration only, displaying detected corners."""
  calibrate(showPics=True)


def runRemoveDistortion():
  """Run calibration then show an undistorted sample image."""
  camMatrix, distCoeffs = calibrate(showPics=False)
  removeDistortion(camMatrix, distCoeffs)


if __name__ == "__main__":
  # runCalibration()
  runRemoveDistortion()