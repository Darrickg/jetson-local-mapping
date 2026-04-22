# !/usr/bin/env python3

import numpy as np
import cv2 as cv
import glob
import os
import matplotlib.pyplot as plt

def calibrate(showPics = True):
  # Read Image
  root = os.getcwd()
  calibrationDir = os.path.join(root, 'camera_calibration/calibration_data/images')
  imgPathList = glob.glob(os.path.join(calibrationDir, '*.png'))

  # Initialize
  Rows = 9
  Cols = 6
  termination = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
  worldPtsCur = np.zeros((Rows*Cols, 3), np.float32)
  worldPtsCur[:, :2] = np.mgrid[0:Cols, 0:Rows].T.reshape(-1, 2)
  worldPtsList = []
  imgPtsList = []

  i = 1
  save_dir = os.path.join(root, 'camera_calibration/calibration_data/chessboard_images')
  os.makedirs(save_dir, exist_ok=True)
  # Find Corners
  for imgPath in imgPathList:
    imgBGR = cv.imread(imgPath)
    imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(imgGray, (Cols, Rows), None)

    if ret:
      worldPtsList.append(worldPtsCur)
      cornersRefined = cv.cornerSubPix(imgGray, corners, (11, 11), (-1, -1), termination)
      imgPtsList.append(cornersRefined)

      if showPics:
        save_path = os.path.join(save_dir, f'chessboard_{i}.png')
        cv.drawChessboardCorners(imgBGR, (Cols, Rows), cornersRefined, ret)
        cv.imshow('Chessboard', imgBGR)
        save_path = os.path.join(save_dir, f'chessboard_{i}.png')
        cv.imwrite(save_path, imgBGR)
        cv.waitKey(500)
        i += 1
  cv.destroyAllWindows()

  # Calibrate
  repError, cameraMatrix, distCoeffs, rvecs, tvecs = cv.calibrateCamera(worldPtsList, imgPtsList, imgGray.shape[::-1], None, None)
  print("Camera Matrix:\n", cameraMatrix)
  print("Distortion Coefficients:\n", distCoeffs)
  print("Reprojection Error (pixels): {:.4f}".format(repError))

  # save calibration parameters
  curFolder = os.path.dirname(os.path.abspath(__file__))
  paramPath = os.path.join(curFolder, 'calibration_parameters.npz')
  np.savez(paramPath,
           repError=repError,
           cameraMatrix=cameraMatrix,
           distCoeffs=distCoeffs,
           rvecs=rvecs,
           tvecs=tvecs)

  return cameraMatrix, distCoeffs

def removeDistortion(cameraMatrix, distCoeffs):
  root = os.getcwd()
  imgPath = os.path.join(root, 'calibration_data/images/img_048.png')
  img = cv.imread(imgPath)
  height, width = img.shape[:2]
  camMatrixNew, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (width, height), 1, (width, height))
  undistortedImg = cv.undistort(img, cameraMatrix, distCoeffs, None, camMatrixNew)

  plt.imshow(undistortedImg)
  plt.show()


def runCalibration():
  calibrate(showPics=True)

def runRemoveDistortion():
  camMatrix, distCoeffs = calibrate(showPics=False)
  removeDistortion(camMatrix, distCoeffs)

if __name__ == "__main__":
  runCalibration()
#   runRemoveDistortion()