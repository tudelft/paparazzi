# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             05/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.0 
# 
# Description:  
# Do a camera calibration and retrieve camera matrix and distortion coefficients
#  -------------------------------------------------------------------------

import glob
import cv2
import numpy as np


#  ------------------------------------------------------------------------- #
#                               FUNCTIONS                                    #
#  ------------------------------------------------------------------------- #
def img_rescale(img, scale_percent):
# Image downscaling -> USED AS IMAGE INPUT AND AFFECTS PERFORMANCE OF DETECTOR
  scale_percent = 30 # Percent of original size
  width = int(img.shape[1] * scale_percent / 100)
  height = int(img.shape[0] * scale_percent / 100)
  dim = (width, height)
  return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

#  ------------------------------------------------------------------------- #
#             TERMINATION CRITERIA FOR CALIBRATION FUNCTION                  #
#  ------------------------------------------------------------------------- #
maxIter = 100
epsilon = 0.001
termCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, maxIter, epsilon)

#  ------------------------------------------------------------------------- #
#                           CONSTANTS DEFINITION                             #
#  ------------------------------------------------------------------------- #
# Number of intersection points of squares on long side of calibration board
widthCB = 9

# Number of intersection points of squares on short side of calibration board
heightCB = 6

# Size of square on chessboard
square_size = 0.022 # [m]

# # Initialise counter for formatting saved images -> UNCOMMENT TO SAVE IMAGES
# counter = 1

#  ------------------------------------------------------------------------- #
#                           CHESSBOARD CORNERS                               #
#  ------------------------------------------------------------------------- #
# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((heightCB * widthCB, 3), np.float32)
objp[:, :2] = np.mgrid[0:widthCB, 0:heightCB].T.reshape(-1,2)

# Get actual positions of objects
objp = objp * square_size
 
# Arrays to store object points and image points from all the images
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane

path = '/home/kevin/IMAV2023/Camera_Calibration/*.JPG'   
images = glob.glob(path)
 
for file in images:
  img = cv2.imread(file)
  img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  
  # Find the chess board corners
  ret, corners = cv2.findChessboardCorners(img_gray, (widthCB, heightCB), None)
  
  # If found, add object points, image points (after refining them)
  if ret == True:
    objpoints.append(objp)
  
    corners2 = cv2.cornerSubPix(img_gray, corners, (11,11), (-1,-1), termCriteria)
    imgpoints.append(corners2)
  
    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (widthCB, heightCB), corners2, ret)
    
    #  ------------------------------------------------------------------------- #
    #                         DRAW AND SAVE IMAGES                               #
    #  ------------------------------------------------------------------------- #
    # # UNCOMMENT TO VIEW IMAGES
    # img = img_rescale(img, 30)
    # cv2.imshow('img', img)
    # cv2.waitKey(0)
  
    # # --------------------------------------------------------
    # # UNCOMMENT TO SAVE IMAGES
    # # Saving the image
    # cv2.imwrite('/home/kevin/IMAV2023/Camera_Calibration/Results/Pictures/ChessBoard_Detected_{}.JPG'.format(counter), img)
    # # --------------------------------------------------------
    
    # # UNCOMMENT TO VIEW IMAGES
    # cv2.destroyAllWindows()

  # # Increment counter -> UNCOMMENT TO SAVE IMAGES
  # counter += 1

#  ------------------------------------------------------------------------- #
#                           CAMERA CALIBRATION                               #
#  ------------------------------------------------------------------------- #
# Calibrate camera -> [..., camera matrix, distortion coefficients, rotation vectors, translation vectors]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)
print(f"Camera Matrix: {mtx}")
print(f"Dist Coeff: {dist}")

#  ------------------------------------------------------------------------- #
#                             SAVE VARIABLES                                 #
#  ------------------------------------------------------------------------- #
# pathStore = '/home/kevin/IMAV2023/CameraCalibration_Variables/Images/cameraCalibration.xml'   
# cv_file = cv2.FileStorage(pathStore, cv2.FILE_STORAGE_WRITE)
# cv_file.write("cM", mtx)
# cv_file.write("dist", dist)

# cv_file.release()

#  ------------------------------------------------------------------------- #
#                      REPROJECTION ERROR COMPUTATION                        #
#  ------------------------------------------------------------------------- #
# Compute error using L2 Norm -> Error should be as close as possible to 0
totError = 0
for i in range(len(objpoints)):
  imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
  error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
  totError += error
 
print(f"Total Error: {totError/len(objpoints)}") # 0.083 [m] error
