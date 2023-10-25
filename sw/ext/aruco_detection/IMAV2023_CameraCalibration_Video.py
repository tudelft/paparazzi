# ---------------------------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             05/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.0 
# 
# Description:  
# Do a camera calibration and retrieve camera matrix and distortion coefficients on video files
#  ---------------------------------------------------------------------------------------------
import cv2
import numpy as np

# #  ------------------------------------------------------------------------- #
# #                          CAMERA WORKING CHECK                              #
# #  ------------------------------------------------------------------------- #
# cap = cv2.VideoCapture(2)

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#     # Our operations on the frame come here
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Display the resulting frame
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

#  ------------------------------------------------------------------------- #
#             TERMINATION CRITERIA FOR CALIBRATION FUNCTION                  #
#  ------------------------------------------------------------------------- #
maxIter = 30
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
square_size = 0.0225 # [m]

# FLAG: Minimum no. of data points reached
MIN_POINTS = 600

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

#  ------------------------------------------------------------------------- #
#              LOAD VIDEO, DEFINE VIDEO CAPTURE, AND WRITE OBJECTS           #
#  ------------------------------------------------------------------------- #
#  Define video path
# path = '/home/kevin/IMAV2023/Camera_Calibration/Videos/2021_0101_002954_005.MP4'  

# Create a VideoCapture object and read from camera (input is either an rtsp stream from the herelink module or input 2)
# cap = cv2.VideoCapture(2)
cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream")
FPS = cap.get(cv2.CAP_PROP_FPS)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error: cannot open video file or stream")
 
# Default resolutions of the frame are obtained and converted from float to integer
frame_width = int(cap.get(3)*0.6)
frame_height = int(cap.get(4)*0.6)

print(frame_width)
print(frame_height)

# Define video codec (FOURCC code)
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')

# Create VideoWriter object 
out = cv2.VideoWriter('/home/kevin/IMAV2023/Camera_Calibration/Results/Videos/ChessBoard_Detected_V9.mp4', fourcc, FPS, (frame_width, frame_height))

# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()

  # If frame found 
  if ret == True:
    # --------- Resize Frame (Noise Reduction) --------- # 
    scale_percent = 60                               # Percent of original size -> At 60%, dim = (1152, 648), min scale_percent = 50%
    resized_frame_width = int(frame_width * scale_percent / 100)
    resized_frame_height = int(frame_height * scale_percent / 100)
    dim = (resized_frame_width, resized_frame_height)
  
    frame = cv2.resize(frame, dim)  

    # Convert to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray_frame, (widthCB, heightCB), None)
  
    # If corners found, add object points, image points (after refining them)
    if ret == True:
      objpoints.append(objp)
    
      corners2 = cv2.cornerSubPix(gray_frame, corners, (11,11), (-1,-1), termCriteria)
      imgpoints.append(corners2)
      
      # Draw and display the corners
      frame = cv2.drawChessboardCorners(frame, (widthCB, heightCB), corners2, ret)

    #  ------------------------------------------------------------------------- #
    #                         DISPLAY AND SAVE IMAGES                            #
    #  ------------------------------------------------------------------------- #
    # Write the frame into the file
    out.write(frame)
    
    # Display the resulting frame
    cv2.imshow('Frame', frame) 

    # Wait 1 [ms] between each frame until it ends or press 'q' on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

    # Break if minimum no. of data points reached
    if len(imgpoints) > MIN_POINTS:
      print("Sufficient data points retrieved...")
      break   
          
  # Break the loop
  else:    
    print('Error: frame not retrieved')  
    break

#  ------------------------------------------------------------------------- #
#                           CAMERA CALIBRATION                               #
#  ------------------------------------------------------------------------- #
# Calibrate camera -> [..., camera matrix, distortion coefficients, rotation vectors, translation vectors]
print("Starting camera calibration...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_frame.shape[::-1], None, None)
print("Finished camera calibration...")
print("-----------------------------")

print(f"Camera Matrix: {mtx}")
print(f"Dist Coeff: {dist}")

#  ------------------------------------------------------------------------- #
#                             SAVE VARIABLES                                 #
#  ------------------------------------------------------------------------- #
pathStore = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_60_Percent_HERELINK.xml'   
cv_file = cv2.FileStorage(pathStore, cv2.FILE_STORAGE_WRITE)
cv_file.write("cM", mtx)
cv_file.write("dist", dist)

cv_file.release()

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

# Release the video capture and video write objects
cap.release()
out.release()
            
# Close all the frames
cv2.destroyAllWindows()

