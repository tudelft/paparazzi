# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             01/09/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          5.1
# 
# Description:  
# - Detect AND track Aruco marker from video live-stream
# - Input  -> Camera parameters, real-time video stream via Herelink
# - Output -> Relative (X, Y, Z,) position and Euclidean Distance of the Aruco marker to the camera (displayed per frame)
#
# Version Updates:
# - Code clean-up
# - Aruco detection performance improvement to avoid freezing of algorithm
# - Updated aruco marker detection visuals
#  -------------------------------------------------------------------------

                                        # LIBRARY DEFINITION #
# ------------------------------------------------------------------------------------------------------- #
import time
import math
import csv
import cv2
import cv2.aruco
import numpy as np

                                        # STREAM WORKING CHECK #
# # ------------------------------------------------------------------------------------------------------- #
# # cap = cv2.VideoCapture(2)
# # cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream") # Create a VideoCapture object (input is for herelink wifi connection)
# cap = cv2.VideoCapture("rtsp://192.168.42.129:8554/fpv_stream") # Create a VideoCapture object (input is for herelink bluetooth tethering)

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()
# 
#     # Display the resulting frame
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# 
# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

                                      # LOAD CAMERA PARAMETERS #
# ------------------------------------------------------------------------------------------------------- #
# UNCOMMENT FOR ALESSANDROS LAPTOP:
# pathLoad = './CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w1920_h1080_HERELINKV2.xml'

pathLoad = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w1920_h1080_HERELINKV2.xml'
# pathLoad = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w640_h480.xml'
cv_file = cv2.FileStorage(pathLoad, cv2.FILE_STORAGE_READ)
camera_Matrix = cv_file.getNode("cM").mat()
distortion_Coeff = cv_file.getNode("dist").mat()
cv_file.release()

                                        # VARIABLE DEFINITION #
# ------------------------------------------------------------------------------------------------------- #
MARKER_SIZE = 1.107         # Size of Aruco marker in [m] -> 1.107 [m]| | 0.35 [m]

X_m = []                    # Measured X value
Y_m = []                    # Measured Y value
Z_m = []                    # Measured Z value
time_m = []                 # Measured time

scaling_factor_X = -0.2976 # Scaling factor to account for reduced frame size in Aruco marker X measurements
scaling_factor_Y = -0.2468 # Scaling factor to account for reduced frame size in Aruco marker Y measurements
scaling_factor_Z = -0.092  # Scaling factor to account for reduced frame size in Aruco marker Z measurements

                                      # FUNCTION -> VISUALISE LEGEND #
# ------------------------------------------------------------------------------------------------------- #
def visualizeLegend(frame_legend, width, height):
  # --------- Show "ALTITUDE" --------- # 
  org = (int(0.03*width), int(0.04*height))
  text = f"ALTITUDE: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1.5
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)
  
  # --------- Draw Legend Outline --------- # 
  frame_legend = cv2.rectangle(frame_legend, (int(0.02*width), int(0.01*height)), (int(0.155*width), int(0.175*height)), (255, 255, 255), 2)

  # --------- Draw Reference System --------- # 
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.08*height)), (int(0.09*width), int(0.08*height)), (0, 0, 255), 3)                   # X = red
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.08*height)), (int(0.05*width), int(0.14*height)), (0, 255, 0), 3)                   # Y = green
  frame_legend = cv2.circle(frame_legend, (int(0.05*width), int(0.08*height)), 10, (255, 0, 0), 2)                                                 # Z = blue
  frame_legend = cv2.putText(frame_legend, "X", (int(0.0465*width), int(0.088*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "X", (int(0.068*width), int(0.074*height)), font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Y", (int(0.054*width), int(0.12*height)), font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Z", (int(0.035*width), int(0.07*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)

  return frame_legend

                                 # FUNCTION -> VISUALISE MARKER POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseMarkerPosition(X_visual, Y_visual, Z_visual, frame_pos, width, height, r, t, C, d):
  # --------- Create Projection from 3D to 2D --------- # 
  axes_3D = np.float32([[1, 0, 0], [0, -1, 0], [0, 0, -1], [0, 0, 0]]).reshape(-1, 3)    # Points in 3D space
  axisPoints, _ = cv2.projectPoints(axes_3D, r, t, C, d)                                 # Project 3D points into 2D image plane


  # --------- Create X-Marker Position Visualisation --------- # 
  X_start_Xline = width/2
  Y_start_Xline = axisPoints[3][0][1]
  X_end_Xline =  axisPoints[3][0][0]
  Y_end_Xline =  axisPoints[3][0][1]
  
  if X_visual >= 0:    # If postive X-value -> show green 
    cv2.line(frame_pos, (int(X_start_Xline), int(Y_start_Xline)), (int(X_end_Xline), int(Y_end_Xline)), (0, 255, 0), 3)
  else:                # Else -> show red 
    cv2.line(frame_pos, (int(X_start_Xline), int(Y_start_Xline)), (int(X_end_Xline), int(Y_end_Xline)), (0, 0, 255), 3)

  org_1 = (int(width/1.99), int(axisPoints[3][0][1]))   # Show X value on frame -> if positive show green, else show red
  text_1 = f"X: {round(X_visual, 1)}[m]"
  font_1 = cv2.FONT_HERSHEY_PLAIN
  fontScale_1 = 1.5
  lineThickness_1 = 2
  if X_visual >= 0:
    color_1 = (0, 255, 0)
  else:
    color_1 = (0, 0, 255)
  cv2.putText(frame_pos, text_1, org_1, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)

  # --------- Create Y-Marker Position Visualisation --------- # 
  X_start_Yline = width/2
  Y_start_Yline = height/2
  X_end_Yline =  width/2
  Y_end_Yline =  axisPoints[3][0][1]
  
  if Y_visual >= 0:    # If postive X-value -> show green 
    cv2.line(frame_pos, (int(X_start_Yline), int(Y_start_Yline)), (int(X_end_Yline), int(Y_end_Yline)), (0, 255, 0), 3)
  else:                # Else -> show red 
    cv2.line(frame_pos, (int(X_start_Yline), int(Y_start_Yline)), (int(X_end_Yline), int(Y_end_Yline)), (0, 0, 255), 3)

  org_2 = (int(width/2), int(height/2))    # Show Y value on frame -> if positive show green, else show red
  text_2 = f"Y: {round(Y_visual, 1)}[m]"
  font_2 = cv2.FONT_HERSHEY_PLAIN
  fontScale_2 = 1.5
  lineThickness_2 = 2
  if Y_visual >= 0:
    color_2 = (0, 255, 0)
  else:
    color_2 = (0, 0, 255)
  cv2.putText(frame_pos, text_2, org_2, font_2, fontScale_2, color_2, lineThickness_2, cv2.LINE_AA)

  # --------- Z (Altitude) Visualisation --------- # 
  org_3 = (int(0.03*width), int(0.04*height))
  text_3 = f"ALTITUDE: {round(Z_visual, 1)}[m]"
  font_3 = cv2.FONT_HERSHEY_PLAIN
  fontScale_3 = 1.5
  color_3 = (255, 255, 255)
  lineThickness_3 = 2
  cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, color_3, lineThickness_3, cv2.LINE_AA)

  # --------- Euclidean Distance Visualisation --------- # 
  X_start_Distline = width/2
  Y_start_Distline = height/2
  X_end_Distline =  axisPoints[3][0][0]
  Y_end_Distline = axisPoints[3][0][1]
  cv2.line(frame_pos, (int(X_start_Distline), int(Y_start_Distline)), (int(X_end_Distline), int(Y_end_Distline)), (255, 255, 255), 3)
  
  return frame_pos

                                              # VIDEO #
# ------------------------------------------------------------------------------------------------------- #
# --------- Load Video --------- #
# cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream") # Create a VideoCapture object (input is for herelink wifi connection)
cap = cv2.VideoCapture("rtsp://192.168.42.129:8554/fpv_stream") # Create a VideoCapture object (input is for herelink bluetooth tethering)
FPS = cap.get(cv2.CAP_PROP_FPS)                                 # Read FPS from input video

# --------- Functioning? --------- #
if (cap.isOpened()== False):                                                         # Check if camera opened successfully
  print("Error: cannot open video file or stream")
 
# --------- Resolution --------- #
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# --------- Write Video Setup --------- #
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')                                                     # Define video codec (FOURCC code)
out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/VALKENBURG_07_09_23_TEST3_V5_1.mp4', 
                      fourcc, FPS, (frame_width, frame_height))                                      # Create VideoWriter object 

# UNCOMMENT FOR ALESSANDROS LAPTOP
# out = cv2.VideoWriter('./Live_Videos/VALKENBURG_07_09_23_TEST3_V5_1.mp4', 
#                       fourcc, FPS, (frame_width, frame_height))                                      # Create VideoWriter object 


                                    # ARUCO MARKER DETECTION SETUP #
# ------------------------------------------------------------------------------------------------------- #
# --------- Load Specific ID=700 Dictionary --------- # 
baseDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
arucoDictionary = cv2.aruco.Dictionary(baseDictionary.bytesList[700], 5, 6)

# --------- Set Detection Parameters --------- # 
arucoParameters =  cv2.aruco.DetectorParameters()

# STEP 1: Adaptive thresholding parameters
arucoParameters.adaptiveThreshWinSizeMin  = 3
arucoParameters.adaptiveThreshWinSizeMax  = 12
arucoParameters.adaptiveThreshWinSizeStep = 3
arucoParameters.adaptiveThreshConstant    = 11

# STEP 2: Contour filtering parameters
arucoParameters.polygonalApproxAccuracyRate = 0.04
arucoParameters.minDistanceToBorder         = 10

# STEP 3: Bit extraction parameters (large influence on detection performance, default = 4)
arucoParameters.perspectiveRemovePixelPerCell = 1

# STEP 4: Corner refinement -> Improves accuracy of Aruco marker pose estimation
arucoParameters.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
arucoParameters.cornerRefinementWinSize       = 7
arucoParameters.cornerRefinementMinAccuracy   = 0.1

# --------- Build Aruco Marker Detector --------- # 
arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

# --------- Timer Start --------- # 
start_time = time.time()

                                            # RUN MAIN LOOP #
# ------------------------------------------------------------------------------------------------------- #
while(cap.isOpened()):
  # --------- Measure and Save Current Time --------- # 
  live_time = time.time()
  current_time = live_time - start_time
  time_m.append(current_time)

  # --------- Read Frame-by-Frame --------- # 
  ret, frame = cap.read()

  if ret == True:  # If frame read correctly          
    # --------- Resize Frame (Noise Reduction) --------- # 
    scale_percent = 60                               # Percent of original size -> At 60%, dim = (1152, 648), min scale_percent = 50%
    resized_frame_width = int(frame_width * scale_percent / 100)
    resized_frame_height = int(frame_height * scale_percent / 100)
    dim = (resized_frame_width, resized_frame_height)
  
    frame = cv2.resize(frame, dim)
    
    # --------- Convert to Grayscale --------- # 
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # --------- Aruco Marker Detection --------- # 
    (markerCorners, _, _) = arucoDetector.detectMarkers(gray_frame)
    
    # --------- Show Legend --------- # 
    frame = visualizeLegend(frame, resized_frame_width, resized_frame_height)

    if len(markerCorners) > 0:                                               # At least one marker detected       
      # --------- Aruco Marker Pose Estimation --------- # 
      rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camera_Matrix, distortion_Coeff)
      (rvec - tvec).any()                                                    # Remove Numpy value array error

      # --------- Save and Print X, Y, and Z --------- # 
      print(f"-------- ITERATION: {C_STEP} --------") 
      X = tvec[0][0][0]
      X = X*((scale_percent/100) + scaling_factor_X)
      X_m.append(X)          # Save measured X
      print(f"X: {X}")

      Y = tvec[0][0][1]
      Y = Y*((scale_percent/100) + scaling_factor_Y)
      Y_m.append(Y)          # Save measured Y
      print(f"Y: {Y}")

      Z = tvec[0][0][2]
      Z = Z*((scale_percent/100) + scaling_factor_Z)
      Z_m.append(Z)          # Save measured Z
      print(f"ALTITUDE: {Z}")
      print("-------------------------------") 

      # --------- Visualise Aruco Marker Position --------- # 
      frame = visualiseMarkerPosition(X, Y, Z, frame, resized_frame_width, resized_frame_height, rvec, tvec, camera_Matrix, distortion_Coeff)

    # --------- Write Video --------- # 
    out.write(frame)
    
    # --------- Display Output Frame --------- # 
    cv2.imshow('Frame', frame)

    # --------- Stop Code Execution (Press 'q') --------- # 
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
            
  # --------- Break While Loop (No Frame Retrieved) --------- # 
  else:
    print('Error: frame not retrieved')  
    break

                                            # SAVE MEASURED VARIABLES #
# ------------------------------------------------------------------------------------------------------- #
# --------- Outdoor Tests --------- # 
with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_07_09_23_TEST1_X_V5_1', 'w') as csvfile:
    writer=csv.writer(csvfile, delimiter=',')
    writer.writerows(zip(X_m, time_m))

with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_07_09_23_TEST1_Y_V5_1', 'w') as csvfile:
    writer=csv.writer(csvfile, delimiter=',')
    writer.writerows(zip(Y_m, time_m))

with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_07_09_23_TEST1_Z_V5_1', 'w') as csvfile:
    writer=csv.writer(csvfile, delimiter=',')
    writer.writerows(zip(Z_m, time_m))

# UNCOMMENT FOR ALESSANDROS LAPTOP:
# with open('./Measured_Variables/Outdoor_Tests/VALKENBURG_07_09_23_TEST1_X_V5_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/VALKENBURG_07_09_23_TEST1_Y_V5_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/VALKENBURG_07_09_23_TEST1_Z_V5_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# --------- Indoor Tests --------- # 
# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_X_V5_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Y_V5_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Z_V5_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

                                            # CLOSE CODE PROPERLY #
# ------------------------------------------------------------------------------------------------------- #
# --------- Release Objects --------- # 
cap.release()
out.release()

# --------- Close Frames --------- # 
cv2.destroyAllWindows()