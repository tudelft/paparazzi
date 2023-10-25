# --------------------------------------------------------------------------
# Author:           Kevin Malkow and Sergio Marin Petersen
# Date:             05/09/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.0 
# 
# Description:  
# - Detect Aruco markers and estimate Aruco marker position from pre-recorded videos
# - Convert Aruco marker position in image plane coordinates to NED coordinates for navigation algorithm
#
#  -------------------------------------------------------------------------

                                        # LIBRARY DEFINITION #
# ------------------------------------------------------------------------------------------------------- #
# --------- General --------- # 
import time
import math
import csv
import sys
import cv2
import cv2.aruco
import threading
import numpy as np
from typing import Tuple

# --------- Ivybus Specific --------- # 
sys.path.append("/home/kevin/paparazzi/sw/ext/pprzlink/lib/v2.0/python/")

from ivy.std_api import *
import pprzlink.ivy
import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message          

                                      # LOAD CAMERA PARAMETERS #
# ------------------------------------------------------------------------------------------------------- #
pathLoad = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w1920_h1080_HERELINKV2.xml'
cv_file = cv2.FileStorage(pathLoad, cv2.FILE_STORAGE_READ)
camera_Matrix = cv_file.getNode("cM").mat()
distortion_Coeff = cv_file.getNode("dist").mat()
cv_file.release()

                                        # VARIABLE DEFINITION #
# ------------------------------------------------------------------------------------------------------- #
MARKER_SIZE = 1.107         # Size of Aruco marker in [m] -> 1.107 [m]||0.35 [m]

pitch_values = None         # Global variable to store Ivybus received pitch values
roll_values = None          # Global variable to store Ivybus received pitch values
yaw_values = None           # Global variable to store Ivybus received pitch values

X_m = []                    # Variable to save measured X value
Y_m = []                    # Variable to save measured Y value
Z_m = []                    # Variable to save measured Z value
NORTH_m = []                # Variable to save measured NORTH value
EAST_m  = []                # Variable to save measured EAST value
DOWN_m  = []                # Variable to save measured DOWN value
pitch_m = []                # Variable to save measured pitch value
roll_m  = []                # Variable to save measured roll value
yaw_m   = []                # Variable to save measured yaw value
time_m = []                 # Variable to save measured time

                                  # FUNCTIONS -> IVYBUS MESSAGES #
# ------------------------------------------------------------------------------------------------------- #
# --------- Bind to Drone Attitude Message --------- # 
def attitude_callback(ac_id, pprzMsg):
    global pitch_values
    global roll_values
    global yaw_values
    
    pitch = pprzMsg['theta']
    roll  = pprzMsg['phi']
    yaw   = pprzMsg['psi']
    pitch_values = pitch
    roll_values  = roll
    yaw_values   = yaw

# --------- Get Attitude Values --------- # 
def get_attitude_values():
    global pitch_values
    global roll_values
    global yaw_values
    return pitch_values, roll_values, yaw_values

                                  # FUNCTIONS -> NED COORDINATES CONVERSION #
# ------------------------------------------------------------------------------------------------------- #
def ned_conversion(pitch, roll, yaw, aruco_position):
    # --------- Rotation Matrix (Rotation around Z-Axis/Yaw) ------- # 
    RX= np.array([
                 [np.cos(yaw), -np.sin(yaw), 0],
                 [np.sin(yaw), np.cos(yaw),  0],
                 [          0,           0,  1]])

    # --------- Rotation Matrix (Rotation around Y-Axis/Pitch) ------- # 
    RY = np.array([
                  [ np.cos(pitch),  0, np.sin(pitch)],
                  [             0,  1,             0],
                  [-np.sin(pitch),  0, np.cos(pitch)]])

    # --------- Rotation Matrix (Rotation around X-Axis/Roll) ------- # 
    RZ = np.array([
                  [1,            0,             0],
                  [0, np.cos(roll), -np.sin(roll)],
                  [0, np.sin(roll),  np.cos(roll)]])

    # --------- Rotation Matrix ------- # 
    R = RZ @ RY @ RX 

    # --------- Obtain NED Coordinates ------- # 
    ned_vector = np.dot(R, aruco_position)
    north, east, down = ned_vector.squeeze()

    return north, east, down

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
  frame_legend = cv2.rectangle(frame_legend, (int(0.02*width), int(0.01*height)), (int(0.155*width), int(0.25*height)), (255, 255, 255), 2)

  # --------- Draw Reference System --------- # 
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.08*height)), (int(0.09*width), int(0.08*height)), (0, 0, 255), 3)                   # X = red
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.08*height)), (int(0.05*width), int(0.14*height)), (0, 255, 0), 3)                   # Y = green
  frame_legend = cv2.circle(frame_legend, (int(0.05*width), int(0.08*height)), 10, (255, 0, 0), 2)                                                 # Z = blue
  frame_legend = cv2.putText(frame_legend, "X", (int(0.0465*width), int(0.088*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "X", (int(0.068*width), int(0.074*height)), font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Y", (int(0.054*width), int(0.12*height)), font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Z", (int(0.035*width), int(0.07*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)

  return frame_legend

                                # FUNCTION -> VISUALISE DRONE ATTITUDE #
# ------------------------------------------------------------------------------------------------------- #
def visualiseDroneAttitude(frame_attitude, width, height, pitch_visual, roll_visual, yaw_visual):
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1.25
  color = (255, 255, 255)
  lineThickness = 2
  
  org_1 = (int(0.03*width), int(0.175*height))
  text_1 = f"Pitch: {round(pitch_visual, 2)}[deg.]"
  frame_attitude = cv2.putText(frame_attitude, text_1, org_1, font, fontScale, color, lineThickness, cv2.LINE_AA)

  org_2 = (int(0.03*width), int(0.2*height))
  text_2 = f"Roll: {round(roll_visual, 2)}[deg.]"
  frame_attitude = cv2.putText(frame_attitude, text_2, org_2, font, fontScale, color, lineThickness, cv2.LINE_AA)

  org_3 = (int(0.03*width), int(0.225*height))
  text_3 = f"Yaw: {round(yaw_visual, 2)}[deg.]"
  frame_attitude = cv2.putText(frame_attitude, text_3, org_3, font, fontScale, color, lineThickness, cv2.LINE_AA)

  return frame_attitude

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

                                      # Ivybus INITIALISATION #
# ------------------------------------------------------------------------------------------------------- #
# --------- Create Ivy Interface --------- # 
ivy = pprzlink.ivy.IvyMessagesInterface(agent_name="ArucoMarker", start_ivy=False, ivy_bus="127.255.255.255:2010")

# --------- Start Ivy Interface --------- # 
ivy.start()

# --------- Subscribe to Ivy Messages --------- # 
ivy.subscribe(attitude_callback, message.PprzMessage("telemetry", "NPS_RATE_ATTITUDE"))

                                              # VIDEO #
# ------------------------------------------------------------------------------------------------------- #
# --------- Load Video --------- #
path = '/home/kevin/IMAV2023/Live_Videos/VALKENBURG_20_07_23_TEST7_SHORTENED.mp4'               # Define video path	

cap = cv2.VideoCapture(path)                                                          # Create a VideoCapture object r
FPS = cap.get(cv2.CAP_PROP_FPS)                                                       # Read FPS from input video

# --------- Functioning? --------- #
if (cap.isOpened()== False):                                                          # Check if camera opened successfully
  print("Error: cannot open video file or stream")
 
# --------- Resolution --------- #
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# --------- Write Video Setup --------- #
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')                                                     # Define video codec (FOURCC code)
out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/Results/VALKENBURG_20_07_23_RESULT11.mp4', 
                      fourcc, FPS, (frame_width, frame_height))                                      # Create VideoWriter object 

                                    # ARUCO MARKER DETECTION SETUP #
# ------------------------------------------------------------------------------------------------------- #
# --------- Load Specific ID=700 Dictionary --------- # 
baseDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
arucoDictionary = cv2.aruco.Dictionary(baseDictionary.bytesList[700], 5, 6)

# --------- Set Detection Parameters --------- # 
arucoParameters =  cv2.aruco.DetectorParameters()

# STEP 1: Adaptive thresholding parameters
arucoParameters.adaptiveThreshWinSizeMin = 3
arucoParameters.adaptiveThreshWinSizeMax = 3
arucoParameters.adaptiveThreshWinSizeStep = 3
arucoParameters.adaptiveThreshConstant = 10

# STEP 2: Contour filtering parameters
arucoParameters.minMarkerPerimeterRate = 0.038 
arucoParameters.maxMarkerPerimeterRate = 0.5
arucoParameters.polygonalApproxAccuracyRate = 0.035

# # STEP 3: Bit extraction parameters
arucoParameters.perspectiveRemovePixelPerCell = 1 

# STEP 4: Corner refinement -> PERFORMANCE INTENSIVE (remove if necessary)
arucoParameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
arucoParameters.cornerRefinementWinSize = 6
arucoParameters.cornerRefinementMinAccuracy = 0.2

# --------- Build Aruco Marker Detector --------- # 
arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

# --------- Timer Start --------- # 
start_time = time.time()

# --------- Set Iteration Counter --------- # 
C_STEP = 0

                                            # RUN MAIN LOOP #
# ------------------------------------------------------------------------------------------------------- #
while(cap.isOpened()):
  # --------- Measure and Save Current Time --------- # 
  live_time = time.time()
  current_time = live_time - start_time
  time_m.append(current_time)

  # # --------- Get Attitude Values from Ivybus --------- # 
  pitch, roll, yaw = get_attitude_values()

  # --------- Read Frame-by-Frame --------- # 
  ret, frame = cap.read()

  if ret == True: # If frame read correctly          
    # --------- Convert to Grayscale --------- # 
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # --------- Aruco Marker Detection --------- # 
    (markerCorners, _, _) = arucoDetector.detectMarkers(gray_frame)
    
    # --------- Show Legend --------- # 
    frame = visualizeLegend(frame, frame_width, frame_height)

    # --------- Show Drone Attitude --------- # 
    if pitch is not None:
      pitch = float(pitch)
      roll  = float(roll)
      yaw   = float(yaw)

      pitch_m.append(pitch) # Save measured pitch
      roll_m.append(pitch)  # Save measured pitch
      yaw_m.append(pitch)   # Save measured pitch

      frame = visualiseDroneAttitude(frame, frame_width, frame_height, pitch, roll, yaw)

    if len(markerCorners) > 0: # At least one marker detected
      # --------- Update Iteration Counter --------- # 
      C_STEP = C_STEP + 1

      # --------- Aruco Marker Pose Estimation --------- # 
      rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camera_Matrix, distortion_Coeff)
      (rvec - tvec).any()    # Remove Numpy value array error

      # --------- Save and Print X, Y, and Z --------- # 
      print(f"-------- ITERATION: {C_STEP} --------") 
      X = tvec[0][0][0]
      X_m.append(X)          # Save measured X
      print(f"X: {X}")

      Y = tvec[0][0][1]
      Y_m.append(Y)          # Save measured Y
      print(f"Y: {Y}")

      Z = tvec[0][0][2]
      Z_m.append(Z)          # Save measured Z
      print(f"ALTITUDE: {Z}")

      # --------- NED Conversion --------- # 
      if pitch is not None:  
        pitch = float(pitch)
        roll  = float(roll)
        yaw   = float(yaw)

        pitch = math.radians(pitch)
        roll  = math.radians(roll)
        yaw   = math.radians(yaw)

        # --------- Convert Aruco marker Position in Image Coordinates to Body Coordinates --------- #
        # X (body) = Y (image plane), Y(body) = -X (image plane)
        X_B = Y
        Y_B = -X
        Z_B = Z

        aruco_position = np.array([[X_B], [Y_B], [Z_B]])

        # --------- Convert Aruco Position in Image Coordinates to NED Coordinates Relative to Drone --------- # 
        NORTH, EAST, DOWN = ned_conversion(pitch, roll, yaw, aruco_position)

        # --------- Save and Print NORTH, EAST, and DOWN --------- # 
        NORTH_m.append(NORTH)        # Save measured NORTH
        print(f"NORTH: {NORTH}")

        EAST_m.append(EAST)          # Save measured EAST
        print(f"EAST: {EAST}")

        DOWN_m.append(DOWN)          # Save measured DOWN
        print(f"DOWN: {DOWN}")
        print("-------------------------------") 

      # --------- Visualise Aruco Marker Position --------- # 
      frame = visualiseMarkerPosition(X, Y, Z, frame, frame_width, frame_height, rvec, tvec, camera_Matrix, distortion_Coeff)

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
# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_X', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_Y', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_Z', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_NORTH', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_EAST', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_DOWN', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_Pitch', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(pitch_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_Roll', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(roll_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/VALKENBURG_05_09_23_TEST1_Yaw', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(yaw_m, time_m))

# --------- Indoor Tests --------- # 
# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_X', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Y', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Z', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_NORTH', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_EAST', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_DOWN', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Pitch', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(pitch_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Roll', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(roll_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Yaw', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(yaw_m, time_m))

                                            # CLOSE CODE PROPERLY #
# ------------------------------------------------------------------------------------------------------- #
# --------- Release/Stop Objects --------- # 
cap.release()
out.release()
ivy.shutdown()

# --------- Close Frames --------- # 
cv2.destroyAllWindows()