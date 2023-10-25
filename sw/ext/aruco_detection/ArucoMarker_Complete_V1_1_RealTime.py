# --------------------------------------------------------------------------
# Author:           Kevin Malkow and Sergio Marin Petersen
# Date:             08/09/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.1 
# 
# Description:  
# - Detect Aruco markers and estimate Aruco marker position from real-time video stream
# - Get attitude and NED position from drone through Ivybus
# - Convert Aruco marker position in image plane coordinates to NED coordinates for navigation algorithm
#   using drone attitude
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
# UNCOMMENT FOR ALESSANDROS LAPTOP:
# sys.path.append("/home/ppz/paparazzi/sw/ext/pprzlink/lib/v2.0/python/")    

sys.path.append("/home/kevin/paparazzi/sw/ext/pprzlink/lib/v2.0/python/")

from ivy.std_api import *
import pprzlink.ivy
import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message          

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
MARKER_SIZE = 1.107                   # Size of Aruco marker in [m] -> 1.107 [m]||0.35 [m]

PITCH_values = None                   # Global variable to store Ivybus received pitch values
ROLL_values = None                    # Global variable to store Ivybus received roll values
YAW_values = None                     # Global variable to store Ivybus received yaw values
pprz_attitude_conversion = 0.0139882  # Unit conversion from pprz message to degrees

NORTH_values = None                   # Global variable to store Ivybus received NORTH values
EAST_values = None                    # Global variable to store Ivybus received EAST values
DOWN_values = None                    # Global variable to store Ivybus received DOWN values
pprz_NED_conversion = 0.0039063       # Unit conversion from pprz message to meters

scaling_factor_X = -0.2976            # Scaling factor to account for reduced frame size in Aruco marker X measurements
scaling_factor_Y = -0.2468            # Scaling factor to account for reduced frame size in Aruco marker Y measurements
scaling_factor_Z = -0.092             # Scaling factor to account for reduced frame size in Aruco marker Z measurements

X_ARUCO_m = []                        # Variable to save measured X value
Y_ARUCO_m = []                        # Variable to save measured Y value
Z_ARUCO_m = []                        # Variable to save measured Z value
NORTH_ARUCO_m = []                    # Variable to save measured Aruco marker NORTH value
EAST_ARUCO_m  = []                    # Variable to save measured Aruco marker EAST value
DOWN_ARUCO_m  = []                    # Variable to save measured Aruco marker DOWN value
NORTH_DRONE_m = []                    # Variable to save measured drone NORTH value
EAST_DRONE_m  = []                    # Variable to save measured drone EAST value
DOWN_DRONE_m  = []                    # Variable to save measured drone DOWN value
PITCH_DRONE_m = []                    # Variable to save measured pitch value
ROLL_DRONE_m  = []                    # Variable to save measured roll value
YAW_DRONE_m   = []                    # Variable to save measured yaw value
time_m = []                           # Variable to save measured time

                                  # FUNCTIONS -> IVYBUS MESSAGES #
# ------------------------------------------------------------------------------------------------------- #
# --------- Bind to Drone Attitude Message --------- # 
def attitude_callback(ac_id, pprzMsg):
    global PITCH_values
    global ROLL_values
    global YAW_values
    
    pitch = pprzMsg['theta']
    roll  = pprzMsg['phi']
    yaw   = pprzMsg['psi']
    PITCH_values = pitch
    ROLL_values  = roll
    YAW_values   = yaw

# --------- Bind to Drone NED Position Message --------- # 
def NED_callback(ac_id, pprzMsg):
    global NORTH_values
    global EAST_values
    global DOWN_values
    
    NORTH_Drone  = pprzMsg['north']
    EAST_Drone   = pprzMsg['east']
    DOWN_Drone   = pprzMsg['up']
    NORTH_values = NORTH_Drone
    EAST_values  = EAST_Drone
    DOWN_values  = DOWN_Drone

# --------- Get Attitude Values --------- # 
def get_attitude_values():
    global PITCH_values
    global ROLL_values
    global YAW_values
    return PITCH_values, ROLL_values, YAW_values

# --------- Get NED Position Values --------- # 
def get_NED_values():
    global NORTH_values
    global EAST_values
    global DOWN_values
    return NORTH_values, EAST_values, DOWN_values

                                  # FUNCTIONS -> NED COORDINATES CONVERSION #
# ------------------------------------------------------------------------------------------------------- #
def NED_conversion(pitch, roll, yaw, Aruco_position):
    # --------- Rotation Matrix (Rotation around Z-Axis/Yaw) ------- # 
    RZ= np.array([
                 [np.cos(yaw), -np.sin(yaw), 0],
                 [np.sin(yaw), np.cos(yaw),  0],
                 [          0,           0,  1]])

    # --------- Rotation Matrix (Rotation around Y-Axis/Pitch) ------- # 
    RY = np.array([
                  [ np.cos(pitch),  0, np.sin(pitch)],
                  [             0,  1,             0],
                  [-np.sin(pitch),  0, np.cos(pitch)]])

    # --------- Rotation Matrix (Rotation around X-Axis/Roll) ------- # 
    RX = np.array([
                  [1,            0,             0],
                  [0, np.cos(roll), -np.sin(roll)],
                  [0, np.sin(roll),  np.cos(roll)]])

    # --------- Rotation Matrix ------- # 
    R = RX @ RY @ RZ 

    # --------- Obtain NED Coordinates ------- # 
    NED_vector = np.dot(R, Aruco_position)
    NORTH, EAST, DOWN = NED_vector.squeeze()

    return NORTH, EAST, DOWN


                                # FUNCTION -> VISUALISE LEGEND #
# ------------------------------------------------------------------------------------------------------- #
def visualizeLegend(frame_legend, width, height):
  # --------- Show "ARUCO" --------- # 
  org = (int(0.02*width), int(0.165*height))
  text = "ARUCO"
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1.25
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco Z --------- # 
  org = (int(0.03*width), int(0.205*height))
  text = f"Z: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco DOWN --------- # 
  org = (int(0.03*width), int(0.23*height))
  text = f"DOWN: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

    # --------- Show Aruco ALTITUDE --------- # 
  org = (int(0.03*width), int(0.255*height))
  text = f"ALTITUDE: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show "DRONE" --------- # 
  org = (int(0.02*width), int(0.42*height))
  text = "DRONE"
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1.25
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)   
  
  # --------- Show Drone NORTH --------- # 
  org = (int(0.03*width), int(0.57*height))
  text = f"NORTH: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)
  
  # --------- Show Drone EAST --------- # 
  org = (int(0.03*width), int(0.605*height))
  text = f"EAST: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Drone DOWN --------- # 
  org = (int(0.03*width), int(0.64*height))
  text = f"DOWN: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Drone Latitude --------- # 
  org = (int(0.03*width), int(0.675*height))
  text = f"Latitude: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)
  
  # --------- Show Drone Longitude --------- # 
  org = (int(0.03*width), int(0.71*height))
  text = f"Longitude: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Drone DOWN --------- # 
  org = (int(0.03*width), int(0.745*height))
  text = f"Altitude: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Draw Aruco Legend Outline --------- # 
  frame_legend = cv2.rectangle(frame_legend, (int(0.02*width), int(0.175*height)), (int(0.165*width), int(0.38*height)), (255, 255, 255), 2)

  # --------- Draw Drone Legend Outline --------- # 
  frame_legend = cv2.rectangle(frame_legend, (int(0.02*width), int(0.43*height)), (int(0.18*width), int(0.77*height)), (255, 255, 255), 2)

  # --------- Draw Reference System --------- # 
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.3*height)), (int(0.09*width), int(0.3*height)), (0, 0, 255), 3)                   # X = red
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.3*height)), (int(0.05*width), int(0.37*height)), (0, 255, 0), 3)                   # Y = green
  frame_legend = cv2.circle(frame_legend, (int(0.05*width), int(0.3*height)), 10, (255, 0, 0), 2)                                                 # Z = blue
  frame_legend = cv2.putText(frame_legend, "X", (int(0.046*width), int(0.309*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "X", (int(0.075*width), int(0.29*height)), font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Y", (int(0.033*width), int(0.36*height)), font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Z", (int(0.03*width), int(0.285*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)

  return frame_legend

                          # FUNCTION -> VISUALISE X, Y, Z ARUCO MARKER POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoXYZMarkerPosition(X_visual, Y_visual, Z_visual, frame_pos, width, height, r, t, C, d):
  # --------- Create Projection from 3D to 2D --------- # 
  axes_3D = np.float32([[1, 0, 0], [0, -1, 0], [0, 0, -1], [0, 0, 0]]).reshape(-1, 3)    # Points in 3D space
  axisPoints, _ = cv2.projectPoints(axes_3D, r, t, C, d)                                 # Project 3D points into 2D image plane

  # --------- Create X-Marker Position Visualisation --------- # 
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

  # --------- Aruco Z Visualisation --------- # 
  org_3 = (int(0.1*width), int(0.205*height))
  text_3 = f" {round(Z_visual, 1)}[m]"
  font_3 = cv2.FONT_HERSHEY_PLAIN
  fontScale_3 = 1
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

                                # FUNCTION -> VISUALISE NED MARKER POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoNEDMarkerPosition(NORTH_visual, EAST_visual, DOWN_visual, frame_pos, width, height, r, t, C, d):
    # --------- Create Projection from 3D to 2D --------- # 
  axes_3D = np.float32([[1, 0, 0], [0, -1, 0], [0, 0, -1], [0, 0, 0]]).reshape(-1, 3)    # Points in 3D space
  axisPoints, _ = cv2.projectPoints(axes_3D, r, t, C, d)                                 # Project 3D points into 2D image plane

  # --------- Create NORTH Marker Position Visualisation --------- # 
  X_start_Xline = width/2
  Y_start_Xline = axisPoints[3][0][1]
  X_end_Xline =  axisPoints[3][0][0]
  Y_end_Xline =  axisPoints[3][0][1]
  
  if NORTH_visual >= 0:  # If postive NORTH value -> show green 
    cv2.line(frame_pos, (int(X_start_Xline), int(Y_start_Xline)), (int(X_end_Xline), int(Y_end_Xline)), (0, 255, 0), 3)
  else:                  # Else -> show red 
    cv2.line(frame_pos, (int(X_start_Xline), int(Y_start_Xline)), (int(X_end_Xline), int(Y_end_Xline)), (0, 0, 255), 3)

  org_1 = (int(width/1.99), int(axisPoints[3][0][1]/0.9))   # Show X value on frame -> if positive show green, else show red
  text_1 = f"NORTH: {round(NORTH_visual, 1)}[m]"
  font_1 = cv2.FONT_HERSHEY_PLAIN
  fontScale_1 = 1.5
  lineThickness_1 = 2
  if NORTH_visual >= 0:
    color_1 = (0, 255, 0)
  else:
    color_1 = (0, 0, 255)
  cv2.putText(frame_pos, text_1, org_1, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)

  # --------- Create EAST Marker Position Visualisation --------- # 
  X_start_Yline = width/2
  Y_start_Yline = height/2
  X_end_Yline =  width/2
  Y_end_Yline =  axisPoints[3][0][1]
  
  if EAST_visual >= 0: # If postive X-value -> show green 
    cv2.line(frame_pos, (int(X_start_Yline), int(Y_start_Yline)), (int(X_end_Yline), int(Y_end_Yline)), (0, 255, 0), 3)
  else:                # Else -> show red 
    cv2.line(frame_pos, (int(X_start_Yline), int(Y_start_Yline)), (int(X_end_Yline), int(Y_end_Yline)), (0, 0, 255), 3)

  org_2 = (int(width/2), int(height/1.88))    # Show Y value on frame -> if positive show green, else show red
  text_2 = f"EAST: {round(EAST_visual, 1)}[m]"
  font_2 = cv2.FONT_HERSHEY_PLAIN
  fontScale_2 = 1.5
  lineThickness_2 = 2
  if EAST_visual >= 0:
    color_2 = (0, 255, 0)
  else:
    color_2 = (0, 0, 255)
  cv2.putText(frame_pos, text_2, org_2, font_2, fontScale_2, color_2, lineThickness_2, cv2.LINE_AA)
  
  # --------- Aruco DOWN Visualisation --------- # 
  org_3 = (int(0.1*width), int(0.235*height))
  text_3 = f" {round(DOWN_visual, 1)}[m]"
  font_3 = cv2.FONT_HERSHEY_PLAIN
  fontScale_3 = 1
  color_3 = (255, 255, 255)
  lineThickness_3 = 2
  cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, color_3, lineThickness_3, cv2.LINE_AA)

  return frame_pos

                                # FUNCTION -> VISUALISE DRONE ATTITUDE #
# ------------------------------------------------------------------------------------------------------- #
def visualiseDroneAttitude(frame_attitude, width, height, pitch_visual, roll_visual, yaw_visual):
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  
  org_1 = (int(0.03*width), int(0.465*height))
  text_1 = f"Pitch: "
  frame_attitude = cv2.putText(frame_attitude, text_1, org_1, font, fontScale, color, lineThickness, cv2.LINE_AA)

  org_2 = (int(0.03*width), int(0.5*height))
  text_2 = f"Roll: "
  frame_attitude = cv2.putText(frame_attitude, text_2, org_2, font, fontScale, color, lineThickness, cv2.LINE_AA)

  org_3 = (int(0.03*width), int(0.535*height))
  text_3 = f"Yaw: "
  frame_attitude = cv2.putText(frame_attitude, text_3, org_3, font, fontScale, color, lineThickness, cv2.LINE_AA)

  if pitch_visual >= 0:   # If postive pitch value -> show green 
    org_1 = (int(0.07*width), int(0.465*height))
    text_1 = f" {round(pitch_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_1, org_1, font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_1 = (int(0.07*width), int(0.465*height))
    text_1 = f" {round(pitch_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_1, org_1, font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)

  if roll_visual >= 0:    # If postive roll value -> show green 
    org_2 = (int(0.07*width), int(0.5*height))
    text_2 = f" {round(roll_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_2, org_2, font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_2 = (int(0.07*width), int(0.5*height))
    text_2 = f" {round(roll_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_2, org_2, font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)
  
  if yaw_visual >= 0:    # If postive yaw value -> show green 
    org_3 = (int(0.07*width), int(0.535*height))
    text_3 = f" {round(yaw_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_3, org_3, font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.07*width), int(0.535*height))
    text_3 = f" {round(yaw_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_3, org_3, font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)

  return frame_attitude

                                 # FUNCTION -> VISUALISE NED DRONE POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseDroneNEDPosition(NORTH_visual, EAST_visual, DOWN_visual, frame_pos, width, height):
  # --------- Drone NORTH Visualisation --------- # 
  if DOWN_visual >= 0:    # If postive DOWN value -> show green 
    org_3 = (int(0.1*width), int(0.57*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.1*width), int(0.57*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone EAST Visualisation --------- # 
  if DOWN_visual >= 0:    # If postive DOWN value -> show green 
    org_3 = (int(0.1*width), int(0.605*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.1*width), int(0.605*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone DOWN Visualisation --------- # 
  org_3 = (int(0.1*width), int(0.64*height))
  text_3 = f" {round(DOWN_visual, 2)}[m]"
  font_3 = cv2.FONT_HERSHEY_PLAIN
  fontScale_3 = 1
  lineThickness_3 = 2
  cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (255, 255, 255), lineThickness_3, cv2.LINE_AA)
 
  return frame_pos

                                      # Ivybus INITIALISATION #
# ------------------------------------------------------------------------------------------------------- #
# --------- Create Ivy Interface --------- # 
ivy = pprzlink.ivy.IvyMessagesInterface(agent_name="ArucoMarker", start_ivy=False, ivy_bus="127.255.255.255:2010")

# --------- Start Ivy Interface --------- # 
ivy.start()

# --------- Subscribe to Ivy Messages --------- # 
ivy.subscribe(attitude_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))
ivy.subscribe(NED_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))

                                              # VIDEO #
# ------------------------------------------------------------------------------------------------------- #
# --------- Load Video --------- #
# cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream") # Create a VideoCapture object (input is for herelink wifi connection)
# cap = cv2.VideoCapture("rtsp://192.168.42.129:8554/fpv_stream") # Create a VideoCapture object (input is for herelink bluetooth tethering)
path = '/home/kevin/IMAV2023/Live_Videos/VALKENBURG_20_07_23_TEST7_SHORTENED.mp4'        # Define video path	
# path = '/home/kevin/IMAV2023/Aruco_Marker_Data/06_07_2023/Videos/2023_0706_001.MP4'    # Define video path	

cap = cv2.VideoCapture(path)                                                             # Create a VideoCapture object
FPS = cap.get(cv2.CAP_PROP_FPS)                                                          # Read FPS from input video

# --------- Functioning? --------- #
if (cap.isOpened()== False):                                                             # Check if camera opened successfully
  print("Error: cannot open video file or stream")
 
# --------- Resolution --------- #
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# --------- Write Video Setup --------- #
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')                                                     # Define video codec (FOURCC code)
out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/IMAV_09_09_23_TEST1_CompleteV1_1.mp4', 
                      fourcc, FPS, (frame_width, frame_height))                                      # Create VideoWriter object 

# UNCOMMENT FOR ALESSANDROS LAPTOP:
# out = cv2.VideoWriter('./Live_Videos/IMAV_09_09_23_TEST1_CompleteV1_1.mp4', 
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

# --------- Set Iteration Counter --------- # 
C_STEP = 0

                                            # RUN MAIN LOOP #
# ------------------------------------------------------------------------------------------------------- #
while(cap.isOpened()): 
  # --------- Measure and Save Current Time --------- # 
  live_time = time.time()
  current_time = live_time - start_time
  time_m.append(current_time)

  # --------- Get Attitude Values from Ivybus --------- # 
  PITCH_DRONE, ROLL_DRONE, YAW_DRONE = get_attitude_values()
  
  # --------- Get NED Values from Ivybus --------- # 
  NORTH_DRONE, EAST_DRONE, DOWN_DRONE = get_NED_values()

  # --------- Read Frame-by-Frame --------- # 
  ret, frame = cap.read()

  if ret == True: # If frame read correctly          
    # --------- Resize Frame (Noise Reduction) --------- # 
    scale_percent = 60 # Percent of original size -> At 60%, dim = (1152, 648), min scale_percent = 50%
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

    # --------- Show Drone Attitude --------- # 
    if PITCH_DRONE is not None:
      PITCH_DRONE = float(PITCH_DRONE)
      ROLL_DRONE  = float(ROLL_DRONE)
      YAW_DRONE   = float(YAW_DRONE)
      
      PITCH_DRONE = PITCH_DRONE*pprz_attitude_conversion
      ROLL_DRONE  = ROLL_DRONE*pprz_attitude_conversion
      YAW_DRONE   = YAW_DRONE*pprz_attitude_conversion 

      PITCH_DRONE_m.append(PITCH_DRONE) # Save measured pitch
      ROLL_DRONE_m.append(ROLL_DRONE)   # Save measured roll
      YAW_DRONE_m.append(YAW_DRONE)     # Save measured yaw

      frame = visualiseDroneAttitude(frame, resized_frame_width, resized_frame_height, PITCH_DRONE, ROLL_DRONE, YAW_DRONE)

    # --------- Save and Print Drone NORTH, EAST, and DOWN --------- # 
    if NORTH_DRONE is not None:
      NORTH_DRONE = float(NORTH_DRONE)
      EAST_DRONE  = float(EAST_DRONE)
      DOWN_DRONE  = float(DOWN_DRONE)

      NORTH_DRONE = NORTH_DRONE*pprz_NED_conversion
      EAST_DRONE  = EAST_DRONE*pprz_NED_conversion
      DOWN_DRONE  = -DOWN_DRONE*pprz_NED_conversion # Drone sends UP value, so negate axis

      NORTH_DRONE_m.append(NORTH_DRONE) # Save measured drone NORTH
      print(f"Drone NORTH: {NORTH_DRONE}")
      
      EAST_DRONE_m.append(EAST_DRONE)   # Save measured drone EAST
      print(f"Drone EAST: {EAST_DRONE}")      
      
      DOWN_DRONE_m.append(DOWN_DRONE)   # Save measured drone DOWN
      print(f"Drone DOWN: {DOWN_DRONE}")
      print("-------------------------------") 
      
      # --------- Visualise NED Drone Position --------- # 
      frame = visualiseDroneNEDPosition(NORTH_DRONE, EAST_DRONE, -DOWN_DRONE, frame, resized_frame_width, resized_frame_height)

    if len(markerCorners) > 0: # At least one marker detected
      # --------- Update Iteration Counter --------- # 
      C_STEP = C_STEP + 1

      # --------- Aruco Marker Pose Estimation --------- # 
      rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camera_Matrix, distortion_Coeff)
      (rvec - tvec).any()    # Remove Numpy value array error

      # --------- Save and Print X, Y, and Z --------- # 
      print(f"-------- ITERATION: {C_STEP} --------") 
      X_ARUCO = tvec[0][0][0]
      X_ARUCO = X_ARUCO*((scale_percent/100) + scaling_factor_X)
      X_ARUCO_m.append(X_ARUCO)          # Save measured X
      print(f"Aruco X: {X_ARUCO}")

      Y_ARUCO = tvec[0][0][1]
      Y_ARUCO = Y_ARUCO*((scale_percent/100) + scaling_factor_Y)
      Y_ARUCO_m.append(Y_ARUCO)          # Save measured Y
      print(f"Aruco Y: {Y_ARUCO}")

      Z_ARUCO = tvec[0][0][2]
      Z_ARUCO = Z_ARUCO*((scale_percent/100) + scaling_factor_Z)
      Z_ARUCO_m.append(Z_ARUCO)          # Save measured Z
      print(f"Aruco Z: {Z_ARUCO}")

      # --------- Visualise X, Y, Z Aruco Marker Position --------- # 
      frame = visualiseArucoXYZMarkerPosition(X_ARUCO, Y_ARUCO, Z_ARUCO, frame, resized_frame_width, resized_frame_height, rvec, tvec, camera_Matrix, distortion_Coeff)

      # --------- NED Conversion and Moving to Relative Position --------- # 
      if PITCH_DRONE is not None: 
        PITCH_DRONE = float(PITCH_DRONE)
        ROLL_DRONE  = float(ROLL_DRONE)
        YAW_DRONE   = float(YAW_DRONE)
        
        PITCH_DRONE = PITCH_DRONE*pprz_attitude_conversion
        ROLL_DRONE  = ROLL_DRONE*pprz_attitude_conversion
        YAW_DRONE   = YAW_DRONE*pprz_attitude_conversion 

        PITCH_DRONE = math.radians(PITCH_DRONE)
        ROLL_DRONE  = math.radians(ROLL_DRONE)
        YAW_DRONE   = math.radians(YAW_DRONE)

        # --------- Convert Aruco marker Position in Image Coordinates to Body Coordinates --------- #
        # X (body) = Y (image plane), Y(body) = -X (image plane)
        X_ARUCO_B = Y_ARUCO
        Y_ARUCO_B = -X_ARUCO
        Z_ARUCO_B = Z_ARUCO

        ARUCO_POSITION_B = np.array([[X_ARUCO_B], [Y_ARUCO_B], [Z_ARUCO_B]])

        # --------- Convert Aruco Position in Image Coordinates to NED Coordinates Relative to Drone --------- # 
        NORTH_REL, EAST_REL, DOWN_REL = NED_conversion(PITCH_DRONE, ROLL_DRONE, YAW_DRONE, ARUCO_POSITION_B)

        # --------- NED Aruco Marker Position --------- # 
        if NORTH_DRONE is not None:
          # --------- NED Relative and NED Drone Summation --------- # 
          NORTH_ARUCO = NORTH_REL + NORTH_DRONE
          EAST_ARUCO = EAST_REL + EAST_DRONE
          DOWN_ARUCO = DOWN_REL + DOWN_DRONE

          # --------- Save and Print Aruco Marker NORTH, EAST, and DOWN --------- # 
          NORTH_ARUCO_m.append(NORTH_ARUCO)        # Save measured Aruco Marker NORTH
          print(f"Aruco NORTH: {NORTH_ARUCO}")

          EAST_ARUCO_m.append(EAST_ARUCO)          # Save measured Aruco Marker EAST
          print(f"Aruco EAST: {EAST_ARUCO}")

          DOWN_ARUCO_m.append(DOWN_ARUCO)          # Save measured Aruco Marker DOWN
          print(f"Aruco DOWN: {DOWN_ARUCO}")

          # --------- Visualise NED Aruco Marker Position --------- # 
          frame = visualiseArucoNEDMarkerPosition(NORTH_ARUCO, EAST_ARUCO, DOWN_ARUCO, frame, resized_frame_width, resized_frame_height, rvec, tvec, camera_Matrix, distortion_Coeff)

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
# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoX_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoY_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoZ_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoNORTH_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoEAST_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoDOWN_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DronePitch_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(PITCH_DRONE_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneRoll_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ROLL_DRONE_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneYaw_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(YAW_DRONE_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneNORTH_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_DRONE_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneEAST_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_DRONE_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneDOWN_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_DRONE_m, time_m))

# UNCOMMENT FOR ALESSANDROS LAPTOP:
# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoX_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoY_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoZ_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoNORTH_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoEAST_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoDOWN_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DronePitch_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(PITCH_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneRoll_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ROLL_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneYaw_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(YAW_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneNORTH_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneEAST_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneDOWN_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_DRONE_m, time_m))

# --------- Indoor Tests --------- # 
# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_X_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Y_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Z_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_NORTH_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_Aruco_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_EAST_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_Aruco_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_DOWN_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(DOWN_Aruco_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Pitch_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(pitch_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Roll_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(roll_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Yaw_V1', 'w') as csvfile:
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