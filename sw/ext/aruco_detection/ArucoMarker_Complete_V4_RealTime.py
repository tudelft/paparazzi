# --------------------------------------------------------------------------
# Author:           Kevin Malkow, Sergio Marin Petersen, and Christophe de Wagter
# Date:             02/10/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          4.0 (Tested) - Onboard Processing
#  
# 
# Description:  
# - Detect Aruco markers and estimate Aruco marker position from real-time video stream
# - Get attitude, NED position, reference latitude, reference longitude, and reference altitude from drone through Ivybus
# - Convert Aruco marker position in image plane coordinates to NED coordinates for navigation algorithm
#   using drone attitude
# - Applying Kalman filter to NED values to get smooth waypoint moving (very special thanks to Christophe de Wagter)
# - Convert Aruco marker position to latitude, longitude, and altitude
# - Moving waypoint to Aruco marker position in latitude, longitude, and altitude 
#
# Updates: 
# - Bug fixes to avoid crashing 
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
import signal
import pymap3d
import cv2.aruco
import threading
import numpy as np
from KalmanFilter_V1 import init, predict, update, route

# # --------- Ivybus Specific --------- # 
# sys.path.append("/home/kevin/paparazzi/sw/ext/pprzlink/lib/v2.0/python/")

# from ivy.std_api import *
# import pprzlink.ivy
# import pprzlink.messages_xml_map as messages_xml_map
# import pprzlink.message as message          

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
# pathLoad = './CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w1152_h648_HERELINKV2.xml'
pathLoad = './CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w640_h480.xml'
cv_file = cv2.FileStorage(pathLoad, cv2.FILE_STORAGE_READ)
camera_Matrix = cv_file.getNode("cM").mat()
distortion_Coeff = cv_file.getNode("dist").mat()
cv_file.release()

                                        # VARIABLE DEFINITION #
# ------------------------------------------------------------------------------------------------------- #
MARKER_SIZE              = 1.107        # Size of Aruco marker in [m] -> 1.107 [m]||0.35 [m]

PITCH_values             = None       # Global variable to store Ivybus received pitch values
ROLL_values              = None       # Global variable to store Ivybus received roll values
YAW_values               = None       # Global variable to store Ivybus received yaw values
pprz_attitude_conversion = 0.0139882  # Unit conversion from pprz message to degrees

NORTH_values             = None       # Global variable to store Ivybus received NORTH values
EAST_values              = None       # Global variable to store Ivybus received EAST values
UP_values                = None       # Global variable to store Ivybus received UP values
pprz_NED_conversion      = 0.0039063  # Unit conversion from pprz message to meters

REF_LAT_values           = None       # Global variable to store Ivybus received LATITUDE values
REF_LONG_values          = None       # Global variable to store Ivybus received LONGITUDE values
REF_ALT_values           = None       # Global variable to store Ivybus received ALTITUDE values
pprz_lat_long_conversion = 0.0000001  # Unit conversion from pprz message to lat, long 
pprz_alt_conversion      = 0.001      # Unit conversion from pprz message to alt 

X_ARUCO_m     = []                    # Variable to save measured X value
Y_ARUCO_m     = []                    # Variable to save measured Y value
Z_ARUCO_m     = []                    # Variable to save measured Z value
NORTH_ARUCO_m = []                    # Variable to save measured Aruco marker NORTH value
EAST_ARUCO_m  = []                    # Variable to save measured Aruco marker EAST value
UP_ARUCO_m    = []                    # Variable to save measured Aruco marker UP value
LAT_ARUCO_m   = []                    # Variable to save measured Aruco marker LAT value
LONG_ARUCO_m  = []                    # Variable to save measured Aruco marker LONG value
ALT_ARUCO_m   = []                    # Variable to save measured Aruco marker ALT value
NORTH_DRONE_m = []                    # Variable to save measured drone NORTH value
EAST_DRONE_m  = []                    # Variable to save measured drone EAST value
UP_DRONE_m    = []                    # Variable to save measured drone UP value
PITCH_DRONE_m = []                    # Variable to save measured drone pitch value
ROLL_DRONE_m  = []                    # Variable to save measured droneroll value
YAW_DRONE_m   = []                    # Variable to save measured drone yaw value
LAT_0_m       = []                    # Variable to save measured drone LAT value
LONG_0_m      = []                    # Variable to save measured drone LONG value
ALT_0_m       = []                    # Variable to save measured drone ALT value
time_m        = []                    # Variable to save measured time
FILT_N_PRED_m = []                    # Variable to save predicted Kalman filter NORTH value
FILT_E_PRED_m = []                    # Variable to save predicted Kalman filter EAST value
FILT_N_UPD_m  = []                    # Variable to save updated Kalman filter NORTH value
FILT_E_UPD_m  = []                    # Variable to save updated Kalman filter EAST value

NORTH_ARUCO   = 0                     # Define global variable for Aruco NORTH 
EAST_ARUCO    = 0                     # Define global variable for Aruco EAST
UP_ARUCO      = 0                     # Define global variable for Aruco UP

FILT_N        = 0                     # Define global variable for Kalman filter NORTH 
FILT_E        = 0                     # Define global variable for Kalman filter EAST 
FILT_U        = 0                     # Define global variable for Kalman filter U 

IS_FILT_INIT = False                  # Flag to check if Kalman filter is initialised 

wp_id_KF      = 11                    # Waypoint ID for LAT, LONG from Kalman filter
wp_id_RAW     = 9                     # Waypoint ID for LAT, LONG from Aruco detection


#                                   # FUNCTIONS -> IVYBUS MESSAGES #
# # ------------------------------------------------------------------------------------------------------- #
# # --------- Bind to Drone Attitude Message --------- # 
# def attitude_callback(ac_id, pprzMsg):
#     global PITCH_values
#     global ROLL_values
#     global YAW_values
    
#     pitch = pprzMsg['theta']
#     roll  = pprzMsg['phi']
#     yaw   = pprzMsg['psi']
#     PITCH_values = pitch
#     ROLL_values  = roll
#     YAW_values   = yaw

# # --------- Bind to Drone NED Position Message --------- # 
# def NED_callback(ac_id, pprzMsg):
#     global NORTH_values
#     global EAST_values
#     global UP_values
    
#     NORTH_Drone  = pprzMsg['north']
#     EAST_Drone   = pprzMsg['east']
#     UP_Drone     = pprzMsg['up']
#     NORTH_values = NORTH_Drone
#     EAST_values  = EAST_Drone
#     UP_values    = UP_Drone

# # --------- Bind to Drone Latitude, Longitude, and Altitude Message --------- # 
# def ref_lat_long_alt_callback(ac_id, pprzMsg):
#     global REF_LAT_values
#     global REF_LONG_values
#     global REF_ALT_values
    
#     lat_drone    = pprzMsg['lat0']
#     long_drone   = pprzMsg['lon0']
#     alt_drone    = pprzMsg['alt0']
#     REF_LAT_values   = lat_drone
#     REF_LONG_values  = long_drone
#     REF_ALT_values   = alt_drone

# # --------- Get Attitude Values --------- # 
# def get_attitude_values():
#     global PITCH_values
#     global ROLL_values
#     global YAW_values
#     return PITCH_values, ROLL_values, YAW_values

# # --------- Get NED Position Values --------- # 
# def get_NED_values():
#     global NORTH_values
#     global EAST_values
#     global UP_values
#     return NORTH_values, EAST_values, UP_values

# # --------- Get Reference Latitude, Longitude, and Altitude --------- # 
# def get_ref_lat_long_alt_values():
#     global REF_LAT_values
#     global REF_LONG_values
#     global REF_ALT_values
#     return REF_LAT_values, REF_LONG_values, REF_ALT_values

#                                   # FUNCTIONS -> NED COORDINATES CONVERSION #
# # ------------------------------------------------------------------------------------------------------- #
# def NED_conversion(pitch, roll, yaw, Aruco_position):
#     # --------- Rotation Matrix (Rotation around Z-Axis/Yaw) ------- # 
#     RZ= np.array([
#                  [np.cos(yaw), -np.sin(yaw), 0],
#                  [np.sin(yaw), np.cos(yaw),  0],
#                  [          0,           0,  1]])

#     # --------- Rotation Matrix (Rotation around Y-Axis/Pitch) ------- # 
#     RY = np.array([
#                   [ np.cos(pitch),  0, np.sin(pitch)],
#                   [             0,  1,             0],
#                   [-np.sin(pitch),  0, np.cos(pitch)]])

#     # --------- Rotation Matrix (Rotation around X-Axis/Roll) ------- # 
#     RX = np.array([
#                   [1,            0,             0],
#                   [0, np.cos(roll), -np.sin(roll)],
#                   [0, np.sin(roll),  np.cos(roll)]])

#     # --------- Rotation Matrix ------- # 
#     R = RZ @ RY @ RX 

#     # --------- Obtain NED Coordinates ------- #
#     NED_vector = np.dot(R, Aruco_position)
#     NORTH, EAST, UP = NED_vector.squeeze()

#     return NORTH, EAST, UP

                                # FUNCTION -> VISUALISE LEGEND #
# ------------------------------------------------------------------------------------------------------- #
def visualizeLegend(frame_legend, width, height):
  # --------- Show "ARUCO" --------- # 
  org = (int(0.02*width), int(0.05*height))
  text = "ARUCO"
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1.25
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco Z --------- # 
  org = (int(0.03*width), int(0.09*height))
  text = f"Z: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco Altitude --------- # 
  org = (int(0.03*width), int(0.115*height))
  text = f"ALT: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco X_B --------- # 
  org = (int(0.03*width), int(0.27*height))
  text = f"X-BODY: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco Y_B --------- # 
  org = (int(0.03*width), int(0.31*height))
  text = f"Y-BODY: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco RAW NORTH --------- # 
  org = (int(0.03*width), int(0.35*height))
  text = f"RAW NORTH: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco RAW EAST --------- # 
  org = (int(0.03*width), int(0.39*height))
  text = f"RAW EAST: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco FILTERED NORTH --------- # 
  org = (int(0.03*width), int(0.43*height))
  text = f"FILT NORTH: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco FILTERED EAST --------- # 
  org = (int(0.03*width), int(0.47*height))
  text = f"FILT EAST: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

   # --------- Show Aruco LAT --------- # 
  org = (int(0.03*width), int(0.51*height))
  text = f"LAT: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Aruco LONG --------- # 
  org = (int(0.03*width), int(0.55*height))
  text = f"LONG: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show "DRONE" --------- # 
  org = (int(0.02*width), int(0.62*height))
  text = "DRONE"
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1.25
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)   
  
  # --------- Show Drone PITCH --------- # 
  org_1 = (int(0.03*width), int(0.66*height))
  text_1 = f"PITCH: "
  font_1 = cv2.FONT_HERSHEY_PLAIN
  fontScale_1 = 1
  color_1 = (255, 255, 255)
  lineThickness_1 = 2
  frame_legend = cv2.putText(frame_legend, text_1, org_1, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)

  # --------- Show Drone ROLL --------- # 
  org_2 = (int(0.03*width), int(0.7*height))
  text_2 = f"ROLL: "
  frame_legend = cv2.putText(frame_legend, text_2, org_2, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)

  # --------- Show Drone YAW --------- # 
  org_3 = (int(0.03*width), int(0.74*height))
  text_3 = f"YAW: "
  frame_legend = cv2.putText(frame_legend, text_3, org_3, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)

  # --------- Show Drone NORTH --------- # 
  org = (int(0.03*width), int(0.78*height))
  text = f"NORTH: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)
  
  # --------- Show Drone EAST --------- # 
  org = (int(0.03*width), int(0.82*height))
  text = f"EAST: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Show Drone UP --------- # 
  org = (int(0.03*width), int(0.86*height))
  text = f"UP: "
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  frame_legend = cv2.putText(frame_legend, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Draw Aruco Legend Outline --------- # 
  frame_legend = cv2.rectangle(frame_legend, (int(0.02*width), int(0.06*height)), (int(0.22*width), int(0.57*height)), (255, 255, 255), 2)

  # --------- Draw Drone Legend Outline --------- # 
  frame_legend = cv2.rectangle(frame_legend, (int(0.02*width), int(0.63*height)), (int(0.22*width), int(0.88*height)), (255, 255, 255), 2)

  # --------- Draw Reference System --------- # 
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.155*height)), (int(0.09*width), int(0.155*height)), (0, 0, 255), 3)                   # X = red
  frame_legend = cv2.line(frame_legend,(int(0.05*width), int(0.155*height)), (int(0.05*width), int(0.225*height)), (0, 255, 0), 3)                   # Y = green
  frame_legend = cv2.circle(frame_legend, (int(0.05*width), int(0.155*height)), 10, (255, 0, 0), 2)                                                 # Z = blue
  frame_legend = cv2.putText(frame_legend, "X", (int(0.046*width), int(0.164*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "X", (int(0.075*width), int(0.145*height)), font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Y", (int(0.033*width), int(0.215*height)), font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  frame_legend = cv2.putText(frame_legend, "Z", (int(0.03*width), int(0.140*height)), font, fontScale, (255, 0, 0), lineThickness, cv2.LINE_AA)

  return frame_legend

                          # FUNCTION -> VISUALISE X, Y, Z ARUCO MARKER POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoXYZMarkerPosition(X_visual, Y_visual, Z_visual, frame_pos, width, height, r, t, C, d):
# --------- Create Projection from 3D to 2D --------- # 
  axes_3D = np.float32([[1, 0, 0], [0, -1, 0], [0, 0, -1], [0, 0, 0]]).reshape(-1, 3)    # Points in 3D space
  axisPoints, _ = cv2.projectPoints(axes_3D, r, t, C, d)                                 # Project 3D points into 2D image plane

  # --------- Create X Marker Position Visualisation --------- # 
  X_start_Xline = width/2
  Y_start_Xline = axisPoints[3][0][1]
  X_end_Xline =  axisPoints[3][0][0]
  Y_end_Xline =  axisPoints[3][0][1]
  
  if X_visual >= 0:      # If postive X value -> show green 
    cv2.line(frame_pos, (int(X_start_Xline), int(Y_start_Xline)), (int(X_end_Xline), int(Y_end_Xline)), (0, 255, 0), 3)
  else:                  # Else -> show red 
    cv2.line(frame_pos, (int(X_start_Xline), int(Y_start_Xline)), (int(X_end_Xline), int(Y_end_Xline)), (0, 0, 255), 3)

  org_1 = (int(width/2), int(axisPoints[3][0][1]))   # Show X value on frame -> if positive show green, else show red
  text_1 = f"X: {round(X_visual, 2)}[m]"
  font_1 = cv2.FONT_HERSHEY_PLAIN
  fontScale_1 = 1.5
  lineThickness_1 = 2
  if X_visual >= 0:
    color_1 = (0, 255, 0)
  else:
    color_1 = (0, 0, 255)
  cv2.putText(frame_pos, text_1, org_1, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)

  # --------- Create Y Marker Position Visualisation --------- # 
  X_start_Yline = width/2
  Y_start_Yline = height/2
  X_end_Yline =  width/2
  Y_end_Yline =  axisPoints[3][0][1]
  
  if Y_visual >= 0:    # If postive Y-value -> show green 
    cv2.line(frame_pos, (int(X_start_Yline), int(Y_start_Yline)), (int(X_end_Yline), int(Y_end_Yline)), (0, 255, 0), 3)
  else:                # Else -> show red 
    cv2.line(frame_pos, (int(X_start_Yline), int(Y_start_Yline)), (int(X_end_Yline), int(Y_end_Yline)), (0, 0, 255), 3)

  org_2 = (int(width/2), int(height/2))    # Show Y value on frame -> if positive show green, else show red
  text_2 = f"Y: {round(Y_visual, 2)}[m]"
  font_2 = cv2.FONT_HERSHEY_PLAIN
  fontScale_2 = 1.5
  lineThickness_2 = 2
  if Y_visual >= 0:
    color_2 = (0, 255, 0)
  else:
    color_2 = (0, 0, 255)
  cv2.putText(frame_pos, text_2, org_2, font_2, fontScale_2, color_2, lineThickness_2, cv2.LINE_AA)
  
  # --------- Aruco Z Visualisation --------- # 
  org_3 = (int(0.11*width), int(0.09*height))
  text_3 = f" {round(Z_visual, 2)}[m]"
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

                                # FUNCTION -> VISUALISE BODY ARUCO POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoBODYMarkerPosition(X_B_visual, Y_B_visual, frame_pos, width, height):
  # --------- Drone X_B Visualisation --------- # 
  if X_B_visual >= 0:    # If postive X_B value -> show green 
    org_3 = (int(0.11*width), int(0.27*height))
    text_3 = f" {round(X_B_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.27*height))
    text_3 = f" {round(X_B_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone Y_B Visualisation --------- # 
  if Y_B_visual >= 0:    # If postive Y_B value -> show green 
    org_3 = (int(0.11*width), int(0.31*height))
    text_3 = f" {round(Y_B_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.31*height))
    text_3 = f" {round(Y_B_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)

  return frame_pos

                                # FUNCTION -> VISUALISE NED ARUCO POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoNEDMarkerPosition(NORTH_visual, EAST_visual, frame_pos, width, height):
  # --------- Drone NORTH Visualisation --------- # 
  if NORTH_visual >= 0:    # If postive NORTH value -> show green 
    org_3 = (int(0.11*width), int(0.35*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.35*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone EAST Visualisation --------- # 
  if EAST_visual >= 0:    # If postive EAST value -> show green 
    org_3 = (int(0.11*width), int(0.39*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.39*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)

  return frame_pos

                               # FUNCTION -> VISUALISE FILTERED NED ARUCO POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoFNEDMarkerPosition(NORTH_visual, EAST_visual, frame_pos, width, height):
  # --------- Drone NORTH Visualisation --------- # 
  if NORTH_visual >= 0:    # If postive NORTH value -> show green 
    org_3 = (int(0.11*width), int(0.43*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.43*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone EAST Visualisation --------- # 
  if EAST_visual >= 0:    # If postive EAST value -> show green 
    org_3 = (int(0.11*width), int(0.47*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.47*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)

  return frame_pos
                          # FUNCTION -> VISUALISE LAT, LONG, ALT ARUCO MARKER POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseArucoGeodeticMarkerPosition(LAT_visual, LONG_visual, ALT_visual, frame_pos, width, height):
    # --------- Create LATITUDE Marker Position Visualisation --------- # 
  org = (int(0.11*width), int(0.51*height))
  text = f" {round(LAT_visual, 4)}[deg.]"
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  cv2.putText(frame_pos, text, org, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Create LONGITUDE Position Visualisation --------- # 
  org_1 = (int(0.11*width), int(0.55*height))
  text_1 = f" {round(LONG_visual, 4)}[deg.]"
  cv2.putText(frame_pos, text_1, org_1, font, fontScale, color, lineThickness, cv2.LINE_AA)

  # --------- Aruco ALTITUDE Visualisation --------- # 
  org_2 = (int(0.11*width), int(0.115*height))
  text_2 = f" {round(ALT_visual, 2)}[m]"
  cv2.putText(frame_pos, text_2, org_2, font, fontScale, color, lineThickness, cv2.LINE_AA)

  return frame_pos

                                # FUNCTION -> VISUALISE DRONE ATTITUDE #
# ------------------------------------------------------------------------------------------------------- #
def visualiseDroneAttitude(frame_attitude, width, height, pitch_visual, roll_visual, yaw_visual):
  font = cv2.FONT_HERSHEY_PLAIN
  fontScale = 1
  color = (255, 255, 255)
  lineThickness = 2
  
  if pitch_visual >= 0:   # If postive pitch value -> show green 
    org_1 = (int(0.11*width), int(0.66*height))
    text_1 = f" {round(pitch_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_1, org_1, font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_1 = (int(0.11*width), int(0.66*height))
    text_1 = f" {round(pitch_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_1, org_1, font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)

  if roll_visual >= 0:    # If postive roll value -> show green 
    org_2 = (int(0.11*width), int(0.7*height))
    text_2 = f" {round(roll_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_2, org_2, font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_2 = (int(0.11*width), int(0.7*height))
    text_2 = f" {round(roll_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_2, org_2, font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)
  
  if yaw_visual >= 0:    # If postive yaw value -> show green 
    org_3 = (int(0.11*width), int(0.74*height))
    text_3 = f" {round(yaw_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_3, org_3, font, fontScale, (0, 255, 0), lineThickness, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.74*height))
    text_3 = f" {round(yaw_visual, 2)}[deg.]"
    frame_attitude = cv2.putText(frame_attitude, text_3, org_3, font, fontScale, (0, 0, 255), lineThickness, cv2.LINE_AA)

  return frame_attitude

                                 # FUNCTION -> VISUALISE NED DRONE POSITION #
# ------------------------------------------------------------------------------------------------------- #
def visualiseDroneNEDPosition(NORTH_visual, EAST_visual, UP_visual, frame_pos, width, height):
  # --------- Drone NORTH Visualisation --------- # 
  if NORTH_visual >= 0:    # If postive NORTH value -> show green 
    org_3 = (int(0.11*width), int(0.78*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.78*height))
    text_3 = f" {round(NORTH_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone EAST Visualisation --------- # 
  if EAST_visual >= 0:    # If postive EAST value -> show green 
    org_3 = (int(0.11*width), int(0.82*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 255, 0), lineThickness_3, cv2.LINE_AA)
  else:                   # Else -> show red 
    org_3 = (int(0.11*width), int(0.82*height))
    text_3 = f" {round(EAST_visual, 2)}[m]"
    font_3 = cv2.FONT_HERSHEY_PLAIN
    fontScale_3 = 1
    lineThickness_3 = 2
    cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (0, 0, 255), lineThickness_3, cv2.LINE_AA)
  
  # --------- Drone UP Visualisation --------- # 
  org_3 = (int(0.11*width), int(0.86*height))
  text_3 = f" {round(UP_visual, 2)}[m]"
  font_3 = cv2.FONT_HERSHEY_PLAIN
  fontScale_3 = 1
  lineThickness_3 = 2
  cv2.putText(frame_pos, text_3, org_3, font_3, fontScale_3, (255, 255, 255), lineThickness_3, cv2.LINE_AA)
 
  return frame_pos

#                                       # Ivybus INITIALISATION #
# # ------------------------------------------------------------------------------------------------------- #
# # --------- Create Ivy Interface --------- # 
# ivy = pprzlink.ivy.IvyMessagesInterface(agent_name="ArucoMarker", start_ivy=False, ivy_bus="127.255.255.255:2010")

# # --------- Start Ivy Interface --------- # 
# ivy.start()

# # --------- Subscribe to Ivy Messages --------- # 
# ivy.subscribe(attitude_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))
# ivy.subscribe(NED_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))
# ivy.subscribe(ref_lat_long_alt_callback, message.PprzMessage("telemetry", "INS_REF"))

                                # FUNCTION -> VISUALISE LEGEND #
# ------------------------------------------------------------------------------------------------------- #
def timeout(timeout_duration=1):
  class TimeoutError(Exception):
    pass

  def handler(A, B):
    raise TimeoutError()
  
  # --------- Set Timeout Handler --------- #
  signal.signal(signal.SIGALRM, handler)
  signal.alarm(timeout_duration)

  try:
    # --------- Read Frame-by-Frame --------- # 
    ret, frame = cap.read()

  except TimeoutError as exc:
    print("-------------------------------") 
    print("Error: Frame TIMEOUT")
    print("-------------------------------") 
    ret = False
    frame = None

  finally:
    signal.alarm(0)

  return ret, frame 

#                                  # FUNCTION -> MOVE WAYPOINT #
# # ------------------------------------------------------------------------------------------------------- #
# def move_waypoint_KF(ac_id, wp_id, aruco_lat, aruco_long, aruco_alt):
#     msg = message.PprzMessage("ground", "MOVE_WAYPOINT")
#     msg['ac_id'] = ac_id
#     msg['wp_id'] = wp_id
#     msg['lat'] = aruco_lat
#     msg['long'] = aruco_long
#     msg['alt'] = aruco_alt
#     ivy.send(msg)

# def move_waypoint_RAW(ac_id, wp_id, aruco_lat, aruco_long, aruco_alt):
#     msg = message.PprzMessage("ground", "MOVE_WAYPOINT")
#     msg['ac_id'] = ac_id
#     msg['wp_id'] = wp_id
#     msg['lat'] = aruco_lat
#     msg['long'] = aruco_long
#     msg['alt'] = aruco_alt
#     ivy.send(msg)

# ac_id = input("Enter Aicraft ID: ")

                                              # VIDEO #
# ------------------------------------------------------------------------------------------------------- #
# --------- Load Video --------- #
# cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream")   # Create a VideoCapture object (input is for herelink wifi connection)
# cap = cv2.VideoCapture("rtsp://192.168.42.129:8554/fpv_stream") # Create a VideoCapture object (input is for herelink bluetooth tethering)
cap = cv2.VideoCapture(0)
FPS = cap.get(cv2.CAP_PROP_FPS)                                 # Read FPS from input video

if FPS == 0:
    print("Error: FPS is 0")
    quit()

# --------- Functioning? --------- #
if (cap.isOpened()== False):                                    # Check if camera opened successfully
  print("Error: cannot open video file or stream")
 
# --------- Resolution --------- #
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# --------- Write Video Setup --------- #
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')                                             # Define video codec (FOURCC code)
out = cv2.VideoWriter('./Live_Videos/IMAV_CompleteV3.mp4', 
                      fourcc, FPS, (frame_width, frame_height))                                              # Create VideoWriter object 

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

  # --------- Update Iteration Counter --------- # 
  C_STEP = C_STEP + 1

  # # --------- Get Attitude Values from Ivybus --------- # 
  # PITCH_DRONE, ROLL_DRONE, YAW_DRONE = get_attitude_values()
  
  # # --------- Get NED Values from Ivybus --------- # 
  # NORTH_DRONE, EAST_DRONE, UP_DRONE = get_NED_values()

  # # --------- Get LAT, LONG, and ALT Values from Ivybus --------- # 
  # LAT_0, LONG_0, ALT_0 = get_ref_lat_long_alt_values()

  # --------- Read Frame-by-Frame --------- # 
  ret, frame = timeout()

  if ret == True: # If frame read correctly          
    # # --------- Resize Frame (Noise Reduction) --------- # 
    # scale_percent = 60 # Percent of original size -> At 60%, dim = (1152, 648), min scale_percent = 50%
    resized_frame_width = frame_width #int(frame_width * scale_percent / 100)
    resized_frame_height = frame_height #int(frame_height * scale_percent / 100)
    # dim = (resized_frame_width, resized_frame_height)
  
    # frame = cv2.resize(frame, dim)
    
    # --------- Convert to Grayscale --------- # 
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # --------- Aruco Marker Detection --------- # 
    (markerCorners, _, _) = arucoDetector.detectMarkers(gray_frame)
    
    # --------- Show Legend --------- # 
    frame = visualizeLegend(frame, resized_frame_width, resized_frame_height)

    # # --------- Save, Print, and Show Drone Attitude --------- # 
    # if PITCH_DRONE is not None:
    #   PITCH_DRONE = float(PITCH_DRONE)
    #   ROLL_DRONE  = float(ROLL_DRONE)
    #   YAW_DRONE   = float(YAW_DRONE)
      
    #   PITCH_DRONE = PITCH_DRONE*pprz_attitude_conversion
    #   ROLL_DRONE  = ROLL_DRONE*pprz_attitude_conversion
    #   YAW_DRONE   = YAW_DRONE*pprz_attitude_conversion 

    #   PITCH_DRONE_m.append(PITCH_DRONE) # Save measured pitch
    #   print(f"-------- ITERATION: {C_STEP} --------")      
    #   print(f"Drone PITCH: {PITCH_DRONE}")

    #   ROLL_DRONE_m.append(ROLL_DRONE)   # Save measured roll
    #   print(f"Drone ROLL: {ROLL_DRONE}")

    #   YAW_DRONE_m.append(YAW_DRONE)     # Save measured yaw
    #   print(f"Drone YAW: {YAW_DRONE}")

    #   frame = visualiseDroneAttitude(frame, resized_frame_width, resized_frame_height, PITCH_DRONE, ROLL_DRONE, YAW_DRONE)

    # # --------- Save, Print, and Show Drone NORTH, EAST, and UP --------- # 
    # if NORTH_DRONE is not None:
    #   NORTH_DRONE = float(NORTH_DRONE)
    #   EAST_DRONE  = float(EAST_DRONE)
    #   UP_DRONE    = float(UP_DRONE)

    #   NORTH_DRONE = NORTH_DRONE*pprz_NED_conversion
    #   EAST_DRONE  = EAST_DRONE*pprz_NED_conversion
    #   UP_DRONE    = UP_DRONE*pprz_NED_conversion 

    #   NORTH_DRONE_m.append(NORTH_DRONE) # Save measured drone NORTH
    #   print(f"Drone NORTH: {NORTH_DRONE}")
      
    #   EAST_DRONE_m.append(EAST_DRONE)   # Save measured drone EAST
    #   print(f"Drone EAST: {EAST_DRONE}")      
      
    #   UP_DRONE_m.append(UP_DRONE)   # Save measured drone UP
    #   print(f"Drone UP: {UP_DRONE}")
      
    #   # --------- Visualise NED Drone Position --------- # 
    #   frame = visualiseDroneNEDPosition(NORTH_DRONE, EAST_DRONE, UP_DRONE, frame, resized_frame_width, resized_frame_height)

    # # --------- FLAG -> Aruco Marker Not Detected --------- # 
    # DETECTION = 0

    if len(markerCorners) > 0: # At least one marker detected
      # --------- Aruco Marker Pose Estimation --------- # 
      rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camera_Matrix, distortion_Coeff)
      
      # (rvec - tvec).any()    # Remove Numpy value array error
         
      # --------- Save and Print X, Y, and Z --------- # 
      try:
        X_ARUCO = tvec[0][0][0]
        X_ARUCO_m.append(X_ARUCO)          # Save measured X
        print(f"Aruco X: {X_ARUCO}")

        Y_ARUCO = tvec[0][0][1]
        Y_ARUCO_m.append(Y_ARUCO)          # Save measured Y
        print(f"Aruco Y: {Y_ARUCO}")

        Z_ARUCO = tvec[0][0][2]
        Z_ARUCO_m.append(Z_ARUCO)          # Save measured Z
        print(f"Aruco Z: {Z_ARUCO}")

      # --------- Visualise X, Y, Z Aruco Marker Position --------- # 
        frame = visualiseArucoXYZMarkerPosition(X_ARUCO, Y_ARUCO, Z_ARUCO, frame, resized_frame_width, resized_frame_height, rvec[0], tvec[0], camera_Matrix, distortion_Coeff)
              
      except:
        print("-------------------------------") 
        print("Error: rvec/tvec empty")
        print("-------------------------------")
        continue 

      # # --------- NED Conversion and Moving to Relative Position --------- # 
      # if PITCH_DRONE is not None: 
      #   PITCH_DRONE = math.radians(PITCH_DRONE)
      #   ROLL_DRONE  = math.radians(ROLL_DRONE)
      #   YAW_DRONE   = math.radians(YAW_DRONE)

      #   # --------- Convert Aruco marker Position in Image Coordinates to Body Coordinates --------- #
      #   # X (body) = Y (image plane), Y(body) = -X (image plane)
      #   X_ARUCO_B = Y_ARUCO
      #   Y_ARUCO_B = -X_ARUCO
      #   Z_ARUCO_B = Z_ARUCO

      #   # --------- Visualise Body Aruco Marker Position --------- # 
      #   frame = visualiseArucoBODYMarkerPosition(X_ARUCO_B, Y_ARUCO_B, frame, resized_frame_width, resized_frame_height)

      #   # Generate Aruco position row vector
      #   ARUCO_POSITION_B = np.array([[X_ARUCO_B], [Y_ARUCO_B], [Z_ARUCO_B]])

      #   # --------- Convert Aruco Position in Image Coordinates to NED Coordinates Relative to Drone --------- # 
      #   NORTH_REL, EAST_REL, UP_REL = NED_conversion(PITCH_DRONE, ROLL_DRONE, YAW_DRONE, ARUCO_POSITION_B)

      #   # --------- NED Aruco Marker Position --------- # 
      #   if NORTH_DRONE is not None:
      #     # --------- NED Relative and NED Drone Summation --------- # 
      #     NORTH_ARUCO = NORTH_REL + NORTH_DRONE
      #     EAST_ARUCO  = EAST_REL + EAST_DRONE
      #     UP_ARUCO    = UP_REL                     # Application based decision -> take Aruco relative UP value (Z not relevant)  

      #     # --------- Save and Print Aruco Marker NORTH, EAST, and UP --------- # 
      #     NORTH_ARUCO_m.append(NORTH_ARUCO)        # Save measured Aruco Marker NORTH
      #     print(f"Aruco NORTH: {NORTH_ARUCO}")

      #     EAST_ARUCO_m.append(EAST_ARUCO)          # Save measured Aruco Marker EAST
      #     print(f"Aruco EAST: {EAST_ARUCO}")

      #     UP_ARUCO_m.append(UP_ARUCO)              # Save measured Aruco Marker UP
      #     print(f"Aruco UP: {UP_ARUCO}")

      #     # --------- Visualise NED Aruco Marker Position --------- # 
      #     frame = visualiseArucoNEDMarkerPosition(NORTH_ARUCO, EAST_ARUCO, frame, resized_frame_width, resized_frame_height)
          
          # # --------- FLAG -> Aruco Marker Detected --------- # 
          # DETECTION = 1
                                              # KALMAN FILTER #
    # # ------------------------------------------------------------------------------------------------------- #
    # if IS_FILT_INIT:            # Check if KF is initialised
    #   dt = 1/FPS                # Set time step 
      
    #   # --------- PREDICTION STEP --------- # 
    #   NED_PRED = predict(dt)
    #   FILT_N = NED_PRED[0]
    #   FILT_E = NED_PRED[1]

    #   FILT_N = float(FILT_N)
    #   FILT_E = float(FILT_E)
    #   FILT_U = UP_ARUCO

    #   FILT_N_PRED_m.append(FILT_N)
    #   FILT_E_PRED_m.append(FILT_E)

    # if DETECTION == 1:
    #   if not IS_FILT_INIT:
    #     # --------- FILTER INITIALISATION --------- # 
    #     init([NORTH_ARUCO, EAST_ARUCO, UP_ARUCO])
    #     IS_FILT_INIT = True
    #   else:
    #     # --------- UPDATE STEP --------- # 
    #     NED_UPD = update([NORTH_ARUCO, EAST_ARUCO, UP_ARUCO])
    #     FILT_N = NED_UPD[0]
    #     FILT_E = NED_UPD[1]
    #     FILT_U = UP_ARUCO 

    #     FILT_N = float(FILT_N)
    #     FILT_E = float(FILT_E)
        
    #     FILT_N_UPD_m.append(FILT_N)
    #     FILT_E_UPD_m.append(FILT_E)

    # # --------- Visualise Filtered NED Aruco Marker Position --------- # 
    # frame = visualiseArucoFNEDMarkerPosition(FILT_N, FILT_E, frame, resized_frame_width, resized_frame_height)

    # # --------- RAW MEASUREMENTS --------- # 
    # FILT_N_RAW = NORTH_ARUCO
    # FILT_E_RAW = EAST_ARUCO
    # FILT_U_RAW = UP_ARUCO

    # # --------- Convert To LAT, LONG, and ALT Aruco Marker Position and Move Waypoint --------- #
    # if LAT_0 is not None: 
    #   LAT_0  = float(LAT_0)
    #   LONG_0 = float(LONG_0)
    #   ALT_0  = float(ALT_0)
      
    #   LAT_0  = LAT_0*pprz_lat_long_conversion
    #   LONG_0 = LONG_0*pprz_lat_long_conversion
    #   ALT_0  = ALT_0*pprz_alt_conversion
            
    #   LAT_0_m.append(LAT_0)             # Save measured drone Ref LAT
    #   print(f"Drone Latitude0: {LAT_0}")
      
    #   LONG_0_m.append(LONG_0)           # Save measured drone Ref LONG
    #   print(f"Drone Longitude0: {LONG_0}")      
      
    #   ALT_0_m.append(ALT_0)             # Save measured drone Ref ALT
    #   print(f"Drone Altitude0: {ALT_0}")
            
    #   # --------- Conversion --------- #
    #   LAT_ARUCO, LONG_ARUCO, _         = pymap3d.ned2geodetic(FILT_N, FILT_E, FILT_U, LAT_0, LONG_0, ALT_0)
    #   LAT_ARUCO_RAW, LONG_ARUCO_RAW, _ = pymap3d.ned2geodetic(FILT_N_RAW, FILT_E_RAW, FILT_U_RAW, LAT_0, LONG_0, ALT_0)

    #   # --------- Altitude -> Aldenhoven Testing Centre Specific (42m AGL) --------- #
    #   ALT_ARUCO = 25                       # Check GCS Alt in Paparazzi by clicking on standby waypoint and manually fill in reference altitude

    #   # --------- Save and Print Aruco Marker NORTH, EAST, and UP --------- # 
    #   LAT_ARUCO_m.append(LAT_ARUCO)            # Save measured Aruco Marker LATITUDE
    #   print(f"Aruco Latitude: {LAT_ARUCO}")

    #   LONG_ARUCO_m.append(LONG_ARUCO)          # Save measured Aruco Marker LONGITUDE
    #   print(f"Aruco Longitude: {LONG_ARUCO}")

    #   ALT_ARUCO_m.append(ALT_ARUCO)            # Save measured Aruco Marker ALTITUDE
    #   print(f"Aruco Altitude: {ALT_ARUCO}")
    #   print("-------------------------------") 

    #   # --------- Visualise NED Aruco Marker Position --------- # 
    #   frame = visualiseArucoGeodeticMarkerPosition(LAT_ARUCO, LONG_ARUCO, ALT_ARUCO, frame, resized_frame_width, resized_frame_height)

      # # --------- Move Waypoint --------- #
      # move_waypoint_KF(ac_id, wp_id_KF, LAT_ARUCO, LONG_ARUCO, ALT_ARUCO)
      # move_waypoint_RAW(ac_id, wp_id_RAW, LAT_ARUCO_RAW, LONG_ARUCO_RAW, ALT_ARUCO)
      
    # --------- Write Video --------- # 
    # out.write(frame)
    
    # --------- Display Output Frame --------- # 
    cv2.imshow('Frame', frame)

    # --------- Stop Code Execution (Press 'q') --------- # 
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
            
  # --------- Break While Loop (No Frame Retrieved) --------- # 
  else:
    print('Error: frame not retrieved')  
    continue

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

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoUP_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(UP_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoLAT_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LAT_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoLONG_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LONG_ARUCO_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_ArucoALT_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ALT_ARUCO_m, time_m))

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

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneUP_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(UP_DRONE_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneLAT_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LAT_0_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneLONG_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LONG_0_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/IMAV_09_09_23_TEST1_DroneALT_V1_1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ALT_0_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoX_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoY_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoZ_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoNORTH_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoEAST_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoUP_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(UP_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoLAT_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LAT_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoLONG_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LONG_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_ArucoALT_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ALT_ARUCO_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_FILTER_N_PRED_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(FILT_N_PRED_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_FILTER_E_PRED_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(FILT_E_PRED_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_FILTER_N_EST_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(FILT_N_UPD_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_FILTER_E_EST_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(FILT_E_UPD_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DronePitch_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(PITCH_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneRoll_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ROLL_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneYaw_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(YAW_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneNORTH_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneEAST_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneUP_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(UP_DRONE_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneLAT_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LAT_0_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneLONG_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(LONG_0_m, time_m))

# with open('./Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneALT_V3', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(ALT_0_m, time_m))

# --------- Indoor Tests --------- # 
# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_X_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Y_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Z_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_NORTH_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(NORTH_Aruco_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_EAST_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EAST_Aruco_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_UP_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(UP_Aruco_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Pitch_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(pitch_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Roll_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(roll_m, time_m))

# with open('~/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Yaw_V1', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(yaw_m, time_m))

                                            # CLOSE CODE PROPERLY #
# ------------------------------------------------------------------------------------------------------- #
# --------- Release/Stop Objects --------- # 
cap.release()
out.release()
# ivy.shutdown()

# --------- Close Frames --------- # 
cv2.destroyAllWindows()
