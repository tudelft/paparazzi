# ------------------------------------------------------------------------------------------------------- #
# --------- General --------- # 
import time
import math
import csv
import sys
import cv2
import cv2.aruco
import numpy as np
from datetime import datetime
import os

# # --------- Ivybus Specific --------- # 
sys.path.append("/home/orangepi/paparazzi/sw/ext/pprzlink/lib/v2.0/python/src")

from ivy.std_api import *
import pprzlink.ivy
import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message          

#-----------------------Parameters--------------------------#
MARKER_SIZE = 0.15 # meters
timestep = None
desired_aruco_dictionary = "DICT_5X5_1000"
pathLoad = '/home/orangepi/paparazzi/sw/ext/aruco_detection/cameraCalibration_mapir_1440p.xml'
pixel_w = 1920  # Example: 1920 pixels wide
pixel_h = 1440  # Example: 1440 pixels tall
usb_num = '/dev/mapir'
#-----------------------------------------------------------#

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
time_m        = []                    # Variable to save measured time

NORTH_ARUCO   = 0                     # Define global variable for Aruco NORTH 
EAST_ARUCO    = 0                     # Define global variable for Aruco EAST
UP_ARUCO      = 0                     # Define global variable for Aruco UP


#                                   # FUNCTIONS -> IVYBUS MESSAGES #
# # ------------------------------------------------------------------------------------------------------- #
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
    global UP_values
    
    NORTH_Drone  = pprzMsg['north']
    EAST_Drone   = pprzMsg['east']
    UP_Drone     = pprzMsg['up']
    NORTH_values = NORTH_Drone
    EAST_values  = EAST_Drone
    UP_values    = UP_Drone

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
    global UP_values
    return NORTH_values, EAST_values, UP_values

# --------- Get Reference Latitude, Longitude, and Altitude --------- # 
def get_ref_lat_long_alt_values():
    global REF_LAT_values
    global REF_LONG_values
    global REF_ALT_values
    return REF_LAT_values, REF_LONG_values, REF_ALT_values

#                                   # FUNCTIONS -> NED COORDINATES CONVERSION #
# # ------------------------------------------------------------------------------------------------------- #
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
    R = RZ @ RY @ RX 

    # --------- Obtain NED Coordinates ------- #
    NED_vector = np.dot(R, Aruco_position)
    NORTH, EAST, UP = NED_vector.squeeze()

    return NORTH, EAST, UP

#                                  # FUNCTION -> SEND ARUCO POSITION MESSAGE ON IVY BUS#
# # ------------------------------------------------------------------------------------------------------- #

def update_aruco_position_NED(ac_id, flag, aruco_NED_pos_x, aruco_NED_pos_y, aruco_NED_pos_z):
    msg = message.PprzMessage("ground", "DESIRED_SP")
    msg['ac_id'] = ac_id
    msg['flag'] = flag
    msg['ux'] = aruco_NED_pos_x
    msg['uy'] = aruco_NED_pos_y
    msg['uz'] = aruco_NED_pos_z
    ivy.send(msg)

ac_id = 1 #input("Enter Aicraft ID: ")

                                              # VIDEO #
# ------------------------------------------------------------------------------------------------------- #
def load_cam_para():
    cv_file = cv2.FileStorage(pathLoad, cv2.FILE_STORAGE_READ)
    camera_Matrix = cv_file.getNode("cM").mat()
    distortion_Coeff = cv_file.getNode("dist").mat()
    cv_file.release()
    return camera_Matrix, distortion_Coeff


#                                       # Ivybus INITIALISATION #
# # ------------------------------------------------------------------------------------------------------- #
# --------- Create Ivy Interface --------- # 
ivy = pprzlink.ivy.IvyMessagesInterface(agent_name="ArucoMarker", start_ivy=False, ivy_bus="127.255.255.255:2010")

# --------- Start Ivy Interface --------- # 
ivy.start()

# --------- Subscribe to Ivy Messages --------- # 
ivy.subscribe(attitude_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))
ivy.subscribe(NED_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))


while True:
    time.sleep(1)
    try:

        #                                       # OpenCV Stuff #
        # # ------------------------------------------------------------------------------------------------------- #
        # --------- Load Video --------- #
        cap = cv2.VideoCapture(usb_num)
        if not cap.isOpened():
            raise ValueError(f"Failed to open camera on port {usb_num}. Please check the port number and try again.")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, pixel_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, pixel_h)

                                            # ARUCO MARKER DETECTION SETUP #
        # ------------------------------------------------------------------------------------------------------- #
        ARUCO_DICT = {"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000}
        camera_Matrix, distortion_Coeff = load_cam_para()

        print(f"[INFO] detecting '{desired_aruco_dictionary}' markers...")
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        this_aruco_parameters = cv2.aruco.DetectorParameters()

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

            # --------- Get Attitude Values from Ivybus --------- # 
            PITCH_DRONE, ROLL_DRONE, YAW_DRONE = get_attitude_values()
        
            # --------- Get NED Values from Ivybus --------- # 
            NORTH_DRONE, EAST_DRONE, UP_DRONE = get_NED_values()

            # --------- Get LAT, LONG, and ALT Values from Ivybus --------- # 
            LAT_0, LONG_0, ALT_0 = get_ref_lat_long_alt_values()

            # --------- Read Frame-by-Frame --------- # 
            ret, frame = cap.read()
            if frame.shape[0] != pixel_h or frame.shape[1] != pixel_w:
                print(frame.shape)
                raise ValueError(f'Input and requested frame dims do not agree!')


            if ret == True: # If frame read correctly          
                # Detect ArUco markers in the video frame
                (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, this_aruco_dictionary, parameters=this_aruco_parameters)

                # # --------- Save, Print, and Show Drone Attitude --------- # 
                if PITCH_DRONE is not None:
                    PITCH_DRONE = float(PITCH_DRONE)
                    ROLL_DRONE  = float(ROLL_DRONE)
                    YAW_DRONE   = float(YAW_DRONE)
                    
                    PITCH_DRONE = PITCH_DRONE*pprz_attitude_conversion
                    ROLL_DRONE  = ROLL_DRONE*pprz_attitude_conversion
                    YAW_DRONE   = YAW_DRONE*pprz_attitude_conversion 

                # --------- Save, Print, and Show Drone NORTH, EAST, and UP --------- # 
                if NORTH_DRONE is not None:
                    NORTH_DRONE = float(NORTH_DRONE)
                    EAST_DRONE  = float(EAST_DRONE)
                    UP_DRONE    = float(UP_DRONE)

                    NORTH_DRONE = NORTH_DRONE*pprz_NED_conversion
                    EAST_DRONE  = EAST_DRONE*pprz_NED_conversion
                    UP_DRONE    = UP_DRONE*pprz_NED_conversion 

                    print(f'Drone position in NED: ({NORTH_DRONE, EAST_DRONE, UP_DRONE})')
                
                # --------- FLAG -> Aruco Marker Not Detected --------- # 
                DETECTION = 0

                if len(corners) > 0: # At least one marker detected
                    # --------- Aruco Marker Pose Estimation --------- # 
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_Matrix, distortion_Coeff)
                        
                    # --------- Save and Print X, Y, and Z --------- # 
                    try:
                        X_ARUCO = tvec[0][0][0]
                        Y_ARUCO = tvec[0][0][1]
                        Z_ARUCO = tvec[0][0][2]
                        print(X_ARUCO, Y_ARUCO, Z_ARUCO)

                    except:
                        print("-------------------------------") 
                        print("Error: rvec/tvec empty")
                        print("-------------------------------")
                        continue 

                # # --------- NED Conversion and Moving to Relative Position --------- # 
                    if PITCH_DRONE is not None: 
                        PITCH_DRONE = math.radians(PITCH_DRONE)
                        ROLL_DRONE  = math.radians(ROLL_DRONE)
                        YAW_DRONE   = math.radians(YAW_DRONE)

                        # --------- Convert Aruco marker Position in Image Coordinates to Body Coordinates --------- #
                        # X (body) = Y (image plane), Y(body) = -X (image plane)
                        # camera pointing ground, top backwards
                        X_ARUCO_B = Y_ARUCO * 1 # bottom of cam is front pos x of drone
                        Y_ARUCO_B = -X_ARUCO * 1 # right of cam is left neg y of drone
                        Z_ARUCO_B = Z_ARUCO

                    #   # Generate Aruco position row vector
                        ARUCO_POSITION_B = np.array([[X_ARUCO_B], [Y_ARUCO_B], [Z_ARUCO_B]])
                        print(f'Rel position in Drone sys: {ARUCO_POSITION_B}')

                    #   # --------- Convert Aruco Position in Image Coordinates to NED Coordinates Relative to Drone --------- # 
                        NORTH_REL, EAST_REL, UP_REL = NED_conversion(PITCH_DRONE, ROLL_DRONE, YAW_DRONE, ARUCO_POSITION_B)
                    
                    #   # --------- NED Aruco Marker Position --------- # 
                        if NORTH_DRONE is not None:
                            # --------- NED Relative and NED Drone Summation --------- # 
                            NORTH_ARUCO = NORTH_REL + NORTH_DRONE
                            EAST_ARUCO  = EAST_REL + EAST_DRONE
                            UP_ARUCO    = UP_DRONE - UP_REL             # Application based decision -> take Aruco relative UP value (Z not relevant)  

                            DETECTION = 1

                    # # --------- Move Waypoint --------- #
                if  DETECTION == 1 : 
                    # # --------- Move Waypoint --------- #
                    update_aruco_position_NED(ac_id, 0, NORTH_ARUCO, EAST_ARUCO, -UP_ARUCO)
                    print(f'Drone position in NED: ({NORTH_ARUCO, EAST_ARUCO, UP_ARUCO})')
                        
            # --------- Break While Loop (No Frame Retrieved) --------- # 
            else:
                print('Error: frame not retrieved')  
                continue

    except Exception:
        pass