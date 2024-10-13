# ------------------------------------------------------------------------------------------------------- #
# --------- General --------- # 
import time
import math
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


#-----------------------Parameters--------------------------#
MARKER_SIZE = 0.259 # meters
desired_aruco_dictionary = "DICT_5X5_1000"
pathLoad = '/home/orangepi/paparazzi/sw/ext/aruco_detection/cameraCalibration_mapir_1440p.xml'
pixel_w = 1920  # Example: 1920 pixels wide
pixel_h = 1440  # Example: 1440 pixels tall
usb_num = '/dev/mapir'
#-----------------------------------------------------------#

#                                   # FUNCTIONS -> IVYBUS MESSAGES #
# # ------------------------------------------------------------------------------------------------------- #
time_last_heartbeat = 0
# --------- Bind to main program message --------- # 
def on_am7_hearbeat(agent, *args):
    print("Received heartbeat from main program, main program time is:", args[0])
    print("Current program time is:", time.monotonic())


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
IvyBindMsg(on_am7_hearbeat, "HEARTBEAT_AM7 (\\S*)")

while True:
    time.sleep(1)
    #Prepare and send a heartbeat message on the ivy bus to say that we are attemting connecting to the camera: 
    #prepare the string with the message id (HEARTBEAT_ARUCO), the current time and the system number, this case -1:
    msg = "HEARTBEAT_ARUCO " + str(time.monotonic()) + " -1"
    IvySendMsg(msg)
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
            if(time.monotonic() - time_last_heartbeat > 1):
                #send a heartbeat message on the ivy bus to say that we are still running:
                msg = "HEARTBEAT_ARUCO " + str(time.monotonic()) + " 1"
                time_last_heartbeat = time.monotonic()

            # --------- Update Iteration Counter --------- # 
            C_STEP = C_STEP + 1

            # --------- Read Frame-by-Frame --------- # 
            ret, frame = cap.read()
            if frame.shape[0] != pixel_h or frame.shape[1] != pixel_w:
                print(frame.shape)
                raise ValueError(f'Input and requested frame dims do not agree!')

            if ret == True: # If frame read correctly          
                # Detect ArUco markers in the video frame
                (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, this_aruco_dictionary, parameters=this_aruco_parameters)

                if ids is not None and len(corners) > 0:  # At least one marker detected
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

                    # --------- Convert Aruco marker Position in Image Coordinates to Body Coordinates --------- #
                    # X (body) = Y (image plane), Y(body) = -X (image plane)
                    # camera pointing ground, top backwards
                    X_ARUCO_B = Y_ARUCO * 1 # bottom of cam is front pos x of drone
                    Y_ARUCO_B = -X_ARUCO * 1 # right of cam is left neg y of drone
                    Z_ARUCO_B = Z_ARUCO

                    # # --------- Send update on ivy bus with timestamp --------- #
                    msg = "ARUCO_RELATIVE_POS " + str(time.monotonic()) + " " + str(ids) + " " + str(X_ARUCO_B) + " " + str(Y_ARUCO_B) + " " + str(Z_ARUCO_B)
                    IvySendMsg(msg)
                    print(f'ARUCO position in BODY: ({X_ARUCO_B, Y_ARUCO_B, Z_ARUCO_B})')
                        
            # --------- Break While Loop (No Frame Retrieved) --------- # 
            else:
                print('Error: frame not retrieved')  
                continue

    except Exception:
        pass