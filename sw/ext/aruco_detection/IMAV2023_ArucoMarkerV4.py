# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             05/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          4.0 
# 
# Description:  
# Detect AND track Aruco marker from pre-recorded videos -> Used as test and debug platform using pre-recorded videos
# 
# Upcoming Version: 5.0
# Detect AND track Aruco marker from video live-stream.
#  -------------------------------------------------------------------------

#  ------------------------------------------------------------------------- #
#                            LIBRARY DEFINITION                              #
#  ------------------------------------------------------------------------- #
import time
import math
import csv
import cv2
import cv2.aruco
import numpy as np

# #  ------------------------------------------------------------------------- #
# #                       IVYBUS -> SENDING/RECEIVING DATA                     #
# #  ------------------------------------------------------------------------- #
# import sys

# sys.path.append("/home/kevin/paparazzi/sw/ext/pprzlink/lib/v2.0/python/")

# from ivy.std_api import *
# import pprzlink.ivy
# import pprzlink.messages_xml_map as messages_xml_map
# import pprzlink.message as message

# # Bind to drone attitude message (callback function for when message is received)
# def attitude_callback(ac_id, pprzMsg):
#   # Print the message and the sender id
#   print ("Received message %s from %s" % (pprzMsg,ac_id))
#   pitch = pprzMsg['theta']
#   roll  = pprzMsg['phi']
#   yaw   = pprzMsg['psi']

# # Create Ivy interface -> receive messages from Ivybus
# ivy = pprzlink.ivy.IvyMessagesInterface(agent_name="ArucoMarker", start_ivy=False, ivy_bus="127.255.255.255:2010")

# # Start the ivy interface
# ivy.start()

# # Subscribe to Ivybus messages
# ivy.subscribe(attitude_callback, message.PprzMessage("telemetry", "NPS_RATE_ATTITUDE"))

# Stop Ivy interface
# ivy.shutdown()
            

#  ------------------------------------------------------------------------- #
#                            CONSTANT DEFINITION                             #
#  ------------------------------------------------------------------------- #
# Size of Aruco marker in [m]
MARKER_SIZE = 1.107 # For small marker change to 0.35 [m]

# Size of Coordinate System axis in 3D
axes_3D = np.float32([[MARKER_SIZE, 0, 0], [0, -MARKER_SIZE, 0], [0, 0, -MARKER_SIZE], [0, 0, 0]]).reshape(-1, 3)

# Arrays to store measured X, Y, Z, Euclidean Distance, and time values 
X_m = []                    # Measured X value
Y_m = []                    # Measured Y value
Z_m = []                    # Measured Z value
EuclideanDistance_m = []    # Measured Euclidean Distance value
time_m = []                 # Measured time

# Drone attitude 
# pitch = 0 # Theta
# roll  = 0 # Phi
# yaw   = 0 # Psi

#  ------------------------------------------------------------------------- #
#                                FUNCTIONS                                   #
#  ------------------------------------------------------------------------- #
def frame_rescale(frame, scale_percent):
# Frame downscaling -> USED AS FRAME INPUT AND AFFECTS PERFORMANCE OF DETECTOR
  scale_percent = 30 # Percent of original size
  width = int(frame.shape[1] * scale_percent / 100)
  height = int(frame.shape[0] * scale_percent / 100)
  dim = (width, height)
  return cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

#  ------------------------------------------------------------------------- #
#                           LOAD CAMERA VARIABLES                            #
#  ------------------------------------------------------------------------- #
pathLoad = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w1920_h1080_HERELINKV2.xml'
cv_file = cv2.FileStorage(pathLoad, cv2.FILE_STORAGE_READ)
camera_Matrix = cv_file.getNode("cM").mat()
distortion_Coeff = cv_file.getNode("dist").mat()

cv_file.release()

#  ------------------------------------------------------------------------- #
#              LOAD VIDEO, DEFINE VIDEO CAPTURE, AND WRITE OBJECTS           #
#  ------------------------------------------------------------------------- #
#  Define video path
path = '/home/kevin/IMAV2023/Live_Videos/VALKENBURG_20_07_23_TEST7.mp4'   
	
# Create a VideoCapture object and read from input file
cap = cv2.VideoCapture(path)
FPS = cap.get(cv2.CAP_PROP_FPS)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error: cannot open video file or stream")
 
# Default resolutions of the frame are obtained and converted from float to integer
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Define video codec (FOURCC code)
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')

# Create VideoWriter object 
out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/Results/VALKENBURG_20_07_23_RESULT.mp4', fourcc, FPS, (frame_width, frame_height))

# Measure start time
start_time = time.time()

# Read until video is completed
while(cap.isOpened()):
  
  # Measure and save current time
  live_time = time.time()
  current_time = live_time - start_time
  time_m.append(current_time)
  
  # Capture frame-by-frame
  ret, frame = cap.read()

  if ret == True:    
    #  ------------------------------------------------------------------------- #
    #                         ARUCO MARKER DETECTION                             #
    #  ------------------------------------------------------------------------- #      
    # Convert to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Load dictionary for aruco marker
    arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    # Initiate aruco marker detection parameters
    arucoParameters =  cv2.aruco.DetectorParameters()

    # Aruco marker detection setup
    arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

    # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
    (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(gray_frame)
    
    # AT LEAST ONE MARKER DETECTED   
    if len(markerCorners) > 0:        
        # Failsafe -> Only consider Marker ID=700 (to nullify false positives)
        if any(x == 700 for x in markerIDs):
            # Iterate over aruco markers (Allow only 1 marker detection aince we know only ID=700 should be detected)
            totalMarkers = min(1, len(markerIDs))
            for i in range(0, totalMarkers):  
                #  ------------------------------------------------------------------------- #
                #                       ARUCO MARKER POSE ESTIMATION                         #
                #  ------------------------------------------------------------------------- #
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], MARKER_SIZE, camera_Matrix, distortion_Coeff)

                # Remove Numpy value array error
                (rvec - tvec).any()  

                # (X, Y, Z)
                X = tvec[i][0][0]
                Y = tvec[i][0][1]
                Z = tvec[i][0][2]
                

                              
                #  ------------------------------------------------------------------------- #
                #             COMPUTE AND SHOW EUCLIDEAN DISTANCE, X, Y, AND Z               #
                #  ------------------------------------------------------------------------- #
                # Compute Euclidean distance between two points in space (origin to 3D point in space) -> sqrt(x^2 + y^2 + z^2)   
                euclideanDist = np.sqrt(X ** 2 + Y ** 2 + Z ** 2)

                # Print Euclidean distance, X, Y, and Z 
                print(f"Euclidean Distance: {euclideanDist}")
                print(f"X: {X}")
                print(f"Y: {Y}")
                print(f"Z: {Z}") 

                # Save measured X, Y, Z, and Euclidean Distance
                X_m.append(X)
                Y_m.append(Y) 
                Z_m.append(Z)
                EuclideanDistance_m.append(euclideanDist)

                #  ------------------------------------------------------------------------- #
                #                DRAW MARKERS, AXES, AND ADD TEXT TO IMAGES                  #
                #  ------------------------------------------------------------------------- #
                # Draw around the correctly detected aruco markers
                cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)  
  
                # DRAW COORDINATE SYSTEM IN 2D IMAGE PLANE -> Project 3D points into 2D image plane
                axisPoints, _ = cv2.projectPoints(axes_3D, rvec, tvec, camera_Matrix, distortion_Coeff)

                # DRAW COORDINATE SYSTEM IN 2D IMAGE PLANE -> Define start (X, Y) coordinates
                X_start =  axisPoints[3][0][0] + (frame_width/2 - axisPoints[3][0][0])
                Y_start = axisPoints[3][0][1] + (frame_height/2 - axisPoints[3][0][1]) 
                      
                # print(frame_width/2)
                # print(axisPoints[3][0][0])
                # print(frame_width/2 - axisPoints[3][0][0])
                # print(axisPoints[3][0][0] + (frame_width/2 - axisPoints[3][0][0]))

                # print("-----------------------------------")      

                # print(frame_height/2)
                # print(axisPoints[3][0][1])
                # print(frame_height/2 - axisPoints[3][0][1])
                # print(axisPoints[3][0][1] + (frame_height/2 - axisPoints[3][0][1]))

                # DRAW COORDINATE SYSTEM IN 2D IMAGE PLANE -> Define end (X, Y) coordinates for X-axis, Y-axis, and Z-axis
                X_end_Xaxis =  X_start + (axisPoints[0][0][0] - axisPoints[3][0][0])
                Y_end_Xaxis =  Y_start + (axisPoints[0][0][1] - axisPoints[3][0][1])
                X_end_Yaxis =  X_start + (axisPoints[1][0][0] - axisPoints[3][0][0])
                Y_end_Yaxis =  Y_start + (axisPoints[1][0][1] - axisPoints[3][0][1])
                
                # DRAW COORDINATE SYSTEM IN 2D IMAGE PLANE -> Draw axis on Aruco markers -> X = red, Y = green, Z = blue     
                cv2.line(frame, (int(X_start), int(Y_start)), (int(X_end_Xaxis), int(Y_end_Xaxis)), (0,0,255), 3)
                cv2.line(frame, (int(X_start), int(Y_start)), (int(X_end_Yaxis), int(Y_end_Yaxis)), (0,255,0), 3)
                
                # Add text to images 
                org_1 = (int(markerCorners[i][0][0][0]), int(markerCorners[i][0][0][1])) # origin
                text_1 = f"Dist: {round(euclideanDist, 2)}[m]"
                font_1 = cv2.FONT_HERSHEY_PLAIN
                fontScale_1 = 1.3
                color_1 = (0, 0, 255)
                lineThickness_1 = 2

                org_2 = (int(markerCorners[i][0][3][0]), int(markerCorners[i][0][3][1])) # origin
                text_2 = f"X: {round(tvec[i][0][0], 2)}[m] Y: {round(tvec[i][0][1], 2)}[m] Z: {round(tvec[i][0][2], 2)}[m]"
                font_2 = cv2.FONT_HERSHEY_PLAIN
                fontScale_2 = 1.0
                color_2 = (0, 0, 255)
                lineThickness_2 = 2
                
                cv2.putText(frame, text_1, org_1, font_1, fontScale_1, color_1, lineThickness_1, cv2.LINE_AA)
                cv2.putText(frame, text_2, org_2, font_2, fontScale_2, color_2, lineThickness_2, cv2.LINE_AA)

    #  ------------------------------------------------------------------------- #
    #                         DISPLAY AND SAVE IMAGES                            #
    #  ------------------------------------------------------------------------- #
    # Write the frame into the file
    # out.write(frame)
    
    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Wait 1 [ms] between each frame until it ends or press 'q' on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
            
  # Break the loop
  else:
    print('Error: frame not retrieved')  
    break

# #  ------------------------------------------------------------------------- #
# #                          SAVE MEASURED VARIABLES                           #
# #  ------------------------------------------------------------------------- #
# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/PRE_VALKENBURG_20_07_23_TEST_X', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/PRE_VALKENBURG_20_07_23_TEST_Y', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/PRE_VALKENBURG_20_07_23_TEST_Z', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/PRE_VALKENBURG_20_07_23_TEST_Dist', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EuclideanDistance_m, time_m))

# #  ------------------------------------------------------------------------- #
# #                          SAVE MEASURED VARIABLES                           #
# #  ------------------------------------------------------------------------- #
# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_X', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Y', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_ZChanges', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST1_Dist', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EuclideanDistance_m, time_m))
   
# Release the video capture and video write objects
cap.release()
out.release()

# Close all the frames
cv2.destroyAllWindows()