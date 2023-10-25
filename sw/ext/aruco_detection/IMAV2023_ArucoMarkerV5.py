# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             05/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          5.0 
# 
# Description:  
# - Detect AND track Aruco marker from video live-stream
# - Input  -> Camera parameters, real-time video stream via Herelink
# - Output -> Relative (X, Y, Z,) position and Euclidean Distance of the Aruco marker to the camera (displayed per frame)
#  -------------------------------------------------------------------------
#  ------------------------------------------------------------------------- #
#                            LIBRARY DEFINITION                              #
#  ------------------------------------------------------------------------- #
import time
import csv
import glob
import cv2
import cv2.aruco
import numpy as np

# #  ------------------------------------------------------------------------- #
# #                          STREAM WORKING CHECK                              #
# #  ------------------------------------------------------------------------- #
# cap = cv2.VideoCapture(2)
# cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream")

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

#  ------------------------------------------------------------------------- #
#                      CONSTANT/VARIABLE DEFINITION                          #
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

#  ------------------------------------------------------------------------- #
#                           LOAD CAMERA VARIABLES                            #
#  ------------------------------------------------------------------------- #
pathLoad = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w1920_h1080_HERELINKV2.xml' # Calibrated for video streaming with resolution w=640, h=480
# pathLoad = '/home/kevin/IMAV2023/CameraCalibration_Variables/Videos/MAPIR_cameraCalibration_Video_w640_h480.xml' # Calibrated for video streaming with resolution w=640, h=480
cv_file = cv2.FileStorage(pathLoad, cv2.FILE_STORAGE_READ)
camera_Matrix = cv_file.getNode("cM").mat()
distortion_Coeff = cv_file.getNode("dist").mat()

cv_file.release()

#  ------------------------------------------------------------------------- #
#              LOAD VIDEO, DEFINE VIDEO CAPTURE, AND WRITE OBJECTS           #
#  ------------------------------------------------------------------------- #
# Create a VideoCapture object and read from camera (input is either an rtsp stream from the herelink module or input 2)
# cap = cv2.VideoCapture(2)
cap = cv2.VideoCapture("rtsp://192.168.43.1:8554/fpv_stream")

# Get FPS from Video Capture object
FPS = cap.get(cv2.CAP_PROP_FPS)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error: cannot open video file or stream")
 
# Default resolutions of the frame are obtained and converted from float to integer
frame_width = int(cap.get(3)) 
frame_height = int(cap.get(4))

print(frame_width)
print(frame_height)

# Define video codec (FOURCC code)
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')

# Create VideoWriter object 
# out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/VALKENBURG_20_07_23_TEST8.mp4', fourcc, FPS, (frame_width, frame_height))
out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/TEST_1x1MarkerV4.mp4', fourcc, FPS, (frame_width, frame_height))

# Measure start time
start_time = time.time()

# Read until video is completed
while(cap.isOpened()):
  
  # Measure and save current time
  live_time = time.time()
  current_time = live_time - start_time
  time_m.append(current_time)
  # print(f"Running for: {time_m}", end='\n')

  # Capture frame-by-frame
  ret, frame = cap.read()

  # Change resolution
  # frame = cv2.resize(frame, (640, 480))

  # Image blurring (Averaging filter)
  # frame = cv2.blur(frame, (5, 5))

  # If frame found
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
            for i in range(0, 1):  
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
                text_2 = f"X: {round(X, 2)}[m] Y: {round(Y, 2)}[m] Z: {round(Z, 2)}[m]"
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
    # out.write(frame) # THIS MIGHT SLOW DOWN/OVERLOAD CODE
    
    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Wait 1 [ms] between each frame until it ends or press 'q' on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
            
  # Break the loop
  else:
    print('Error: frame not retrieved')  
    break
  print(time_m)

#  ------------------------------------------------------------------------- #
#                          SAVE MEASURED VARIABLES                           #
#  ------------------------------------------------------------------------- #
# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/LIVE_VALKENBURG_20_07_23_TEST2_X', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/LIVE_VALKENBURG_20_07_23_TEST2_Y', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/LIVE_VALKENBURG_20_07_23_TEST2_Z', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Outdoor_Tests/LIVE_VALKENBURG_20_07_23_TEST2_Dist', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EuclideanDistance_m, time_m))

#  ------------------------------------------------------------------------- #
#                          SAVE MEASURED VARIABLES                           #
#  ------------------------------------------------------------------------- #
# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST_X_V2', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(X_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST_Y_V2', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Y_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST_Z_V2', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(Z_m, time_m))

# with open('/home/kevin/IMAV2023/Measured_Variables/Indoor_Tests/TEST_Dist_V2', 'w') as csvfile:
#     writer=csv.writer(csvfile, delimiter=',')
#     writer.writerows(zip(EuclideanDistance_m, time_m))

# Release the video capture and video write objects
cap.release()
out.release()
            
# Close all the frames
cv2.destroyAllWindows()