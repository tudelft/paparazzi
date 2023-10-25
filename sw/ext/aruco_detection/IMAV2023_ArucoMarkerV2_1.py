# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             05/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          2.1 
# 
# Description:  
# Detect Aruco marker from video live-stream.
# 
# Upcoming Version: 3.0
# Detect AND track Aruco marker from images -> requires camera calibration step (done in seperate code)
# 
# Upcoming Version: 4.0
# Detect AND track Aruco marker from pre-recorded videos.
# 
# Upcoming Version: 5.0
# Detect AND track Aruco marker from video live-stream.
#  -------------------------------------------------------------------------

import glob
import cv2
import cv2.aruco
import numpy as np
#  ------------------------------------------------------------------------- #
#                                FUNCTIONS                                   #
#  ------------------------------------------------------------------------- #

# -------------- Frame Rescaling Function --------------
def frame_rescale(frame, scale_percent):
# Frame downscaling -> USED AS FRAME INPUT AND AFFECTS PERFORMANCE OF DETECTOR
  scale_percent = 30 # Percent of original size
  width = int(frame.shape[1] * scale_percent / 100)
  height = int(frame.shape[0] * scale_percent / 100)
  dim = (width, height)
  return cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

#  ------------------------------------------------------------------------- #
#                            VIDEO PROCESSING                                #
#  ------------------------------------------------------------------------- #

# -------------- Load Video Live-stream --------------
# Create a VideoCapture object and read from camera (input is either 0 or 1, for first and second camera, respectively)
cap = cv2.VideoCapture(2)

# -------------- Detect Aruco Markers, Write Video, and View Video --------------
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error: cannot open video file or stream")
 
# Default resolutions of the frame are obtained and converted from float to integer
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Define video codec (FOURCC code)
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')

# Create VideoWriter object 
out = cv2.VideoWriter('/home/kevin/IMAV2023/Live_Videos/ArucoMarker_LIVEVideo_Detected_1.mp4', fourcc, 30, (frame_width, frame_height))

# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  
  # Rescale frame
#   frame = frame_rescale(frame, 30)

  if ret == True:
    # -------------- Aruco Marker Detection --------------
    # Load dictionary for aruco marker
    arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    # Initiate aruco marker detection parameters
    arucoParameters =  cv2.aruco.DetectorParameters()

    # Aruco marker detection setup
    arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

    # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
    (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(frame)
    
    # AT LEAST ONE MARKER DETECTED   
    if len(markerCorners) > 0:
      for i in range(0, len(markerIDs)):  
        # Draw around the correctly detected aruco markers
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)  
            
        # Draw around the rejected candidates
        # cv2.aruco.drawDetectedMarkers(frame, rejectedCandidates, borderColor=(100, 200, 255))

    # Write the frame into the file
    out.write(frame)
    
    # Display the resulting frame
    cv2.imshow('Frame', frame)
 
    # Wait 1 [ms] between each frame until it ends or press 'q' on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else:
    print('Error: frame not retrieved') 
    break
 
# Release the video capture and video write objects
cap.release()
out.release()
 
# Close all the frames
cv2.destroyAllWindows()