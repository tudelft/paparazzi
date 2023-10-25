# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             04/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.1 
# 
# Description:  
# Detect Aruco marker from images.
# 
# Updates:  
# - Tried several methods, but chosen to include scaling before inputting image to detector.
# - Deleted commented out code.
# 
# Upcoming Version: 2.0
# Detect Aruco marker from pre-recorded video.
# 
# Upcoming Version: 3.0
# Detect Aruco marker from live video stream.
#  -------------------------------------------------------------------------

import glob
import cv2
import cv2.aruco
from pathlib import Path

# -------------- Image Rescaling Function --------------
def img_rescale(img, scale_percent):
# Image downscaling -> USED AS IMAGE INPUT AND AFFECTS PERFORMANCE OF DETECTOR
  scale_percent = 30 # Percent of original size
  width = int(img.shape[1] * scale_percent / 100)
  height = int(img.shape[0] * scale_percent / 100)
  dim = (width, height)
  return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# -------------- Load Image and Resize Image --------------
#  Define image path
path = '/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_2/*.JPG'   

# Initialise counter for formatting saved images
counter = 1

# Read images in a batch
for file in glob.iglob(path): 
    # Read image
    img = cv2.imread(file)
    # img = cv2.imread(file, cv2.IMREAD_GRAYSCALE)

    # Image rescaling
    img = img_rescale(img, 30)

# -------------- Aruco Marker Detection --------------
    # Load dictionary for aruco marker
    arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    # Initiate aruco marker detection parameters
    arucoParameters =  cv2.aruco.DetectorParameters()# Upcoming Version: 4.0

    # Aruco marker detection setup
    arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

    # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
    (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(img)

    # -------------- Draw Resulting Detection and Save Image --------------
    # AT LEAST ONE MARKER DETECTED   
    if len(markerCorners) > 0:
        # FALSE DETECTETION (Marker ID != 700) 
        if any(x != 700 for x in markerIDs):
            for i in range(0, len(markerIDs)):  
                # Draw around the correctly detected aruco markers
                cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIDs)  
                
                # Draw around the rejected candidates
                cv2.aruco.drawDetectedMarkers(img, rejectedCandidates, borderColor=(100, 200, 255))

            # Display the resulting image
            cv2.imshow('image', img)

            # Wait until the "0" key is pressed to close the image window
            cv2.waitKey(0)

            # --------------------------------------------------------
            # UNCOMMENT TO SAVE IMAGE
            # Saving the image
            cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_2/Results_SCALING_30/FALSE_DETECTIONS/FALSE_ArucoMarkerDetected_{}.JPG'.format(counter), img)
            # --------------------------------------------------------

            # Remove image from memory
            cv2.destroyAllWindows()    
        
        # MARKER CORRECTLY DETECTED
        else:
            # Iterate over aruco markers
            for i in range(0, len(markerIDs)):  
                # Draw around the correctly detected aruco markers
                cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIDs)  
                
                # Draw around the rejected candidates
                cv2.aruco.drawDetectedMarkers(img, rejectedCandidates, borderColor=(100, 200, 255))

            # Display the resulting image
            cv2.imshow('image', img)

            # Wait until the "0" key is pressed to close the image window
            cv2.waitKey(0)

            # --------------------------------------------------------
            # UNCOMMENT TO SAVE IMAGE            
            # Saving the image
            cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_2/Results_SCALING_30/ArucoMarkerDetected_{}.JPG'.format(counter), img)
            # --------------------------------------------------------

            # Remove image from memory
            cv2.destroyAllWindows()

    # MARKER NOT DETECTED 
    else:
        # Display the resulting image
        cv2.imshow('image', img)

        # Wait until the "0" key is pressed to close the image window
        cv2.waitKey(0)
    
        # --------------------------------------------------------
        # UNCOMMENT TO SAVE IMAGE                
        # Saving the image
        cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_2/Results_SCALING_30/NOT_DETECTED/NOT_ArucoMarkerDetected_{}.JPG'.format(counter), img)
        # --------------------------------------------------------

        # Remove image from memory
        cv2.destroyAllWindows()
    
    # Increment counter
    counter += 1
