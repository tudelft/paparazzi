# --------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             02/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.0 
# 
# Description:  
# Detect Aruco marker from images.
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

# -------------- Load Image and Resize Image --------------
#  Define image path
path = '/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/*.JPG'   

# Initialise counter for formatting saved images
counter = 1

# Read images in a batch
for file in glob.iglob(path): 
    # Read image
    img = cv2.imread(file)
    # img = cv2.imread(file, cv2.IMREAD_GRAYSCALE)

    # Image downscaling -> USED AS IMAGE INPUT AND AFFECTS PERFORMANCE OF DETECTOR
    scale_percent = 30 # Percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# -------------- Aruco Marker Detection --------------
    # Load dictionary for aruco marker
    arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    # Initiate aruco marker detection parameters
    arucoParameters =  cv2.aruco.DetectorParameters()

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
            cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results_SCALING_EXPERIMENT/FALSE_DETECTIONS/FALSE_ArucoMarkerDetected_{}.JPG'.format(counter), img)
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
            cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results_SCALING_EXPERIMENT/ArucoMarkerDetected_{}.JPG'.format(counter), img)
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
        cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results_SCALING_EXPERIMENT/NOT_DETECTED/NOT_ArucoMarkerDetected_{}.JPG'.format(counter), img)
        # --------------------------------------------------------

        # Remove image from memory
        cv2.destroyAllWindows()
    
    # Increment counter
    counter += 1





    # # FALSE DETECTETION (At least two markers detected) -> Try again with original image and no downscaling as input
    # elif len(markerCorners) > 1:
    #     # -------------- Load Image and Resize Image --------------
    #     # Read image
    #     # img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    #     img_retry_1 = cv2.imread(file)

    #     # -------------- Aruco Marker Detection --------------
    #     # Load dictionary for aruco marker
    #     arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    #     # Initiate aruco marker detection parameters
    #     arucoParameters =  cv2.aruco.DetectorParameters()

    #     # Aruco marker detection setup
    #     arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

    #     # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
    #     (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(img_retry_1)

    #     # -------------- Draw Resulting Detection and Save Image --------------
    #     # Check if at least one aruco marker was detected in unscaled image
    #     if len(markerCorners) > 0:
    #         # Iterate over aruco markers
    #         for i in range(0, len(markerIDs)):  
    #             # Draw around the correctly detected aruco markers
    #             cv2.aruco.drawDetectedMarkers(img_retry_1, markerCorners, markerIDs)  
                
    #             # Draw around the rejected candidates
    #             cv2.aruco.drawDetectedMarkers(img_retry_1, rejectedCandidates, borderColor=(100, 200, 255))

    #         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
    #         scale_percent = 30 # Percent of original size
    #         width = int(img_retry_1.shape[1] * scale_percent / 100)
    #         height = int(img_retry_1.shape[0] * scale_percent / 100)
    #         dim = (width, height)
    #         img_retry_1 = cv2.resize(img_retry_1, dim, interpolation = cv2.INTER_AREA)

    #         # Display the resulting image
    #         cv2.imshow('image', img_retry_1)

    #         # Wait until the "0" key is pressed to close the image window
    #         cv2.waitKey(0)
        
    #         # --------------------------------------------------------
    #         # UNCOMMENT TO SAVE IMAGE    
    #         # Saving the image
    #         cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results/ArucoMarkerDetected_{}.JPG'.format(i), img_retry_1)
    #         # --------------------------------------------------------

    #         # Remove image from memory
    #         cv2.destroyAllWindows()
    
    #     elif len(markerCorners) > 1:
    #         for i in range(0, len(markerIDs)):  
    #             # Draw around the correctly detected aruco markers
    #             cv2.aruco.drawDetectedMarkers(img_retry_1, markerCorners, markerIDs)  
                
    #             # Draw around the rejected candidates
    #             cv2.aruco.drawDetectedMarkers(img_retry_1, rejectedCandidates, borderColor=(100, 200, 255))

    #         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
    #         scale_percent = 30 # Percent of original size
    #         width = int(img_retry_1.shape[1] * scale_percent / 100)
    #         height = int(img_retry_1.shape[0] * scale_percent / 100)
    #         dim = (width, height)
    #         img_retry_1 = cv2.resize(img_retry_1, dim, interpolation = cv2.INTER_AREA)

    #         # Display the resulting image
    #         cv2.imshow('image', img_retry_1)

    #         # Wait until the "0" key is pressed to close the image window
    #         cv2.waitKey(0)

    #         # --------------------------------------------------------
    #         # UNCOMMENT TO SAVE IMAGE
    #         # Saving the image
    #         cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results/FALSE_DETECTIONS/FALSE_ArucoMarkerDetected_{}.JPG'.format(i), img_retry_1)
    #         # --------------------------------------------------------

    #         # Remove image from memory
    #         cv2.destroyAllWindows()    

    #     else:
    #         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
    #         scale_percent = 30 # Percent of original size
    #         width = int(img_retry_1.shape[1] * scale_percent / 100)
    #         height = int(img_retry_1.shape[0] * scale_percent / 100)
    #         dim = (width, height)
    #         img_retry_1 = cv2.resize(img_retry_1, dim, interpolation = cv2.INTER_AREA)
            
    #         # Display the resulting image
    #         cv2.imshow('image', img_retry_1)

    #         # Wait until the "0" key is pressed to close the image window
    #         cv2.waitKey(0)

    #         # --------------------------------------------------------
    #         # UNCOMMENT TO SAVE IMAGE                
    #         # Saving the image
    #         cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results/NOT_DETECTED/NOT_ArucoMarkerDetected_{}.JPG'.format(i), img_retry_1)
    #         # --------------------------------------------------------

    #         # Remove image from memory
    #         cv2.destroyAllWindows()

    # # MARKER NOT DETECTED -> Try again with original image and no downscaling as input
    # else:
    #     # -------------- Load Image and Resize Image --------------
    #     # Read image
    #     # img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    #     img_retry_2 = cv2.imread(file)

    #     # -------------- Aruco Marker Detection --------------
    #     # Load dictionary for aruco marker
    #     arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    #     # Initiate aruco marker detection parameters
    #     arucoParameters =  cv2.aruco.DetectorParameters()

    #     # Aruco marker detection setup
    #     arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

    #     # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
    #     (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(img_retry_2)

    #     # -------------- Draw Resulting Detection and Save Image --------------
    #     # Check if at least one aruco marker was detected in unscaled image
    #     if len(markerCorners) > 0:
    #         # Iterate over aruco markers
    #         for i in range(0, len(markerIDs)):  
    #             # Draw around the correctly detected aruco markers
    #             cv2.aruco.drawDetectedMarkers(img_retry_2, markerCorners, markerIDs)  
                
    #             # Draw around the rejected candidates
    #             cv2.aruco.drawDetectedMarkers(img_retry_2, rejectedCandidates, borderColor=(100, 200, 255))

    #         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
    #         scale_percent = 30 # Percent of original size
    #         width = int(img_retry_2.shape[1] * scale_percent / 100)
    #         height = int(img_retry_2.shape[0] * scale_percent / 100)
    #         dim = (width, height)
    #         img_retry_2 = cv2.resize(img_retry_2, dim, interpolation = cv2.INTER_AREA)

    #         # Display the resulting image
    #         cv2.imshow('image', img_retry_2)

    #         # Wait until the "0" key is pressed to close the image window
    #         cv2.waitKey(0)
        
    #         # --------------------------------------------------------
    #         # UNCOMMENT TO SAVE IMAGE                
    #         # Saving the image
    #         cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results/ArucoMarkerDetected_{}.JPG'.format(i), img_retry_2)
    #         # --------------------------------------------------------

    #         # Remove image from memory
    #         cv2.destroyAllWindows()
    
    #     elif len(markerCorners) > 1:
    #         for i in range(0, len(markerIDs)):  
    #             # Draw around the correctly detected aruco markers
    #             cv2.aruco.drawDetectedMarkers(img_retry_2, markerCorners, markerIDs)  
                
    #             # Draw around the rejected candidates
    #             cv2.aruco.drawDetectedMarkers(img_retry_2, rejectedCandidates, borderColor=(100, 200, 255))

    #         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
    #         scale_percent = 30 # Percent of original size
    #         width = int(img_retry_2.shape[1] * scale_percent / 100)
    #         height = int(img_retry_2.shape[0] * scale_percent / 100)
    #         dim = (width, height)
    #         img_retry_2 = cv2.resize(img_retry_2, dim, interpolation = cv2.INTER_AREA)

    #         # Display the resulting image
    #         cv2.imshow('image', img_retry_2)

    #         # Wait until the "0" key is pressed to close the image window
    #         cv2.waitKey(0)

    #         # --------------------------------------------------------
    #         # UNCOMMENT TO SAVE IMAGE
    #         # Saving the image
    #         cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results/FALSE_DETECTIONS/FALSE_ArucoMarkerDetected_{}.JPG'.format(i), img_retry_2)
    #         # --------------------------------------------------------

    #         # Remove image from memory
    #         cv2.destroyAllWindows()    

    #     else:
    #         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
    #         scale_percent = 30 # Percent of original size
    #         width = int(img_retry_2.shape[1] * scale_percent / 100)
    #         height = int(img_retry_2.shape[0] * scale_percent / 100)
    #         dim = (width, height)
    #         img_retry_2 = cv2.resize(img_retry_2, dim, interpolation = cv2.INTER_AREA)

    #         # Display the resulting image
    #         cv2.imshow('image', img_retry_2)

    #         # Wait until the "0" key is pressed to close the image window
    #         cv2.waitKey(0)

    #         # --------------------------------------------------------
    #         # UNCOMMENT TO SAVE IMAGE                
    #         # Saving the image
    #         cv2.imwrite('/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/Results/NOT_DETECTED/NOT_ArucoMarkerDetected_{}.JPG'.format(i), img_retry_2)
    #         # --------------------------------------------------------

    #         # Remove image from memory
    #         cv2.destroyAllWindows()












# ----------------------------------------------------------------------------------------------------------------------- #
                            # --- Code that works by manually going through each image --- #
# ----------------------------------------------------------------------------------------------------------------------- #

# import os
# import glob
# import cv2
# import cv2.aruco

# # -------------- Load Image and Resize Image --------------
# #  Define image path
# path = "/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Pictures/Storey_3/.JPG"  

# img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
# img = cv2.imread(path)

# # Image downscaling -> USED AS IMAGE INPUT AND AFFECTS PERFORMANCE OF DETECTOR
# scale_percent = 30 # Percent of original size
# width = int(img.shape[1] * scale_percent / 100)
# height = int(img.shape[0] * scale_percent / 100)
# dim = (width, height)
# img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# # -------------- Aruco Marker Detection --------------
# # Load dictionary for aruco marker
# arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

# # Initiate aruco marker detection parameters
# arucoParameters =  cv2.aruco.DetectorParameters()

# # Aruco marker detection setup
# arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

# # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
# (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(img)

# # -------------- Draw Resulting Detection and Save Image --------------
# # MARKER CORRECTLY DETECTED
# if len(markerCorners) > 0:
#     # Iterate over aruco markers
#     for i in range(0, len(markerIDs)):  
#         # Draw around the correctly detected aruco markers
#         cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIDs)  
        
#         # Draw around the rejected candidates
#         cv2.aruco.drawDetectedMarkers(img, rejectedCandidates, borderColor=(100, 200, 255))

#     # Display the resulting image
#     cv2.imshow('image', img)

#     # Wait until the "0" key is pressed to close the image window
#     cv2.waitKey(0)

#     # --------------------------------------------------------
#     # UNCOMMENT TO SAVE IMAGE
#     # Change the current directory to specified directory 
#     os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3")
        
#     # Saving the image
#     cv2.imwrite('ArucoMarkerDetected_011.JPG', img)
#     # --------------------------------------------------------

#     # Remove image from memory
#     cv2.destroyAllWindows()

# # FALSE DETECTETION (At least two markers detected) -> Try again with original image and no downscaling as input
# elif len(markerCorners) > 1:
#     # -------------- Load Image and Resize Image --------------
#     #  Define image path
#     path = "/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/2023_0704_034316_011.JPG"  

#     # Read image
#     # img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
#     img_retry_1 = cv2.imread(path)

#     # -------------- Aruco Marker Detection --------------
#     # Load dictionary for aruco marker
#     arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

#     # Initiate aruco marker detection parameters
#     arucoParameters =  cv2.aruco.DetectorParameters()

#     # Aruco marker detection setup
#     arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

#     # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
#     (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(img_retry_1)

#     # -------------- Draw Resulting Detection and Save Image --------------
#     # Check if at least one aruco marker was detected in unscaled image
#     if len(markerCorners) > 0:
#         # Iterate over aruco markers
#         for i in range(0, len(markerIDs)):  
#             # Draw around the correctly detected aruco markers
#             cv2.aruco.drawDetectedMarkers(img_retry_1, markerCorners, markerIDs)  
            
#             # Draw around the rejected candidates
#             cv2.aruco.drawDetectedMarkers(img_retry_1, rejectedCandidates, borderColor=(100, 200, 255))

#         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
#         scale_percent = 30 # Percent of original size
#         width = int(img_retry_1.shape[1] * scale_percent / 100)
#         height = int(img_retry_1.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         img_retry_1 = cv2.resize(img_retry_1, dim, interpolation = cv2.INTER_AREA)

#         # Display the resulting image
#         cv2.imshow('image', img_retry_1)

#         # Wait until the "0" key is pressed to close the image window
#         cv2.waitKey(0)
    
#         # --------------------------------------------------------
#         # UNCOMMENT TO SAVE IMAGE
#         # Change the current directory to specified directory 
#         os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3")
            
#         # Saving the image
#         cv2.imwrite('ArucoMarkerDetected_011.JPG', img_retry_1)
#         # --------------------------------------------------------

#         # Remove image from memory
#         cv2.destroyAllWindows()
   
#     elif len(markerCorners) > 1:
#         for i in range(0, len(markerIDs)):  
#             # Draw around the correctly detected aruco markers
#             cv2.aruco.drawDetectedMarkers(img_retry_1, markerCorners, markerIDs)  
            
#             # Draw around the rejected candidates
#             # cv2.aruco.drawDetectedMarkers(img, rejectedCandidates, borderColor=(100, 0, 240))
#             cv2.aruco.drawDetectedMarkers(img_retry_1, rejectedCandidates, borderColor=(100, 200, 255))

#        # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
#         scale_percent = 30 # Percent of original size
#         width = int(img_retry_1.shape[1] * scale_percent / 100)
#         height = int(img_retry_1.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         img_retry_1 = cv2.resize(img_retry_1, dim, interpolation = cv2.INTER_AREA)

#         # Display the resulting image
#         cv2.imshow('image', img_retry_1)

#         # Wait until the "0" key is pressed to close the image window
#         cv2.waitKey(0)

#         # --------------------------------------------------------
#         # UNCOMMENT TO SAVE IMAGE
#         # Change the current directory to specified directory 
#         os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3/FALSE_DETECTIONS")
            
#         # Saving the image
#         cv2.imwrite('FALSE_ArucoMarkerDetected_011.JPG', img_retry_1)
#         # --------------------------------------------------------

#         # Remove image from memory
#         cv2.destroyAllWindows()    

#     else:
#         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
#         scale_percent = 30 # Percent of original size
#         width = int(img_retry_1.shape[1] * scale_percent / 100)
#         height = int(img_retry_1.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         img_retry_1 = cv2.resize(img_retry_1, dim, interpolation = cv2.INTER_AREA)
        
#         # Display the resulting image
#         cv2.imshow('image', img_retry_1)

#         # Wait until the "0" key is pressed to close the image window
#         cv2.waitKey(0)

#         # --------------------------------------------------------
#         # UNCOMMENT TO SAVE IMAGE
#         # Change the current directory to specified directory 
#         os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3/NOT_DETECTED")
            
#         # Saving the image
#         cv2.imwrite('NOT_ArucoMarkerDetected_011.JPG', img_retry_1)
#         # --------------------------------------------------------

#         # Remove image from memory
#         cv2.destroyAllWindows()

# # MARKER NOT DETECTED -> Try again with original image and no downscaling as input
# else:
#     # -------------- Load Image and Resize Image --------------
#     #  Define image path
#     path = "/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/2023_0704_034316_011.JPG"  

#     # Read image
#     # img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
#     img_retry_2 = cv2.imread(path)

#     # -------------- Aruco Marker Detection --------------
#     # Load dictionary for aruco marker
#     arucoDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

#     # Initiate aruco marker detection parameters
#     arucoParameters =  cv2.aruco.DetectorParameters()

#     # Aruco marker detection setup
#     arucoDetector = cv2.aruco.ArucoDetector(arucoDictionary, arucoParameters)

#     # List of aruco marker detected corners, IDs corresponding to each aruco marker, and rejected aruco markers
#     (markerCorners, markerIDs, rejectedCandidates) = arucoDetector.detectMarkers(img_retry_2)

#     # -------------- Draw Resulting Detection and Save Image --------------
#     # Check if at least one aruco marker was detected in unscaled image
#     if len(markerCorners) > 0:
#         # Iterate over aruco markers
#         for i in range(0, len(markerIDs)):  
#             # Draw around the correctly detected aruco markers
#             cv2.aruco.drawDetectedMarkers(img_retry_2, markerCorners, markerIDs)  
            
#             # Draw around the rejected candidates
#             cv2.aruco.drawDetectedMarkers(img_retry_2, rejectedCandidates, borderColor=(100, 200, 255))

#         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
#         scale_percent = 30 # Percent of original size
#         width = int(img_retry_2.shape[1] * scale_percent / 100)
#         height = int(img_retry_2.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         img_retry_2 = cv2.resize(img_retry_2, dim, interpolation = cv2.INTER_AREA)

#         # Display the resulting image
#         cv2.imshow('image', img_retry_2)

#         # Wait until the "0" key is pressed to close the image window
#         cv2.waitKey(0)
    
#         # --------------------------------------------------------
#         # UNCOMMENT TO SAVE IMAGE
#         # Change the current directory to specified directory 
#         os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3")
            
#         # Saving the image
#         cv2.imwrite('ArucoMarkerDetected_011.JPG', img_retry_2)
#         # --------------------------------------------------------

#         # Remove image from memory
#         cv2.destroyAllWindows()
   
#     elif len(markerCorners) > 1:
#         for i in range(0, len(markerIDs)):  
#             # Draw around the correctly detected aruco markers
#             cv2.aruco.drawDetectedMarkers(img_retry_2, markerCorners, markerIDs)  
            
#             # Draw around the rejected candidates
#             # cv2.aruco.drawDetectedMarkers(img, rejectedCandidates, borderColor=(100, 0, 240))
#             cv2.aruco.drawDetectedMarkers(img_retry_2, rejectedCandidates, borderColor=(100, 200, 255))

#         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
#         scale_percent = 30 # Percent of original size
#         width = int(img_retry_2.shape[1] * scale_percent / 100)
#         height = int(img_retry_2.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         img_retry_2 = cv2.resize(img_retry_2, dim, interpolation = cv2.INTER_AREA)

#         # Display the resulting image
#         cv2.imshow('image', img_retry_2)

#         # Wait until the "0" key is pressed to close the image window
#         cv2.waitKey(0)

#         # --------------------------------------------------------
#         # UNCOMMENT TO SAVE IMAGE
#         # Change the current directory to specified directory 
#         os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3/FALSE_DETECTIONS")
            
#         # Saving the image
#         cv2.imwrite('FALSE_ArucoMarkerDetected_011.JPG', img_retry_2)
#         # --------------------------------------------------------

#         # Remove image from memory
#         cv2.destroyAllWindows()    

#     else:
#         # Image downscaling -> DOES NOT AFFECT DETECTOR (NOT USED AS INPUT)
#         scale_percent = 30 # Percent of original size
#         width = int(img_retry_2.shape[1] * scale_percent / 100)
#         height = int(img_retry_2.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         img_retry_2 = cv2.resize(img_retry_2, dim, interpolation = cv2.INTER_AREA)

#         # Display the resulting image
#         cv2.imshow('image', img_retry_2)

#         # Wait until the "0" key is pressed to close the image window
#         cv2.waitKey(0)

#         # --------------------------------------------------------
#         # UNCOMMENT TO SAVE IMAGE
#         # Change the current directory to specified directory 
#         os.chdir("/home/kevin/IMAV2023/Aruco_Marker_Pictures/04_07_2023/Results/Storey3/NOT_DETECTED")
            
#         # Saving the image
#         cv2.imwrite('NOT_ArucoMarkerDetected_011.JPG', img_retry_2)
#         # --------------------------------------------------------

#         # Remove image from memory
#         cv2.destroyAllWindows()

