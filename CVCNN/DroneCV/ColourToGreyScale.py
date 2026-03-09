import cv2
import numpy as np
import glob
import os

# input and output folders
input_folder = r'C:\Users\willi\Documents\DroneCV\real_depth'
output_folder = r'C:\Users\willi\Documents\DroneCV\real_depth_gray'

# create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# get all PNG images
files = glob.glob(os.path.join(input_folder, '*.png'))

for f in files:
    # read color image
    img = cv2.imread(f)  # BGR

    # convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # optional: invert if closer = brighter
    #gray_inverted = 255 - gray

    # optional: normalize to 0-255 for better contrast
    gray_norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)

    # new filename in output folder
    base_name = os.path.basename(f)  # keeps original filename
    out_path = os.path.join(output_folder, base_name.replace('.png','_gray.png'))

    # save grayscale duplicate
    cv2.imwrite(out_path, gray_norm)

print("Done! Grayscale duplicates saved in:", output_folder)