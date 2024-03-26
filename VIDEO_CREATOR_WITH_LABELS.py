import os
import cv2
import pandas as pd
from datetime import datetime, timedelta
import imageio
import numpy as np


# Define the paths to your image folder and CSV file
image_folder_path = '/tmp/paparazzi/images/20240326-154548'
csv_file_path = '/tmp/paparazzi/log/20240326-154615.csv'

# Load the CSV file containing the optical flow data
optical_flow_data = pd.read_csv(csv_file_path)

optical_flow_data['time'] = optical_flow_data['time'].astype(str)


def calculate_rms(values):
   return np.sqrt(np.mean(np.square(values)))


# Function to extract timestamp from image filename
def extract_timestamp(image_filename):
   timestamp_ms_str = image_filename.split('.')[0]
   timestamp_sec = float(timestamp_ms_str)
   return timestamp_sec


# Convert CSV timestamp to seconds since the Unix epoch for comparison
def csv_timestamp_to_seconds(csv_timestamp):
   epoch_time = csv_timestamp.replace(".", '')
   return float(epoch_time)


# Pre-calculate all the CSV timestamps in seconds to speed up processing
optical_flow_data['timestamp_seconds'] = optical_flow_data['time'].apply(csv_timestamp_to_seconds)


# Iterate over the images in the folder
for image_filename in sorted(os.listdir(image_folder_path)):
    if image_filename.endswith('.jpg'):
        image_path = os.path.join(image_folder_path, image_filename)
        image_timestamp_seconds = extract_timestamp(image_filename)


        optical_flow_data['time_difference'] = optical_flow_data['timestamp_seconds'].apply(
            lambda x: abs(x - image_timestamp_seconds))
        closest_row_index = optical_flow_data['time_difference'].idxmin()

        closest_row = optical_flow_data.iloc[closest_row_index]

        # Adjust the coordinates for the text
        text_1 = f"CCount: {closest_row['color_count']}"
        text_2 = f"FCount: {closest_row['floor_count']}"
        text_3 = f"FCENTRAL: {closest_row['floor_count_central']}"
        text_4 = f"PLANT: {closest_row['plat_count']}"
        text_5 = f"HEADING: {closest_row['heading']}"
        text_6 = f"NAV: {closest_row['navigation_state_msg']}"
        text_7 = f"CFC: {closest_row['central_floor_count_threshold']}"

        # Load the image
        img = cv2.imread(image_path)


        # Rotate the image by 90 degrees clockwise
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)


        # Overlay the optical flow data onto the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5  # Smaller font size
        thickness = 2
        height, width = img.shape[:2]

        # Calculate text size for proper alignment
        text_width, _ = cv2.getTextSize(text_1, font, font_scale, thickness)

        # Adjust the coordinates for the text
        text_x = 10
        text_y_spacing = 30  # Adjust the spacing between each text line

        # Overlay each text variable on the image
        cv2.putText(img, text_1, (text_x, text_y_spacing * 1), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(img, text_2, (text_x, text_y_spacing * 2), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(img, text_3, (text_x, text_y_spacing * 3), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(img, text_4, (text_x, text_y_spacing * 4), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(img, text_5, (text_x, text_y_spacing * 5), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(img, text_6, (text_x, text_y_spacing * 6), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(img, text_7, (text_x, text_y_spacing * 7), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)

        # Save or display the image
        cv2.imwrite(os.path.join(image_folder_path, f'modified_{image_filename}'), img)


# VIDEO GENERATION
frame_size = (width, height)  # Specify the frame size

images = []
output_video_path = '/tmp/paparazzi/New_testoutput_video.mp4'
for image_filename in sorted(os.listdir(image_folder_path)):
   if image_filename.startswith('modified_') and image_filename.endswith('.jpg'):
       image_path = os.path.join(image_folder_path, image_filename)
       images.append(imageio.imread(image_path))


writer = imageio.get_writer(output_video_path, fps=5)


for img in images:
   writer.append_data(img)


writer.close()



