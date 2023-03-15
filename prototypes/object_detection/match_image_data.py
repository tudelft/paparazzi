import numpy as np
import pandas as pd
import os
import cv2


def convert_csv_2_dict(datafile: str) -> dict:
    """
    converts csv files of the state of the drone into dictionaries
    """
    df = pd.read_csv(datafile)
    dict_data = dict()
    for col in df.columns:
        dict_data[col] = np.array(df[col].array)
    return dict_data

def load_images_from_folder(folder: str) -> dict:
    """
    loads the images from the folder in to dictionary consisting of the images and the timestamps
    """
    images = {"images": [], "timestamps": []}
    images_list = []
    timestamps_list = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        timestamp = get_timestamp_image(filename)
        if img is not None:
            images_list.append(img)
            timestamps_list.append(timestamp)
    sorted_timestamps = np.sort(timestamps_list)
    images_arr = np.array(images_list)
    sorted_images = images_arr[np.argsort(timestamps_list)]
    images["images"] = sorted_images
    images["timestamps"] = sorted_timestamps
    return images

def get_timestamp_image(filename: str) -> float:
    timestamp_ns = int(filename.split(".")[0])
    timestamp = timestamp_ns * 10**-6
    return timestamp

def get_drone_state(data_dict: dict, timestamp_img: float) -> dict:
    """
    puts the corresponding state of the drone to the given image into a dictionary
    """
    next_data_index = np.min(np.argwhere(timestamp_img - data_dict["time"] < 0))
    factor_right = (timestamp_img - data_dict["time"][next_data_index-1])/(data_dict["time"][next_data_index] - data_dict["time"][next_data_index-1])
    image_data = {}
    for key, value in data_dict.items():
        image_data[key] = (1 - factor_right) * value[next_data_index-1] + (factor_right) * value[next_data_index]
    return image_data
