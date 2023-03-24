import cv2
import matplotlib as plt
import numpy as np
import os 

# positive samples
folder_pos = '/home/korneel/vscode/AE4317_2019_datasets/Corner_Poles'
target_pos = '/home/korneel/vscode/AE4317_2019_datasets/ViolaJones/positive_reduced'
# negative samplse
folder_neg = '/home/korneel/vscode/AE4317_2019_datasets/Without_Corner_poles'
target_neg = '/home/korneel/vscode/AE4317_2019_datasets/ViolaJones/negative_reduced'

def reduce_files(folder_og, folder_tg, width = 24, height = 52):
    '''Reduces all files in a source folder and saves them in a target folder.
    folder_og: source folder
    folder_tg: target folder'''

    for file in os.listdir(folder_og):
        image = cv2.imread(os.path.join(folder_og,file))
        image_red = cv2.resize(image, (width,height))

        os.chdir(folder_tg)
        cv2.imwrite(str(file),image_red)
        os.chdir(folder_og)

# reduce_files(folder_pos, target_pos)
# reduce_files(folder_neg, target_neg)

def generate_info(folder_pos, folder_neg):
    for img in os.listdir(folder_pos):

        line = folder_pos+'/'+img+' 1 0 0 24 52\n'
        with open('/home/korneel/vscode/AE4317_2019_datasets/ViolaJones/info.dat','a') as f:
            f.write(line)
    for img in os.listdir(folder_pos):
        line = folder_neg+'/'+img+'\n'
        with open('/home/korneel/vscode/AE4317_2019_datasets/ViolaJones/bg.txt','a') as f:
            f.write(line)

# generate_info(target_pos, target_neg)

# todo: create samples
#       train cascades
#       check object detection