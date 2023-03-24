#  First implement decision tree from learning classifier notebook

# from image_processing import filter_color
import cv2
import matplotlib.pyplot as plt
import numpy as np

import os
from scipy.ndimage import convolve

# file = 'AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/25181969.jpg'
file = 'AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/41815146.jpg'
file = 'test.jpg'
assert os.path.exists(file)

im = cv2.imread(file)

# filter_color(im,y_low=0,y_high=0,u_low=0,u_high=115.5,v_low=0,v_high=145)

def ground_decision_tree(im):
    # # u has to be smaller than 115.5
    # im[:,:,1][im[:,:,1]<= 115.5] = 0
    # im[:,:,2][im[:,:,1]>= 115.5] = 255
    # im[:,:,0][im[:,:,1]<= 115.5] = 0
    im = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)

    indices = np.where((im[:,:,1]<=115.5) & (im[:,:,2]<=145))
    # v smaller than 145
    im[:,:,1][indices] = 255
    im[:,:,2][indices] = 128
    im[:,:,0][indices] = 128
    mask = np.zeros((im.shape[0],im.shape[1]))
    mask[indices[0],indices[1]] = 255
    mask[mask<254] = 0
    
    # im[:,:,1][np.where((im[:,:,2]<=145.5))] = 255
    # im[:,:,2][np.where((im[:,:,2]<=145.5))] = 128
    # im[:,:,0][np.where((im[:,:,2]<=145.5))] = 128

    # im[im[:,:,1]>=115.5 and im[:,:,2]>=145] = 0
    # im_v = im[im_u]
    im = cv2.cvtColor(im, cv2.COLOR_YUV2RGB)
    return im, mask

def isophote(im):
    pass
# im_ground = ground_decision_tree(im)[0]
# mask = ground_decision_tree(im)[1]
# plt.figure();
# plt.imshow(mask);
# # plt.title('Original image');
# plt.show()