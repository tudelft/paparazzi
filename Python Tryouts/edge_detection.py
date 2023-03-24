import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.ndimage import convolve
from floor_detection import ground_decision_tree

# file = 'AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/25181969.jpg'
file_01 = 'AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/41815146.jpg'
file = 'test.jpg'

assert os.path.exists(file)

# standard edge detection
ridge_01 = np.array([[0,-1,0],
                  [-1,4,-1],
                  [0,-1,0]])

ridge_02 = np.array([[-1,-1,-1],
                  [-1,8,-1],
                  [-1,-1,-1]])

# Sobel Operators
sobel_x = np.array([[1,0,-1],
                  [2,0,-2],
                  [1,0,-1]])
sobel_y = sobel_x.T

hor_filter = np.array([[1,1,1],
                  [0,0,0],
                  [-1,-1,-1]])

ver_filter = np.array([[1,0,1],
                  [1,0,1],
                  [1,0,1]])

im = cv2.imread(file)
# sub_width = 25
# im_sub = im[:,260-sub_width:260+sub_width,:]
def contour_finder(im):
    ground = ground_decision_tree(im)[1]
    ground = np.float32(ground)

    

    # ground_edges = convolve(ground,ridge_01)

    # ground_edges[ground_edges!=0] = 1

    ground[ground != 0] = 1
    ground = cv2.convertScaleAbs(ground)
    # plt.imshow(ground)
    # plt.show()

    contours, order = cv2.findContours(ground, 1,2)
    # Sort the contours by length
    cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
  

    contour = np.zeros(im.shape)
    cv2.drawContours(contour, cntsSorted,0, (0,255,0), 3)
    contour[-6:-1,:,1] = 0
    contour[-1,:,1] = 0

    middle = im.shape[1]//2
    vector_str = contour[:,middle,1]
    # more efficient if traverse wrong way around
    pixel = 0
    for i, val in enumerate(vector_str):
        if int(val)!=0 and i>pixel:
            pixel = i


    cv2.drawContours(im, cntsSorted, 0, (0,255,0), 3)


    # try to find corners
    line1 = contour[:,0,1]
    pix_val1 = line1[0]
    i = 0
    while pix_val1 == 0 and i<241:
        i+=1
        pix_val1 = line1[i]

    line2 = contour[:,-2,1]
    pix_val2 = line2[0]
    j = 0
    while pix_val2 == 0 and j<241:
        j+=1
        pix_val2 = line2[j]


    return contour, [0,i], [contour.shape[1]-1, j]

def MSE_hor(contour, hor_coord_x, hor_coord_y, step = 10, threshold = 10):
    '''Inputs: contour has contours of ground
    hor_coord is x and y coordinates of end points of estimated horizon
    step is at what amount of pixels we check the correspondence of error and horizon'''

    x_1, x_2 = hor_coord_x[0],hor_coord_x[1]
    y_1, y_2 = hor_coord_y[0],hor_coord_y[1]
    points = [[x_1,y_1]]
    for x in range(x_1,x_2, step):
        y = int(y_1 + (y_2-y_1)/(x_2-x_1)*x)
        points.append([x,y])
    points.append([x_2,y_2])
    mse= 0
    for point in points:
        pixelfound = False
        for i,pixel in enumerate(contour[:,int(point[0]),1]):
            
            if pixel != 0 and not pixelfound:
                mse += (i - point[1])**2
                pixelfound = True
                

    object_det = False
    if mse > threshold:
        object_det = True
    return mse, object_det


# contour, coord1, coord2 = contour_finder(im_sub)
# mse, obj = MSE_hor(contour,(coord1[0], coord2[0]),(coord1[1], coord2[1]))

im = cv2.imread(file)
sub_width = 25

sub_images = []
for i in range(sub_width,im.shape[1],2*sub_width):
    sub_images.append(im[:,i-sub_width:i+sub_width,:])


for sub_image in sub_images:
    contour, coord1, coord2 = contour_finder(sub_image)
    mse, obj = MSE_hor(contour,(coord1[0], coord2[0]),(coord1[1], coord2[1]))

    if obj == True:
        plt.title('Object Detected')
        
    else:
        plt.title('No Object Detected')
    plt.imshow(sub_image)
    plt.show()





















def clean_up(im, min_size = 16):
    '''Cleans up black and white image with vals = 0 or 255'''

    kernel = np.ones((min_size,min_size))/(min_size**2)
    out = convolve(im,kernel)
    out[out<255] = 0
    out[out>254] = 255
    return out

def edge_mom(im_processed):
    avg_grad = [0,0]

    edges = convolve(im_processed,ridge_01)

    # find horizon (longest line)
    sensitivity = 3
    indices = np.where(edges != 0)



def fit_horizon(countour):
    step_radius = 10
    # first find horizontal pos




# cleaned = clean_up(ground)
plt.figure()
# plt.imshow(cleaned, cmap='gray')
# plt.show()



def ogcode():
    image_og = cv2.imread(file)

    #reduce and see if it still works
    # image = cv2.resize(image_og,(24,52))

    image_gray = cv2.cvtColor(image_og,cv2.COLOR_BGR2GRAY)
    image_hsv = cv2.cvtColor(image_og, cv2.COLOR_BGR2HSV)
    image_sat = np.copy(image_hsv)
    image_sat[:,:,1] = image_sat[:,:,1]+20
    image_sat[image_sat>255] = 255
    plt.imshow(image_sat)
    # low = np.array([0, 0, 0])
    # high = np.array([215, 51, 51])

    # mask = cv2.inRange(img, low, high)

    image_gray[image_gray<70] = 0
    image_blur = cv2.GaussianBlur(image_gray,(5,5),0)
    image_filtered = convolve(image_gray,ridge_01)

    # filter edges a bit
    image_filterd_blur = cv2.GaussianBlur(image_filtered, (5,5),0)
    image_gray[image_gray>70] = 255
    image_edges = convolve(image_gray,ridge_01)

    image_edges = image_edges*255
    image_og[:,:,0] = image_og[:,:,0] + image_edges
    image_og[:,:,1][image_edges != 0] = 0
    image_og[:,:,2][image_edges != 0] = 0
    image_rgb = cv2.cvtColor(image_og, cv2.COLOR_BGR2RGB)

    plt.imshow(image_og)
    plt.show()
    # cv2.imshow('grayscale',image_gray)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


    # plt.figure()
    # plt.imshow(image_gray, cmap='gray')
    # plt.figure()
    # plt.imshow(image_filtered)
    # plt.figure()
    # plt.imshow(image_edges)
    # plt.show()