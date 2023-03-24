import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

# file = 'AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/25181969.jpg'
file = 'AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/41815146.jpg'
assert os.path.exists(file)

# image = 
# def reduce_size(im, vert_pix = 24, hor_pix = 52):
#     ''' input from Bebop has resolution: 240 x 520'''
#     out_im = np.zeros((24,52))
#     i = 4
#     j = 4
#     sub_arr = im[j-5:j+5, i-5:i+5,:]
#     print(sub_arr)
#     # for i_count, i in enumerate(range(4,235, 10)):
#     #     for j_count, j in enumerate(range(4,515,10)):
#     #         sub_arr = im[j-5:j+5, i-5:i+5,:]

im = cv2.imread(file)

plt.figure()
# reduce image size for viola jones
im_red = cv2.resize(im, (24,52))
rgb = cv2.cvtColor(im_red, cv2.COLOR_BGR2RGB)

yuv = cv2.cvtColor(im_red, cv2.COLOR_RGB2YUV)
# plt.imshow(yuv)
# plt.show()

ones = np.ones((4,4))
print(np.triu(ones))
def integral_image(im_reduced):
    ''' Work out on OneNote'''
    int_im = np.zeros(im_reduced.shape)

    upper_triangle_1 = np.triu(np.ones(int_im.shape[1]))
    upper_triangle_2 = np.triu(np.ones(int_im.shape[0]))
    # print(upper_triangle_1)

    im_Y = im_reduced[:,:,0]
    im_U = im_reduced[:,:,1]
    im_V = im_reduced[:,:,2]

    int_im_Y = np.matmul(np.matmul(im_Y, upper_triangle_1).T,upper_triangle_2).T
    int_im_U = np.matmul(np.matmul(im_U, upper_triangle_1).T,upper_triangle_2).T
    int_im_V = np.matmul(np.matmul(im_V, upper_triangle_1).T,upper_triangle_2).T
    print(int_im_U)
    int_m = np.stack([int_im_Y,int_im_U,int_im_V])
    
    return int_im

def filter_color(image_name = file, y_low = 83, y_high = 110, \
                 u_low = 100, u_high = 130, v_low = 140, v_high = 200, resize_factor=10):
    im = cv2.imread(image_name);
    im = cv2.resize(im, (int(im.shape[1]/resize_factor), int(im.shape[0]/resize_factor)));
    YUV = cv2.cvtColor(im, cv2.COLOR_BGR2YUV);
    Filtered = np.zeros([YUV.shape[0], YUV.shape[1]]);
    for y in range(YUV.shape[0]):
        for x in range(YUV.shape[1]):
            if(YUV[y,x,0] >= y_low and YUV[y,x,0] <= y_high and \
               YUV[y,x,1] >= u_low and YUV[y,x,1] <= u_high and \
               YUV[y,x,2] >= v_low and YUV[y,x,2] <= v_high):
                Filtered[y,x] = 1;
    
    plt.figure();
    RGB = cv2.cvtColor(im, cv2.COLOR_BGR2RGB);
    plt.imshow(RGB);
    plt.title('Original image');
    
    plt.figure()
    plt.imshow(Filtered);
    plt.title('Filtered image');

def detect_horizon_line(image_grayscaled):
    """Detect the horizon's starting and ending points in the given image
    The horizon line is detected by applying Otsu's threshold method to
    separate the sky from the remainder of the image.
    :param image_grayscaled: grayscaled image to detect the horizon on, of
     shape (height, width)
    :type image_grayscale: np.ndarray of dtype uint8
    :return: the (x1, x2, y1, y2) coordinates for the starting and ending
     points of the detected horizon line
    :rtype: tuple(int)
    """

    msg = ('`image_grayscaled` should be a grayscale, 2-dimensional image '
           'of shape (height, width).')
    assert image_grayscaled.ndim == 2, msg
    image_blurred = cv2.GaussianBlur(image_grayscaled, ksize=(3, 3), sigmaX=0)

    _, image_thresholded = cv2.threshold(
        image_blurred, thresh=0, maxval=1,
        type=cv2.THRESH_BINARY+cv2.THRESH_OTSU
    )
    image_thresholded = image_thresholded - 1
    image_closed = cv2.morphologyEx(image_thresholded, cv2.MORPH_CLOSE,
                                    kernel=np.ones((9, 9), np.uint8))

    horizon_x1 = 0
    horizon_x2 = image_grayscaled.shape[1] - 1
    horizon_y1 = max(np.where(image_closed[:, horizon_x1] == 0)[0])
    horizon_y2 = max(np.where(image_closed[:, horizon_x2] == 0)[0])

    return horizon_x1, horizon_x2, horizon_y1, horizon_y2
image = integral_image(rgb)
# im_rot = cv2.rotate(im,rotateCode=2)

# print(image)
# filter_color()
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
hor_x1,hor_x2,hor_y1,hor_y2 = detect_horizon_line(gray)
print(hor_x1, hor_x2,hor_y1,hor_y2)
plt.imshow(hsv)
plt.plot([hor_x1,hor_x2],[hor_y1,hor_y2])
plt.show()

