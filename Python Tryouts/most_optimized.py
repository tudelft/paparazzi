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

im = cv2.imread(file)
sub_width = 25
height = 75
sub_images = []
for i in range(sub_width,im.shape[1],2*sub_width):
    sub_images.append(im[240-height:-1,i-sub_width:i+sub_width,:])

O = 0

### START HERE ###
def find_next(im_bw, prev_points, h,w,step = 3, alpha = .8):
    '''Input should be image mask, give previous points such that it starts out in kinda the right direction.'''
    # if prev points is only the initial point, dir is [0,step]
    # global O
    # print(prev_points)
    prev_points = [prev_points[0],prev_points[0],*prev_points]

    dir = [int((prev_points[-3][0]*1+prev_points[-2][0]*2+prev_points[-1][0]*3)/6)-prev_points[-2][0],3*(len(prev_points)-2)]
    # if straight edge, we will overshoot and go to end point
    if abs(dir[0]/step)> 2:
        dir[0] = 0
    init_guess = [prev_points[-1][0] + dir[0], dir[1]]

    
    edge_found = False
    while not edge_found:
        # print(init_guess)
        if init_guess[0]+1<h:
            # print(init_guess[0], init_guess[1],h)
            # O+=1
            # if on edge, pixel above + pixel below == 1 if on ground == 2 if in air == 0
            if (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 1:
                edge_found = True

            elif (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 0:
                # its in the air
                init_guess[0] += 1

            else:
                # its on ground
                init_guess[0] -= 1
        else:
            return [h,init_guess[1]]
            # return [init_guess[0],init_guess[1]]

    return init_guess
def hor_tracer(sub_im, step = 5):
    h,w,_ = sub_im.shape

    # first we needmask
    ground = ground_decision_tree(sub_im)[1]
    ground = np.float32(ground)


    ground[ground != 0] = 1
    ground = cv2.convertScaleAbs(ground)
    plt.imshow(ground)
    plt.show()
    # start looking for ground
    ground_found = False
    i = 0

    # find first point
    col = 0
    while not ground_found and i < (h-5) and col < (w-5):
        if ground[i,col] != 0:
            area = 0
            for j in range(5):
                for k in range(5):
                    area+= ground[i+j,k]
            if area > 20:
                ground_found = True
        
        i+=1
        if i == h-5:
            i = 0
            col += 1
    start_row = i

    # find edge
    edge_dir_found = False
    hor_points = [[start_row,0]]
    dirs = []
    i = 0
    end = False
    while hor_points[-1][1]<= w-step and i < 100 and not end:
        next_point = find_next(ground, hor_points,h=h, w=w,step = 5)
        # print(next_point)
        # if next_point == False:
        #     end = True
        
        # else:
        hor_points.append(next_point)
        dirs.append(dir)
        plt.plot(next_point[1], next_point[0],'o',color='y')
        
        i+=1


    for point in hor_points:
        plt.plot(point[1],point[0], 'o', c='y')
    plt.imshow(sub_im)
    plt.show()
    return hor_points


def second_derivative(hor_points):
    # f''(x) ≈ [f(x+h) - 2f(x) + f(x-h)] / h^2
    '''This function uses the central difference formula to find the second derivatives'''
    delta_x = hor_points[1][1] - hor_points[0][1]
    y_lst = []
    for i, point in enumerate(hor_points):
        y_lst.append(point[0])

    fdd_lst = []
    for i,y in enumerate(y_lst[1:-1]):   
        # print(i,y_lst[i-1])
        fdd = (y_lst[i] - 2*y + y_lst[i+2])/(delta_x**2)
        # print(fdd)
        fdd_lst.append(fdd)
    return fdd_lst

def find_mom(fdd, thr = 1):
    count = 0
    for i in fdd:
        if  abs(i) >= thr:
            count+=1
    return count

def confidence(hor_points, im_height, thr = 5):
    y_lst = []
    for i, point in enumerate(hor_points):
        y_lst.append(point[0])
    
    left = True
    y_left = y_lst[0:len(y_lst)//2]
    y_right = y_lst[len(y_lst)//2:]

    if np.average(y_left) < np.average(y_right):
        left = True
    else:
        left = False


    min_y = max(y_lst)
    conf = (im_height-min_y)/im_height
    
    return conf, left


    


# hor_points = hor_tracer(sub_images[3])  
# fdd = second_derivative(hor_points)
# x_arr = np.arange(0,len(fdd))

# print(find_mom(fdd, thr=.5))
# print(confidence(hor_points, sub_images[3].shape[0]))
for sub_image in sub_images:
    hor_points = hor_tracer(sub_image)  
    fdd = second_derivative(hor_points)

    x_arr = np.arange(0,len(fdd))
    print(find_mom(fdd, thr=.5))
    print(confidence(hor_points, sub_image.shape[0]))



### END HERE ###




def find_next(im_bw, prev_points, step = 3, alpha = .8):
    # if prev points is only the initial point, dir is [0,step]
    global O
    # print(prev_points)
    prev_points = [prev_points[0],prev_points[0],*prev_points]
    '''
    # print(prev_points)
    # if len(prev_points) == 1:
    #     dir = [0,step]
    #     init_guess = [prev_points[-1][0] + dir[0], step]
    #     edge_found = False
    #     while not edge_found:
    #         O+=1
    #         # if on edge, pixel above + pixel below == 1 if on ground == 2 if in air == 0
    #         if (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 1:
    #             edge_found = True

    #         elif (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 0:
    #             # its in the air
    #             init_guess[0] += 1

    #         else:
    #             # its on ground
    #             init_guess[0] -= 1

    #     return init_guess
    
    # if len(prev_points) == 2:
    #     dir = [int((prev_points[0][0]*2+prev_points[1][0]*3)/5)-prev_points[0][0],6]
    #     init_guess = [prev_points[-1][0] + dir[0], dir[1]]
    #     edge_found = False
    #     while not edge_found:
    #         O+=1
    #         # if on edge, pixel above + pixel below == 1 if on ground == 2 if in air == 0
    #         if (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 1:
    #             edge_found = True

    #         elif (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 0:
    #             # its in the air
    #             init_guess[0] += 1

    #         else:
    #             # its on ground
    #             init_guess[0] -= 1

    #     return init_guess
    
    # else:
    # weighted average of last 3 points ((x_-3*1 + x_-2*2+x_-1*3)/6)''' 
    
    dir = [int((prev_points[-2][0]*1+prev_points[-2][0]*2+prev_points[-1][0]*3)/6)-prev_points[-2][0],3*(len(prev_points)-2)]
    init_guess = [prev_points[-1][0] + dir[0], dir[1]]

    edge_found = False
    while not edge_found:
        O+=1
        # if on edge, pixel above + pixel below == 1 if on ground == 2 if in air == 0
        if (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 1:
            edge_found = True

        elif (im_bw[init_guess[0], init_guess[1]] + im_bw[init_guess[0]-1, init_guess[1]] ) == 0:
            # its in the air
            init_guess[0] += 1

        else:
            # its on ground
            init_guess[0] -= 1

    return init_guess
















def find_mom_changes(horizon_points, rho = 0.6):
    y_lst = []
    for point in horizon_points:
        y_lst.append(point[0])
    S_prev = y_lst[0]
    S_lst = [S_prev]
    S_dir = [S_prev,S_prev,S_prev]
    S_avg_prev = S_prev
    # if abs(S_curr - S_prev) > 3 than 45 degree angle in S plot
    for i,y in enumerate(y_lst[1:-1]):
        # print(S_prev)
        S_curr = rho*S_prev + (1-rho)*y
        S_lst.append(S_curr)
        S_prev = S_curr

        # print(Delta_S)
        # if Delta_S>3:
        #     print('Momentum change detected! Corner found at:', 3*i)
            
         
    return S_lst

















# S_lst = find_mom_changes(hor_points)
# plt.plot(x_arr, S_lst)
# S_lst = find_mom_changes(hor_points, rho = .2)
# x_arr = np.linspace(0,50,len(S_lst))

# x_lst = []
# y_lst = []
# for i, point in enumerate(hor_points):
#     y_lst.append(point[0])
#     x_lst.append(point[1])

# from scipy.interpolate import UnivariateSpline

# y_spl = UnivariateSpline(x_lst,y_lst,s=0,k=4)

# plt.figure()
# plt.semilogy(x_lst,y_lst,'ro',label = 'data')
# x_range = np.linspace(x_lst[0],x_lst[-1],1000)

# plt.semilogy(x_range,y_spl(x_range))
# y_spl_2d = y_spl.derivative(n=2)
# plt.figure()
# plt.plot(x_range,y_spl_2d(x_range))

# plt.plot(x_arr, S_lst)
# fit_lines(hor_points)
# plt.show()



def is_green(sub_im, sensitivity = 2):
    '''Checks number of pixel rows fully green in subimage.'''
    w,h,_ = sub_im.shape

    ground = ground_decision_tree(sub_im)[1]

    

    # ground_edges = convolve(ground,ridge_01)

    # ground_edges[ground_edges!=0] = 1

    ground[ground != 0] = 1
    ground = cv2.convertScaleAbs(ground)

    # plt.imshow(ground)
    # plt.show()


is_green(sub_images[2])