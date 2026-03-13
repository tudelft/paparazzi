import cv2
import numpy as np
import glob




images = glob.glob( './data/*c.jpg', recursive=True)
print('images=',images)

for f in images:
    img = cv2.imread(f)
    h,w,d = img.shape
    print('img=',f,'lbl=' ,w,'x',h)
    
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    integral = cv2.integral(greyscale)
    print('integral=',integral)