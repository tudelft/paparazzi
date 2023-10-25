from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np


y = np.asarray([-31.9311054713794,
271.783338767474,
-32.0712612077143,
354.330707476033,
6.54081246301252,
383.589318217342,
69.7688247447596,
371.79730057929,
82.4293454201474,
317.952569707543,
41.2149464895494,
283.464634824714,
-19.0602618491586,
254.205805061623,
-55.9858755301192,
184.452443674776,
-112.958411922747,
93.2285411270681,
-216.217248725545,
-67.4128541508385,
-207.285803430584,
-102.679390260187,
-178.800825545386,
-157.081574962641,
-176.129043959345,
-189.121590913013,
-195.051149417886,
-212.928357567232,
-230.080445408295,
-219.156908501874,
-249.845234939409,
-206.584775444945,
-258.565968964039,
-179.661912656902,
-232.887278638881,
-68.7471303050234,
-129.135408800651,
97.678909647592,
-69.3491451054297,
198.915079108026,
-31.9311054713794,
271.783338767474])

n = len(y)
y = y.reshape((int(n/2),2))
n = len(y)
x = range(0, n)

tck1 = interpolate.splrep(x, y[:,0], s=0.001, k=3)
tck2 = interpolate.splrep(x, y[:,1], s=0.001, k=3)

# 10000 = 4m/s
x_new = np.linspace(min(x), max(x), 10000)
y_fitx = interpolate.BSpline(*tck1)(x_new)
y_fity = interpolate.BSpline(*tck2)(x_new)

fit = np.vstack((y_fitx, y_fity)).T

#print(fit)

start = np.asarray([[-113.6], [67]])
#start = np.asarray([[-113.6], [87]])

print(start)

print(fit[:, 0] - start[0]**2)

distances = np.sqrt((fit[:, 0] - start[0])**2 + (fit[:, 1] - start[1])**2)
closest_index = np.argmin(distances)

print(closest_index)


plt.title("BSpline curve fitting")
plt.plot(y[:,1], y[:,0], 'ro', label="original")
plt.plot(y_fity[closest_index:], y_fitx[closest_index:], '-c', label="B-spline")
plt.plot(start[1], start[0], 'X', label="Start")
plt.legend(loc='best', fancybox=True, shadow=True)
plt.axis('equal')
plt.grid()
plt.show() 
