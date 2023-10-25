                                        # LIBRARY DEFINITION #
# ------------------------------------------------------------------------------------------------------- #
from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np

                                          # SPLINED TRACKS #
# ------------------------------------------------------------------------------------------------------- #
# --------- Aldenhoven Testing Centre Track --------- # 
racetrack_at_competition = np.asarray([-31.9311054713794,
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

# --------- Belgium Testing Track --------- # 
test_field_track = np.asarray([-81.9,
133.7,
-43.4,
80.7,
27.7,
116.7,
-28.7,
183.9,
-141.7,
199.9,
-81.9,
133.7])

# --------- Valkenburg Testing Track --------- # 
valkenburg_track = np.asarray([10.2766958604546,
17.1690562541347,
40.9424685964964,
30.0988525001902,
50.2341861676278,
0.946053855489006,
23.7829369045678,
-7.3437990762182,
10.2766958604546,
17.1690562541347])

# --------- Select Track --------- # 
y = valkenburg_track
# y = racetrack_at_competition
# y = test_field_track

# --------- Spline Track --------- # 
n = len(y)
y = y.reshape((int(n/2),2))
n = len(y)
x = range(0, n)

tck1 = interpolate.splrep(x, y[:,0], s=0.001, k=3)
tck2 = interpolate.splrep(x, y[:,1], s=0.001, k=3)

x_new = np.linspace(min(x), max(x), 1500)  # Important value -> 10000 = 4 m/s
y_fitx = interpolate.BSpline(*tck1)(x_new)
y_fity = interpolate.BSpline(*tck2)(x_new)

fit = np.vstack((y_fitx, y_fity)).T

                  # FUNCTIONS -> FIND CLOSEST POINT TO DRONE FROM SPLINED TRACK #
# ------------------------------------------------------------------------------------------------------- #
def find_closest_point( P ):
    global fit

    distances = np.sqrt((fit[:, 1] - P[0])**2 + (fit[:, 0] - P[1])**2)
    closest_index = np.argmin(distances)
    return closest_index

# --------- Track Starting Point --------- # 
start = np.asarray([[-113.6], [67]])
nr = find_closest_point(start)
print('Start at:', nr)

                                  # FUNCTIONS -> DEFINE ROUTE #
# ------------------------------------------------------------------------------------------------------- #
def route():
    global nr
    global y_fitx
    global y_fity

    nr -= 1      # Hack: Move along the track blindly

    if nr <= 0:
        nr = len(y_fitx) - 1

    zk = [y_fity[nr], y_fitx[nr], 0, 0 ]

    return zk

                        # FUNCTIONS -> DETERMINE DISTANCE TO POINT IN SPLINED TRACK #
# ------------------------------------------------------------------------------------------------------- #
def determine_distance():
    global fit
    dist = 0
    for i in range(fit.shape[0]-1):
        dist += np.sqrt((fit[i, 1] - fit[i+1, 1])**2 + (fit[i, 0] - fit[i+1, 0])**2)

    return dist

                                        # KALMAN FILTER #
# ------------------------------------------------------------------------------------------------------- #
x = np.asarray([start[0][0],
                start[1][0],
                0,
                0])

H = np.asarray([[1,  0, 0 ,0],
                [0,  1, 0 ,0]] )

K0 = 1e5
Kp = 1
Kv = 0.0001
Kpv = 0
Km = 1e5

Q = np.asarray([[Kp, 0, Kpv, 0],
                [0, Kp, 0, Kpv],
                [Kpv, 0, Kv, 0],
                [0, Kpv, 0 ,Kv]])
R = np.asarray([[Km],[Km]])
P = np.asarray([[K0, 0, 0, 0],
                [0, K0, 0, 0],
                [0, 0,  0, 0],
                [0, 0,  0, 0]])

KP = 0.6    # Important value -> Defines gain/aggresiveness of filter (higher value = measurements trusted more and vice versa)
KV = 0.01

def init( X0 ):
    global x

    N = X0[0]
    E = X0[1]
    D = X0[2]

    x = np.asarray([[N],
                [E],
                [0],
                [0]])
    
    print('x0 set to:', x)

vision_update_counter = 0

def predict(dt):
    global x
    global P
    global Q
    global vision_update_counter
    global nr

    # Do Kalman predict
    A = np.asarray([[1,  0,  dt,  0],
                    [0,  1,  0,   dt],
                    [0,  0,  1,   0],
                    [0,  0,  0,   1]] )

    x = A @ x
    P = ((A @ P) @ A.T) + Q

    # Keep track of the number of predictions since the last update
    vision_update_counter += 1

    # On timeout, predict that the car follows the track
    TIMEOUT_SEC = 5
    FPS = 15
    if vision_update_counter == (TIMEOUT_SEC * FPS):
        # For every ArUco: update the track to the closest point
        nr = find_closest_point(x)
        print(nr)

    if vision_update_counter >= (TIMEOUT_SEC * FPS):
        # Get the next position on the route as the next measurement
        z = route()

        # Update the filter as if we were following the track
        update(z, True)

    return x

def update(Z, route=False):
    global x
    global H
    global R
    global P
    global KP
    global KV
    global vision_update_counter

    if not route:
        vision_update_counter = 0

    zk = np.asarray([[Z[0]],
                     [Z[1]]])
    yk = zk - H@x
    S = H @ P @ H.T + R
    #Si = np.linalg.inv(S)
    #K = (P @ H.T) @ Si
    K = np.asarray([[KP, 0],[0, KP],[KV, 0],[0, KV]])
    x = x + (K @ yk)
    P = (np.eye(4) - (K @ H)) @ P

    return x