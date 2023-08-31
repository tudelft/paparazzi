import sys
import numpy as np
from os import path, getenv
import time
import itertools

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC =  getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


ref_lat = 50.909553
ref_long = 6.227969
deg2m = 111.139
with open('./sw/ground_segment/python/waypoints.txt', 'r') as file:
    longitude = []
    latitude = []
    altitude = []
    for line in file:
        values = line.strip().split()
        latitude.append(float(values[0]))
        longitude.append(float(values[1]))
        altitude.append(float(values[2]))
home_long = ref_long*np.ones(len(longitude))
home_lat = ref_lat*np.ones(len(latitude))

y = deg2m*(longitude-home_long)
x = deg2m*(latitude-home_lat)
waypoints = np.column_stack((x, y, altitude))

distances = []
for i in range(len(x)):
    row = []
    for j in range(len(x)):
        distance = ((x[j]-x[i])**2 + (y[j]-y[i])**2)**0.5
        row.append(distance)
    distances.append(row)

optimal_distance = float('inf')
optimal_order = []

permutations = itertools.permutations(range(len(x)))

for order in permutations:
    total_distance = 0
    prev = order[-1]
    for i in order:
        total_distance += distances[prev][i]
        prev = i


    if total_distance < optimal_distance:
        optimal_distance = total_distance
        optimal_order = order


lat = [latitude[i] for i in optimal_order]
long = [longitude[i] for i in optimal_order]
alt = [altitude[i] for i in optimal_order]

_interface = IvyMessagesInterface("Tiltprop Waypoints")

for i in range(len(x)):
    msgw = PprzMessage("ground", "MOVE_WAYPOINT")
    msgw['ac_id'] = 2
    msgw['wp_id'] = i + 4
    msgw['lat'] = lat[i]
    msgw['long'] = long[i]
    msgw['alt'] = alt[i]

    _interface.send(msgw)
    time.sleep(0.5)

#TODO: check if waypoints are confirmed!