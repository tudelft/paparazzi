import sys
import numpy as np
from os import path, getenv
import time
import itertools
import math as m

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC =  getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


ref_lat = 50.910595
ref_long = 6.227356
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

optimal_distance = float('inf')
optimal_order = []

permutations = itertools.permutations(range(len(x)))

for order in permutations:
    total_distance = 0
    total_distance = total_distance + m.sqrt(waypoints[order[0]][0]**2+waypoints[order[0]][1]**2)
    for i in range(3):
        idxcurrent = order[i]
        idxnext = order[i+1]
        total_distance += m.sqrt((waypoints[idxcurrent][0]-waypoints[idxnext][0])**2 + (waypoints[idxcurrent][1]-waypoints[idxnext][1])**2)
        prev = i
    total_distance += + m.sqrt(waypoints[order[-1]][0]**2+waypoints[order[-1]][1]**2)

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
