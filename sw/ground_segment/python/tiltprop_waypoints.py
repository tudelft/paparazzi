import sys
import numpy as np
from os import path, getenv
import time


PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

ref_lat = 50.909553
ref_long = 6.227969
deg2m = 111.139
with open('waypoints.txt', 'r') as file:
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

_interface = IvyMessagesInterface("Tiltprop Waypoints")

for i in range(len(x)):
    msgw = PprzMessage("datalink", "MOVE_WP")
    msgw['wp_id'] = i
    msgw['ac_id'] = 2
    msgw['lat'] = latitude[i]
    msgw['long'] = longitude[i]
    msgw['alt'] = altitude[i]

    _interface.send(msgw)
    time.sleep(0.5)