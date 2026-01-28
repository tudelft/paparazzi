import numpy as np
import matplotlib.pyplot as plt
import random
import time
from shapely.geometry import Polygon, Point, LineString
import numpy as np
import math
import os
import sys
import xml.etree.ElementTree as ET
import pymap3d as pm
from scipy.special import expit  # stable sigmoid


USE_PPRZ = False  # set True if you want to send to Paparazzi (not configured here)
RANDOM_SEED = 112
np.random.seed(RANDOM_SEED)
random.seed(RANDOM_SEED)

# --- Paparazzi home ---

################## Communication example part ###########################################################

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

# sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# --- ENU origin (from flight plan) ---
lat0, lon0, alt0 = 52.1681551, 4.4126468, 0.0

# --- Parse waypoints from XML (same as you had) ---
xml_file = os.path.expanduser("~/paparazzi2/paparazzi/conf/flight_plans/tudelft/rotwing_EHVB_Damian.xml")
tree = ET.parse(xml_file)
root = tree.getroot()

waypoints = {}
for wp in root.findall(".//waypoint"):
    name = wp.attrib.get("name")
    if "lat" in wp.attrib and "lon" in wp.attrib:
        lat, lon = float(wp.attrib["lat"]), float(wp.attrib["lon"])
        alt = float(wp.attrib.get("alt", 0.0))
        x, y, z = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
        waypoints[name] = (x, y, z)
    elif "x" in wp.attrib and "y" in wp.attrib:
        x, y = float(wp.attrib["x"]), float(wp.attrib["y"])
        z = float(wp.attrib.get("z", 0.0))
        waypoints[name] = (x, y, z)

ehvb_xy = np.array([waypoints[wp][:2] for wp in ["C1","C2","C3","C4","C5","C6","C7","C8","C9"]])
softgeo_xy = np.array([waypoints[wp][:2] for wp in ["S1","S2","S3","S4","S5","S6","S7","S8","S9"]])

soft_poly = Polygon(softgeo_xy)
boundary_margin = 20
soft_poly_inner = soft_poly.buffer(-boundary_margin)  # e.g., boundary_margin = 20–30 m

# Victims
amount_of_victims = 8

def generate_victims_inside_polygon(polygon, n_victims):
    victims = []
    minx, miny, maxx, maxy = polygon.bounds
    while len(victims) < n_victims:
        x = np.random.uniform(minx, maxx)
        y = np.random.uniform(miny, maxy)
        pt = Point(x, y)
        if polygon.contains(pt):
            victims.append(np.array([x, y]))
    return victims

safe_poly = soft_poly_inner.buffer(-10.0)
victims = generate_victims_inside_polygon(safe_poly, n_victims=10)
victims = np.array(victims)  # shape will be (n_victims, 2)



detected_victims = np.zeros(len(victims), dtype=bool)

# Grid
grid_res_fine = 5.0
margin = 20.0
min_x, min_y, max_x, max_y = soft_poly.bounds[0]-margin, soft_poly.bounds[1]-margin, soft_poly.bounds[2]+margin, soft_poly.bounds[3]+margin
xv, yv = np.meshgrid(np.arange(min_x, max_x, grid_res_fine),
                     np.arange(min_y, max_y, grid_res_fine))

# ---------------------------
# UAV initial state
# ---------------------------

# Base probability map (uniform prior, 0.5 everywhere)
p0 = 0.5
l0 = np.log(p0 / (1 - p0))   # log-odds(0.5) = 0.0
l_map = np.zeros_like(xv, dtype=float) + l0

# --- Parameters ---
BASE_PROB = 0.5
BASE_LOGODDS = np.log(BASE_PROB / (1 - BASE_PROB))
FALSE_ALARM_BASE = 0.02       # min p(z=1|free)
FALSE_ALARM_EDGE = 0.08       # higher near geofence edges
P_HIT_AT_ZERO = 0.9
SENSOR_SIGMA = 40.0
LOGODDS_CLIP = 50.0           # numerical bound