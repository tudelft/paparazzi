import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from shapely.geometry import Polygon, Point, LineString
import os
import sys
import xml.etree.ElementTree as ET
import pymap3d as pm

# ======================================================
# 1. LOAD SATELLITE IMAGE
# ======================================================
sat_path = "/home/shivesh/paparazzi2/paparazzi/sw/ground_segment/python/Occupancy_Map/EHVB_sat.png"
sat_img = Image.open(sat_path)

# ======================================================
# 2. LOAD PAPARAZZI WAYPOINTS
# ======================================================
PPRZ_HOME = os.getenv(
    "PAPARAZZI_HOME",
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
)
sys.path.append(PPRZ_HOME + "/sw/ext/pprzlink/lib/v1.0/python")

lat0, lon0, alt0 = 52.1681551, 4.4126468, 0.0
xml_file = os.path.expanduser("~/paparazzi2/paparazzi/conf/flight_plans/SDB/sdb_rotwing_EHVB.xml")

tree = ET.parse(xml_file)
root = tree.getroot()

waypoints = {}
for wp in root.findall(".//waypoint"):
    name = wp.attrib.get("name")
    if name is None:
        continue

    # Use ENU conversion when needed
    if "lat" in wp.attrib and "lon" in wp.attrib:
        lat, lon = float(wp.attrib["lat"]), float(wp.attrib["lon"])
        alt = float(wp.attrib.get("alt", 0.0))
        x, y, z = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
    else:
        x, y = float(wp.attrib["x"]), float(wp.attrib["y"])
        z = float(wp.attrib.get("z", 0.0))

    waypoints[name] = (x, y, z)


# ======================================================
# 3. DEFINE SOFTGEOFENCE POLYGON
# ======================================================
softgeo_xy = np.array([waypoints[wp][:2] for wp in ["S1","S2","S3","S4","S5","S6","S7","S8","S9"]])
soft_poly = Polygon(softgeo_xy)


# ======================================================
# 4. CREATE GRID INSIDE POLYGON
# ======================================================
x_min, y_min, x_max, y_max = soft_poly.bounds

grid_resolution = 10.0
grid_x = np.arange(x_min, x_max + grid_resolution, grid_resolution)
grid_y = np.arange(y_min, y_max + grid_resolution, grid_resolution)

XX, YY = np.meshgrid(grid_x, grid_y)
grid_points_all = np.column_stack([XX.ravel(), YY.ravel()])

inside_mask = np.array([soft_poly.contains(Point(p)) for p in grid_points_all])
grid_points = grid_points_all[inside_mask]

belief = 0.5 * np.ones(len(grid_points))   # uniform belief for visualization


# ======================================================
# 5. PER-EDGE TRIMMING
# ======================================================
# Format: (("WP1","WP2"), trim_distance)
trim_edges = [
    (("S2", "S1"), 15.0),
    (("S1", "S9"), 25.0),
    (("S9", "S8"), 35.0),
    (("S8", "S7"), 30.0),
    (("S7", "S6"), 15.0),
    (("S6", "S5"), 10.0),
    (("S5", "S4"), 10.0)
]

trim_mask = np.ones(len(grid_points), dtype=bool)

for (wpA, wpB), trim_dist in trim_edges:
    A = np.array(waypoints[wpA][:2])
    B = np.array(waypoints[wpB][:2])
    edge_line = LineString([A, B])

    # distance of every grid point to this edge
    d = np.array([edge_line.distance(Point(p)) for p in grid_points])

    # mask out points closer than trim_dist
    trim_mask &= (d > trim_dist)

# Apply trimming
grid_points = grid_points[trim_mask]
belief = belief[trim_mask]


# ======================================================
# 6. PLOT
# ======================================================
plt.figure(figsize=(14, 10))

margin = 5

plt.imshow(
    np.flipud(sat_img),
    extent=[x_min-margin, x_max+margin, y_min-margin, y_max+margin],
    origin="lower",
    alpha=0.90
)

plt.scatter(
    grid_points[:,0],
    grid_points[:,1],
    c=belief,
    cmap="RdYlBu_r",
    s=14,
    alpha=0.9,
    edgecolors="none",
    vmin=0.0,
    vmax=1.0,
    label="Occupancy probability"
)

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Multi-UAV Probabilistic Occupancy Map over EHVB Test Area")
plt.colorbar(label="Occupancy probability")
plt.legend()
plt.grid(False)
plt.tight_layout()
plt.show()
