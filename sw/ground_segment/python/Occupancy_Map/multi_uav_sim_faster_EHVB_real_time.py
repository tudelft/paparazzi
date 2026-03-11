import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, Point
import pymap3d as pm
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter
from shapely.affinity import rotate, translate
from scipy.ndimage import shift
from numba import jit
import time
import matplotlib.cm as cm
import threading
import math
import csv
from datetime import datetime

# --- PAPER EXPERIMENT TOGGLE ---
EXPERIMENT_MODE = "ENERGY_AWARE" # Change to "BASELINE" for the control run
SHOW_REAL_TIME_PLOT = True

if EXPERIMENT_MODE == "ENERGY_AWARE":
    lam = 0.5       # Planner cares about energy
else:
    lam = 0.0       # BASELINE: Planner ignores energy completely (Standard IPP)
    

# --- PAPARAZZI BRIDGE GLOBALS ---
UAVS = {}   # Stores state: {ac_id: {'state': {'x':..., 'y':...}}}
LOCK = threading.Lock()

# Scales for fixed-point INS messages (Standard Paparazzi)
POS_SCALE = 0.0039063
VEL_SCALE = 0.0000019
ANGLE_SCALE = 0.0139882  # <--- ADD THIS LINE

# --- Paparazzi setup ---
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))
sys.path.append(PPRZ_HOME + "/sw/ext/pprzlink/lib/v1.0/python")


lat0, lon0, alt0 = 52.1681551, 4.4126468, 0.0
LAT0, LON0, ALT0 =lat0, lon0, alt0 
xml_file = os.path.expanduser("~/paparazzi2/paparazzi/conf/flight_plans/SDB/sdb_bebop_EHVB.xml")
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

softgeo_xy = np.array([waypoints[wp][:2] for wp in ["S1","S2","S3","S4","S5","S6","S7","S8","S9"]])
soft_poly = Polygon(softgeo_xy)

x_min, y_min, x_max, y_max = soft_poly.bounds
grid_resolution = 10.0  

# 1. Create the axes
grid_x = np.arange(x_min, x_max + grid_resolution, grid_resolution)
grid_y = np.arange(y_min, y_max + grid_resolution, grid_resolution)
rows, cols = len(grid_y), len(grid_x)

# 2. Create the 2D Coordinate Grids (The replacement for grid_points)
# XX contains the x-coord for every pixel, YY contains the y-coord
XX, YY = np.meshgrid(grid_x, grid_y) 

# 3. Create Belief and Mask
belief_map = 0.5 * np.ones((rows, cols))
inside_mask = np.zeros((rows, cols), dtype=bool)

# 4. Generate Geofence Mask (Slow but run once)
print("Generating Geofence Mask...")
from matplotlib.path import Path as MplPath
# Use matplotlib Path for fast "point inside polygon" check on the whole grid
poly_path = MplPath(list(soft_poly.exterior.coords))
points_flat = np.vstack((XX.flatten(), YY.flatten())).T
mask_flat = poly_path.contains_points(points_flat)
inside_mask = mask_flat.reshape(rows, cols)

# 5. Apply to belief
belief_map[~inside_mask] = 0.5


from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

def sync_drone_state(d_idx, ac_id, default_pos):
    """
    Reads the latest position from the global UAVS dictionary.
    Thread-safe due to LOCK.
    """
    # 1. Access the global variable 'UAVS'
    global UAVS 
    
    # 2. Acquire the lock so we don't read while on_ins is writing
    with LOCK:
        if ac_id in UAVS:
            s = UAVS[ac_id]["state"]
            
            # Create a clean numpy array for the planner to use
            # We use max(0.2, z) to prevent the planner from thinking it's underground
            real_pos = np.array([s['x'], s['y'], max(0.2, s['z'])])
            
            # Get velocity for the smoothing filter
            real_vel = np.array([s['vx'], s['vy'], s['vz']])
            
            return real_pos, real_vel
            
    # 3. Fallback: If Paparazzi hasn't sent data yet, return the old/default position
    return default_pos, np.zeros(3)

def ensure_uav(ac_id):
    """Creates a new UAV entry if seen for the first time."""
    with LOCK:
        if ac_id not in UAVS:
            # Initial state structure
            st = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "heading": 0.0}
            UAVS[ac_id] = {"state": st}
            print(f"[BRIDGE] Registered new AC{ac_id}")
        return UAVS[ac_id]

# Add this GLOBAL variable at the top
FIRST_GPS_RECEIVED = False
CALIB_OFFSET_X = 0.0
CALIB_OFFSET_Y = 0.0

def on_gps_int(ac_id, msg):
    global FIRST_GPS_RECEIVED, CALIB_OFFSET_X, CALIB_OFFSET_Y
    if msg.name != "GPS_INT": return
    ac_id = int(ac_id)
    uav = ensure_uav(ac_id)

    lat = float(msg["lat"]) * 1e-7
    lon = float(msg["lon"]) * 1e-7
    alt = float(msg["alt"]) / 1000.0 

    # Convert using your XML Lat0/Lon0
    x, y, z = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)

    # --- CALIBRATION HACK ---
    # The first time we get a GPS message, assuming the drone is at "Home" (0,0)
    # we calculate the error.
    if not FIRST_GPS_RECEIVED:
        # Ask yourself: Is the drone physically at the XML Home (S1/Start) right now?
        # If yes, x and y SHOULD be 0.0. If they are 50.0, your offset is 50.0.
        print(f"\n[CALIBRATION] Drone is at Python Coords: X={x:.2f}, Y={y:.2f}")
        print(f"[CALIBRATION] If drone is actually at HOME (0,0), subtract these values!\n")
        FIRST_GPS_RECEIVED = True
        
    # Apply Calibration (Uncomment if you want to force it to 0,0)
    # x -= 54.0  (Example: Replace with value printed above)
    # y -= -12.0 (Example)
    
    with LOCK:
        uav["state"]["x"] = x
        uav["state"]["y"] = y
        uav["state"]["z"] = z

def on_ins(ac_id, msg):
    if msg.name != "INS":
        return

    ac_id = int(ac_id)
    uav = ensure_uav(ac_id)

    ins_x  = float(msg["ins_x"])
    ins_y  = float(msg["ins_y"])
    ins_z  = float(msg["ins_z"])
    ins_xd = float(msg["ins_xd"])
    ins_yd = float(msg["ins_yd"])
    ins_zd = float(msg["ins_zd"])

    # ENU Conversion:
    # Paparazzi NED X (North) -> Planner Y
    # Paparazzi NED Y (East)  -> Planner X
    # Paparazzi NED Z (Down)  -> Planner -Z
    
    north_m = ins_x * POS_SCALE
    east_m  = ins_y * POS_SCALE
    up_m    = -ins_z * POS_SCALE

    north_v = ins_xd * VEL_SCALE
    east_v  = ins_yd * VEL_SCALE
    up_v    = -ins_zd * VEL_SCALE

    heading = math.atan2(east_v, north_v) if (abs(east_v)>1e-6 or abs(north_v)>1e-6) else None
    heading_deg = math.degrees(heading) if heading is not None else float('nan')

    # --- store ---
    with LOCK:
        st = uav["state"]
        # st["x"], st["y"], st["z"] = east_m, north_m, up_m
        st["vx"], st["vy"], st["vz"] = east_v, north_v, up_v
        st["heading"] = heading

    # --- DEBUG PRINT ---
    # print(f"INS -> AC{ac_id}: x={east_m:.2f}, y={north_m:.2f}, z={up_m:.2f}")

def on_pprz_msg(ac_id, msg):
    """
    Handles incoming Ivy messages. 
    Prioritizes ROTORCRAFT_FP for smooth, high-speed closed-loop control.
    """
    try:
        ac_id = int(ac_id)
    except:
        return

    # Filter out base station / broadcast IDs
    if ac_id == 0 or ac_id == 255:
        return

    # Ensure UAV exists in our database
    uav = ensure_uav(ac_id)

    with LOCK:
        # --- 1. ROTORCRAFT_FP (High Priority & Smooth) ---
        if msg.name == "ROTORCRAFT_FP":
            
            if uav.get('msg_type') != 'FP':
                print(f"[IVY] AC{ac_id} LOCKED onto ROTORCRAFT_FP (High Speed Mode)")
                uav['msg_type'] = 'FP'
            
            # Extract Data
            x = float(msg['east']) * POS_SCALE
            y = float(msg['north']) * POS_SCALE
            z = float(msg['up']) * POS_SCALE
            
            vx = float(msg['veast']) * VEL_SCALE
            vy = float(msg['vnorth']) * VEL_SCALE
            vz = float(msg['vup']) * VEL_SCALE
            
            psi_deg = float(msg['psi']) * ANGLE_SCALE
            
            # Update State
            uav["state"]["x"] = x
            uav["state"]["y"] = y
            uav["state"]["z"] = z
            uav["state"]["vx"] = vx
            uav["state"]["vy"] = vy
            uav["state"]["vz"] = vz
            uav["state"]["heading"] = math.radians(psi_deg)

# Store the last yaw state and timestamp for each drone to calculate rate
YAW_MEMORY = {} 
# Maximum allowed turn rate: 20 degrees per second (smooth and realistic for fast flight)
MAX_YAW_RATE = math.radians(20.0) 

def send_pprz_velocity(interface, ac_id, vx_enu, vy_enu, vz_enu):
    if interface is None: return

    # 1. Convert ENU -> NED
    vx_ned = vy_enu   # North 
    vy_ned = vx_enu   # East
    vz_ned = -vz_enu  # Down

    # Calculate current ground speed
    speed_2d = math.sqrt(vx_ned**2 + vy_ned**2)

    # 2. Where do we WANT to look?
    # Increased deadband from 0.001 to 1.0 m/s to completely ignore wind-jitter when hovering
    if speed_2d > 1.0: 
        target_yaw = math.atan2(vy_ned, vx_ned)
    else:
        target_yaw = None

    # 3. Rate Limiter Logic
    global YAW_MEMORY
    now = time.time()
    
    # Initialize memory if drone is new
    if ac_id not in YAW_MEMORY:
        # Try to pull its real current heading, default to 0.0
        start_yaw = UAVS.get(ac_id, {}).get("state", {}).get("heading", 0.0)
        YAW_MEMORY[ac_id] = {'yaw': start_yaw, 'time': now}
        
    last_yaw = YAW_MEMORY[ac_id]['yaw']
    last_time = YAW_MEMORY[ac_id]['time']
    dt_yaw = now - last_time
    
    # Prevent giant dt jumps if the loop lags
    if dt_yaw > 1.0: dt_yaw = 0.1 

    if target_yaw is not None:
        # Find the shortest angular path to the target (-pi to pi)
        diff = (target_yaw - last_yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Clamp the angular change so we don't exceed MAX_YAW_RATE
        max_allowed_change = MAX_YAW_RATE * dt_yaw
        
        if abs(diff) > max_allowed_change:
            # math.copysign applies the max_allowed_change in the correct left/right direction
            diff = math.copysign(max_allowed_change, diff)
            
        desired_yaw = last_yaw + diff
    else:
        # If hovering or fighting wind, just keep the nose pointed where it already is
        desired_yaw = last_yaw

    # Keep angles normalized between -pi and pi
    desired_yaw = (desired_yaw + math.pi) % (2 * math.pi) - math.pi

    # Save state for the next loop
    YAW_MEMORY[ac_id] = {'yaw': desired_yaw, 'time': now}

    msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
    msg['ac_id'] = int(ac_id)
    
    # 0x60 explicitly commands X,Y,Z as Velocity, and YAW as Absolute Position
    msg['flags'] = 0x60
    msg['x'] = float(vx_ned)
    msg['y'] = float(vy_ned)
    msg['z'] = float(vz_ned)
    msg['yaw'] = float(desired_yaw)  
    
    interface.send(msg, ac_id=int(ac_id))


# Global set for discovery
DISCOVERED_IDS = set()


# Helper to map World (x,y) -> Grid (col, row)
def world_to_grid(x, y):
    c = int(round((x - x_min) / grid_resolution))
    r = int(round((y - y_min) / grid_resolution))
    return r, c

# -----------------------------
# GLOBAL PARAMETERS
# -----------------------------

dt_step = 1.0
v_drift = np.array([1.0, 0.0])
theta_FOV = np.deg2rad(45)
E_scale = 20.0
E_scale_track = 20.0
gamma_wind = 5.0
v_wind = np.array([0.0, 0.0])
v_max = 8.0        # max velocity [m/s]

confirm_pconf = 0.6
peak_tresh = 0.75
mean_thresh = 0.4
rho_th = 0.25

# -----------------------------
# CORE FUNCTIONS
# -----------------------------

# --- Sensor model parameters (sec. 7.3.1) ---
h_ref = 20.0  # [m] altitude at which sensor reaches max resolution (confirmation height)
P0 = 0.9
P_false = 1e-5        # example

def f_res(h):
    """
    Resolution factor (Sec. 7.3.1).
    Use inverse-square scaling up to h_ref, then saturate at 1:

        f_res(h) = min(1, (h_ref / h)^2)

    So:
      - at h = h_ref:  f_res = 1
      - at h > h_ref:  f_res < 1  (worse resolution, lower P_hit)
      - at h < h_ref:  f_res = 1  (no >100% "super sensor")
    """
    h_eff = max(h, 1e-3)
    return min(1.0, (h_ref / h_eff))


# --- Empirical VSQP Power Model ---
def get_vsqp_power(v):
    """
    Returns instantaneous power (W) for a given horizontal airspeed v (m/s).
    Based on empirical data fusion (Hover=1751W, Cruise=323W).
    """
    if v < 12.0:
        return 1751.0  # Hover Regime
    elif 12.0 <= v < 17.0:
        return 154.5 * v - 104.6  # Transition Regime (Linear Fit)
    else:
        return 323.0   # Fixed-Wing Cruise Regime

def get_quadcopter_power(v):
    """
    Returns instantaneous electrical power (W) for a given horizontal airspeed v (m/s).
    Empirically derived from flight log '26_03_06__16_15_17.data'.
    """
    # U-shaped power curve: P(v) = P_hover - (Translational Lift) + (Parasite Drag)
    power = 81.3 - (1.626 * v) + (0.219 * v**2)
    
    # Safety floor just in case extreme tailwinds cause weird math
    return max(power, 50.0)
    
'''For VSQP:'''
# def energy_of_path(path):
#     # Use the global wind vector
#     global v_wind 
    
#     total_energy = 0.0
#     v_climb, v_desc = 2.0, 3.0
#     P_climb = get_vsqp_power(0.0) # 1751 W

#     for i in range(len(path) - 1):
#         p0, p1 = path[i], path[i+1]
#         vec_g = (p1[:2] - p0[:2])
#         dist_xy = np.linalg.norm(vec_g)
#         dz = p1[2] - p0[2]

#         if dist_xy > 1e-3:
#             # Calculate the ground velocity vector for this segment
#             # Note: We use the wind-aware vg we just fixed in the planner
#             unit_vec = vec_g / (dist_xy + 1e-6)
#             v_headwind_comp = np.dot(v_wind[:2], unit_vec)
#             vg_allowed = max(2.0, min(20.0, 20.0 + v_headwind_comp))
            
#             v_ground_vec = unit_vec * vg_allowed
            
#             # Find the resulting airspeed vector
#             v_air_vec = v_ground_vec - v_wind[:2]
#             v_air_mag = np.linalg.norm(v_air_vec)
            
#             # Get power based on AIRSPEED
#             P_segment = get_vsqp_power(v_air_mag)
            
#             t_segment = dist_xy / vg_allowed
#             E_horiz = P_segment * t_segment
#         else:
#             E_horiz = 0.0

#         # Vertical Energy (unchanged)
#         if dz > 0:   
#             E_vert = P_climb * (dz / v_climb)
#         elif dz < 0: 
#             E_vert = P_climb * (-dz / v_desc) * 0.5
#         else:        
#             E_vert = 0.0
        
#         total_energy += (E_horiz + E_vert)
        
#     return total_energy

def energy_of_path(path):
    # Use the global wind vector
    global v_wind 
    
    total_energy = 0.0
    v_climb, v_desc = 2.0, 3.0
    
    # --- QUADCOPTER VERTICAL POWER MODEL ---
    # Base hover power is ~70W
    P_hover = get_quadcopter_power(0.0) 
    
    # Climbing takes roughly 20% more power than hovering
    P_climb = P_hover * 1.20  
    
    # Descending takes roughly 20% less power than hovering
    P_desc = P_hover * 0.80   

    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i+1]
        vec_g = (p1[:2] - p0[:2])
        dist_xy = np.linalg.norm(vec_g)
        dz = p1[2] - p0[2]

        if dist_xy > 1e-3:
            # Calculate the ground velocity vector for this segment
            unit_vec = vec_g / (dist_xy + 1e-6)
            v_headwind_comp = np.dot(v_wind[:2], unit_vec)
            
            # --- See derivation of 13.26 m/s ---
            vg_allowed = max(2.0, min(13.26, 13.26 + v_headwind_comp))
            
            v_ground_vec = unit_vec * vg_allowed
            
            # Find the resulting airspeed vector
            v_air_vec = v_ground_vec - v_wind[:2]
            v_air_mag = np.linalg.norm(v_air_vec)
            
            # --- CALL THE NEW QUADCOPTER POWER CURVE ---
            P_segment = get_quadcopter_power(v_air_mag)
            
            t_segment = dist_xy / vg_allowed
            E_horiz = P_segment * t_segment
        else:
            E_horiz = 0.0

        # --- UPDATED VERTICAL ENERGY CALCULATION ---
        if dz > 0:   
            E_vert = P_climb * (dz / v_climb)
        elif dz < 0: 
            E_vert = P_desc * (-dz / v_desc) 
        else:        
            E_vert = 0.0
        
        total_energy += (E_horiz + E_vert)
        
    return total_energy

# ==============================================================================
# 2. THE PLANNER (Supports 'constraint_poly' and 'drone_vel')
# ==============================================================================
@jit(nopython=True)
def get_p_hit_numba(h, h_ref=20.0, P0=0.9):
    # Calculates Detection Probability P_hit(h)
    if h < 1e-3: h = 1e-3
    f_res = 1.0
    if h > h_ref:
        f_res = h_ref / h # Simple inverse scaling
    return P0 * f_res

@jit(nopython=True)
def calc_ig_numba(path_arr, belief_map, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols):
    """
    Numba-compiled Information Gain Calculator.
    Executes explicit loops at C-speeds (~50-100x faster than Numpy slicing).
    """
    total_IG = 0.0
    
    # We must work on a copy so we don't mutate the actual simulation map
    pred_belief = belief_map.copy()
    
    steps = len(path_arr) - 1
    
    for i in range(steps):
        p0 = path_arr[i]
        p1 = path_arr[i+1]
        
        # 3D Euclidean Distance
        dist = np.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2)
        
        # Sample approx every 10 meters
        n_samples = int(np.ceil(dist / 10.0))
        if n_samples < 1: n_samples = 1
        
        dx = (p1[0] - p0[0]) / n_samples
        dy = (p1[1] - p0[1]) / n_samples
        dz = (p1[2] - p0[2]) / n_samples
        
        for k in range(n_samples):
            # Interpolated Drone Position
            px = p0[0] + k * dx
            py = p0[1] + k * dy
            pz = p0[2] + k * dz
            
            # Calculate FOV Radius on Ground
            r_m = pz * fov_tan
            r_cells = int(np.ceil(r_m / grid_res))
            r_sq = r_m * r_m
            
            # Grid Indices of Drone
            # (int(x + 0.5) is fast rounding in Numba)
            c_center = int((px - x_min) / grid_res + 0.5)
            r_center = int((py - y_min) / grid_res + 0.5)
            
            # Clamp Window to Map Bounds
            r_start = max(0, r_center - r_cells)
            r_end   = min(rows, r_center + r_cells + 1)
            c_start = max(0, c_center - r_cells)
            c_end   = min(cols, c_center + r_cells + 1)
            
            # Get P_hit for this altitude
            q = get_p_hit_numba(pz)
            
            # Explicit Loop over the FOV Window
            for r in range(r_start, r_end):
                for c in range(c_start, c_end):
                    
                    # 1. Check Geofence Mask
                    if not inside_mask[r, c]:
                        continue
                    
                    # 2. Check Distance (Circular Footprint)
                    # Convert grid index back to meters relative to drone
                    dr = (r - r_center) * grid_res
                    dc = (c - c_center) * grid_res
                    dist_sq = dr*dr + dc*dc
                    
                    if dist_sq <= r_sq:
                        p = pred_belief[r, c]
                        
                        # Skip if already fully known (0.0 or 1.0)
                        if p < 1e-6 or p > (1.0 - 1e-6):
                            continue
                        
                        # --- ENTROPY BEFORE ---
                        # Manual clipping for safety
                        p_safe = p
                        if p_safe < 1e-9: p_safe = 1e-9
                        elif p_safe > (1.0-1e-9): p_safe = 1.0-1e-9
                        
                        H_prior = -p_safe * np.log2(p_safe) - (1-p_safe) * np.log2(1-p_safe)
                        
                        # --- BAYES UPDATE (No Detection) ---
                        # p_new = p(1-q) / (1 - pq)
                        num = p * (1.0 - q)
                        den = 1.0 - p * q
                        if den < 1e-12: den = 1e-12
                        p_post = num / den
                        
                        # --- ENTROPY AFTER ---
                        pp_safe = p_post
                        if pp_safe < 1e-9: pp_safe = 1e-9
                        elif pp_safe > (1.0-1e-9): pp_safe = 1.0-1e-9
                        
                        H_post = -pp_safe * np.log2(pp_safe) - (1-pp_safe) * np.log2(1-pp_safe)
                        
                        # Accumulate Gain
                        total_IG += (H_prior - H_post)
                        
                        # Update the temp belief so we don't double-count gain from overlap
                        pred_belief[r, c] = p_post
                        
    return total_IG


# logging entropy:
@jit(nopython=True)
def calculate_global_map_entropy_numba(prob_map, inside_mask):
    """
    Calculates the actual current entropy of the entire global map.
    Only counts cells that are actually inside the mission geofence.
    """
    total_entropy = 0.0
    rows, cols = prob_map.shape
    
    for r in range(rows):
        for c in range(cols):
            # Only calculate entropy for valid flying area
            if not inside_mask[r, c]:
                continue
                
            p = prob_map[r, c]
            
            # Safe clipping just like in your IG calculator
            if p < 1e-9: 
                p_safe = 1e-9
            elif p > (1.0 - 1e-9): 
                p_safe = 1.0 - 1e-9
            else:
                p_safe = p
                
            H = -p_safe * np.log2(p_safe) - (1.0 - p_safe) * np.log2(1.0 - p_safe)
            total_entropy += H
            
    return total_entropy

# ==============================================================================
# 2. UPDATED PLANNER FUNCTION (CALLS NUMBA)
# ==============================================================================

def plan_velocity_ipp_3D(drone_pos, drone_vel, belief_map, inside_mask, grid_origin, grid_resolution, soft_poly,
                         constraint_poly=None,
                         fov_angle=theta_FOV, v_max=8.0, n_directions=16,
                         step_length=40.0, altitude_candidates=[30, 35, 40, 45, 50],
                         E_scale=100.0,    
                         lam=0.5,          
                         buffer=0.0, pred_depth=3):

    # Unpack origin for Numba
    x_min, y_min = grid_origin
    rows, cols = belief_map.shape
    fov_tan = np.tan(fov_angle)

    # 1. Constraint Logic
    if constraint_poly is not None:
        valid_area = soft_poly.intersection(constraint_poly).buffer(0)
    else:
        valid_area = Polygon(soft_poly)
        if buffer > 0.0: valid_area = valid_area.buffer(-buffer)

    if valid_area.is_empty:
        return 0.0, 0.0, 0.0, [drone_pos, drone_pos], None, 0.0

    # 2. Motion Primitives
    cx, cy, cz = drone_pos
    vx, vy, vz = drone_vel
    current_speed = np.linalg.norm([vx, vy])

    if current_speed > 2.0:
        current_heading = np.arctan2(vy, vx)
        angles = current_heading + np.linspace(-np.pi/3, np.pi/3, n_directions)
    else:
        angles = np.linspace(0, 2*np.pi, n_directions, endpoint=False)

    candidates = []

    # 3. Generate Candidates
    # (Geometric check is fast enough in Python for ~50 paths)
    for a in angles:
        dx, dy = np.cos(a), np.sin(a)
        for alt in altitude_candidates:
            trajectory = [np.array([cx, cy, cz])]
            valid_traj = True
            
            for k in range(pred_depth):
                prev = trajectory[-1]
                nx = prev[0] + dx * step_length
                ny = prev[1] + dy * step_length
                nz = alt
                
                if not valid_area.contains(Point(nx, ny)):
                    valid_traj = False
                    break
                trajectory.append(np.array([nx, ny, nz]))
            
            if valid_traj:
                candidates.append(trajectory)

    if not candidates and constraint_poly is not None:
        # Fallback for tight spaces
        small_step = 10.0
        for a in angles:
            dx, dy = np.cos(a), np.sin(a)
            end = np.array([cx + dx*small_step, cy + dy*small_step, cz])
            if valid_area.contains(Point(end[0], end[1])):
                candidates.append([np.array([cx, cy, cz]), end])

    # 4. Evaluation Loop (Using Numba)
    best_J = -np.inf
    best_path = None
    # We drop 'best_mask' to save massive amounts of time
    best_mask = None 

    for path in candidates:
        # Convert path to float64 numpy array for Numba
        path_arr = np.array(path, dtype=np.float64)
        
        # --- CALL NUMBA FUNCTION ---
        # Returns only IG (float), no mask
        I_p = calc_ig_numba(
            path_arr, 
            belief_map, 
            inside_mask, 
            x_min, y_min, 
            grid_resolution, 
            fov_tan, 
            rows, cols
        )
        
        E_p = energy_of_path(path)
        E_norm = E_p / E_scale
        J = I_p - (lam * E_norm)
        print(f" IG={I_p:.2f}, E={E_p:.1f}J, J={J:.2f}")
        
        if J > best_J:
            best_J = J
            best_path = path

    if best_path is None:
        return 0.0, 0.0, 0.0, [drone_pos, drone_pos], None, 0.0

    # 5. Output Velocity Calculation (Matches energy_of_path logic)
    p0, p1 = best_path[0], best_path[1]
    vec = p1 - p0
    dist = np.linalg.norm(vec)
    unit_vec = vec / (dist + 1e-6)

    # Replicate the exact physics used in the Planner's energy check
    # V_ground_target = V_cruise_air (20) + V_wind_parallel
    v_headwind_comp = np.dot(v_wind[:2], unit_vec[:2])
    
    # Calculate the max allowed ground speed for this heading
    # (Matches the logic you pasted in energy_of_path)
    v_g_allowed = max(1.0, min(8.0, 8.0 + v_headwind_comp))
    
    # Calculate velocity components
    travel_time = max(dist / v_g_allowed, 0.1)
    
    vx = vec[0] / travel_time
    vy = vec[1] / travel_time
    vz = vec[2] / travel_time

    return float(vx), float(vy), float(vz), best_path, None, best_J


def tracking_region(t_now, detection_pos, detection_time, v_drift,
                    base_radius=60.0, uncertainty_gain=1.0,
                    cone_angle_deg=90):
    """
    Creates a drift cone region aligned with v_drift.
    Uncertainty radius grows proportionally to |v_drift|.
    """

    dt = max(t_now - detection_time, 0.0)

    # Drift speed magnitude
    drift_speed = np.linalg.norm(v_drift)

    # Drifted center
    drift_vec = v_drift * dt
    center_R = detection_pos + drift_vec

    # Expansion based on drift
    radius_R = base_radius + uncertainty_gain * drift_speed * dt

    # Cone geometry
    theta = np.arctan2(v_drift[1], v_drift[0])
    half_angle = np.deg2rad(cone_angle_deg / 2)

    angles = np.linspace(-half_angle, half_angle, 50)
    x_arc = radius_R * np.cos(angles)
    y_arc = radius_R * np.sin(angles)

    # Build polygon
    cone_points = np.vstack((
        [0,0],
        np.column_stack((x_arc, y_arc)),
        [0,0]
    ))

    cone_poly = Polygon(cone_points)
    cone_poly = rotate(cone_poly, np.degrees(theta), origin=(0,0))
    cone_poly = translate(cone_poly, detection_pos[0], detection_pos[1])

    return cone_poly, center_R, radius_R



def clear_confirmed_region_2d(belief_map, XX, YY, victim_pos, wind_vec,
                              major_axis=80.0, minor_axis=60.0, decay=0.01):
    
    # Normalize wind direction
    w = wind_vec[:2]
    norm_w = np.linalg.norm(w)
    wind_hat = w / (norm_w + 1e-6) if norm_w > 1e-6 else np.array([1.0, 0.0])
    perp_hat = np.array([-wind_hat[1], wind_hat[0]])

    # Vectorized relative positions (2D Array math)
    dx = XX - victim_pos[0]
    dy = YY - victim_pos[1]
    
    # Project onto axes (Dot product broadcast)
    a = dx * wind_hat[0] + dy * wind_hat[1]  # Along-wind dist
    b = dx * perp_hat[0] + dy * perp_hat[1]  # Cross-wind dist
    
    # Ellipse Mask
    mask = (a / major_axis)**2 + (b / minor_axis)**2 <= 1.0
    
    # Apply decay
    belief_map[mask] = decay
    
    return belief_map


def active_cone_mask_2d(t_now, XX, YY, track, v_drift):
    """
    Returns a 2D boolean mask (same shape as XX, YY) that is True
    inside any active tracker's drift cone.
    """
    total_mask = np.zeros(XX.shape, dtype=bool)
    
    # Flatten coordinates once for fast checking
    points_flat = np.vstack((XX.flatten(), YY.flatten())).T
    
    for tr in track:
        if not tr["active"] or tr["pos"] is None or tr["time"] is None:
            continue

        # Get the Shapely polygon for this cone
        region_poly, _, _ = tracking_region(
            t_now=t_now,
            detection_pos=tr["pos"],
            detection_time=tr["time"],
            v_drift=v_drift
        )

        if region_poly and not region_poly.is_empty:
            # Buffer it as before
            poly_buffered = region_poly.buffer(20.0)
            
            # Rasterize: Convert Polygon -> Boolean Mask
            mpl_path = MplPath(list(poly_buffered.exterior.coords))
            mask_flat = mpl_path.contains_points(points_flat)
            
            # Reshape back to 2D and combine
            total_mask |= mask_flat.reshape(XX.shape)

    return total_mask

def get_view_window(pos_xyz, fov_angle=theta_FOV):
    """
    Returns the slice objects (rows, cols) and the effective radius in meters.
    """
    x, y, h = pos_xyz
    
    # 1. Determine radius in meters and grid cells
    r_m = h * np.tan(fov_angle)
    r_cells = int(np.ceil(r_m / grid_resolution))
    
    # 2. Get center index
    r_center, c_center = world_to_grid(x, y)
    
    # 3. Determine start/end indices (clamped to map boundaries)
    r_start = max(0, r_center - r_cells)
    r_end   = min(rows, r_center + r_cells + 1)
    
    c_start = max(0, c_center - r_cells)
    c_end   = min(cols, c_center + r_cells + 1)
    
    return (slice(r_start, r_end), slice(c_start, c_end)), r_m, (r_center, c_center)



victims_template = np.array([[100.0,-150.0],[300.0,0.0], [0.0, 0.0], [50.0, 50.0]])
DISCOVERED_POSITIONS = {}

VICTIM_IDS = [123, 219]  

if __name__ == "__main__":
    # --- SIMULATION CONFIG ---
    # Set True to force internal physics (drones move themselves).
    # Set False if you are running the Paparazzi simulator/real drone.
    USE_INTERNAL_PHYSICS = True

    # ---------------------------------------------------------
    # 1. IVY DISCOVERY (Split Searchers vs Real Victims)
    # ---------------------------------------------------------
    try:
        # NOTE: 127.255.255.255 can sometimes fail on local Linux. 
        # If it fails, try "127.0.0.1:2010"
        ivy_bus = os.getenv("PPRZ_IVY_BUS", "127.255.255.255:2010")
        ivy = IvyMessagesInterface("cyberzoo_planner", ivy_bus=ivy_bus)
        ivy.subscribe(on_pprz_msg)  # <--- USES YOUR NEW HANDLER
        ivy.start()
        print(f"[BRIDGE] Listening on {ivy_bus}...")
        
        print("Waiting 5s for drones...")
        time.sleep(5)
        
        # Get ALL seen IDs from the global UAVS dict
        ALL_IDS = sorted(list(UAVS.keys()))
        
        # A. Identify Real Victims
        active_victim_ids = [ac for ac in ALL_IDS if ac in VICTIM_IDS]
        
        if active_victim_ids:
            HAS_REAL_VICTIM = True
            print(f"[BRIDGE] Found Real Victim Drones: {active_victim_ids}")
        else:
            HAS_REAL_VICTIM = False
            # USE_INTERNAL_PHYSICS = False
            # print(f"[WARN] No Real Victim Drones found (looked for {VICTIM_IDS}). Using virtual.")

        # B. Identify Searchers (Everyone else)
        AC_IDS = [ac for ac in ALL_IDS if ac not in VICTIM_IDS]

        # Handle case where no searchers connected
        # Handle case where no searchers connected via Ivy
        if not AC_IDS:
            print("[WARN] No SEARCH drones found via Ivy. Using Sim Defaults.")
            AC_IDS = [122, 215, 216, 221, 222]
            
            # Initialize states for all 6 drones
            for ac in AC_IDS:
                ensure_uav(ac)
            
            # Spread them out along the X-axis so they don't immediately collide
            # UAVS[121]['state']['x'] = 0.0;   UAVS[121]['state']['y'] = 0.0;  UAVS[121]['state']['z'] = 30.0 // This one is broken now ;()
            UAVS[122]['state']['x'] = 20.0;  UAVS[122]['state']['y'] = 0.0;  UAVS[122]['state']['z'] = 30.0
            UAVS[215]['state']['x'] = 40.0;  UAVS[215]['state']['y'] = 0.0;  UAVS[215]['state']['z'] = 32.0
            UAVS[216]['state']['x'] = 60.0;  UAVS[216]['state']['y'] = 0.0;  UAVS[216]['state']['z'] = 35.0
            UAVS[221]['state']['x'] = 80.0;  UAVS[221]['state']['y'] = 0.0;  UAVS[221]['state']['z'] = 37.0
            UAVS[222]['state']['x'] = 100.0; UAVS[222]['state']['y'] = 0.0;  UAVS[222]['state']['z'] = 40.0
        else:
            print(f"[BRIDGE] Active Search Drones: {AC_IDS}")
            USE_INTERNAL_PHYSICS = False

    except Exception as e:
        print(f"[ERR] Ivy fail: {e}")
        ivy = None
        AC_IDS = [122, 215, 216, 221, 222]
        for ac in AC_IDS:
            ensure_uav(ac)
        HAS_REAL_VICTIM = False
        active_victim_ids = []
        USE_INTERNAL_PHYSICS = True

    # ---------------------------------------------------------
    # 2. SETUP (Initialize Victims & Searchers)
    # ---------------------------------------------------------
    
    # --- A. Setup Victims ---
    if HAS_REAL_VICTIM:
        # If we have real victim drones, use their actual positions
        real_victim_positions = []
        
        for v_id in active_victim_ids:
            # Sync state to get starting position (default to safe spot if jittery)
            # We use index 0 arbitrarily here as the 'owner' index since victims don't own themselves in the planner
            v_pos, _ = sync_drone_state(0, v_id, np.array([-40.0, 20.0, 10.0]))
            real_victim_positions.append(v_pos[:2])
            print(f" -> Init Real Victim AC{v_id} at {v_pos}")
        
        # Create the master victims array from real data
        victims = np.array(real_victim_positions)
        
    else:
        # Fallback to manual/virtual input if no real victims connected
        try:
            val = input(f"Victims [2]: ")
            num_victims = int(val) if val.strip() else 2
            USE_INTERNAL_PHYSICS = True
        except: num_victims = 2
        victims = victims_template[:num_victims]


    # --- B. Setup Search Drones ---
    num_drones = len(AC_IDS)
    drone_positions = []
    uav_nominal_altitudes = []


    # --- B. Setup Search Drones ---
    num_drones = len(AC_IDS)
    drone_positions = []
    uav_nominal_altitudes = []
    
    # Initialize from the UAVS dictionary populated by on_gps_int/on_ins
    for i, ac_id in enumerate(AC_IDS):
        uav = UAVS[ac_id]
        with LOCK:
            s = uav['state']
            # Read the actual Z from the drone
            real_z = max(0.5, s['z'])
            pos = np.array([s['x'], s['y'], real_z])
        
        # We now use the drone's actual current altitude as its nominal tier
        # (Assuming it is currently flying at its assigned XML altitude)
        assigned_alt = real_z
        
        print(f" -> Init Searcher AC{ac_id} at {pos[:2]} | Assigned Alt: {assigned_alt:.1f}m")
        drone_positions.append(pos)
        uav_nominal_altitudes.append(assigned_alt)

    uav_nominal_altitudes = np.array(uav_nominal_altitudes)

    drone_vels = [np.zeros(3) for _ in range(num_drones)]
    drone_modes = ['explore'] * num_drones
    track = [{"active":False, "confirmed":False, "pos":None, "time":None, "phase":None} for _ in range(num_drones)]
    pred_depth = 5
    # Assume all UAVs start with 100% energy (normalized)
    E_max = 100.0
    E_rem = np.ones(num_drones) * E_max

    # ...
    cmap = cm.RdYlBu_r.copy()  # Create a copy so we can modify it safely
    cmap.set_bad(color='white')

    # --- Main occupancy map figure ---
    # if SHOW_REAL_TIME_PLOT:
    fig, ax_map = plt.subplots(figsize=(8, 8))

    # 1. Prepare initial display data
    display_data = belief_map.copy()
    display_data[~inside_mask] = np.nan

    # 2. Render the Grid using imshow
    im_display = ax_map.imshow(
        display_data,
        origin='lower',
        extent=[x_min, x_max, y_min, y_max],
        cmap=cmap,
        vmin=0.0, vmax=1.0,
        interpolation='nearest',
        zorder=1  # Puts image on the bottom layer
    )

    # 3. FORCE GRID LINES ON TOP
    # This creates the "cells" look you are missing
    ax_map.grid(visible=True, which='major', color='white', 
                linestyle='-', linewidth=0.5, alpha=0.5)
    ax_map.set_axisbelow(False) # Important: Brings grid to front!

    # Drone markers (High zorder to stay on top)
    drone_plots = []
    for d_idx in range(num_drones):
        (p,) = ax_map.plot(
            drone_positions[d_idx][0], drone_positions[d_idx][1],
            'o', markersize=8, markeredgecolor='k', 
            label=f"UAV{d_idx}", zorder=10
        )
        drone_plots.append(p)

    # ... (rest of setup)

    # Victim markers
    victim_plot = ax_map.scatter(
        victims[:, 0], victims[:, 1],
        c='r', marker='x', s=60, label='Victims', zorder=10
    )


    ax_map.cone_fill = None  # placeholder for the red cone fill patch

    # Colorbar and labels
    cbar = plt.colorbar(im_display, ax=ax_map)
    cbar.set_label('Occupancy probability')


    # --- ADD THIS LINE FOR GRID LOOK ---
    ax_map.grid(which='major', color='w', linestyle='-', linewidth=0.5, alpha=0.3)

    # Lock the view to the geofence bounds
    ax_map.set_xlim(x_min - 20, x_max + 20)
    ax_map.set_ylim(y_min - 20, y_max + 20)
    ax_map.set_xlabel('X [m]')
    ax_map.set_ylabel('Y [m]')
    ax_map.set_title('Multi-UAV Occupancy Map (High Speed)')
    ax_map.legend(loc='upper right')

    # Optional: Draw the polygon boundary line for visual reference
    if soft_poly:
        x_poly, y_poly = soft_poly.exterior.xy
        ax_map.plot(x_poly, y_poly, 'k-', linewidth=2, alpha=0.5)

    plt.ion()
    plt.show()

    # -----------------------------
    # SIMULATION LOOP (multi-UAV)
    # -----------------------------
    max_dheading = np.deg2rad(10)  # max heading change per second
    buffer = 30.0 # safe buffer for polygon

    # Create a buffered polygon for safe navigation
    safe_poly = soft_poly.buffer(-buffer) if buffer > 0 else soft_poly

    # Initialize UAV states and tracking flags
    drone_modes = ["explore"] * num_drones   # 'explore' or 'track'
    detection_active = [False]*num_drones
    detection_confirmed = [False]*num_drones
    detection_pos = [None]*num_drones
    detection_time = [None]*num_drones
    tracker_phase = [None]*num_drones
    cooldown_until = np.zeros(num_drones)
    COOLDOWN_TIME = 30  # seconds of detection lockout
    cone_artists = []
    # --- TRACKING TIMERS ---
    MIN_ALT = 20.0                # Minimum flight altitude
    # --- Tracking timing parameters ---
    initial_lock_duration = 8.0    # seconds tracking cone center before IPP
    cone_search_timeout   = 30.0   # max time doing IPP inside cone before abort
    dt = dt_step           # simulation time step [s]
 

    detection_event = [False]*num_drones

    track = []
    for _ in range(num_drones):
        track.append({
            "active": False,
            "confirmed": False,
            "pos": None,            # detection_pos for THIS UAV
            "time": None,           # detection_time for THIS UAV
            # phases: "to_detection", "initial_lock", "cone_tracking", "hover_confirm"
            "phase": None,
            "lock_start": None,     # time when initial_lock starts
            "cone_start": None      # time when cone_tracking (IPP in cone) starts
        })


    # Pre-allocate cone graphics for each UAV
    cone_lines = []
    cone_centers = []
    cone_fills = []

    # =====================================
    # SIMULATION GENERAL CONSTANTS
    # =====================================


    for d_idx in range(num_drones):
        color = plt.cm.tab10(d_idx)

        (line,) = ax_map.plot([], [], '--', lw=2, color=color, alpha=0.9)
        (center,) = ax_map.plot([], [], 'o', color=color, markersize=6)

        from matplotlib.patches import Polygon as MplPolygon
        fill = MplPolygon([[0,0],[0,0],[0,0]], closed=True,
                        facecolor=color, alpha=0.15)
        ax_map.add_patch(fill)

        cone_lines.append(line)
        cone_centers.append(center)
        cone_fills.append(fill)


    # -----------------------------
    # DATA LOGGING SETUP (Dual File)
    # -----------------------------
    # Create a unique filename based on the current time
    timestamp = datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
    map_log_filename = f"map_metrics_{timestamp}.csv"

    # Open the file and write the headers
    map_log_file = open(map_log_filename, mode='w', newline='')
    map_writer = csv.writer(map_log_file)
    map_writer.writerow(['time_s', 'total_entropy', 'entropy_reduced_percent', 'compute_time_ms'])

    initial_entropy = None  # Will be set on the first loop iteratio
    previous_entropy = None
    
    # Initialize frequency variable to prevent crash on first loop
    current_freq = 0.0    

    try:
        # Initialize clocks
        start_time = time.time()
        last_loop_time = time.time()
        frame_count = 0
                
        # Use 'while True' for continuous bridge operation
        while True:
            # -------------------------------------------------------
            # 1. CALCULATE REAL DT (Fixes Time Dilation)
            # -------------------------------------------------------
            now = time.time()
            t = now
            dt = now - last_loop_time
            last_loop_time = now
            
            # Safety Clamp: Prevent huge jumps if computer lags
            # (If lag > 0.2s, we pretend it was only 0.2s to stop teleporting)
            dt = np.clip(dt, 0.001, 0.5)

            # =========================================================
            # STEP 0: SYNC STATE (INPUT)
            # =========================================================
            if not USE_INTERNAL_PHYSICS:
                for d_idx in range(num_drones):
                    ac_id = AC_IDS[d_idx]
                    
                    # 1. READ from Bridge (This gets the latest INS/GPS data)
                    real_pos, real_vel = sync_drone_state(d_idx, ac_id, drone_positions[d_idx])
                    
                    # 2. OVERWRITE the planner's state
                    drone_positions[d_idx] = real_pos 
                    
                    # 3. OPTIONAL: Sync Velocity too (helps planner know about wind/fighting)
                    drone_vels[d_idx] = real_vel 

                    # Debug Print (Optional) to prove it's working
                    if frame_count % 10 == 0:
                       print(f"[SYNC] UAV{d_idx} Real Pos: {real_pos}")

            # ------------------------------------------
            # 0. Dynamic mapping
            # ------------------------------------------
            shift_y = v_drift[1] * dt / grid_resolution
            shift_x = v_drift[0] * dt / grid_resolution
            
            # Shift
            belief_map = shift(belief_map, shift=[shift_y, shift_x], order=1, cval=0.5)
            
            # Reset boundaries (Prevents map shrinking)
            belief_map[~inside_mask] = 0.5 
            
            # INCREASE SIGMA: Makes "cleared" areas decay back to "unknown" faster
            belief_map = gaussian_filter(belief_map, sigma=1.0)
            # ------------------------------------------
            # 1. Victim motion (Drift)
            # ------------------------------------------

            # Move victims in Real Time
            # ------------------------------------------
            # 1. Victim motion (Drift + Altitude Hold)
            # ------------------------------------------
            if HAS_REAL_VICTIM:
                for i, v_id in enumerate(active_victim_ids):
                    # A. Get current state
                    vic_pos, _ = sync_drone_state(0, v_id, np.zeros(3))
                    
                    # B. Altitude Hold Logic (Adjust target_z for outdoors!)
                    target_z = 10.0  # <--- Make sure this is a safe outdoor altitude!
                    z_err = target_z - vic_pos[2]
                    
                    # P-Controller: Correct drift with vertical velocity
                    # Limit max climb/sink for safety
                    vz_hold = np.clip(z_err * 0.5, -1.0, 1.0)
                    
                    # C. Send Command (Drift XY + Hold Z)
                    if ivy:
                        send_pprz_velocity(ivy, v_id, v_drift[0], v_drift[1], vz_hold)
                    
                    # D. Update internal map variable
                    victims[i] = vic_pos[:2]

            else:
                # Virtual drift
                victims[:, :2] += v_drift * dt

            # ------------------------------------------
            # 4. Bayesian mapping update (Per UAV - OPTIMIZED)
            # ------------------------------------------
            
            # Reset detection events for this timestep
            detection_event = [False] * num_drones

            for d_idx in range(num_drones):
                drone_pos = drone_positions[d_idx]
                h = drone_pos[2]
                
                # --- A. Get the "View Window" (The 2D Slice) ---
                # This effectively "zooms in" on the small square around the drone
                # You must have the 'get_view_window' helper from Step 2 defined!
                (r_slice, c_slice), r_m, (rc, cc) = get_view_window(drone_pos, theta_FOV)
                
                # Extract local patches (Views into the main array)
                local_belief = belief_map[r_slice, c_slice]
                local_mask   = inside_mask[r_slice, c_slice] # Geofence mask
                
                # If the drone is out of bounds or looking at nothing, skip
                if local_belief.size == 0:
                    continue

                # --- B. Create Local Coordinate Grid for Distance Check ---
                # We generate coordinates ONLY for this small window (~20x20 pixels)
                # grid_y and grid_x are your global 1D coordinate arrays
                local_Y, local_X = np.meshgrid(grid_y[r_slice], grid_x[c_slice], indexing='ij')
                
                dx = local_X - drone_pos[0]
                dy = local_Y - drone_pos[1]
                d2 = dx*dx + dy*dy
                
                # Visibility Mask: Inside Radius AND Inside Geofence
                vis_mask = (d2 <= r_m*r_m) & local_mask
                
                if not np.any(vis_mask):
                    continue
                
                # --- C. Check for "True" Victim Detection (Oracle) ---
                # Instead of checking 1 million grid cells, just check if 
                # any victim is physically close to the drone.
                # dist(drone, victim) < FOV_radius
                
                # Vectorized distance check against ALL victims
                # (Assumes victims array is shape [N, 2])
                vic_dists_sq = np.sum((victims[:, :2] - drone_pos[:2])**2, axis=1)
                
                # Is any victim inside the FOV radius?
                # We add a small buffer (cell_size) to account for discretization errors
                victims_in_fov = vic_dists_sq <= (r_m + grid_resolution)**2
                
                # Calculate Detection Probability
                # p_hit_val = P_hit(h)
                p_hit_val = get_p_hit_numba(h, h_ref, P0)
                
                if np.any(victims_in_fov):
                    # Victim is physically there!
                    # Boost probability to ensure we don't miss it (simulating strong signal)
                    p_detect = min(1.0, p_hit_val + 0.4)
                else:
                    # Empty space
                    p_detect = P_false
                
                # Roll the dice
                is_hit = (np.random.rand() < p_detect)
                detection_event[d_idx] = is_hit
                
                # --- D. Bayesian Update (Vectorized on Local Patch) ---
                p_prior = local_belief[vis_mask]
                
                if is_hit:
                    # HIT Update
                    num = p_prior * p_hit_val
                    den = num + (1 - p_prior) * P_false
                else:
                    # MISS Update
                    num = p_prior * (1 - p_hit_val)
                    den = 1 - p_prior * p_hit_val
                
                # Avoid division by zero
                p_post = num / (den + 1e-12)
                
                # --- E. WRITE BACK (Modifies Global Map Instantly) ---
                # We clip to prevent numerical lock (0.0 or 1.0)
                local_belief[vis_mask] = np.clip(p_post, 1e-6, 1 - 1e-6)

            # ------------------------------------------
            # 5. Centralized detection & tracker assignment
            # ------------------------------------------

            # (5.1) Build mask of existing cones (Fast 2D Rasterization)
            # REPLACES: in_active_cone = active_cone_mask(...)
            in_active_cone = active_cone_mask_2d(t, XX, YY, track, v_drift)

            # (5.2) Select eligible explorers
            eligible_explorers = [
                d_idx for d_idx, mode in enumerate(drone_modes)
                if mode == "explore" and t >= cooldown_until[d_idx]
            ]

            # (5.3) Check for NEW detections
            NEW_DETECTION = False
            detection_uav = None
            detection_pos = None

            for d_idx in eligible_explorers:
                # 1. Did Step 4 flag a detection event?
                if not detection_event[d_idx]:
                    continue

                # 2. Localize the signal (Find peak in the 2D window)
                # REPLACES: looping through vis_mask indices
                (r_slice, c_slice), r_m, _ = get_view_window(drone_positions[d_idx])
                
                local_patch = belief_map[r_slice, c_slice]
                
                if local_patch.size == 0: continue

                # Find peak index in local patch
                flat_idx = np.argmax(local_patch)
                max_r, max_c = np.unravel_index(flat_idx, local_patch.shape)
                
                # Convert to Global Grid Index
                global_r = r_slice.start + max_r
                global_c = c_slice.start + max_c
                
                # 3. Check if inside an existing cone (using the 2D mask)
                if in_active_cone[global_r, global_c]:
                    continue
                
                # 4. Convert to World Position
                det_x = grid_x[global_c]
                det_y = grid_y[global_r]
                detection_pos = np.array([det_x, det_y])

                # Found valid new detection
                NEW_DETECTION = True
                detection_uav = d_idx
                break 


            # (5.4) If no new detection, skip
            if (not NEW_DETECTION) or detection_uav is None:
                pass
            else:
                # (5.5) Duplicate Suppression (Geometric check)
                # This logic remains largely the same, just checking the new detection_pos
                det_point = Point(detection_pos[0], detection_pos[1])
                is_duplicate = False
                duplicate_thresh = 100.0

                for tr_exist in track:
                    if not tr_exist["active"] or tr_exist["pos"] is None: continue

                    region_poly_exist, _, radius_R = tracking_region(
                        t_now=t,
                        detection_pos=tr_exist["pos"],
                        detection_time=tr_exist["time"],
                        v_drift=v_drift
                    )

                    if region_poly_exist and not region_poly_exist.is_empty:
                        if region_poly_exist.buffer(10.0).contains(det_point):
                            is_duplicate = True; break

                    dist = np.linalg.norm(detection_pos - tr_exist["pos"])
                    if dist < max(duplicate_thresh, radius_R * 0.8):
                        is_duplicate = True; break

                if is_duplicate:
                    print(f"[t={t}] Detection at {detection_pos} overlaps existing tracker.")
                
                else:
                    # (5.7) Assign Nearest Explorer (SAME AS BEFORE)
                    print(f"[t={t}] TRUE DETECTION at {detection_pos}")

                    best_d, best_cost = None, np.inf

                    for d_idx in eligible_explorers:
                        r = detection_pos - drone_positions[d_idx][:2]
                        dist = np.linalg.norm(r)
                        r_hat = r / (dist + 1e-6)
                        # Wind penalty cost
                        Deff = dist + gamma_wind * abs(np.dot(v_wind[:2], r_hat))

                        if Deff < best_cost:
                            best_cost = Deff
                            best_d = d_idx

                    if best_d is not None:
                        # --- THIS PART IS YOUR ORIGINAL CODE ---
                        tr_assign = track[best_d]
                        tr_assign["active"] = True
                        tr_assign["confirmed"] = False
                        tr_assign["pos"] = detection_pos.copy()
                        tr_assign["pos_at_detection"] = detection_pos.copy()
                        tr_assign["time"] = t
                        tr_assign["phase"] = "to_detection"

                        drone_modes[best_d] = "track"
                        cooldown_until[best_d] = t + COOLDOWN_TIME

                        print(f"[t={t}] Assigning UAV{best_d} to track region.")


            # ------------------------------------------
            # 6. Plan next velocities (Logic Updated for 2D Arrays)
            # ------------------------------------------
            commanded_vels = np.zeros((num_drones, 3))

            for d_idx in range(num_drones):
                tr = track[d_idx]

                # --- PREVENT BLEEDING VARIABLES BETWEEN DRONES! ---
                vx_des, vy_des, vz_des = 0.0, 0.0, None

                # --- A. EXPLORATION MODE ---
                if drone_modes[d_idx] == "explore":

                    target_alt = uav_nominal_altitudes[d_idx]

                    vx_des, vy_des, vz_des, *_ = plan_velocity_ipp_3D(
                        drone_positions[d_idx], 
                        drone_vels[d_idx],
                        belief_map,           # <--- 2D Matrix
                        inside_mask,          # <--- 2D Mask
                        (x_min, y_min),       # <--- Origin
                        grid_resolution,      # <--- Res
                        soft_poly,
                        constraint_poly=None,
                        step_length=20.0,
                        fov_angle=theta_FOV, v_max=v_max, n_directions=16,
                        altitude_candidates=[target_alt], pred_depth=3, E_scale=E_scale,
                        lam=lam,        
                        buffer=buffer
                    )

        

                    # 3. Strict P-controller to guarantee it sticks to the altitude outdoors
                    z_err = target_alt - drone_positions[d_idx][2]
                    vz_des = np.clip(z_err * 0.5, -2.0, 2.0) # Increased from [-1.0, 1.0]
                    


                # --- B. TRACKING MODE ---
                elif drone_modes[d_idx] == "track" and tr["active"] and not tr["confirmed"]:

                    # --- THE HARD DECK ABORT ---
                    if drone_positions[d_idx][2] < 17.5: # 17.5 to allow tiny sensor jitter
                        print(f"[t={t}] UAV{d_idx} breached hard deck (20m)! Aborting track.")
                        tr["active"] = False
                        drone_modes[d_idx] = "explore"
                        vz_des = 2.0  # Command an immediate climb
                        commanded_vels[d_idx] = [0.0, 0.0, vz_des]
                        continue

                    # 1. Abort Check
                    if tr["pos"] is not None:
                        # ... (Keep your existing clear check here) ...
                        tc = int(round((tr["pos"][0] - x_min) / grid_resolution))
                        tr_idx = int(round((tr["pos"][1] - y_min) / grid_resolution))
                        if 0 <= tr_idx < rows and 0 <= tc < cols:
                            if belief_map[tr_idx, tc] < 0.05:
                                tr["active"] = False; drone_modes[d_idx] = "explore"
                                continue

                    # --- PHASE: APPROACH ("to_detection") ---
                    if tr["phase"] == "to_detection":
                        t_elapsed = t - tr["time"]
                        raw_target = tr["pos"] + v_drift * t_elapsed
                        
                        # 1. Geofence Clamp
                        target_point = Point(raw_target[0], raw_target[1])
                        if not soft_poly.contains(target_point):
                            nearest_pt = soft_poly.exterior.interpolate(
                                soft_poly.exterior.project(target_point))
                            target_now = np.array([nearest_pt.x, nearest_pt.y])
                        else:
                            target_now = raw_target
                        
                        vec = target_now - drone_positions[d_idx][:2]
                        dist = np.linalg.norm(vec)

                        # 2. PREDICTIVE BRAKING (The Fix)
                        # Slow down proportional to distance so we don't overshoot.
                        if dist > 75.0:
                            # Far away? Cruise at max speed.
                            vx_des, vy_des = (vec / (dist + 1e-6)) * 8.0
                        else:
                            # Closer than 15m? Slow down.
                            # At 15m -> 7.5 m/s
                            # At 2m  -> 1.0 m/s
                            # speed = np.clip(dist * 1.5, 1.0, 20.0)
                            # Gain 0.8 requires ~2.5 m/s² braking (Feasible)
                            speed = np.clip(dist * 0.3, 0.5, 8.0)
                            vx_des, vy_des = (vec / (dist + 1e-6)) * speed
                        
                        # vz_des = 0.0
                        z_err = uav_nominal_altitudes[d_idx] - drone_positions[d_idx][2]
                        vz_des = np.clip(z_err * 0.5, -2.0, 2.0)

                        # Switch to cone tracking when VERY close
                        if dist < 5.0:
                            tr["phase"] = "cone_tracking"
                            tr["cone_start"] = t

                    # --- PHASE: INITIAL LOCK ---
                    elif tr["phase"] == "initial_lock":
                        det_pos = tr["pos"]
                        det_time = tr["time"]

                        # Follow drifting cone center
                        region_poly, center_R, _ = tracking_region(t, det_pos, det_time, v_drift)
                        vec_center = center_R - drone_positions[d_idx][:2]
                        dist_center = np.linalg.norm(vec_center)

                        if dist_center > 5.0:
                            dir_unit = vec_center / (dist_center + 1e-6)
                            vx_des, vy_des = dir_unit * 5.0
                        else:
                            vx_des, vy_des = 0.0, 0.0

                        # vz_des = 0.0 
                        z_err = uav_nominal_altitudes[d_idx] - drone_positions[d_idx][2]
                        vz_des = np.clip(z_err * 0.5, -2.0, 2.0)

                        if (t - tr["lock_start"]) >= initial_lock_duration:
                            tr["phase"] = "cone_tracking"
                            tr["cone_start"] = t


                    # --- PHASE: CONE TRACKING (IPP inside cone) ---
                    elif tr["phase"] == "cone_tracking":
                        det_pos = tr["pos"]
                        det_time = tr["time"]

                        # 1. Update Cone
                        region_poly, center_R, _ = tracking_region(t, det_pos, det_time, v_drift)
                        valid_search_area = soft_poly.intersection(region_poly)
                        
                        if valid_search_area.is_empty:
                            tr["active"] = False; drone_modes[d_idx] = "explore"
                            continue

                        # 2. Rasterize Cone (REPLACES: list comprehension)
                        # We need to find belief stats inside the cone
                        # Use MplPath for fast point-in-polygon on the grid
                        poly_path = MplPath(list(region_poly.exterior.coords))
                        points_flat = np.vstack((XX.flatten(), YY.flatten())).T
                        mask_flat = poly_path.contains_points(points_flat)
                        in_R_mask = mask_flat.reshape(rows, cols)
                        
                        # 3. Calculate Stats
                        if np.any(in_R_mask):
                            p_vals = belief_map[in_R_mask]
                            frac_high = np.mean(p_vals > confirm_pconf)
                            mean_p = np.mean(p_vals)
                            max_p = np.max(p_vals)

                            # Success -> Hover
                            if (frac_high >= rho_th or mean_p >= 0.30 or max_p >= peak_tresh):
                                tr["phase"] = "hover_confirm"
                                tr["fail_timer"] = 0.0
                                tr["pos"] = drone_positions[d_idx][:2].copy() # Anchor
                                tr["time"] = t
                                continue

                            # Descent Logic
                            if max_p > 0.20 and drone_positions[d_idx][2] > MIN_ALT + 0.5:
                                vz_des = -2.0 # Descend
                            
                            # Timeout/Failure Logic
                            z = drone_positions[d_idx][2]
                            if z <= MIN_ALT + 0.5:
                                tr.setdefault("fail_timer", 0.0)
                                tr["fail_timer"] += dt
                                if tr["fail_timer"] > 8.0:
                                    print(f"UAV{d_idx} Bad Detection. Clearing area.")
                                    # Clear local area in 2D
                                    dx = XX - tr["pos_at_detection"][0]
                                    dy = YY - tr["pos_at_detection"][1]
                                    belief_map[(dx**2 + dy**2) < 40.0**2] = 0.0
                                    tr["active"] = False; drone_modes[d_idx] = "explore"
                                    vz_des = 2.0
                                    continue
                            
                            if (t - tr["cone_start"]) >= cone_search_timeout and max_p < 0.50:
                                tr["active"] = False; drone_modes[d_idx] = "explore"
                                vz_des = 2.0
                                continue

                        # 4. Motion: IPP constrained to Cone
                        soft_cone_constraint = region_poly.buffer(10.0)
                        
                        vx_ipp, vy_ipp, vz_ipp, _, _, _ = plan_velocity_ipp_3D(
                            drone_positions[d_idx], 
                            drone_vels[d_idx],
                            belief_map, inside_mask,
                            (x_min, y_min), grid_resolution,
                            soft_poly,
                            constraint_poly=soft_cone_constraint,
                            step_length=20.0,
                            fov_angle=theta_FOV, v_max=5.0, 
                            altitude_candidates=[drone_positions[d_idx][2]], 
                            pred_depth=2, E_scale=E_scale_track,
                            lam=lam,       
                            buffer=buffer
                        )

                        # vx_des = vx_ipp + v_drift[0] * 0.8
                        # vy_des = vy_ipp + v_drift[1] * 0.8
                        vx_des = vx_ipp 
                        vy_des = vy_ipp 
                        # if 'vz_des' not in locals(): vz_des = vz_ipp

                        if vz_des is None: 
                            z_err = uav_nominal_altitudes[d_idx] - drone_positions[d_idx][2]
                            vz_des = np.clip(z_err * 0.5, -2.0, 2.0)

                    # --- PHASE: HOVER CONFIRM ---
                    elif tr["phase"] == "hover_confirm":
                        det_pos = tr["pos"]
                        det_time = tr["time"]
                        if "hover_start" not in tr: tr["hover_start"] = t

                        # 1. Get Visibility Window
                        (r_s, c_s), r_m, _ = get_view_window(drone_positions[d_idx])
                        local_belief = belief_map[r_s, c_s]
                        
                        # 2. Stabilized Targeting
                        hotspot_mask_local = (local_belief > confirm_pconf)
                        
                        raw_target = None
                        has_visual_lock = False
                        theoretical_center = det_pos + v_drift * (t - det_time)

                        if np.sum(hotspot_mask_local) > 3:
                            # Reconstruct World Coordinates locally
                            y_indices = np.arange(r_s.start, r_s.stop) * grid_resolution + y_min
                            x_indices = np.arange(c_s.start, c_s.stop) * grid_resolution + x_min
                            loc_x_mesh, loc_y_mesh = np.meshgrid(x_indices, y_indices)
                            
                            coords_x = loc_x_mesh[hotspot_mask_local]
                            coords_y = loc_y_mesh[hotspot_mask_local]
                            weights = local_belief[hotspot_mask_local]**2
                            
                            vis_x = np.average(coords_x, weights=weights)
                            vis_y = np.average(coords_y, weights=weights)
                            visual_centroid = np.array([vis_x, vis_y])

                            # Gating
                            if np.linalg.norm(visual_centroid - theoretical_center) < 20.0:
                                has_visual_lock = True
                                raw_target = visual_centroid
                            else:
                                raw_target = theoretical_center
                        else:
                            raw_target = theoretical_center

                        # --- FIX: LOW PASS FILTER ON TARGET POSITION ---
                        # This prevents the drone from jerking when switching between visual/model
                        if "target_smooth" not in tr or tr["target_smooth"] is None:
                            tr["target_smooth"] = raw_target  # Snap on first frame

                        # Alpha 0.4 means 40% new data, 60% old position (smooth drift)
                        alpha_pos = 0.3
                        tr["target_smooth"] = (1 - alpha_pos) * tr["target_smooth"] + alpha_pos * raw_target
                        target_xy = tr["target_smooth"]

                        # 3. Control (Chase the Smoothed Target)
                        err_vec = target_xy - drone_positions[d_idx][:2]
                        dist_err = np.linalg.norm(err_vec)

                        if dist_err > 0.2:
                            # P-Controller
                            speed = np.clip(dist_err * 0.5, 0.5, v_max)
                            vx_cmd, vy_cmd = (err_vec / (dist_err + 1e-6)) * speed
                        else:
                            vx_cmd, vy_cmd = 0.0, 0.0

                    
                        if dist_err > 0.2:
                            speed = np.clip(dist_err * 0.5, 0.5, v_max)
                            vx_cmd, vy_cmd = (err_vec / (dist_err + 1e-6)) * speed
                        else:
                            vx_cmd, vy_cmd = 0.0, 0.0
                        
                        vx_des = vx_cmd + v_drift[0]
                        vy_des = vy_cmd + v_drift[1]

                        # Vertical Logic
                        current_alt = drone_positions[d_idx][2]
                        # Only descend if we are > 20.0m high
                        if current_alt > 20.0:
                            # SCALED CHECK: Only drop if within 5.0m of target horizontally
                            if dist_err < 10.0:
                                vz_des = -1.5 # Safe, controlled descent
                            else:
                                vz_des = 0.0  # Wait to align horizontally!
                        else:
                            vz_des = 0.0

                        # 4. Confirmation Stats
                        p_vals = local_belief[hotspot_mask_local]
                        
                        if len(p_vals) > 0:
                            mean_p = np.mean(p_vals)
                            max_p = np.max(p_vals)
                            
                            # --- RESTORED LOGIC START ---
                            # Dynamic Confirmation Ceiling (Higher Altitudes Allowed!)
                            if mean_p >= 0.90:
                                required_confirmation_alt = 35.0
                            elif mean_p >= 0.70:
                                required_confirmation_alt = 28.0
                            else:
                                required_confirmation_alt = 22.0
                                
                            alt_ready = (current_alt <= required_confirmation_alt)

                            # Check Confirmation
                            if (alt_ready and max_p >= peak_tresh):
                                print(f"[t={t}] UAV{d_idx} CONFIRMED victim at alt={current_alt:.1f}m.")
                                
                                # 1. Clear Ellipse (2D Version)
                                belief_map = clear_confirmed_region_2d(belief_map, XX, YY, det_pos, v_drift)
                                
                                # 2. Hard Clear Center (2D Version)
                                dx = XX - det_pos[0]
                                dy = YY - det_pos[1]
                                belief_map[(dx**2 + dy**2) < 50.0**2] *= 0.05
                                
                                print(f"[t={t}] Fully cleared victim region.")
                                
                                tr["active"] = False; tr["confirmed"] = True
                                drone_modes[d_idx] = "explore"
                                vz_des = 2.0
                            # --- RESTORED LOGIC END ---

                        # Timeout Logic
                        # (Uses the same alt_ready logic to decide if we timed out waiting for confirmation)
                        # We recalculate alt_ready here just in case p_vals was empty above
                        if mean_p >= 0.70: req_alt = 28.0 
                        else: req_alt = 22.0
                        
                        if (
                            (current_alt <= req_alt) 
                            and (t - tr["hover_start"]) > 10.0
                        ):
                            print(f"[t={t}] UAV{d_idx} hover-confirm timeout → aborting.")
                            tr["active"] = False; drone_modes[d_idx] = "explore"
                            vz_des = 2.0

                    else:
                        vx_des, vy_des, vz_des = 0.0, 0.0, 0.0

                # --- Fallback ---
                else:
                    vx_des, vy_des, vz_des, *_ = plan_velocity_ipp_3D(
                        drone_positions[d_idx], 
                        drone_vels[d_idx],
                        belief_map, inside_mask,
                        (x_min, y_min), grid_resolution,
                        soft_poly,
                        constraint_poly=None,
                        step_length=20.0,
                        fov_angle=theta_FOV, v_max=v_max, n_directions=16,
                        altitude_candidates=[30, 50, 70], pred_depth=3, E_scale=E_scale,
                        lam=lam,        
                        buffer=buffer
                    )

                commanded_vels[d_idx] = [vx_des, vy_des, vz_des]

                # ====================================================
                # UNIVERSAL HARD DECK (Safety Failsafe)
                # ====================================================
                if drone_positions[d_idx][2] < 17.5:
                    print(f"[URGENT] UAV{d_idx} breached hard deck ({drone_positions[d_idx][2]:.1f}m)! PULL UP!")
                    drone_modes[d_idx] = "explore" 
                    if "active" in track[d_idx]: track[d_idx]["active"] = False
                    # Override previous planner commands: Hover XY, Climb Fast!
                    commanded_vels[d_idx] = [0.0, 0.0, 2.5]

                # --- MOTION SMOOTHING (The Fix) ---

                # CASE A: REAL DRONE (Ivy Connected)
                # We send the RAW desired velocity directly. 
                # Paparazzi has its own internal PID controller that handles acceleration/braking physics.
                # If we smooth it here, we double-filter the command, causing lag and slow braking.
                if not USE_INTERNAL_PHYSICS:
                    drone_vels[d_idx] = np.array([vx_des, vy_des, vz_des])

                # CASE B: INTERNAL SIMULATION
                # We must simulate inertia manually so the dots don't teleport on the map.
                else:
                    # 1. Calculate the requested jump from current velocity to desired
                    v_current = drone_vels[d_idx][:2]
                    v_target = np.array([vx_des, vy_des])
                    
                    delta_v = v_target - v_current
                    dist_v = np.linalg.norm(delta_v)
                    
                    # 2. Universal Acceleration Limit (4.0 m/s^2)
                    max_accel = 4.0  
                    max_change = max_accel * dt
                    
                    if dist_v > max_change:
                        # Clamp the change vector magnitude
                        ratio = max_change / dist_v
                        delta_v = delta_v * ratio
                        
                    # 3. Apply Horizontal Velocity
                    drone_vels[d_idx][:2] = v_current + delta_v
                    
                    # 4. Vertical Smoothing (Limit vertical accel to 1.0 m/s^2)
                    z_diff = vz_des - drone_vels[d_idx][2]
                    drone_vels[d_idx][2] += np.clip(z_diff, -1.0, 1.0)
          


            repulsion_vels = np.zeros((num_drones, 3))
            safe_separation = 15.0  # [m]
            repulsion_gain = 2.0

            for i in range(num_drones):
                for j in range(i + 1, num_drones):
                    # Use only X and Y for distance so they don't push each other up/down
                    diff_vec_2d = drone_positions[i][:2] - drone_positions[j][:2]
                    dist_2d = np.linalg.norm(diff_vec_2d)
                    
                    if dist_2d < safe_separation and dist_2d > 0.1:
                        push_2d = (diff_vec_2d / dist_2d) * (safe_separation - dist_2d) * repulsion_gain
                        repulsion_vels[i][:2] += push_2d
                        repulsion_vels[j][:2] -= push_2d

            # B. Apply Physics, Geofence & Output
            # ---------------------------------------------------------
            for d_idx in range(num_drones):
                
                # 1. START WITH PLANNER'S WISH (The "Target")
                # We do not start with current velocity. We start with where we WANT to go.
                v_target = commanded_vels[d_idx].copy()
                
                # 2. ADD REPULSION (High priority reflex)
                v_target += repulsion_vels[d_idx]

                # 3. ACCELERATION LIMITER (The "Smoother")
                # We calculate the change needed from the LAST COMMAND (drone_vels)
                # to the NEW TARGET (v_target). We do NOT use real velocity here.
                v_last_cmd = drone_vels[d_idx] # This is the "Memory" of what we sent last time
                
                # Calculate required change
                delta_v = v_target - v_last_cmd
                dist_v = np.linalg.norm(delta_v)
                
                # Clamp change to physical limits (e.g., 4 m/s^2)
                max_accel = 4.0 
                max_change = max_accel * dt 
                
                if dist_v > max_change:
                    scale = max_change / dist_v
                    delta_v = delta_v * scale
                
                # This is our new "Proposed" velocity for this frame
                v_proposed = v_last_cmd + delta_v
                
                # 4. GEOFENCE PREDICTION CHECK
                # We predict where this proposed velocity will put the drone relative to its REAL position
                next_pos = drone_positions[d_idx] + v_proposed * dt
                
                # Check Mask (Fast)
                c_idx = int(round((next_pos[0] - x_min) / grid_resolution))
                r_idx = int(round((next_pos[1] - y_min) / grid_resolution))
                is_safe = (0 <= r_idx < rows and 0 <= c_idx < cols and inside_mask[r_idx, c_idx])
                
                # Apply Wall Slide (Slow) if unsafe
                if not is_safe:
                    point_next = Point(next_pos[0], next_pos[1])
                    if not soft_poly.contains(point_next):
                        nearest = np.array(safe_poly.exterior.interpolate(
                            safe_poly.exterior.project(point_next)).coords[0])
                        
                        # Calculate "Sliding" Vector
                        wall_vec = nearest - drone_positions[d_idx][:2]
                        norm = np.linalg.norm(wall_vec)
                        
                        if norm > 0.01:
                            # Project velocity onto wall (simple clamp for now)
                            limit_speed = min(np.linalg.norm(v_proposed[:2]), 2.0) # Slow down near wall
                            v_proposed[:2] = (wall_vec / norm) * limit_speed
                        else:
                            v_proposed[:2] = [0.0, 0.0]

                # 5. WIND & MAX SPEED CLAMP
                v_g_mag = np.linalg.norm(v_proposed[:2])
                if v_g_mag > 0.1:
                    unit_v = v_proposed[:2] / v_g_mag
                    # Allow higher speed if flying INTO wind
                    v_headwind = np.dot(v_wind[:2], unit_v)
                    limit = 15.0 + v_headwind
                    if v_g_mag > limit:
                        v_proposed[:2] = unit_v * max(2.0, limit)
                
                # 6. UPDATE STATE & SEND
                # Update the "Memory" for the next loop
                drone_vels[d_idx] = v_proposed
                
                # Send to Paparazzi (Real Drone)
                if ivy:
                     send_pprz_velocity(ivy, AC_IDS[d_idx], *drone_vels[d_idx])
                     print(f"  UAV{d_idx}: Cmd=[{drone_vels[d_idx][0]:5.2f}, {drone_vels[d_idx][1]:5.2f}]")

            # =========================================================
            # 9. INTERNAL PHYSICS FALLBACK (Sim Only)
            # =========================================================
            if USE_INTERNAL_PHYSICS:
                for d_idx in range(num_drones):
                    drone_positions[d_idx] += drone_vels[d_idx] * dt


           # =========================================================
            # 10. DATA LOGGING (Map Entropy & Mission Success)
            # =========================================================
            # 1. Take a snapshot of the current global entropy using your belief_map
            current_entropy = calculate_global_map_entropy_numba(belief_map, inside_mask)

            # 2. If this is the very first loop, set our baselines
            if initial_entropy is None:
                initial_entropy = current_entropy
                previous_entropy = current_entropy

            # 3. Calculate Entropy Reduced PER TIMESTEP (Current step vs Last step)
            entropy_reduced_this_step = previous_entropy - current_entropy

            # 4. Calculate TOTAL Entropy Reduced (Current step vs Very beginning)
            total_entropy_reduced = initial_entropy - current_entropy
            percent_reduced = (total_entropy_reduced / initial_entropy) * 100.0 if initial_entropy > 0 else 0.0

            # 5. Write everything to your CSV
            map_writer.writerow([
                f"{t:.3f}",                           # Current time
                f"{current_entropy:.3f}",             # Absolute entropy remaining
                f"{entropy_reduced_this_step:.4f}",   # Entropy reduced in this specific loop
                f"{percent_reduced:.2f}"              # Overall mission progress (%)
            ])
            map_log_file.flush() # Save immediately

            # 6. Update 'previous_entropy' so it is ready for the next loop iteration
            previous_entropy = current_entropy

            # ============================
            #  Visualization Update
            # ============================

            if SHOW_REAL_TIME_PLOT:

                # A. Update Belief Map (IMSHOW)
                display_data = belief_map.copy()
                display_data[~inside_mask] = np.nan
                im_display.set_data(display_data)

                # B. Update Markers
                for d_idx in range(num_drones):
                    drone_plots[d_idx].set_data(drone_positions[d_idx][0], drone_positions[d_idx][1])
                victim_plot.set_offsets(victims[:, :2])

                # C. Update Cone Graphics
                for d_idx in range(num_drones):
                    tr = track[d_idx]
                    
                    # Show cone only if tracking and active
                    if tr["active"] and tr["pos"] is not None:
                        region_poly, center_R, _ = tracking_region(
                            t_now=t,
                            detection_pos=tr["pos"],
                            detection_time=tr["time"],
                            v_drift=v_drift
                        )
                        
                        if region_poly and not region_poly.is_empty:
                            xR, yR = region_poly.exterior.xy
                            cone_lines[d_idx].set_data(xR, yR)
                            cone_centers[d_idx].set_data(center_R[0], center_R[1])
                            cone_fills[d_idx].set_xy(np.column_stack([xR, yR]))
                        else:
                            # Hide
                            cone_lines[d_idx].set_data([], [])
                            cone_centers[d_idx].set_data([], [])
                            cone_fills[d_idx].set_xy(np.zeros((3,2))) # Safe dummy coords
                    else:
                        # Hide
                        cone_lines[d_idx].set_data([], [])
                        cone_centers[d_idx].set_data([], [])
                        cone_fills[d_idx].set_xy(np.zeros((3,2)))

                # Draw Main Map
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

        

            # 2. CONDITIONAL Rendering (Draw every 5th frame)
            VIZ_INTERVAL = 10
            
            if t % VIZ_INTERVAL == 0:
                
                # D. PRINT COMMANDED VS GROUND VELOCITY
                # D. PRINT FULL STATUS (Cmd vs Gnd vs Air)
                print(f"--- Time t={t} ---")
                for d_idx in range(num_drones):
                    # 1. Ground Velocity (What you see on the map)
                    vx, vy, vz = drone_vels[d_idx]
                    g_speed = np.linalg.norm([vx, vy])
                    
                    # 2. Commanded Velocity (What the planner requested)
                    cvx, cvy, cvz = commanded_vels[d_idx]
                    
                    # 3. Airspeed (How fast the props are spinning relative to air)
                    #    Vector Subtraction: V_air = V_ground - V_wind
                    v_air_vec = np.array([vx, vy]) - v_wind[:2]
                    air_speed = np.linalg.norm(v_air_vec)
                    
                    print(f"  UAV{d_idx}: Cmd=[{cvx:5.2f}, {cvy:5.2f}] Gnd=[{vx:5.2f}, {vy:5.2f}] | GS: {g_speed:5.1f} m/s | AS: {air_speed:5.1f} m/s")

            # ============================
            # 12. DEBUG: Frequency Measurement
            # ============================
            frame_count += 1
            now = time.time()
            elapsed = now - start_time
            
            # Update frequency calculation every 1.0 second
            if elapsed >= 1.0:
                current_freq = frame_count / elapsed
                print(f"--- PERFORMANCE: Loop Rate = {current_freq:.2f} Hz ---")
                
                # Reset counters
                start_time = now
                frame_count = 0

            # --- Small delay for real-time smoothness ---
            plt.pause(0.0001)

            # ============================
            # DATA LOGGING & SNAPSHOTS
            # ============================
            
            # Save visual snapshots of the map every 10 frames
            if frame_count % 10 == 0:
                # Make sure the directory exists!
                os.makedirs("EHVB_flight_logs", exist_ok=True)
                f_name = f"EHVB_flight_logs/frame_{int(t*10):06d}.png"
                plt.savefig(f_name, dpi=60, bbox_inches='tight')

    except KeyboardInterrupt:
        print("\n[BRIDGE] Stopping...")
    finally:
        if ivy:
            ivy.stop()
            print("[BRIDGE] Ivy Stopped.")

    plt.ioff()
    plt.show()