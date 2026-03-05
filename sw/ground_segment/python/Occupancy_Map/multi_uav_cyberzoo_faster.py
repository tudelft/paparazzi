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

# --- PAPARAZZI BRIDGE GLOBALS ---
UAVS = {}   # Stores state: {ac_id: {'state': {'x':..., 'y':...}}}
LOCK = threading.Lock()

# --- SCALING FACTORS (Derived from Paparazzi XML) ---
POS_SCALE = 0.0039063        # Converts integer position to Meters (1/256)
VEL_SCALE = 0.0000019        # Converts integer velocity to m/s (1/2^19)
ACCEL_SCALE = 0.0009766      # Converts integer accel to m/s^2 (1/1024)
ANGLE_SCALE = 0.0139882      # Converts integer angle to Degrees
DEG2RAD = math.pi / 180.0    # Helper for Trig functions

# --- Paparazzi setup ---
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))
sys.path.append(PPRZ_HOME + "/sw/ext/pprzlink/lib/v1.0/python")


lat0, lon0, alt0 = 51.990634, 4.376789, 0.0
LAT0, LON0, ALT0 =lat0, lon0, alt0 
# --- CyberZoo Geofence Setup ---
# Extracted from cyberzoo XML
cz_waypoints = {
    "_CZ1": (51.9905834, 4.3767710),
    "_CZ2": (51.9906440, 4.3767060),
    "_CZ3": (51.9906860, 4.3768080),
    "_CZ4": (51.9906238, 4.3768729),
}

# Convert Geodetic to ENU (Local meters)
# Using lat0="51.990634" lon0="4.376789" from the XML header
lat0, lon0 = 51.990634, 4.376789
softgeo_xy = []
for name in ["_CZ1", "_CZ2", "_CZ3", "_CZ4"]:
    lat, lon = cz_waypoints[name]
    x, y, _ = pm.geodetic2enu(lat, lon, 0, lat0, lon0, 0)
    softgeo_xy.append([x, y])

soft_poly = Polygon(softgeo_xy)


x_min, y_min, x_max, y_max = soft_poly.bounds
grid_resolution = 0.05  # Finer resolution (0.5m)

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


def ensure_uav(ac_id):
    """Creates a new UAV entry if seen for the first time."""
    with LOCK:
        if ac_id not in UAVS:
            # Initial state structure
            st = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "heading": 0.0}
            UAVS[ac_id] = {"state": st}
            print(f"[BRIDGE] Registered new AC{ac_id}")
        return UAVS[ac_id]


FIRST_GPS_RECEIVED = False
CALIB_OFFSET_X = 0.0
CALIB_OFFSET_Y = 0.0


KNOWN_START_HEIGHTS = {
    121: 2.0,  # HOver heights
    122: 3.0, 
    123: 0.5, 
    219: 2.0, 
}

# specific AC_IDs you are using
ac_ids = [121, 122, 123, 219] 

GROUND_REF_AMSL = None  # Will store the calculated Sea Level of the floor

def on_gps_int(ac_id, msg):
    global GROUND_REF_AMSL
    if msg.name != "GPS_INT": return
    ac_id = int(ac_id)
    uav = ensure_uav(ac_id)

    # 1. Raw GPS Data
    lat = float(msg["lat"]) * 1e-7
    lon = float(msg["lon"]) * 1e-7
    alt_amsl = float(msg["alt"]) / 1000.0  # Raw Height above Mean Sea Level (e.g. 45.2m)

    # 2. Get X, Y (Standard conversion)
    x, y, _ = pm.geodetic2enu(lat, lon, alt_amsl, LAT0, LON0, ALT0)

    # 3. Handle Z Calibration (One-time setup)
    if GROUND_REF_AMSL is None:
        # We use the FIRST drone seen to establish where the "Floor" is.
        # Look up this drone's known start height (default to 0.0 if unknown)
        start_h = KNOWN_START_HEIGHTS.get(ac_id, 0.0)
        
        # Calculate Ground AMSL: (Current GPS Alt) - (Physical Height)
        GROUND_REF_AMSL = alt_amsl - start_h
        
        print(f"\n[CALIBRATION] Global Ground Reference set!")
        print(f"  -> Based on AC{ac_id} reading {alt_amsl:.2f}m AMSL")
        print(f"  -> Known physical height was {start_h:.2f}m")
        print(f"  -> Calculated Ground Level: {GROUND_REF_AMSL:.2f}m AMSL\n")

    # 4. Calculate Z relative to the calculated Ground Level
    # This works for ALL drones now, regardless of their height.
    z_agl = alt_amsl - GROUND_REF_AMSL

    with LOCK:
        uav["state"]["x"] = x
        uav["state"]["y"] = y
        uav["state"]["z"] = z_agl

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


def on_pprz_msg(ac_id, msg):
    """
    Handles incoming Ivy messages. 
    Correct signature for pprzlink: (ac_id, msg)
    """
    # 1. Ensure ID is an integer
    try:
        ac_id = int(ac_id)
    except (ValueError, TypeError):
        return

    with LOCK:
        # ---------------------------------------------------------
        # 1. DISCOVERY: Register ANY new drone immediately
        # ---------------------------------------------------------
        if ac_id not in UAVS:
            # Prevent clutter by not printing every single message type for new drones
            print(f"[IVY] Found AC{ac_id} (msg: {msg.name})")
            UAVS[ac_id] = {
                'state': np.zeros(6),       # [x, y, z, vx, vy, vz]
                'accel': np.zeros(3),
                'wind': np.zeros(2),
                'status': {'bat': 12.0, 'flight': True}, 
                'last_update': time.time(),
                'msg_type': 'UNKNOWN'
            }

        # Keep alive
        UAVS[ac_id]['last_update'] = time.time()

        # ---------------------------------------------------------
        # 2. DATA PARSING
        # ---------------------------------------------------------
        
        # A. ROTORCRAFT_FP (High Precision)
        if msg.name == "ROTORCRAFT_FP":
            UAVS[ac_id]['msg_type'] = 'FP'
            # Convert units
            new_pos = np.array([float(msg['east']), float(msg['north']), float(msg['up'])]) * POS_SCALE
            new_vel = np.array([float(msg['veast']), float(msg['vnorth']), float(msg['vup'])]) * VEL_SCALE
            
            # Simple jump filter (ignore 0,0,0 glitches)
            if np.linalg.norm(new_pos) > 0.1:
                UAVS[ac_id]['state'] = np.concatenate((new_pos, new_vel))

        # B. INS (Fallback)
        elif msg.name == "INS" and UAVS[ac_id]['msg_type'] != 'FP':
            # Only use if we haven't seen FP yet
            ix = float(msg["ins_x"]) * POS_SCALE
            iy = float(msg["ins_y"]) * POS_SCALE
            iz = float(msg["ins_z"]) * POS_SCALE
            
            # Convert NED -> ENU (North=Y, East=X, Up=-Z)
            # Standard Paparazzi INS is NED
            pos_enu = np.array([iy, ix, -iz]) 
            
            ivx = float(msg["ins_xd"]) * VEL_SCALE
            ivy = float(msg["ins_yd"]) * VEL_SCALE
            ivz = float(msg["ins_zd"]) * VEL_SCALE
            vel_enu = np.array([ivy, ivx, -ivz])
            
            if np.linalg.norm(pos_enu) > 0.1:
                UAVS[ac_id]['state'] = np.concatenate((pos_enu, vel_enu))

        # C. WIND INFO
        elif msg.name == "WIND_INFO":
            # flags bit 0 usually means "active"
            if 'flags' in msg.fieldnames and (int(msg['flags']) & 1):
                UAVS[ac_id]['wind'] = np.array([float(msg['east']), float(msg['north'])])

        # D. STATUS (Battery)
        elif msg.name == "ROTORCRAFT_STATUS":
            if 'vsupply' in msg.fieldnames:
                UAVS[ac_id]['status']['bat'] = float(msg['vsupply']) / 10.0
            if 'ap_in_flight' in msg.fieldnames:
                UAVS[ac_id]['status']['flight'] = (int(msg['ap_in_flight']) == 1)

def on_msg(ac_id, msg):
    """
    Handles incoming Ivy messages.
    Switches to ROTORCRAFT_FP for smooth, glitch-free control.
    """
    with LOCK:
        # Initialize structure if new drone appears
        if ac_id not in UAVS:
            UAVS[ac_id] = {
                'state': np.zeros(6),       # [x, y, z, vx, vy, vz] (Required by your planner)
                'accel': np.zeros(3),       # [ax, ay, az] (For smooth braking)
                'wind': np.zeros(2),        # [wind_x, wind_y] (For victim prediction)
                'heading': 0.0,             # Yaw in degrees
                'status': {'bat': 0, 'flight': False}, 
                'last_update': time.time()
            }

        # --- 1. POSITION & VELOCITY (The Glitch Fix) ---
        if msg.name == "ROTORCRAFT_FP":
            # Extract filtered data (Smoothed by Drone EKF)
            # This removes the 200ms lag and the 165m/s spikes
            new_pos = np.array([float(msg['east']), float(msg['north']), float(msg['up'])]) * POS_SCALE
            new_vel = np.array([float(msg['veast']), float(msg['vnorth']), float(msg['vup'])]) * VEL_SCALE
            
            # Heading (Critical for correct scanning direction)
            heading_deg = float(msg['psi']) * ANGLE_SCALE
            
            # SANITY CHECK: Block teleportation glitches > 5 meters
            # This is the "Safety Net" against bad OptiTrack data
            current_pos = UAVS[ac_id]['state'][:3]
            dist_jump = np.linalg.norm(new_pos - current_pos)
            
            # Update only if jump is physical (or if it's the first reading)
            if dist_jump < 5.0 or np.linalg.norm(current_pos) == 0:
                UAVS[ac_id]['state'] = np.concatenate((new_pos, new_vel))
                UAVS[ac_id]['heading'] = heading_deg
                UAVS[ac_id]['last_update'] = time.time()

        # --- 2. ACCELERATION (For Smoother Tracking) ---
        elif msg.name == "INS":
            # Knowing acceleration lets you predict stopping distance
            ax = float(msg['ins_xdd']) * ACCEL_SCALE
            ay = float(msg['ins_ydd']) * ACCEL_SCALE
            az = float(msg['ins_zdd']) * ACCEL_SCALE
            UAVS[ac_id]['accel'] = np.array([ax, ay, az])

        # --- 3. WIND ESTIMATION (For Victim Prediction) ---
        elif msg.name == "WIND_INFO":
            # Flag bit 0 checks if horizontal wind is valid
            if (int(msg['flags']) & 1):
                # The wind vector blowing ON the drone
                w_e = float(msg['east'])
                w_n = float(msg['north'])
                UAVS[ac_id]['wind'] = np.array([w_e, w_n])

        # --- 4. SAFETY & STATUS ---
        elif msg.name == "ROTORCRAFT_STATUS":
            # Automatically detect if drone is flying and check battery
            UAVS[ac_id]['status']['bat'] = float(msg['vsupply']) / 10.0
            UAVS[ac_id]['status']['flight'] = (int(msg['ap_in_flight']) == 1)

def sync_drone_state(current_searcher_ids):
    """ 
    Updates global arrays from the UAVS dict safely.
    Moves Virtual Victims = Manual Drift + Real Measured Wind
    """
    global drone_positions, drone_vels, drone_modes, victims
    
    avg_wind = np.zeros(2)
    wind_count = 0
    dt = 0.1 # Simulation step size

    with LOCK:
        # 1. Update SEARCHER Drones (Real Telemetry)
        for i, ac_id in enumerate(current_searcher_ids):
            if ac_id in UAVS:
                state = UAVS[ac_id]['state']
                
                # Update global arrays
                if i < len(drone_positions):
                    drone_positions[i] = state[:3]
                    drone_vels[i] = state[3:]
                    
                    if not UAVS[ac_id]['status']['flight']:
                        drone_modes[i] = "GROUND"

                # Aggregate Real Wind Data
                w = UAVS[ac_id]['wind']
                if np.linalg.norm(w) > 0.1: 
                    avg_wind += w
                    wind_count += 1
                    
        # 2. Update VICTIMS
        if HAS_REAL_VICTIM:
            # Case A: Real Victims (Trust their Telemetry)
            if 'victims' in globals():
                for v_idx, v_id in enumerate(VICTIM_IDS):
                    if v_id in UAVS and v_idx < len(victims):
                         victims[v_idx] = UAVS[v_id]['state'][:2]
        else:
            # Case B: Virtual Victims (Simulate Physics)
            # Movement = Manual Drift + Real Wind Effect
            
            # Start with your manual drift (e.g. water current)
            total_drift = v_drift.copy() 
            
            # Add real wind bias if drones detect it
            if wind_count > 0:
                measured_wind = avg_wind / wind_count
                # You can tune this "0.5" factor (how much wind pushes the victim)
                total_drift += measured_wind * 0.5 
            
            # Apply to all virtual victims
            # (Assuming victims is a numpy array of shape [N, 2])
            for i in range(len(victims)):
                victims[i] += total_drift * dt


def send_pprz_velocity(interface, ac_id, vx_enu, vy_enu, vz_enu):
    if interface is None: return

    # 1. Convert ENU -> NED
    vx_ned = vy_enu   # North 
    vy_ned = vx_enu   # East
    vz_ned = -vz_enu  # Down

    if (vx_ned**2 + vy_ned**2) > 0.001: 
        desired_yaw = math.atan2(vy_ned, vx_ned)
    else:
        desired_yaw = 0.0

    msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
    msg['ac_id'] = int(ac_id)
    msg['flags'] = 0x60 
    
    msg['x'] = float(vx_ned)
    msg['y'] = float(vy_ned)
    msg['z'] = float(vz_ned)
    msg['yaw'] = float(desired_yaw)  # <--- Send calculated heading
    
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

dt_step = 0.2 #was 1.0
v_drift = np.array([0.06, 0.0])
theta_FOV = np.deg2rad(25)
E_scale = 0.1
E_scale_track = 0.04
gamma_wind = 2.0
v_wind = np.array([0.0, 0.0])
v_max = 1.0        # max velocity [m/s]

confirm_pconf = 0.6
peak_tresh = 0.75
mean_thresh = 0.4
rho_th = 0.25

# -----------------------------
# CORE FUNCTIONS
# -----------------------------

# --- Sensor model parameters (sec. 7.3.1) ---
h_ref = 1.0  # [m] altitude at which sensor reaches max resolution (confirmation height)
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
    

def energy_of_path(path):
    # Use the global wind vector
    global v_wind 
    
    total_energy = 0.0
    v_climb, v_desc = 0.7, 0.8
    P_climb = get_vsqp_power(0.0) # 1751 W

    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i+1]
        vec_g = (p1[:2] - p0[:2])
        dist_xy = np.linalg.norm(vec_g)
        dz = p1[2] - p0[2]

        if dist_xy > 1e-3:
            # Calculate the ground velocity vector for this segment
            # Note: We use the wind-aware vg we just fixed in the planner
            unit_vec = vec_g / (dist_xy + 1e-6)
            v_headwind_comp = np.dot(v_wind[:2], unit_vec)
            vg_allowed = max(2.0, min(2.0, 2.0 + v_headwind_comp))
            
            v_ground_vec = unit_vec * vg_allowed
            
            # Find the resulting airspeed vector
            v_air_vec = v_ground_vec - v_wind[:2]
            v_air_mag = np.linalg.norm(v_air_vec)
            
            # Get power based on AIRSPEED
            P_segment = get_vsqp_power(v_air_mag)
            
            t_segment = dist_xy / vg_allowed
            E_horiz = P_segment * t_segment
        else:
            E_horiz = 0.0

        # Vertical Energy (unchanged)
        if dz > 0:   
            E_vert = P_climb * (dz / v_climb)
        elif dz < 0: 
            E_vert = P_climb * (-dz / v_desc) * 0.5
        else:        
            E_vert = 0.0
        
        total_energy += (E_horiz + E_vert)
        
    return total_energy

# ==============================================================================
# 2. THE PLANNER (Supports 'constraint_poly' and 'drone_vel')
# ==============================================================================
@jit(nopython=True)
def get_p_hit_numba(h, h_ref=1.0, P0=0.9):
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
        n_samples = int(np.ceil(dist / 1.0))
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

# ==============================================================================
# 2. UPDATED PLANNER FUNCTION (CALLS NUMBA)
# ==============================================================================

def plan_velocity_ipp_3D(drone_pos, drone_vel, belief_map, inside_mask, grid_origin, grid_resolution, soft_poly,
                         constraint_poly=None,
                         fov_angle=theta_FOV, v_max=1.0, n_directions=8,
                         step_length=2.0, altitude_candidates=[2, 3, 4],
                         E_scale=100.0,    
                         lam=0.4,          
                         buffer=0.0, pred_depth=2):

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
        small_step = 1.0
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
    v_g_allowed = max(0.7, min(1.0, 1.0 + v_headwind_comp))
    
    # Calculate velocity components
    travel_time = max(dist / v_g_allowed, 0.1)
    
    vx = vec[0] / travel_time
    vy = vec[1] / travel_time
    vz = vec[2] / travel_time

    return float(vx), float(vy), float(vz), best_path, None, best_J


def tracking_region(t_now, detection_pos, detection_time, v_drift,
                    base_radius=1.0, uncertainty_gain=1.0,
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
                              major_axis=1.0, minor_axis=1.0, decay=0.01):
    
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
            poly_buffered = region_poly.buffer(0.7)
            
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

def apply_path_to_belief(belief_map, path, grid_origin, grid_resolution, fov_tan, inside_mask):
    """
    Paints the path onto the belief map as 'scanned' (0.001) so other drones
    don't try to go there in the same timestep.
    """
    x_min, y_min = grid_origin
    rows, cols = belief_map.shape
    
    for i in range(len(path)-1):
        p0, p1 = path[i], path[i+1]
        dist = np.linalg.norm(p1[:2] - p0[:2])
        n_samples = max(1, int(dist / 0.5))
        
        for k in range(n_samples+1):
            alpha = k / n_samples
            pos = p0 * (1-alpha) + p1 * alpha
            
            # Simple square approximation of FOV for reservation
            r_m = pos[2] * fov_tan
            r_cells = int(np.ceil(r_m / grid_resolution))
            
            c_cen = int((pos[0] - x_min) / grid_resolution)
            r_cen = int((pos[1] - y_min) / grid_resolution)
            
            r_start = max(0, r_cen - r_cells)
            r_end = min(rows, r_cen + r_cells + 1)
            c_start = max(0, c_cen - r_cells)
            c_end = min(cols, c_cen + r_cells + 1)
            
            # Mark as low probability (effectively "reserved/scanned")
            belief_map[r_start:r_end, c_start:c_end] = 0.001

    return belief_map

victims_template = np.array([[-1.0,-2.0],[-3.0,0.0], [0.0, 0.0], [2.0, -1.0]])
DISCOVERED_POSITIONS = {}

VICTIM_IDS = [123]  

if __name__ == "__main__":
    # --- SIMULATION CONFIG ---
    USE_INTERNAL_PHYSICS = False  # Assume Real Flight first

    # ---------------------------------------------------------
    # 1. IVY DISCOVERY (Fixed Import + Smart Wait)
    # ---------------------------------------------------------
    try:
        # [FIX] Use the import that works on your system (pprzlink)
        from pprzlink.ivy import IvyMessagesInterface
        
        # Connect
        ivy_bus = os.getenv("PPRZ_IVY_BUS", "127.255.255.255:2010")
        ivy = IvyMessagesInterface("cyberzoo_planner", ivy_bus=ivy_bus)
        ivy.subscribe(on_pprz_msg)
        ivy.start()
        print(f"[BRIDGE] Listening on {ivy_bus}...")
        
        # --- SMART WAIT FOR VICTIM (AC 123) ---
        print(f"Waiting for Victim(s) {VICTIM_IDS}...")
        waiting_start = time.time()
        
        while True:
            # 1. Check who we have seen
            seen_ids = list(UAVS.keys())
            
            # 2. Check if victim is present
            found_victims = [vid for vid in VICTIM_IDS if vid in seen_ids]
            
            if len(found_victims) == len(VICTIM_IDS):
                print(f" -> SUCCESS: Found all victims: {found_victims}")
                break
            
            # 3. Timeout Logic (10 seconds)
            if time.time() - waiting_start > 10.0:
                 print(f"[WARN] Timed out waiting for {VICTIM_IDS}. Proceeding with: {seen_ids}")
                 break
                 
            time.sleep(0.5)

        # --- CLASSIFICATION ---
        ALL_IDS = sorted(list(UAVS.keys()))
        
        # A. Identify Real Victims
        active_victim_ids = [ac for ac in ALL_IDS if ac in VICTIM_IDS]
        
        if active_victim_ids:
            HAS_REAL_VICTIM = True
            print(f"[BRIDGE] Real Victims Active: {active_victim_ids}")
            
            real_victim_positions = []
            for v_id in active_victim_ids:
                if v_id in UAVS:
                    pos = UAVS[v_id]['state'][:3]
                    real_victim_positions.append(pos[:2])
            victims = np.array(real_victim_positions)
        else:
            HAS_REAL_VICTIM = False
            print(f"[WARN] No Real Victim Drones found. Using Virtual.")
            victims = victims_template[:2]

        # B. Identify Searchers (Everyone NOT in VICTIM_IDS)
        AC_IDS = [ac for ac in ALL_IDS if ac not in VICTIM_IDS]

        if not AC_IDS:
            print("[WARN] No SEARCH drones found. Defaulting to Sim IDs [121, 122].")
            AC_IDS = [121, 122]
            # Fake init positions for simulation
            ensure_uav(121); ensure_uav(122)
            UAVS[121]['state'] = np.array([-3.0, -2.0, 1.5, 0,0,0])
            UAVS[122]['state'] = np.array([3.0, 2.0, 1.5, 0,0,0])
            USE_INTERNAL_PHYSICS = True
        else:
            print(f"[BRIDGE] Active Search Drones: {AC_IDS}")
            USE_INTERNAL_PHYSICS = False

    except ImportError:
        print("[CRITICAL ERR] Could not import 'pprzlink'. Is it installed?")
        sys.exit(1)
        
    except Exception as e:
        print(f"[ERR] Ivy Setup Failed: {e}")
        # Fallback
        AC_IDS = [121, 122]
        HAS_REAL_VICTIM = False
        victims = victims_template[:2]
        USE_INTERNAL_PHYSICS = True
        
        # 2. Force Separate Starting Positions (Sim Only)
        # We manually populate the UAVS dict so the rest of the code works
        with LOCK:
            # Drone 1: Bottom Left
            UAVS[121] = {
                'state': np.array([-3.0, -2.0, 1.5, 0, 0, 0]), 
                'wind': np.zeros(2),
                'status': {'bat': 12.0, 'flight': True}
            }
            # Drone 2: Top Right
            UAVS[122] = {
                'state': np.array([3.0, 2.0, 2.0, 0, 0, 0]), 
                'wind': np.zeros(2),
                'status': {'bat': 12.0, 'flight': True}
            }

    # --- D. Setup Search Drones ---
    num_drones = len(AC_IDS)
    drone_positions = []
    uav_nominal_altitudes = []
    
    # Initialize from the UAVS dictionary
    for ac_id in AC_IDS:
        # Create entry if missing (for simulation fallback)
        if ac_id not in UAVS:
            with LOCK:
                 UAVS[ac_id] = {
                    'state': np.zeros(6), 
                    'wind': np.zeros(2),
                    'status': {'bat': 12.0, 'flight': False}
                 }
        
        # Get start position
        with LOCK:
            s = UAVS[ac_id]['state']
            # Default to (0,0,1) if the drone hasn't sent data yet
            if np.all(s == 0): 
                pos = np.array([0.0, 0.0, 1.0])
            else:
                pos = np.array([s[0], s[1], max(0.5, s[2])])
        
        print(f" -> Init Searcher AC{ac_id} at {pos}")
        drone_positions.append(pos)
        uav_nominal_altitudes.append(pos[2])

    uav_nominal_altitudes = np.array(uav_nominal_altitudes)
    drone_vels = [np.zeros(3) for _ in range(num_drones)]
    drone_modes = ['explore'] * num_drones
    track = [{"active":False, "confirmed":False, "pos":None, "time":None, "phase":None} for _ in range(num_drones)]
    pred_depth = 1
    # Assume all UAVs start with 100% energy (normalized)
    E_max = 100.0
    E_rem = np.ones(num_drones) * E_max

    # ...
    cmap = cm.RdYlBu_r.copy()  # Create a copy so we can modify it safely
    cmap.set_bad(color='white')

    # --- Main occupancy map figure ---
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
    ax_map.set_xlim(x_min - 1, x_max + 1)
    ax_map.set_ylim(y_min - 1, y_max + 1)
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
    buffer = 0.7 # safe buffer for polygon

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
    initial_lock_duration = 5.0   # Seconds to follow cone center before spiraling
    MIN_ALT = 0.5                # Minimum flight altitude
    DESCENT_STEP = 0.2            # Meters per step to descend
    # --- Tracking timing parameters ---
    cone_search_timeout   = 15.0   # max time doing IPP inside cone before abort
    HOVER_CONFIRM_TIME = 7.0  # seconds at low altitude before abort if not confirmed
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
    # DATA LOGGING SETUP
    # -----------------------------
    # -----------------------------
    # DATA LOGGING SETUP (Dual File)
    # -----------------------------
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # File 1: Searcher Drone Data
    log_uav_filename = f"log_uavs_real_{timestamp_str}.csv"
    f_uav = open(log_uav_filename, mode='w', newline='')
    writer_uav = csv.writer(f_uav)
    writer_uav.writerow([
        "timestamp", "uav_id", 
        "x", "y", "z", 
        "vx", "vy", "vz", 
        "mode", "target_est_x", "target_est_y", "loop_rate"
    ])
    
    # File 2: True Victim Ground Truth
    log_vic_filename = f"log_victims_{timestamp_str}.csv"
    f_vic = open(log_vic_filename, mode='w', newline='')
    writer_vic = csv.writer(f_vic)
    writer_vic.writerow(["timestamp", "victim_id", "true_x", "true_y"])
    
    print(f"[LOG] Logging UAVs to:    {log_uav_filename}")
    print(f"[LOG] Logging Victims to: {log_vic_filename}")

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
            t = now - start_time # Update total simulation time
            dt = now - last_loop_time
            last_loop_time = now
            
            # (If lag > 0.2s, we pretend it was only 0.2s to stop teleporting)
            dt = np.clip(dt, 0.001, 0.2) 

            # =========================================================
            # STEP 0: SYNC STATE (INPUT)
            # =========================================================
            # [FIXED] The new function updates global arrays directly!
            # It also handles the "USE_INTERNAL_PHYSICS" check internally.
            sync_drone_state(AC_IDS)

            # Debug Print (Optional - Every 10 frames)
            # [FIX] Loop through all drones for the debug print
            if frame_count % 10 == 0:
                for i, ac_id in enumerate(AC_IDS):
                    if i < len(drone_positions):
                        p = drone_positions[i]
                        m = drone_modes[i]
                        # Print condensed info
                        print(f"[SYNC] AC{ac_id} Pos: [{p[0]:.2f}, {p[1]:.2f}] Mode: {m}")

            # ------------------------------------------
            # 0. Dynamic mapping (Drift the Search Grid)
            # ------------------------------------------
            # This shifts the probability map to follow the water current
            shift_y = v_drift[1] * dt / grid_resolution
            shift_x = v_drift[0] * dt / grid_resolution
            
            # Shift the belief map
            # Note: order=1 is faster, order=3 is smoother. 1 is fine for real-time.
            belief_map = shift(belief_map, shift=[shift_y, shift_x], order=1, cval=0.5)
            
            # Reset boundaries (Prevents map shrinking artifacts)
            belief_map[~inside_mask] = 0.5 
            
            # INCREASE SIGMA: Makes "cleared" areas decay back to "unknown" faster
            # This represents "uncertainty growing over time"
            belief_map = gaussian_filter(belief_map, sigma=1.5)
            # ------------------------------------------


            # ------------------------------------------
            # 1. Victim motion (Commands for REAL drones)
            # ------------------------------------------
            # NOTE: Virtual victims are moved in Step 0 (sync_drone_state), 
            # so we ONLY need to handle REAL victim commands here.
            
            if HAS_REAL_VICTIM:
                for i, v_id in enumerate(active_victim_ids):
                    # A. Get current state (Directly from UAVS dict)
                    if v_id in UAVS:
                        vic_pos = UAVS[v_id]['state'][:3]
                        
                        # B. Altitude Hold Logic
                        # Target: 0.5 meters
                        target_z = 0.5  
                        z_err = target_z - vic_pos[2]
                        
                        # P-Controller: Correct drift with vertical velocity
                        # Limit max climb/sink to 0.2 m/s for safety
                        vz_hold = np.clip(z_err * 1.0, -0.2, 0.2)
                        
                        # C. Send Command (Drift XY + Hold Z)
                        # We send the manual v_drift preference, but you could add wind compensation here too
                        send_pprz_velocity(ivy, v_id, v_drift[0], v_drift[1], vz_hold)
                        
                        # D. Update internal map variable (Redundant but safe)
                        victims[i] = vic_pos[:2]

            # ------------------------------------------
            # 4. Bayesian mapping update (Per UAV)
            # ------------------------------------------
            
            # Reset detection events for this timestep
            detection_event = [False] * num_drones

            for d_idx in range(num_drones):
                drone_pos = drone_positions[d_idx]
                h = drone_pos[2]
                
                # --- A. Get the "View Window" ---
                # (Assumes you have get_view_window defined elsewhere)
                (r_slice, c_slice), r_m, (rc, cc) = get_view_window(drone_pos, theta_FOV)
                
                # Extract local patches
                local_belief = belief_map[r_slice, c_slice]
                local_mask   = inside_mask[r_slice, c_slice] 
                
                if local_belief.size == 0: continue

                # --- B. Visibility Check ---
                local_Y, local_X = np.meshgrid(grid_y[r_slice], grid_x[c_slice], indexing='ij')
                dx = local_X - drone_pos[0]
                dy = local_Y - drone_pos[1]
                d2 = dx*dx + dy*dy
                vis_mask = (d2 <= r_m*r_m) & local_mask
                
                if not np.any(vis_mask): continue
                
                # --- C. "True" Victim Detection (Oracle) ---
                # Check distance to ALL victims (Real or Virtual)
                # We use the updated 'victims' array from Step 0
                vic_dists_sq = np.sum((victims[:, :2] - drone_pos[:2])**2, axis=1)
                
                # Is any victim inside the FOV radius?
                victims_in_fov = vic_dists_sq <= (r_m + grid_resolution)**2
                
                # Calculate Detection Probability
                p_hit_val = get_p_hit_numba(h, h_ref, P0)
                
                if np.any(victims_in_fov):
                    # Victim is physically there!
                    p_detect = min(1.0, p_hit_val + 0.4)
                else:
                    p_detect = P_false
                
                # Roll the dice
                is_hit = (np.random.rand() < p_detect)
                detection_event[d_idx] = is_hit
                
                # --- D. Bayesian Update ---
                p_prior = local_belief[vis_mask]
                
                if is_hit:
                    num = p_prior * p_hit_val
                    den = num + (1 - p_prior) * P_false
                else:
                    num = p_prior * (1 - p_hit_val)
                    den = 1 - p_prior * p_hit_val
                
                p_post = num / (den + 1e-12)
                local_belief[vis_mask] = np.clip(p_post, 1e-6, 1 - 1e-6)

            # ------------------------------------------
            # 5. Centralized detection & tracker assignment
            # ------------------------------------------

            # [NEW] Calculate Global Wind for Cost Function
            # We need this for the wind penalty in assignment
            total_wind = np.zeros(2)
            w_count = 0
            for ac_id in AC_IDS:
                if ac_id in UAVS: 
                    total_wind += UAVS[ac_id]['wind']
                    w_count += 1
            current_wind = total_wind / w_count if w_count > 0 else np.zeros(2)

            # Select eligible explorers
            eligible_explorers = [
                d_idx for d_idx, mode in enumerate(drone_modes)
                if mode == "explore" and t >= cooldown_until[d_idx]
            ]

            # Check for NEW detections
            NEW_DETECTION = False
            detection_uav = None
            detection_pos = None

            for d_idx in eligible_explorers:
                if not detection_event[d_idx]: continue

                # Localize signal
                (r_slice, c_slice), r_m, _ = get_view_window(drone_positions[d_idx])
                local_patch = belief_map[r_slice, c_slice]
                if local_patch.size == 0: continue

                flat_idx = np.argmax(local_patch)
                max_r, max_c = np.unravel_index(flat_idx, local_patch.shape)
                
                det_x = grid_x[c_slice.start + max_c]
                det_y = grid_y[r_slice.start + max_r]
                candidate_pos = np.array([det_x, det_y])

                # Distance check to existing targets
                is_too_close = False
                for tr in track:
                    if tr["active"] and tr["pos"] is not None:
                         if np.linalg.norm(candidate_pos - tr["pos"]) < 2.0:
                             is_too_close = True
                             break
                
                if is_too_close: continue

                detection_pos = candidate_pos
                NEW_DETECTION = True
                detection_uav = d_idx
                break


            # Process Detection
            if NEW_DETECTION and detection_uav is not None:
                print(f"[t={t:.1f}] TRUE DETECTION at {detection_pos}")

                best_d, best_cost = None, np.inf

                for d_idx in eligible_explorers:
                    r = detection_pos - drone_positions[d_idx][:2]
                    dist = np.linalg.norm(r)
                    r_hat = r / (dist + 1e-6)
                    
                    # [UPDATED] Use 'current_wind' calculated above
                    Deff = dist + gamma_wind * abs(np.dot(current_wind, r_hat))

                    if Deff < best_cost:
                        best_cost = Deff
                        best_d = d_idx

                if best_d is not None:
                    tr_assign = track[best_d]
                    tr_assign["active"] = True
                    tr_assign["confirmed"] = False
                    tr_assign["pos"] = detection_pos.copy()
                    tr_assign["pos_at_detection"] = detection_pos.copy()
                    tr_assign["time"] = t
                    tr_assign["phase"] = "to_detection"

                    drone_modes[best_d] = "track"
                    cooldown_until[best_d] = t + COOLDOWN_TIME

                    print(f"[t={t:.1f}] Assigning UAV{best_d} to track region.")


            # ------------------------------------------
            # 6. Plan next velocities (Logic Updated for 2D Arrays)
            # ------------------------------------------
            commanded_vels = np.zeros((num_drones, 3))

            planning_belief = belief_map.copy()

            for d_idx in range(num_drones):
                tr = track[d_idx]
                #  Define current_pos here so it works in all blocks
                current_pos = drone_positions[d_idx] 
                current_alt = current_pos[2]

                # --- A. EXPLORATION MODE ---
                if drone_modes[d_idx] == "explore":

                    target_alt = uav_nominal_altitudes[d_idx]

                    vx_des, vy_des, vz_des, best_path, best_mask, _= plan_velocity_ipp_3D(
                        drone_positions[d_idx], 
                        drone_vels[d_idx],
                        planning_belief,           # <--- 2D Matrix
                        inside_mask,          # <--- 2D Mask
                        (x_min, y_min),       # <--- Origin
                        grid_resolution,      # <--- Res
                        soft_poly,
                        constraint_poly=None,
                        step_length=2.0,
                        fov_angle=theta_FOV, v_max=v_max, n_directions=8,
                        altitude_candidates=[target_alt], pred_depth=pred_depth, E_scale=E_scale,
                        lam=0.5,        
                        buffer=buffer
                    )
                    # Maintain nominal altitude

                    if best_mask is not None:
                        planning_belief[best_mask] = 0.001

                    # This simple P-controller ensures we stick to the altitude
                    # even if the planner's velocity output drifts slightly.
                    z_err = target_alt - drone_positions[d_idx][2]

                
                    vz_des = np.clip(z_err * 0.8, -0.5, 0.5)

                    # if abs(z_err) > 0.1:
                    #     vz_des = np.clip(z_err, -0.5, 0.5)  

                    

                # --- B. TRACKING MODE ---
                elif drone_modes[d_idx] == "track" and tr["active"] and not tr["confirmed"]:

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
                    # if tr["phase"] == "to_detection":
                    #     t_elapsed = t - tr["time"]
                    #     raw_target = tr["pos"] + v_drift * t_elapsed
                        
                    #     # 1. Geofence Clamp
                    #     target_point = Point(raw_target[0], raw_target[1])
                    #     if not soft_poly.contains(target_point):
                    #         nearest_pt = soft_poly.exterior.interpolate(
                    #             soft_poly.exterior.project(target_point))
                    #         target_now = np.array([nearest_pt.x, nearest_pt.y])
                    #     else:
                    #         target_now = raw_target
                        
                    #     vec = target_now - drone_positions[d_idx][:2]
                    #     dist = np.linalg.norm(vec)

                    #     # 2. PREDICTIVE BRAKING (The Fix)
                    #     # Slow down proportional to distance so we don't overshoot.
                    #     if dist > 2.0:
                    #         # Far away? Cruise at max speed.
                    #         vx_des, vy_des = (vec / (dist + 1e-6)) * 2.0
                    #     else:
                    #         # Closer than 15m? Slow down.
                    #         # At 15m -> 7.5 m/s
                    #         # At 2m  -> 1.0 m/s
                    #         # speed = np.clip(dist * 1.5, 1.0, 20.0)
                    #         # Gain 0.8 requires ~2.5 m/s² braking (Feasible)
                    #         speed = np.clip(dist * 0.5, 0.4, 1.0)
                    #         vx_des, vy_des = (vec / (dist + 1e-6)) * speed
                        
                    #     vz_des = 0.0

                    #     # Switch to cone tracking when VERY close
                    #     if dist < 0.6:
                    #         tr["phase"] = "cone_tracking"
                    #         tr["cone_start"] = t

                    # # --- PHASE: INITIAL LOCK ---
                    # elif tr["phase"] == "initial_lock":
                    #     det_pos = tr["pos"]
                    #     det_time = tr["time"]

                    #     # Follow drifting cone center
                    #     region_poly, center_R, _ = tracking_region(t, det_pos, det_time, v_drift)
                    #     vec_center = center_R - drone_positions[d_idx][:2]
                    #     dist_center = np.linalg.norm(vec_center)

                    #     if dist_center > 1.0:
                    #         dir_unit = vec_center / (dist_center + 1e-6)
                    #         vx_des, vy_des = dir_unit * 1.5
                    #     else:
                    #         vx_des, vy_des = 0.0, 0.0

                    #     vz_des = 0.0 

                    #     if (t - tr["lock_start"]) >= initial_lock_duration:
                    #         tr["phase"] = "cone_tracking"
                    #         tr["cone_start"] = t

                    # # --- PHASE: CONE TRACKING (IPP inside cone) ---
                    # elif tr["phase"] == "cone_tracking":
                    #     det_pos = tr["pos"]
                    #     det_time = tr["time"]

                    #     # 1. Update Cone
                    #     region_poly, center_R, _ = tracking_region(t, det_pos, det_time, v_drift)
                        
                    #     # Safety check
                    #     if region_poly.is_empty:
                    #         tr["active"] = False; drone_modes[d_idx] = "explore"; continue

                    #     # 2. Rasterize Cone (Check Belief Stats FIRST)
                    #     poly_path = MplPath(list(region_poly.exterior.coords))
                    #     points_flat = np.vstack((XX.flatten(), YY.flatten())).T
                    #     mask_flat = poly_path.contains_points(points_flat)
                    #     in_R_mask = mask_flat.reshape(rows, cols)
                        
                    #     # 3. Calculate Stats & Logic
                    #     if np.any(in_R_mask):
                    #         p_vals = belief_map[in_R_mask]
                    #         frac_high = np.mean(p_vals > confirm_pconf)
                    #         mean_p = np.mean(p_vals)
                    #         max_p = np.max(p_vals)

                    #         # Success -> Hover
                    #         if (frac_high >= rho_th or mean_p >= 0.25 or max_p >= peak_tresh):
                    #             tr["phase"] = "hover_confirm"
                    #             tr["fail_timer"] = 0.0
                    #             tr["pos"] = drone_positions[d_idx][:2].copy() # Anchor
                    #             tr["time"] = t
                    #             tr["hover_start"] = t
                    #             tr["target_smooth"] = None # Reset filter
                    #             continue

                    #         # Timeout/Failure Logic
                    #         # Allow 30s max for tracking
                    #         if (t - tr["cone_start"]) >= 30.0 and max_p < 0.40:
                    #             tr["active"] = False; drone_modes[d_idx] = "explore"
                    #             vz_des = 0.5
                    #             continue

                    #     # 4. Motion Planning
                        
                    #     # [CRITICAL FIX] Distance Check to Cone Center
                    #     # If we are falling behind the drifting cone, forget IPP and CHASE IT.
                    #     err_vec = center_R - drone_positions[d_idx][:2]
                    #     dist_err = np.linalg.norm(err_vec)
                        
                    #     if dist_err > 1.5:
                    #         # Chase Mode: Fly directly to center
                    #         speed = 1.5 # Fast recovery
                    #         vx_des, vy_des = (err_vec / dist_err) * speed
                    #     else:
                    #         # Search Mode: IPP inside the cone
                    #         soft_cone_constraint = region_poly.buffer(0.5)
                            
                    #         vx_ipp, vy_ipp, vz_ipp, _, _, _ = plan_velocity_ipp_3D(
                    #             drone_positions[d_idx], 
                    #             drone_vels[d_idx],
                    #             belief_map, inside_mask,
                    #             (x_min, y_min), grid_resolution,
                    #             soft_poly,
                    #             constraint_poly=soft_cone_constraint,
                    #             step_length=1.5,         
                    #             fov_angle=theta_FOV, 
                    #             v_max=0.8,               # [TUNED] Faster search speed
                    #             altitude_candidates=[drone_positions[d_idx][2]], 
                    #             pred_depth=1, E_scale=E_scale_track,
                    #             lam=0.5,        
                    #             buffer=buffer
                    #         )
                            
                    #         # If planner returns 0 (stuck), force small movement
                    #         if abs(vx_ipp) < 0.01 and abs(vy_ipp) < 0.01:
                    #              vx_des, vy_des = (err_vec / (dist_err+1e-6)) * 0.5
                    #         else:
                    #              vx_des, vy_des = vx_ipp, vy_ipp

                    #     # [CRITICAL FIX] Feedforward Drift
                    #     # Add the victim's drift velocity to our command so we don't lag behind
                    #     vx_des += v_drift[0] 
                    #     vy_des += v_drift[1] 

                    #     # Maintain Altitude or Slow Descent
                    #     if np.any(in_R_mask) and np.max(belief_map[in_R_mask]) > 0.3:
                    #          if current_alt > 1.2: vz_des = -0.2
                    #          else: vz_des = 0.0
                    #     else:
                    #          vz_des = 0.0

                    # --- PHASE: APPROACH ("to_detection") ---
                    if tr["phase"] == "to_detection":
                        t_elapsed = t - tr["time"]
                        raw_target = tr["pos"] + v_drift * t_elapsed
                        
                        # [FIX] Clamp target to SAFETY POLY (Buffer -0.4m), not just Soft Poly
                        # This ensures Step 7 doesn't immediately reject the move
                        target_point = Point(raw_target[0], raw_target[1])
                        if not safety_poly.contains(target_point):
                            nearest_pt = safety_poly.exterior.interpolate(
                                safety_poly.exterior.project(target_point))
                            target_now = np.array([nearest_pt.x, nearest_pt.y])
                        else:
                            target_now = raw_target
                        
                        vec = target_now - current_pos[:2]
                        dist = np.linalg.norm(vec)

                        # Predictive Braking
                        if dist > 1.5:
                            vx_des, vy_des = (vec / (dist + 1e-6)) * 2.0
                        else:
                            speed = np.clip(dist * 0.8, 0.4, 1.5)
                            vx_des, vy_des = (vec / (dist + 1e-6)) * speed
                        
                        vz_des = 0.0 # Maintain height

                        if dist < 0.6:
                            tr["phase"] = "cone_tracking"
                            tr["cone_start"] = t

                    # --- PHASE: CONE TRACKING (IPP inside cone) ---
                    elif tr["phase"] == "cone_tracking":
                        det_pos = tr["pos"]
                        det_time = tr["time"]

                        # 1. Update Cone
                        region_poly, center_R, _ = tracking_region(t, det_pos, det_time, v_drift)
                        
                        if region_poly.is_empty:
                            tr["active"] = False; drone_modes[d_idx] = "explore"; continue

                        # 2. Rasterize Cone
                        poly_path = MplPath(list(region_poly.exterior.coords))
                        points_flat = np.vstack((XX.flatten(), YY.flatten())).T
                        in_R_mask = poly_path.contains_points(points_flat).reshape(rows, cols)
                        
                        # 3. Stats & Mode Switching
                        if np.any(in_R_mask):
                            p_vals = belief_map[in_R_mask]
                            if np.max(p_vals) > peak_tresh or np.mean(p_vals) > 0.3:
                                tr["phase"] = "hover_confirm"
                                tr["hover_start"] = t
                                tr["pos"] = drone_positions[d_idx][:2].copy() # Reset anchor
                                continue
                            
                            # Timeout
                            if (t - tr["cone_start"]) > 30.0:
                                tr["active"] = False; drone_modes[d_idx] = "explore"; continue

                        # 4. Motion Planning
                        # [FIX] Chase Logic: If we are far, just fly to center (clamped)
                        # We clamp 'center_R' to the safety poly
                        center_pt = Point(center_R[0], center_R[1])
                        if not safety_poly.contains(center_pt):
                             nearest = safety_poly.exterior.interpolate(safety_poly.exterior.project(center_pt))
                             safe_center = np.array([nearest.x, nearest.y])
                        else:
                             safe_center = center_R

                        err_vec = safe_center - current_pos[:2]
                        dist_err = np.linalg.norm(err_vec)

                        if dist_err > 1.0:
                            # Direct Chase
                            speed = 1.0 
                            vx_des, vy_des = (err_vec / dist_err) * speed
                        else:
                            # Search Pattern (IPP)
                            soft_cone_constraint = region_poly.buffer(0.5)
                            # Clip constraint to safety poly
                            safe_cone = soft_cone_constraint.intersection(safety_poly)
                            
                            vx_ipp, vy_ipp, vz_ipp, _, _, _ = plan_velocity_ipp_3D(
                                current_pos, drone_vels[d_idx],
                                belief_map, inside_mask,
                                (x_min, y_min), grid_resolution,
                                soft_poly, # Planner uses full map
                                constraint_poly=safe_cone, # But constrained to safe cone
                                step_length=1.5,         
                                fov_angle=theta_FOV, 
                                v_max=0.8,               
                                altitude_candidates=[current_pos[2]], 
                                pred_depth=1, E_scale=E_scale_track,
                                lam=0.5,        
                                buffer=buffer
                            )
                            vx_des, vy_des = vx_ipp, vy_ipp

                        # Add drift feedforward
                        vx_des += v_drift[0]
                        vy_des += v_drift[1]

                        # Descent Logic
                        if np.any(in_R_mask) and np.max(belief_map[in_R_mask]) > 0.3:
                             if current_alt > 1.2: vz_des = -0.2
                             else: vz_des = 0.0
                        else:
                             vz_des = 0.0


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
                        
                        theoretical_center = det_pos + v_drift * (t - det_time)
                        raw_target = theoretical_center
                        has_visual_lock = False

                        if np.sum(hotspot_mask_local) > 3:
                            y_indices = np.arange(r_s.start, r_s.stop) * grid_resolution + y_min
                            x_indices = np.arange(c_s.start, c_s.stop) * grid_resolution + x_min
                            loc_x_mesh, loc_y_mesh = np.meshgrid(x_indices, y_indices)
                            
                            coords_x = loc_x_mesh[hotspot_mask_local]
                            coords_y = loc_y_mesh[hotspot_mask_local]
                            weights = local_belief[hotspot_mask_local]**2
                            
                            vis_x = np.average(coords_x, weights=weights)
                            vis_y = np.average(coords_y, weights=weights)
                            visual_centroid = np.array([vis_x, vis_y])

                            # Gating: Tight check for indoor (2.0m)
                            if np.linalg.norm(visual_centroid - theoretical_center) < 2.0:
                                has_visual_lock = True
                                raw_target = visual_centroid

                        # Low Pass Filter
                        if "target_smooth" not in tr or tr["target_smooth"] is None:
                            tr["target_smooth"] = raw_target

                        alpha_pos = 0.3
                        tr["target_smooth"] = (1 - alpha_pos) * tr["target_smooth"] + alpha_pos * raw_target
                        target_xy = tr["target_smooth"]

                        # 3. Control (Chase the Smoothed Target)
                        err_vec = target_xy - drone_positions[d_idx][:2]
                        dist_err = np.linalg.norm(err_vec)

                        # TENACIOUS CHASE: Keep chasing until 10cm (0.1m)
                        if dist_err > 0.1:
                            speed = np.clip(dist_err * 0.6, 0.3, v_max) # Min speed 0.3 to prevent stalling
                            vx_cmd, vy_cmd = (err_vec / (dist_err + 1e-6)) * speed
                        else:
                            vx_cmd, vy_cmd = 0.0, 0.0
                        
                        vx_des = vx_cmd + v_drift[0]
                        vy_des = vy_cmd + v_drift[1]

                        # 4. Vertical Logic (The "Cylinder" Check)
                        current_alt = drone_positions[d_idx][2]
                        
                        # Only descend if we are > 1.0m high
                        if current_alt > 1.0:
                            # SCALED CHECK: Only drop if within 0.8m of target (80cm)
                            if dist_err < 0.8:
                                vz_des = -0.3 # Slow, controlled descent
                            else:
                                vz_des = 0.0  # Wait to align
                        else:
                            vz_des = 0.0

                        # 5. Confirmation Stats
                        p_vals = local_belief[hotspot_mask_local]
                        max_p = np.max(p_vals) if len(p_vals) > 0 else 0.0
                        mean_p = np.mean(p_vals) if len(p_vals) > 0 else 0.0

                        # Dynamic Ceiling
                        # if mean_p >= 0.90: req_alt = 1.5
                        # elif mean_p >= 0.70: req_alt = 1.0
                        # else: req_alt = 0.5

                        req_alt = 1.0  # We want to get as close as possible for confirmation
                        
                        # A. CONFIRMATION
                        if (current_alt <= req_alt and max_p >= peak_tresh):
                            print(f"[t={t}] UAV{d_idx} CONFIRMED victim at alt={current_alt:.1f}m.")
                            belief_map = clear_confirmed_region_2d(belief_map, XX, YY, det_pos, v_drift)
                            dx = XX - det_pos[0]; dy = YY - det_pos[1]
                            belief_map[(dx**2 + dy**2) < 2.0**2] *= 0.05
                            
                            tr["active"] = False; tr["confirmed"] = True
                            drone_modes[d_idx] = "explore"
                            vz_des = 0.5

                        # B. TIMEOUT (Extended)
                        # Increased to 20s to allow for the align-then-descend behavior
                        if (t - tr["hover_start"]) > 20.0:
                             # Only abort if low enough but still not sure
                             if current_alt <= req_alt:
                                print(f"[t={t}] UAV{d_idx} hover-confirm timeout → aborting.")
                                tr["active"] = False; drone_modes[d_idx] = "explore"
                                vz_des = 0.5
                             # Hard Safety limit (e.g. 25s)
                             elif (t - tr["hover_start"]) > 25.0:
                                tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 0.5


                    else:
                        vx_des, vy_des, vz_des = 0.0, 0.0, 0.0

                # --- Fallback ---
                else:
                    vx_des, vy_des, vz_des, *_ = plan_velocity_ipp_3D(
                        drone_positions[d_idx], 
                        drone_vels[d_idx],
                        planning_belief, inside_mask,
                        (x_min, y_min), grid_resolution,
                        soft_poly,
                        constraint_poly=None,
                        step_length=2.0,
                        fov_angle=theta_FOV, v_max=v_max, n_directions=8,
                        altitude_candidates=[2, 3, 4], pred_depth=pred_depth, E_scale=E_scale,
                        lam=0.5,        
                        buffer=buffer
                    )
              
                commanded_vels[d_idx] = [vx_des, vy_des, vz_des]

       
            # =========================================================
            # 7. ROBUST CONTROL & SAFETY LAYER
            # =========================================================
            
            # =========================================================
            # 7. ROBUST CONTROL & SAFETY LAYER
            # =========================================================
            
            # [CONFIG] Safety Margin
            # We enforce a 40cm "Virtual Wall" inside the real wall.
            # If the drone crosses this, the safety layer kicks in HARD.
            WALL_SAFETY_MARGIN = 0.5 
            safety_poly = soft_poly.buffer(-WALL_SAFETY_MARGIN)

            # A. Repulsion
            repulsion_vels = np.zeros((num_drones, 3))
            safe_separation = 0.8  
            repulsion_gain = 2.0

            for i in range(num_drones):
                for j in range(i + 1, num_drones):
                    diff_vec = drone_positions[i] - drone_positions[j]
                    dist = np.linalg.norm(diff_vec)
                    if dist < safe_separation and dist > 0.1:
                        push = (diff_vec / dist) * (safe_separation - dist) * repulsion_gain
                        repulsion_vels[i] += push
                        repulsion_vels[j] -= push

            # B. Apply Physics & Output
            for d_idx in range(num_drones):
                ac_id = AC_IDS[d_idx]

                # 1. Target Velocity (Planner + Repulsion)
                v_target = commanded_vels[d_idx].copy()
                v_target += repulsion_vels[d_idx]

                # 2. ACCELERATION LIMITER
                current_real_vel = drone_vels[d_idx]
                delta_v = v_target - current_real_vel
                dist_v = np.linalg.norm(delta_v)
                
                if USE_INTERNAL_PHYSICS:
                    max_accel = 5.0    
                else:
                    max_accel = 1.0    
                
                max_change = max_accel * dt 
                
                if dist_v > max_change:
                    scale = max_change / dist_v
                    delta_v = delta_v * scale
                
                v_proposed = current_real_vel + delta_v
                
                # 3. GEOFENCE SAFETY CHECK (The Fix)
                # Predict where we will be in the next timestep
                next_pos = drone_positions[d_idx] + v_proposed * dt
                point_next = Point(next_pos[0], next_pos[1])
                
                # [CRITICAL CHECK]
                # We check against 'safety_poly' (Wall - 0.4m), NOT the hard wall.
                if not safety_poly.contains(point_next):
                    # We are about to violate the safety margin!
                    # Calculate vector to push us back INSIDE the safety buffer
                    
                    # Project predicted point onto the safety boundary
                    nearest_pt = safety_poly.exterior.interpolate(
                        safety_poly.exterior.project(point_next)
                    )
                    safe_spot = np.array([nearest_pt.x, nearest_pt.y])
                    
                    # Create a "Wall Slide" vector (Towards the safe spot)
                    correction_vec = safe_spot - drone_positions[d_idx][:2]
                    norm_c = np.linalg.norm(correction_vec)
                    
                    if norm_c > 0.01:
                        # Redirect velocity ALONG the wall or INWARDS
                        # 1.5 gain ensures we react fast enough to wind/momentum
                        speed_factor = min(np.linalg.norm(v_proposed[:2]), 0.5) 
                        v_proposed[:2] = (correction_vec / norm_c) * speed_factor * 1.5
                    else:
                        # If we are right on the line, just stop moving towards wall
                        v_proposed[:2] = np.zeros(2)

                # 4. Wind Limit
                if not USE_INTERNAL_PHYSICS and ac_id in UAVS:
                    local_wind = UAVS[ac_id]['wind']
                    v_g_mag = np.linalg.norm(v_proposed[:2])
                    if v_g_mag > 0.1:
                        unit_v = v_proposed[:2] / v_g_mag
                        v_headwind = np.dot(local_wind, unit_v)
                        limit = 0.7 + max(0, v_headwind) 
                        if v_g_mag > limit: v_proposed[:2] = unit_v * limit

                # 5. Output
                if not USE_INTERNAL_PHYSICS:
                    if ivy: send_pprz_velocity(ivy, ac_id, *v_proposed)
                else:
                    drone_vels[d_idx] = v_proposed
            
            # 6. INTERNAL PHYSICS INTEGRATION
            if USE_INTERNAL_PHYSICS:
                for d_idx in range(num_drones):
                    drone_positions[d_idx] += drone_vels[d_idx] * dt
            
            # (End of Main Loop)
            # ============================
            #  Visualization Update
            # ============================

            # A. Update Belief Map (IMSHOW)
           # =========================================================
            # 8. VISUALIZATION UPDATE
            # =========================================================
            
            # A. Update Belief Map
            display_data = belief_map.copy()
            display_data[~inside_mask] = np.nan
            im_display.set_data(display_data)

            # B. Update Markers
            for d_idx in range(num_drones):
                # set_data expects lists/arrays
                drone_plots[d_idx].set_data([drone_positions[d_idx][0]], [drone_positions[d_idx][1]])
            
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
                    
                    if not region_poly.is_empty:
                        xR, yR = region_poly.exterior.xy
                        cone_lines[d_idx].set_data(xR, yR)
                        cone_centers[d_idx].set_data([center_R[0]], [center_R[1]])
                        # Polygon Patch Update
                        cone_fills[d_idx].set_xy(np.column_stack([xR, yR]))
                    else:
                        cone_lines[d_idx].set_data([], [])
                        cone_centers[d_idx].set_data([], [])
                        cone_fills[d_idx].set_xy(np.zeros((3,2)))
                else:
                    # Hide
                    cone_lines[d_idx].set_data([], [])
                    cone_centers[d_idx].set_data([], [])
                    cone_fills[d_idx].set_xy(np.zeros((3,2)))

            # =========================================================
            # 9. PERFORMANCE & LOGGING
            # =========================================================
            
            # A. Frequency Calculation (Non-destructive)
            # We use a separate variable 'last_freq_time' so we don't break 't'
            if 'last_freq_time' not in locals(): last_freq_time = time.time()
            
            freq_now = time.time()
            freq_elapsed = freq_now - last_freq_time
            
            if freq_elapsed >= 1.0:
                current_freq = frame_count / freq_elapsed
                print(f"--- t={t:.2f}s | Loop Rate={current_freq:.1f} Hz ---")
                
                # Console Telemetry
                for d_idx in range(num_drones):
                    ac_id = AC_IDS[d_idx]
                    
                    # 1. Ground Speed
                    vx, vy, vz = drone_vels[d_idx]
                    g_speed = np.linalg.norm([vx, vy])
                    
                    # 2. Real Measured Wind (From UAVS dict)
                    real_wind = UAVS[ac_id]['wind'] if ac_id in UAVS else np.zeros(2)
                    
                    # 3. Airspeed Estimate
                    v_air_vec = np.array([vx, vy]) - real_wind
                    air_speed = np.linalg.norm(v_air_vec)
                    
                    # 4. Battery
                    bat = UAVS[ac_id]['status']['bat'] if ac_id in UAVS else 0.0
                    
                    print(f"  AC{ac_id}: GSpd={g_speed:.1f} ASpd={air_speed:.1f} Wind={real_wind} Bat={bat:.1f}V")

                # Reset Frequency Timer
                last_freq_time = freq_now
                frame_count = 0

            # --- GUI Update ---
            plt.pause(0.001) 

            # B. CSV Logging
            # Log Searchers
            for d_idx in range(num_drones):
                pos = drone_positions[d_idx]
                vel = drone_vels[d_idx]
                mode = drone_modes[d_idx]
                tr = track[d_idx]
                
                est_x = tr["pos"][0] if (tr["active"] and tr["pos"] is not None) else ""
                est_y = tr["pos"][1] if (tr["active"] and tr["pos"] is not None) else ""
                
                # Safety check for frequency variable
                freq_val = current_freq if 'current_freq' in locals() else 0.0
                
                writer_uav.writerow([
                    f"{t:.3f}", AC_IDS[d_idx],
                    f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{pos[2]:.3f}",
                    f"{vel[0]:.3f}", f"{vel[1]:.3f}", f"{vel[2]:.3f}",
                    mode, est_x, est_y,
                    f"{freq_val:.2f}"
                ])
                
            # Log Victims
            for v_idx, v_pos in enumerate(victims):
                writer_vic.writerow([
                    f"{t:.3f}", v_idx,
                    f"{v_pos[0]:.3f}", f"{v_pos[1]:.3f}"
                ])
                
            # Flush periodically
            if frame_count % 10 == 0:
                f_uav.flush()
                f_vic.flush()

            frame_count += 1

    except KeyboardInterrupt:
        print("\n[BRIDGE] Stopping...")
    
    finally:
        if 'ivy' in locals() and ivy:
            ivy.stop()
            print("[BRIDGE] Ivy Stopped.")
        
        # Close CSVs properly
        if 'f_uav' in locals(): f_uav.close()
        if 'f_vic' in locals(): f_vic.close()

    plt.ioff()
    plt.show()