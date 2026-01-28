import numpy as np
import matplotlib.pyplot as plt
import random
import pandas as pd  # <--- Add this
import os
import sys
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, Point
import pymap3d as pm
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter
from shapely.affinity import rotate, translate


# --- Paparazzi setup ---
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))
sys.path.append(PPRZ_HOME + "/sw/ext/pprzlink/lib/v1.0/python")


lat0, lon0, alt0 = 52.1681551, 4.4126468, 0.0
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

softgeo_xy = np.array([waypoints[wp][:2] for wp in ["S1","S2","S3","S4","S5","S6","S7","S8","S9"]])
soft_poly = Polygon(softgeo_xy)

x_min, y_min, x_max, y_max = soft_poly.bounds
grid_resolution = 10.0
grid_x = np.arange(x_min, x_max + grid_resolution, grid_resolution)
grid_y = np.arange(y_min, y_max + grid_resolution, grid_resolution)
cell_size = float(grid_x[1] - grid_x[0])


# -----------------------------
# GLOBAL PARAMETERS
# -----------------------------

dt_step = 1.0
v_drift = np.array([1.7, 1.0])
theta_FOV = np.deg2rad(45)
E_scale = 100.0
E_scale_track = 40.0
gamma_wind = 5.0
v_wind = v_drift.copy()             # treat wind ≈ drift for Deff (7.46)
v_max = 20.0        # max velocity [m/s]

confirm_pconf = 0.6
peak_tresh = 0.75
mean_thresh = 0.4
rho_th = 0.25

# -----------------------------
# CORE FUNCTIONS
# -----------------------------

def cell_entropy_map(belief):
    p = np.clip(belief, 1e-9, 1 - 1e-9)
    return -p * np.log2(p) - (1 - p) * np.log2(1 - p)

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


def r_FOV(h):
    """Ground-projected FOV radius: h * tan(theta_FOV)."""
    return h * np.tan(theta_FOV)

def P_hit(h):
    """Detection probability as in thesis: P0 * f_res(h)."""
    return P0 * f_res(h)

def visible_cells_at(pos_xyz, grid_points, fov_angle=theta_FOV):
    """
    Compute which grid cells are visible from pos_xyz and their
    single-pass detection probability.

    fov_angle is the half-angle of the conical sensor model.
    """
    x, y, h = pos_xyz

    # altitude-dependent footprint
    r = h * np.tan(fov_angle)

    dx = grid_points[:, 0] - x
    dy = grid_points[:, 1] - y
    d2 = dx*dx + dy*dy

    mask = d2 <= r*r

    # detection probability depends only on altitude
    p_hit = np.zeros(len(grid_points), dtype=float)
    if np.any(mask):
        p_hit[mask] = P_hit(h)

    return mask, p_hit

def victim_signal_at_alt(grid_points, victim_pos, alt):
    """
    Compute which grid cells would contain the victim *if* the UAV were
    looking at altitude = alt, using the same FOV model as the sensor.
    """
    vx, vy = victim_pos
    r = r_FOV(alt)   # same FOV model as UAV

    dx = grid_points[:, 0] - vx
    dy = grid_points[:, 1] - vy
    d2 = dx*dx + dy*dy

    return (d2 <= r*r).astype(float)


def expected_info_gain(path, belief, grid_points, fov_angle=theta_FOV, pred_depth=3):
    """
    Compute expected information gain along a candidate path.

    We approximate the future by assuming that, along the planned path,
    we get *no detection* and update the belief accordingly using a
    Bayesian "no detection" update:

        p' = p (1 - q) / (1 - p q)

    where p is the prior belief and q = P_hit(h) is the single-pass
    detection probability at altitude h.

    The information gain is the reduction in entropy H(p) - H(p').
    """
    pred_belief = belief.copy()
    total_IG = 0.0
    vis_mask_total = np.zeros(len(grid_points), dtype=bool)

    for i in range(len(path) - 1):
        if i >= pred_depth:
            break

        p0, p1 = path[i], path[i + 1]
        seg_len = np.linalg.norm(p1[:2] - p0[:2])

        # sample points along the segment
        n_samples = max(2, int(np.ceil(seg_len / 20.0)))
        xs = np.linspace(p0[0], p1[0], n_samples)
        ys = np.linspace(p0[1], p1[1], n_samples)
        zs = np.linspace(p0[2], p1[2], n_samples)

        for k in range(n_samples):
            pos = np.array([xs[k], ys[k], zs[k]])
            mask, p_hit = visible_cells_at(pos, grid_points, fov_angle=fov_angle)

            if not np.any(mask):
                continue

            # prior belief and entropy for visible cells
            p_prior = pred_belief[mask]
            H_prior = cell_entropy_map(p_prior)

            # detection probability for visible cells
            q = p_hit[mask]  # already 0..1

            # Bayesian update for "no detection"
            # p' = p (1 - q) / (1 - p q)
            num = p_prior * (1.0 - q)
            denom = 1.0 - p_prior * q
            denom = np.where(denom < 1e-9, 1e-9, denom)   # avoid division by zero
            p_post = num / denom
            p_post = np.clip(p_post, 1e-9, 1.0 - 1e-9)

            H_post = cell_entropy_map(p_post)

            # entropy reduction (information gain) for this sample
            delta_H = H_prior - H_post
            total_IG += np.sum(delta_H)

            # update predicted belief along the path so we don't
            # double-count repeated looks at the same cells
            pred_belief[mask] = p_post
            vis_mask_total |= mask

    return total_IG, vis_mask_total


# ==============================================================================
# 1. HELPER FUNCTIONS (Must support multi-step paths)
# ==============================================================================

def expected_detection(path, belief, grid_points, fov_angle, pred_depth=3):
    DET = 0.0
    # Iterate through segments (p0->p1, p1->p2...)
    for i in range(len(path) - 1):
        if i >= pred_depth: break
        p0, p1 = path[i], path[i+1]
        
        # Sample points along the segment
        seg_len = np.linalg.norm(p1[:2] - p0[:2])
        n_samples = max(2, int(np.ceil(seg_len / 10.0))) # Sample every 10m
        
        xs = np.linspace(p0[0], p1[0], n_samples)
        ys = np.linspace(p0[1], p1[1], n_samples)
        zs = np.linspace(p0[2], p1[2], n_samples)

        for x, y, z in zip(xs, ys, zs):
            pos = np.array([x, y, z])
            vis_mask, p_hit = visible_cells_at(pos, grid_points, fov_angle=fov_angle)
            if np.any(vis_mask):
                DET += np.sum(belief[vis_mask] * p_hit[vis_mask])
    return DET


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
    # Standard travel speed for the planner is v_max (20 m/s)
    # At 20 m/s, get_vsqp_power returns ~323W
    cruise_speed = 20.0 
    P_cruise_travel = get_vsqp_power(cruise_speed) 
    
    # Cost of climbing (fighting gravity) is roughly equal to Hover power
    P_climb = get_vsqp_power(0.0) # 1751 W
    v_climb = 2.0
    v_desc = 3.0

    total_energy = 0.0

    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i+1]
        dist_xy = np.linalg.norm(p1[:2] - p0[:2])
        dz = p1[2] - p0[2]

        # Horizontal Energy: Power * Time
        if dist_xy > 1e-3:
            t_segment = dist_xy / cruise_speed
            E_horiz = P_cruise_travel * t_segment
        else:
            E_horiz = 0.0

        # Vertical Energy
        if dz > 0:   
            t_climb = dz / v_climb
            E_vert = P_climb * t_climb
        elif dz < 0: 
            t_desc = -dz / v_desc
            E_vert = P_climb * t_desc * 0.5 # Descent is cheaper (gravity assists)
        else:        
            E_vert = 0.0
        
        total_energy += (E_horiz + E_vert)
        
    return total_energy

# ==============================================================================
# 2. THE PLANNER (Supports 'constraint_poly' and 'drone_vel')
# ==============================================================================

def plan_velocity_ipp_3D(drone_pos, drone_vel, belief, grid_points, soft_poly,
                         constraint_poly=None,
                         fov_angle=theta_FOV, v_max=20.0, n_directions=16,
                         step_length=40.0, altitude_candidates=[30, 50, 70],
                         E_scale=100.0,    # From Table 5.1
                         lam=0.4,          # From Table 5.1 (Energy Cost Weight)
                         buffer=0.0, pred_depth=3):

    # 1. Constraint Logic (Geofence intersection)
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

    # Dynamic heading sampling
    if current_speed > 2.0:
        current_heading = np.arctan2(vy, vx)
        angles = current_heading + np.linspace(-np.pi/3, np.pi/3, n_directions)
    else:
        angles = np.linspace(0, 2*np.pi, n_directions, endpoint=False)

    candidates = []

    # 3. Generate Candidate Trajectories
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

    # Fallback for tight spaces
    if not candidates and constraint_poly is not None:
        small_step = 10.0
        for a in angles:
            dx, dy = np.cos(a), np.sin(a)
            end = np.array([cx + dx*small_step, cy + dy*small_step, cz])
            if valid_area.contains(Point(end[0], end[1])):
                candidates.append([np.array([cx, cy, cz]), end])

    # 4. Evaluation Loop (Matches Eq: J = I - lambda * E)
    best_J = -np.inf
    best_path = None
    best_mask = None

    for path in candidates:
        # I_p: Expected Information Gain (Raw Entropy Reduction)
        I_p, vis_mask = expected_info_gain(path, belief, grid_points, fov_angle, pred_depth)
        
        # E_p: Energy Cost (Joules)
        E_p = energy_of_path(path)
        
        # E_norm: Normalized Energy (Unitless, based on Table 5.1)
        E_norm = E_p / E_scale

        # --- OBJECTIVE FUNCTION ---
        # Strictly matches J = I - lambda * E
        J = I_p - (lam * E_norm)
        
        if J > best_J:
            best_J = J
            best_path = path
            best_mask = vis_mask

    if best_path is None:
        return 0.0, 0.0, 0.0, [drone_pos, drone_pos], None, 0.0

    # 5. Output Velocity Calculation
    p0, p1 = best_path[0], best_path[1]
    vec = p1 - p0
    dist = np.linalg.norm(vec)
    
    travel_time = max(dist / v_max, 0.1)
    vx = vec[0] / travel_time
    vy = vec[1] / travel_time
    vz = vec[2] / travel_time

    return float(vx), float(vy), float(vz), best_path, best_mask, best_J



def propagate_belief_vectorized(belief, grid_points, v_drift, dt, sigma=5.0):
    prev_positions = grid_points - v_drift * dt
    tree = cKDTree(grid_points)
    _, idx = tree.query(prev_positions)
    new_belief = belief[idx]

    x_coords = np.unique(grid_points[:, 0])
    y_coords = np.unique(grid_points[:, 1])
    x_idx = np.searchsorted(x_coords, grid_points[:, 0])
    y_idx = np.searchsorted(y_coords, grid_points[:, 1])

    grid_2d = np.full((len(x_coords), len(y_coords)), np.nan)
    grid_2d[x_idx, y_idx] = new_belief
    mask = np.isnan(grid_2d)
    grid_2d[mask] = 0.5
    grid_2d = gaussian_filter(grid_2d, sigma=sigma)
    grid_2d = np.clip(grid_2d, 0.0, 1.0)

    new_belief = grid_2d[x_idx, y_idx]
    return new_belief



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



def clear_confirmed_region(belief, grid_points, victim_pos, wind_vec,
                           major_axis=120.0, minor_axis=60.0, decay=0.01):
    """
    Clear belief in a drift-aligned elliptical region after victim confirmation.

    major_axis : length (m) along wind direction
    minor_axis : length (m) perpendicular to wind
    decay      : residual probability after clearing (0.0–0.05 recommended)
    """

    # Normalize wind direction
    w = wind_vec[:2]
    if np.linalg.norm(w) < 1e-6:
        wind_hat = np.array([1.0, 0.0])   # fallback
    else:
        wind_hat = w / np.linalg.norm(w)

    # Perpendicular axis
    perp_hat = np.array([-wind_hat[1], wind_hat[0]])

    # Compute vector from ellipse center
    rel = grid_points - victim_pos

    # Project onto drift-aligned axes
    a = np.dot(rel, wind_hat)      # along-wind
    b = np.dot(rel, perp_hat)      # cross-wind

    # Ellipse mask (a/major)^2 + (b/minor)^2 <= 1
    mask = (a / major_axis) ** 2 + (b / minor_axis) ** 2 <= 1.0

    # Apply clearing with soft decay
    belief[mask] = decay

    return belief


def local_centroid(belief, grid_points, idx, radius=40):
    center = grid_points[idx]
    diffs = grid_points - center
    d2 = diffs[:,0]**2 + diffs[:,1]**2
    mask = d2 < radius**2

    if np.sum(mask) < 3:
        return center

    w = belief[mask]
    pts = grid_points[mask]
    pt = np.average(pts, axis=0, weights=w)
    return pt

def drift_aligned_centroid(belief, grid_points, idx, wind_vec,
                           major=150.0, minor=80.0):
    """
    Compute a weighted centroid around the MAP cell using a drift-aligned ellipse.
    major/minor are axis lengths (meters).
    """
    center = grid_points[idx]

    # Normalize drift direction
    w = wind_vec[:2]
    if np.linalg.norm(w) < 1e-6:
        wind_hat = np.array([1.0, 0.0])
    else:
        wind_hat = w / np.linalg.norm(w)

    # Perpendicular axis
    perp_hat = np.array([-wind_hat[1], wind_hat[0]])

    # Relative vectors to all grid points
    rel = grid_points - center
    a = np.dot(rel, wind_hat)     # along wind
    b = np.dot(rel, perp_hat)     # across wind

    # Elliptical selection region
    mask = (a/major)**2 + (b/minor)**2 <= 1.0

    # Fallback if not enough points
    if np.sum(mask) < 5:
        return center

    # Weighted average
    wts = belief[mask]
    pts = grid_points[mask]
    centroid = np.average(pts, axis=0, weights=wts)

    return centroid

def active_cone_mask(t_now, grid_points, track, v_drift):
    """
    Returns a boolean mask over grid_points that is True where
    ANY active tracker’s cone (buffered) is present.
    """
    mask = np.zeros(len(grid_points), dtype=bool)

    for tr in track:
        if not tr["active"] or tr["pos"] is None or tr["time"] is None:
            continue

        region_poly, _, _ = tracking_region(
            t_now=t_now,
            detection_pos=tr["pos"],
            detection_time=tr["time"],
            v_drift=v_drift
        )

        if region_poly and not region_poly.is_empty:
            # same 20 m buffer you were using
            region_poly_buffered = region_poly.buffer(20.0)
            in_cone = np.array([
                region_poly_buffered.contains(Point(p))
                for p in grid_points
            ])
            mask |= in_cone

    return mask


# --- Build grid and inside-polygon mask ---
XX, YY = np.meshgrid(grid_x, grid_y)
grid_points_all = np.column_stack([XX.ravel(), YY.ravel()])

inside_mask_full = np.array([soft_poly.contains(Point(p)) for p in grid_points_all])
grid_points = grid_points_all[inside_mask_full]

# belief must match grid_points, not inside_mask_full
belief = 0.5 * np.ones(len(grid_points))

# --- Mapping from ALL -> inside ---
all_to_inside = -np.ones(len(grid_points_all), dtype=int)
all_to_inside[np.where(inside_mask_full)[0]] = np.arange(np.sum(inside_mask_full))

# --- CONSTANTS & GEOMETRY (Keep Global) ---

# Master list of spawn points
master_positions = [
    np.array([400.0, 0.0, 80.0]),
    np.array([50.0, -300.0, 40.0]),
    np.array([0.0, -200.0, 40.0]), 
    np.array([300.0, -40.0, 40.0]),
    np.array([80.0, -100.0, 80.0]),
    np.array([10.0, -250.0, 40.0]),
    np.array([0.0, 0.0, 80.0])
]

# Map Boundaries
buffer = 30.0 
safe_poly = soft_poly.buffer(-buffer) if buffer > 0 else soft_poly

# Simulation Parameters
max_dheading = np.deg2rad(10)
dt = 1.0
pred_depth = 3
E_max = 100.0

# Tracking Parameters
initial_lock_duration = 8.0
cone_search_timeout = 30.0
HOVER_CONFIRM_TIME = 10.0
MIN_ALT = 20.0
DESCENT_STEP = 3.0
COOLDOWN_TIME = 30

# ==========================================
# 3. MONTE CARLO WRAPPER (Final: Penalty + Metrics)
# ==========================================

def run_trial(num_drones, num_victims, seed):
    # 1. Set Seed (Ensures reproducible randomness per trial)
    np.random.seed(seed)
    random.seed(seed)

    # 2. Reset Map & Constants
    belief = 0.5 * np.ones(len(grid_points))
    E_scale = 100.0         
    E_scale_track = 40.0
    
    BATTERY_CAPACITY_J = 18000.0 * 60.0
    discharge_factor = 100.0 / BATTERY_CAPACITY_J 

    start_entropy = np.sum(cell_entropy_map(belief))

    # --- ENVIRONMENT STOCHASTICITY ---
    wind_speed = np.random.uniform(0.5, 5.0) 
    wind_angle = np.random.uniform(0, 2*np.pi)
    v_wind = np.array([wind_speed * np.cos(wind_angle), wind_speed * np.sin(wind_angle), 0.0])
    
    drift_speed = np.random.uniform(0.5, 2.0)
    drift_angle = np.random.uniform(0, 2*np.pi)
    v_drift = np.array([drift_speed * np.cos(drift_angle), drift_speed * np.sin(drift_angle)])

    # 3. Initialize Victims
    all_possible_victims = np.array([[200.0, -200.0], [100.0, -100.0]], dtype=float)
    victims = all_possible_victims[:num_victims]
    initial_victim_pos = [list(v) for v in victims] 

    # 4. Initialize Drones (Fixed Start)
    drone_positions = []
    uav_nominal_altitudes = []
    initial_drone_positions = [] 

    # Define your fixed positions
    master_positions = [
        np.array([400.0, 0.0, 80.0]),
        np.array([50.0, -300.0, 40.0]),
        np.array([0.0, -200.0, 40.0]), 
        np.array([300.0, -40.0, 40.0]),
        np.array([80.0, -100.0, 80.0]),
        np.array([10.0, -250.0, 40.0]),
        np.array([0.0, 0.0, 80.0])
    ]
    
    for i in range(num_drones):
        # Use modulo so if num_drones > 7, it cycles back to the start
        pos = master_positions[i % len(master_positions)]
        
        # Extract Z height for the altitude list
        r_z = pos[2] 
        
        drone_positions.append(pos)
        initial_drone_positions.append(list(pos))
        uav_nominal_altitudes.append(r_z)
        
    uav_nominal_altitudes = np.array(uav_nominal_altitudes)

    # 5. Initialize State & Metrics
    drone_vels = [np.zeros(3) for _ in range(num_drones)]
    drone_modes = ['explore' for _ in range(num_drones)]
    # NEW: Track previous mode for transition penalty
    prev_flight_mode = ["Hover"] * num_drones 
    
    E_rem = np.ones(num_drones) * 100.0 
    
    detection_event = [False] * num_drones
    cooldown_until = np.zeros(num_drones)
    
    mode_counts = {"Hover": 0, "Transition": 0, "Cruise": 0}
    total_aborts = 0

    # 6. Initialize Tracks
    track = []
    for _ in range(num_drones):
        track.append({
            "active": False, "confirmed": False, "pos": None, "time": None,
            "pos_at_detection": None, "phase": None, "lock_start": None,
            "cone_start": None, "hover_start": None, "fail_timer": 0.0,
            "target_smooth": None 
        })

    confirmed_locations = []
    success = False
    confirmation_time = None
    
    # Constants
    initial_lock_duration = 8.0
    cone_search_timeout = 30.0
    confirm_pconf = 0.7
    peak_tresh = 0.95
    mean_thresh = 0.85
    rho_th = 0.80
    MIN_ALT = 20.0
    DESCENT_STEP = 1.5

    # ==========================
    # SIMULATION LOOP (180s)
    # ==========================
    for t in range(180):
        # ------------------------------------------
        # 0. Physics & Belief Prop
        # ------------------------------------------
        belief = propagate_belief_vectorized(belief, grid_points, v_drift, dt_step, sigma=1.0)
        victims[:, :2] += v_drift * dt_step

        # 1. Ground Truth
        victim_cell_mask = np.zeros(len(grid_points), dtype=bool)
        for v in victims:
            d2 = np.sum((grid_points - v[:2])**2, axis=1)
            victim_cell_mask |= (d2 == np.min(d2))

        # 2. Sensor Update
        for d_idx in range(num_drones):
            vis_mask, p_hit = visible_cells_at(drone_positions[d_idx], grid_points)
            p_prior = belief[vis_mask]
            
            if p_prior.size > 0:
                visible_victim_mask = vis_mask & victim_cell_mask
                if np.any(visible_victim_mask):
                    p_detect = min(1.0, np.max(p_hit[visible_victim_mask]) + 0.4)
                else:
                    p_detect = P_false 

                detection_event[d_idx] = (np.random.rand() < p_detect)
                q = p_hit[vis_mask]
                if detection_event[d_idx]:
                    num = p_prior * q
                    den = p_prior * q + (1 - p_prior) * P_false
                    belief[vis_mask] = num / np.maximum(den, 1e-12)
                else:
                    num = p_prior * (1 - q)
                    den = 1 - p_prior * q
                    p_post = num / np.maximum(den, 1e-12)
                    belief[vis_mask] = np.maximum(p_post, 0.7 * p_prior) 

        # 3. Detection Assignment
        in_active_cone = active_cone_mask(t, grid_points, track, v_drift)
        eligible_explorers = [d for d, m in enumerate(drone_modes) if m == "explore" and t >= cooldown_until[d]]
        
        for d_idx in eligible_explorers:
            if not detection_event[d_idx]: continue
            vis_mask, _ = visible_cells_at(drone_positions[d_idx], grid_points)
            if not np.any(victim_cell_mask[vis_mask]): continue

            vis_indices = np.where(vis_mask)[0]
            local_idx = vis_indices[np.argmax(belief[vis_mask])]
            detection_pos = grid_points[local_idx].copy()
            
            is_duplicate = False
            for tr_exist in track:
                if tr_exist["active"] and tr_exist["pos"] is not None:
                    if np.linalg.norm(tr_exist["pos"] - detection_pos) < 50.0:
                        is_duplicate = True; break
            for conf_loc in confirmed_locations:
                if np.linalg.norm(conf_loc - detection_pos) < 50.0:
                    is_duplicate = True; break
            
            if not is_duplicate:
                track[d_idx].update({
                    "active": True, "pos": detection_pos, "time": t, "phase": "to_detection",
                    "pos_at_detection": detection_pos.copy()
                })
                drone_modes[d_idx] = "track"
                cooldown_until[d_idx] = t + 30.0
                break 

        # 4. PLANNING (Using All Constants)
        for d_idx in range(num_drones):
            tr = track[d_idx]

            if drone_modes[d_idx] == "explore":
                vx_des, vy_des, vz_des, _, _, _ = plan_velocity_ipp_3D(
                    drone_positions[d_idx], drone_vels[d_idx], belief, grid_points, soft_poly,
                    constraint_poly=None, fov_angle=theta_FOV, v_max=v_max, 
                    E_scale=E_scale, lam=0.5, buffer=buffer
                )
                vz_des = np.clip(uav_nominal_altitudes[d_idx] - drone_positions[d_idx][2], -1.0, 1.0)

            elif drone_modes[d_idx] == "track" and tr["active"]:
                if tr["phase"] == "to_detection":
                    t_elapsed = t - tr["time"]
                    target = tr["pos"] + v_drift * t_elapsed
                    vec = target - drone_positions[d_idx][:2]
                    dist = np.linalg.norm(vec)
                    
                    if dist > 40.0:
                        dir_unit = vec / (dist + 1e-6)
                        speed = np.clip((dist - 40.0) * 0.5, 5.0, v_max)
                        vx_des, vy_des = dir_unit * speed
                        vz_des = 0.0
                    else:
                        tr["phase"] = "initial_lock"
                        tr["lock_start"] = t
                        vx_des, vy_des = v_drift
                        vz_des = 0.0

                elif tr["phase"] == "initial_lock":
                    region_poly, center_R, radius_R = tracking_region(t, tr["pos"], tr["time"], v_drift)
                    vec_center = center_R - drone_positions[d_idx][:2]
                    dist_center = np.linalg.norm(vec_center)
                    
                    if dist_center > 5.0:
                        vx_des = (vec_center[0]/dist_center) * 5.0
                        vy_des = (vec_center[1]/dist_center) * 5.0
                    else:
                        vx_des, vy_des = 0.0, 0.0
                    vz_des = 0.0
                    
                    if (t - tr["lock_start"]) >= initial_lock_duration:
                        tr["phase"] = "cone_tracking"
                        tr["cone_start"] = t

                elif tr["phase"] == "cone_tracking":
                    region_poly, center_R, radius_R = tracking_region(t, tr["pos"], tr["time"], v_drift)
                    
                    if not soft_poly.intersects(region_poly):
                         total_aborts += 1
                         tr["active"] = False; tr["phase"] = None
                         drone_modes[d_idx] = "explore"
                         vx_des, vy_des, vz_des = 0.0, 0.0, 0.0
                         continue

                    soft_cone_constraint = region_poly.buffer(10.0) 
                    vx_ipp, vy_ipp, vz_ipp, _, _, _ = plan_velocity_ipp_3D(
                        drone_positions[d_idx], drone_vels[d_idx], belief, grid_points, soft_poly,
                        constraint_poly=soft_cone_constraint, 
                        E_scale=E_scale_track, lam=0.5, buffer=buffer, v_max=v_max
                    )
                    vx_des = vx_ipp + v_drift[0] * 0.8
                    vy_des = vy_ipp + v_drift[1] * 0.8
                    vz_des = vz_ipp

                    in_R_mask = np.array([region_poly.contains(Point(p)) for p in grid_points])
                    if np.any(in_R_mask):
                        p_vals = belief[in_R_mask]
                        frac_high = np.mean(p_vals > confirm_pconf) 
                        max_p = np.max(p_vals)
                        
                        if max_p >= peak_tresh or frac_high > rho_th:
                            tr["phase"] = "hover_confirm"
                            tr["hover_start"] = t
                            tr["pos"] = drone_positions[d_idx][:2].copy()
                            tr["time"] = t
                    
                    if (t - tr["cone_start"]) > cone_search_timeout:
                        total_aborts += 1
                        tr["active"] = False; tr["phase"] = None
                        drone_modes[d_idx] = "explore"

                elif tr["phase"] == "hover_confirm":
                    t_elapsed = t - tr["time"]
                    target = tr["pos"] + v_drift * t_elapsed
                    err_vec = target - drone_positions[d_idx][:2]
                    dist_err = np.linalg.norm(err_vec)
                    
                    if dist_err > 0.2:
                        speed = np.clip(dist_err * 1.5, 0.5, v_max) 
                        vx_des = (err_vec[0]/dist_err) * speed + v_drift[0]
                        vy_des = (err_vec[1]/dist_err) * speed + v_drift[1]
                    else:
                        vx_des, vy_des = 0.0, 0.0
                    
                    current_alt = drone_positions[d_idx][2]
                    if current_alt > MIN_ALT: 
                        vz_des = -DESCENT_STEP
                    else: 
                        vz_des = 0.0

                    vis_mask_inside, _ = visible_cells_at(drone_positions[d_idx], grid_points)
                    region_poly, _, _ = tracking_region(t, tr["pos"], tr["time"], v_drift)
                    region_mask = np.array([region_poly.contains(Point(p)) for p in grid_points])
                    hover_mask = vis_mask_inside & region_mask
                    
                    if np.sum(hover_mask) > 0:
                        mean_p = np.mean(belief[hover_mask])
                        
                        if mean_p >= mean_thresh and current_alt <= (MIN_ALT + 5.0):
                            is_new = True
                            for loc in confirmed_locations:
                                if np.linalg.norm(tr["pos"] - loc) < 30.0:
                                    is_new = False; break
                            
                            if is_new:
                                confirmed_locations.append(tr["pos"])
                                diff = grid_points - tr["pos"]
                                clear_mask = (diff[:,0]**2 + diff[:,1]**2) <= 50.0**2
                                belief[clear_mask] *= 0.05
                            
                            drone_modes[d_idx] = "explore"
                            tr["active"] = False; tr["phase"] = None
                            vz_des = 2.0 
                    
                    if (t - tr["hover_start"]) > 10.0:
                        total_aborts += 1
                        tr["active"] = False; tr["phase"] = None
                        drone_modes[d_idx] = "explore"

                else:
                    vx_des, vy_des, vz_des = 0.0, 0.0, 0.0
            else:
                vx_des, vy_des, vz_des = 0.0, 0.0, 0.0

            drone_vels[d_idx] = np.array([vx_des, vy_des, vz_des])

        # 5. Position Update & Mode Counting
        for d_idx in range(num_drones):
            next_pos = drone_positions[d_idx] + drone_vels[d_idx] * dt_step
            if not soft_poly.contains(Point(next_pos[0], next_pos[1])):
                drone_vels[d_idx][:2] *= -0.5
            drone_positions[d_idx] += drone_vels[d_idx] * dt_step
            
            # --- FLIGHT MODE LOGIC ---
            v_h = np.linalg.norm(drone_vels[d_idx][:2])
            current_mode = ""
            if v_h < 12.0: 
                current_mode = "Hover"
                mode_counts["Hover"] += 1
            elif v_h < 17.0: 
                current_mode = "Transition"
                mode_counts["Transition"] += 1
            else: 
                current_mode = "Cruise"
                mode_counts["Cruise"] += 1
            
            # --- TRANSITION PENALTY LOGIC ---
            # If changing between Hover and Cruise (skipping or passing transition)
            # Penalty = 1800 W * 5 s = 9000 Joules
            # This simulates the high-power stabilization needed to switch regimes
            if current_mode != prev_flight_mode[d_idx]:
                if (current_mode == "Hover" and prev_flight_mode[d_idx] == "Cruise") or \
                   (current_mode == "Cruise" and prev_flight_mode[d_idx] == "Hover"):
                    penalty_joules = 1800.0 * 5.0
                    E_rem[d_idx] -= penalty_joules * discharge_factor
            
            prev_flight_mode[d_idx] = current_mode

        # 6. Success Check
        if len(confirmed_locations) >= num_victims:
            success = True
            confirmation_time = t
            break

        # 7. Energy Update (Airspeed + Penalty)
        for d_idx in range(num_drones):
            v_ground = drone_vels[d_idx]
            v_air = v_ground - v_wind
            v_air_horiz = np.linalg.norm(v_air[:2])
            
            P_inst = get_vsqp_power(v_air_horiz)
            if drone_vels[d_idx][2] > 0.1: P_inst += 1000.0 * drone_vels[d_idx][2]
            P_inst = max(100.0, P_inst)
            E_rem[d_idx] -= P_inst * dt_step * discharge_factor
            E_rem[d_idx] = max(0, E_rem[d_idx])

    # --- END LOOP ---
    final_entropy = np.sum(cell_entropy_map(belief))
    entropy_reduction = start_entropy - final_entropy
    total_pct_consumed = np.sum(100.0 - E_rem)
    total_joules = (total_pct_consumed / 100.0) * BATTERY_CAPACITY_J
    
    total_steps = (t + 1) * num_drones
    pct_hover = (mode_counts["Hover"] / total_steps) * 100.0
    pct_trans = (mode_counts["Transition"] / total_steps) * 100.0
    pct_cruise = (mode_counts["Cruise"] / total_steps) * 100.0

    return {
        "Drones": num_drones,
        "Victims": num_victims,
        "Seed": seed,
        "Success": success,
        "Time": confirmation_time if success else 180,
        "Energy_Used_J": total_joules,
        "Entropy_Reduction": entropy_reduction,
        "Total_Aborts": total_aborts,
        "Pct_Hover": pct_hover,
        "Pct_Transition": pct_trans,
        "Pct_Cruise": pct_cruise,
        "Wind_Speed": wind_speed,
        "Drift_Speed": drift_speed,
        "Drone_Start_Pos": str(initial_drone_positions), 
        "Victim_Start_Pos": str(initial_victim_pos)      
    }

if __name__ == "__main__":
    SEEDS = 10
    results = []
    configs = [(3, 1), (5, 1), (7, 1), (3, 2), (5, 2), (7, 2)]
    
    print(f"Starting Monte Carlo Campaign ({SEEDS} seeds)...")
    
    for seed in range(SEEDS):
        print(f"Processing Seed {seed}...")
        for nd, nv in configs:
            try:
                res = run_trial(nd, nv, seed)
                results.append(res)
            except Exception as e:
                print(f"Error in Seed {seed} Config {nd}D/{nv}V: {e}")
                
    df = pd.DataFrame(results)
    df.to_csv("mc_results_final3.csv", index=False)
    print("Done. Results saved to mc_results_final.csv")