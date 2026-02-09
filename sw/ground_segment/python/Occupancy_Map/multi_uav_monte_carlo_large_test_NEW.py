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
from matplotlib.path import Path as MplPath


# Helper to map World (x,y) -> Grid (col, row)
def world_to_grid(x, y):
    c = int(round((x - x_min) / grid_resolution))
    r = int(round((y - y_min) / grid_resolution))
    return r, c

# Helper to map Grid (col, row) -> World (x,y)
def grid_to_world(r, c):
    return grid_x[c], grid_y[r]

# -----------------------------
# GLOBAL PARAMETERS
# -----------------------------
dt_step = 1.0
theta_FOV = np.deg2rad(45)
E_scale = 100.0
E_scale_track = 40.0
gamma_wind = 5.0
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
    

def energy_of_path(path, v_wind):
    # Use the global wind vector
    
    
    total_energy = 0.0
    v_climb, v_desc = 2.0, 3.0
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
            vg_allowed = max(2.0, min(20.0, 20.0 + v_headwind_comp))
            
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

# ==============================================================================
# 2. UPDATED PLANNER FUNCTION (CALLS NUMBA)
# ==============================================================================

def plan_velocity_ipp_3D(drone_pos, drone_vel, belief_map, inside_mask, grid_origin, grid_resolution, soft_poly, v_wind,
                         constraint_poly=None,
                         fov_angle=theta_FOV, v_max=20.0, n_directions=16,
                         step_length=40.0, altitude_candidates=[30, 50, 70],
                         E_scale=100.0,    
                         lam=0.4,          
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
        
        E_p = energy_of_path(path, v_wind)
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
    v_g_allowed = max(2.0, min(20.0, 20.0 + v_headwind_comp))
    
    # Calculate velocity components
    travel_time = max(dist / v_g_allowed, 0.1)
    
    vx = vec[0] / travel_time
    vy = vec[1] / travel_time
    vz = vec[2] / travel_time

    return float(vx), float(vy), float(vz), best_path, None, best_J


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

def clear_confirmed_region_2d(belief_map, XX, YY, victim_pos, wind_vec,
                              major_axis=120.0, minor_axis=60.0, decay=0.01):
    
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

def get_random_vector(magnitude):
    theta = np.random.uniform(0, 2 * np.pi)
    return np.array([magnitude * np.cos(theta), magnitude * np.sin(theta)])


def run_single_sim(num_drones, n_victims, map_s, max_time, mixed_alt=False, seed=None, viz=True):
    """
    Headless simulation with:
    - 2 m/s Random Wind
    - 0.1 m/s Random Drift
    - Victims spawned 100m inside boundaries
    - 'Teleport' logic for confirmed victims
    """
    if seed is not None:
        np.random.seed(seed)
    
    # --- 1. SETUP MAP & GRID ---
    # We use globals for speed where possible, but re-init for map size changes
    global map_size, half_s, soft_poly, inside_mask, rows, cols
    global grid_x, grid_y, XX, YY, x_min, x_max, y_min, y_max
    global grid_resolution

    map_size = map_s
    half_s = map_size / 2
    
    # Define Geofence
    softgeo_xy = np.array([[-half_s, -half_s], [half_s, -half_s], [half_s, half_s], [-half_s, half_s]])
    soft_poly = Polygon(softgeo_xy)
    
    # Safe Polygon for Physics (Buffer)
    buffer = 30.0
    safe_poly = soft_poly.buffer(-buffer)

    # Grid Setup
    x_min, y_min, x_max, y_max = soft_poly.bounds
    # grid_resolution = max(10.0, map_s / 100.0)
    grid_resolution = 10.0
    grid_x = np.arange(x_min, x_max + grid_resolution, grid_resolution)
    grid_y = np.arange(y_min, y_max + grid_resolution, grid_resolution)
    rows, cols = len(grid_y), len(grid_x)
    XX, YY = np.meshgrid(grid_x, grid_y)

    # Fast Mask Generation
    poly_path = MplPath(list(soft_poly.exterior.coords))
    points_flat = np.vstack((XX.flatten(), YY.flatten())).T
    inside_mask = poly_path.contains_points(points_flat).reshape(rows, cols)
    
    # Belief Init
    belief_map = 0.5 * np.ones((rows, cols))
    belief_map[~inside_mask] = 0.5

    # --- 2. SETUP ENVIRONMENT & ACTORS ---
    v_wind = get_random_vector(2.0)   # Standard 2 m/s
    v_drift = get_random_vector(0.1)  # Standard 0.1 m/s

    # Victims
    spawn_limit = half_s - 100.0
    if spawn_limit <= 0: spawn_limit = 10.0
    real_victims = []
    for _ in range(n_victims):
        vx = np.random.uniform(-spawn_limit, spawn_limit)
        vy = np.random.uniform(-spawn_limit, spawn_limit)
        real_victims.append([vx, vy])
    victims = np.array(real_victims)

    # Drones
    drone_positions = []
    uav_nominal_altitudes = []
    for i in range(num_drones):
        z = 50.0 if (mixed_alt and i % 2 == 0) else 40.0 if mixed_alt else 50.0
        jx = np.random.uniform(-5, 5)
        jy = np.random.uniform(-5, 5)
        drone_positions.append(np.array([jx, jy, z]))
        uav_nominal_altitudes.append(z)

    drone_positions = np.array(drone_positions)
    uav_nominal_altitudes = np.array(uav_nominal_altitudes)
    drone_vels = np.zeros((num_drones, 3))
    
    # State Init
    drone_modes = ["explore"] * num_drones
    track = [{"active": False, "confirmed": False, "pos": None, "time": None, "phase": None} for _ in range(num_drones)]
    cooldown_until = np.zeros(num_drones)
    confirmed_ids = set()
    energy_consumed = np.zeros(num_drones)

    entropy_history = []
    
    # Constants
    COOLDOWN_TIME = 30
    initial_lock_duration = 15.0
    cone_search_timeout = 30.0
    dt = 1.0
    pred_depth = 3
    E_scale = 100.0
    E_scale_track = 40.0
    MIN_ALT = 20.0

    # --- VISUALIZATION SETUP (ONLY IF VIZ=TRUE) ---
    if viz:
        cmap = cm.RdYlBu_r.copy()
        cmap.set_bad(color='white')
        fig, ax_map = plt.subplots(figsize=(8, 8))
        
        # Display Data
        display_data = belief_map.copy()
        display_data[~inside_mask] = np.nan
        im_display = ax_map.imshow(display_data, origin='lower', extent=[x_min, x_max, y_min, y_max],
                                   cmap=cmap, vmin=0.0, vmax=1.0, interpolation='nearest', zorder=1)
        
        ax_map.grid(visible=True, which='major', color='white', linestyle='-', linewidth=0.5, alpha=0.5)
        
        # Markers
        drone_plots = []
        for d_idx in range(num_drones):
            (p,) = ax_map.plot(drone_positions[d_idx][0], drone_positions[d_idx][1], 'o', markeredgecolor='k', zorder=10)
            drone_plots.append(p)
            
        victim_plot = ax_map.scatter(victims[:, 0], victims[:, 1], c='r', marker='x', s=60, zorder=10)
        
        cone_lines, cone_fills = [], []
        from matplotlib.patches import Polygon as MplPolygon
        for d_idx in range(num_drones):
            (line,) = ax_map.plot([], [], '--', lw=2, alpha=0.9)
            fill = MplPolygon([[0,0],[0,0],[0,0]], closed=True, alpha=0.15)
            ax_map.add_patch(fill)
            cone_lines.append(line)
            cone_fills.append(fill)

        ax_map.set_xlim(x_min - 20, x_max + 20)
        ax_map.set_ylim(y_min - 20, y_max + 20)
        plt.ion()
        plt.show()

    # --- PRE-LOOP INITIALIZATION ---
    start_time = time.time()
    frame_count = 0
    commanded_vels = np.zeros((num_drones, 3)) # Initialize container

    for t in range(600):
        # [INSERT THIS BLOCK]
        # Calculate Map Entropy (ignoring 0/1 to avoid NaN)
        valid_mask = inside_mask & (belief_map > 0.001) & (belief_map < 0.999)
        if np.any(valid_mask):
            p = belief_map[valid_mask]
            curr_ent = np.sum(-p * np.log2(p) - (1 - p) * np.log2(1 - p))
        else:
            curr_ent = 0.0
        entropy_history.append(curr_ent)
        # ------------------------------------------
        # 0. Dynamic mapping
        # ------------------------------------------
        shift_y = v_drift[1] * dt_step / grid_resolution
        shift_x = v_drift[0] * dt_step / grid_resolution
        
        # Shift
        belief_map = shift(belief_map, shift=[shift_y, shift_x], order=1, cval=0.5)
        
        # Reset boundaries (Prevents map shrinking)
        belief_map[~inside_mask] = 0.5 
        
        # INCREASE SIGMA: Makes "cleared" areas decay back to "unknown" faster
        belief_map = gaussian_filter(belief_map, sigma=1.0)
        # ------------------------------------------
        # 1. Victim motion (Drift)
        # ------------------------------------------
        # victims[:, :2] += v_drift * dt_step

        active_vic_mask = victims[:, 0] > -5000
        victims[active_vic_mask, :2] += v_drift * dt

    
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
            p_hit_val = P_hit(h)
            
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

        # END OF STEP 4
        # ... Continue to Step 5 (Tracker Assignment) ...
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
        # 6. Plan next velocities
        # ------------------------------------------
    
        commanded_vels = np.zeros((num_drones, 3))

    
        planning_belief = belief_map.copy()

        for d_idx in range(num_drones):
            tr = track[d_idx]

            if t < 5:
                # 360 degrees divided by N drones
                angle_rad = (2 * np.pi / num_drones) * d_idx 
                speed = v_max # Fast cruise speed
                
                vx_des = speed * np.cos(angle_rad)
                vy_des = speed * np.sin(angle_rad)
                vz_des = 0.0 
                
                # 1. Apply to Physics (This makes them move)
                drone_vels[d_idx] = np.array([vx_des, vy_des, vz_des])
                
                # 2. [FIX] Save to Logging Array (This makes the print correct)
                commanded_vels[d_idx] = [vx_des, vy_des, vz_des]
                continue

            # --- A. EXPLORATION MODE (With Reservation) ---
            if drone_modes[d_idx] == "explore":
                vx_des, vy_des, vz_des, best_path, best_mask, _ = plan_velocity_ipp_3D(
                    drone_positions[d_idx], 
                    drone_vels[d_idx],
                    planning_belief,      # <--- [FIX] Use Scratchpad
                    inside_mask,
                    (x_min, y_min),
                    grid_resolution,
                    soft_poly, v_wind,
                    constraint_poly=None,
                    step_length=40.0,
                    fov_angle=theta_FOV, v_max=v_max, n_directions=16,
                    altitude_candidates=[30, 50, 70], pred_depth=pred_depth, E_scale=E_scale,
                    lam=0.5,        
                    buffer=buffer
                )
                # Maintain nominal altitude
                if best_mask is not None:
                    planning_belief[best_mask] = 0.001

                vz_des = np.clip(uav_nominal_altitudes[d_idx] - drone_positions[d_idx][2], -1.0, 1.0)


            # --- B. TRACKING MODE ---
            elif drone_modes[d_idx] == "track" and tr["active"] and not tr["confirmed"]:

                # --- 1. Abort Check (Optimization) ---
                # Check if target belief has dropped to zero (cleared by someone else)
                if tr["pos"] is not None:
                    # Convert World Pos -> Grid Index
                    tc = int(round((tr["pos"][0] - x_min) / grid_resolution))
                    tr_idx = int(round((tr["pos"][1] - y_min) / grid_resolution))
                    
                    # Boundary check + Value check
                    if 0 <= tr_idx < rows and 0 <= tc < cols:
                        if belief_map[tr_idx, tc] < 0.05:
                            print(f"[t={t}] UAV{d_idx} target cleared by peer → Aborting.")
                            tr["active"] = False; tr["phase"] = None
                            drone_modes[d_idx] = "explore"
                            continue

                # --- PHASE: APPROACH ("to_detection") ---
                if tr["phase"] == "to_detection":
                    t_elapsed = t - tr["time"]
                    raw_target = tr["pos"] + v_drift * t_elapsed
                    
                    # Clamp target to Geofence
                    target_point = Point(raw_target[0], raw_target[1])
                    if not soft_poly.contains(target_point):
                        nearest_pt = soft_poly.exterior.interpolate(soft_poly.exterior.project(target_point))
                        target_now = np.array([nearest_pt.x, nearest_pt.y])
                    else:
                        target_now = raw_target
                    
                    vec = target_now - drone_positions[d_idx][:2]
                    dist = np.linalg.norm(vec)

                    # Braking Logic
                    if dist > 40.0: 
                        dir_unit = vec / (dist + 1e-6)
                        speed = np.clip((dist - 40.0) * 0.5, 5.0, v_max)
                        vx_des, vy_des = dir_unit * speed
                        vz_des = 0.0
                    else:
                        tr["phase"] = "initial_lock"
                        tr["lock_start"] = t
                        # Match drift velocity
                        vx_des, vy_des = v_drift
                        vz_des = 0.0

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

                    vz_des = 0.0 

                    if (t - tr["lock_start"]) >= initial_lock_duration:
                        tr["phase"] = "cone_tracking"
                        tr["cone_start"] = t


                # --- PHASE: CONE TRACKING (IPP inside cone) ---
                # elif tr["phase"] == "cone_tracking":
                #     det_pos = tr["pos"]
                #     det_time = tr["time"]

                #     # 1. Update Cone
                #     region_poly, center_R, _ = tracking_region(t, det_pos, det_time, v_drift)
                #     valid_search_area = soft_poly.intersection(region_poly)
                    
                #     if valid_search_area.is_empty:
                #         tr["active"] = False; drone_modes[d_idx] = "explore"
                #         continue

                #     # 2. Rasterize Cone (REPLACES: list comprehension)
                #     # We need to find belief stats inside the cone
                #     # Use MplPath for fast point-in-polygon on the grid
                #     poly_path = MplPath(list(region_poly.exterior.coords))
                #     points_flat = np.vstack((XX.flatten(), YY.flatten())).T
                #     mask_flat = poly_path.contains_points(points_flat)
                #     in_R_mask = mask_flat.reshape(rows, cols)
                    
                #     # 3. Calculate Stats
                #     if np.any(in_R_mask):
                #         p_vals = belief_map[in_R_mask]
                #         frac_high = np.mean(p_vals > confirm_pconf)
                #         mean_p = np.mean(p_vals)
                #         max_p = np.max(p_vals)

                #         # Success -> Hover
                #         if (frac_high >= rho_th or mean_p >= 0.30 or max_p >= peak_tresh):
                #             tr["phase"] = "hover_confirm"
                #             tr["fail_timer"] = 0.0
                #             tr["pos"] = drone_positions[d_idx][:2].copy() # Anchor
                #             tr["time"] = t
                #             continue

                #         # Descent Logic
                #         if max_p > 0.20 and drone_positions[d_idx][2] > MIN_ALT + 0.5:
                #             vz_des = -1.5 # Descend
                        
                #         # Timeout/Failure Logic
                #         z = drone_positions[d_idx][2]
                #         if z <= MIN_ALT + 0.5:
                #             tr.setdefault("fail_timer", 0.0)
                #             tr["fail_timer"] += dt
                #             if tr["fail_timer"] > 8.0:
                #                 print(f"UAV{d_idx} Bad Detection. Clearing area.")
                #                 # Clear local area in 2D
                #                 dx = XX - tr["pos_at_detection"][0]
                #                 dy = YY - tr["pos_at_detection"][1]
                #                 belief_map[(dx**2 + dy**2) < 40.0**2] = 0.0
                #                 tr["active"] = False; drone_modes[d_idx] = "explore"
                #                 vz_des = 1.5
                #                 continue
                        
                #         if (t - tr["cone_start"]) >= cone_search_timeout and max_p < 0.50:
                #             tr["active"] = False; drone_modes[d_idx] = "explore"
                #             vz_des = 1.0
                #             continue

                #     # 4. Motion: IPP constrained to Cone
                #     soft_cone_constraint = region_poly.buffer(15.0)
                    
                #     vx_ipp, vy_ipp, vz_ipp, _, _, _ = plan_velocity_ipp_3D(
                #         drone_positions[d_idx], 
                #         drone_vels[d_idx],
                #         belief_map, inside_mask,
                #         (x_min, y_min), grid_resolution,
                #         soft_poly, v_wind, 
                #         constraint_poly=soft_cone_constraint,
                #         step_length=20.0,
                #         fov_angle=theta_FOV, v_max=v_max, 
                #         altitude_candidates=[drone_positions[d_idx][2]], 
                #         pred_depth=2, E_scale=E_scale_track,
                #         lam=0.5,       
                #         buffer=buffer
                #     )

                #     vx_des = vx_ipp + v_drift[0] * 0.8
                #     vy_des = vy_ipp + v_drift[1] * 0.8
                #     if 'vz_des' not in locals(): vz_des = vz_ipp


                # # --- PHASE: HOVER CONFIRM ---
                # elif tr["phase"] == "hover_confirm":
                #     det_pos = tr["pos"]
                #     det_time = tr["time"]
                #     if "hover_start" not in tr: tr["hover_start"] = t

                #     # 1. Get Visibility Window & Local Belief
                #     (r_s, c_s), r_m, _ = get_view_window(drone_positions[d_idx])
                #     local_belief = belief_map[r_s, c_s]
                    
                #     # 2. Stabilized Targeting (Visual Lock Logic)
                #     hotspot_mask_local = (local_belief > confirm_pconf)
                    
                #     theoretical_center = det_pos + v_drift * (t - det_time)
                #     raw_target = theoretical_center # Default to model
                #     has_visual_lock = False

                #     if np.sum(hotspot_mask_local) > 3:
                #         # Reconstruct World Coordinates locally
                #         y_indices = np.arange(r_s.start, r_s.stop) * grid_resolution + y_min
                #         x_indices = np.arange(c_s.start, c_s.stop) * grid_resolution + x_min
                #         loc_x_mesh, loc_y_mesh = np.meshgrid(x_indices, y_indices)
                        
                #         coords_x = loc_x_mesh[hotspot_mask_local]
                #         coords_y = loc_y_mesh[hotspot_mask_local]
                #         weights = local_belief[hotspot_mask_local]**2
                        
                #         vis_x = np.average(coords_x, weights=weights)
                #         vis_y = np.average(coords_y, weights=weights)
                #         visual_centroid = np.array([vis_x, vis_y])

                #         # Gating: Only accept visual update if close to model prediction
                #         if np.linalg.norm(visual_centroid - theoretical_center) < 20.0:
                #             has_visual_lock = True
                #             raw_target = visual_centroid

                #     # Low Pass Filter (Prevents jerky movement)
                #     if "target_smooth" not in tr or tr["target_smooth"] is None:
                #         tr["target_smooth"] = raw_target

                #     alpha_pos = 0.4
                #     tr["target_smooth"] = (1 - alpha_pos) * tr["target_smooth"] + alpha_pos * raw_target
                #     target_xy = tr["target_smooth"]

                #     # 3. Control (P-Controller to chase target)
                #     err_vec = target_xy - drone_positions[d_idx][:2]
                #     dist_err = np.linalg.norm(err_vec)

                #     if dist_err > 20.0:
                #         speed = np.clip(dist_err * 0.5, 0.5, v_max)
                #         vx_cmd, vy_cmd = (err_vec / (dist_err + 1e-6)) * speed
                #     else:
                #         vx_cmd, vy_cmd = 0.0, 0.0
                    
                #     # Add Drift Compensation
                #     vx_des = vx_cmd + v_drift[0]
                #     vy_des = vy_cmd + v_drift[1]

                #     # Vertical Logic (Descend only when centered)
                #     current_alt = drone_positions[d_idx][2]
                #     allowed_err = np.interp(current_alt, [20.0, 80.0], [2.0, 10.0])
                    
                #     if has_visual_lock and dist_err > allowed_err:
                #         vz_des = 0.0 # Wait to center
                #     elif current_alt > 20.0:
                #         vz_des = -1.5 # Descend
                #     else:
                #         vz_des = 0.0 # Hold

                #     # 4. Confirmation Stats & Logic
                #     # ---------------------------------------------------------
                #     # Calculate stats first so we can use them for both Confirmation AND Timeout
                #     p_vals = local_belief[hotspot_mask_local]
                #     mean_p = 0.0
                #     max_p = 0.0
                    
                #     if len(p_vals) > 0:
                #         mean_p = np.mean(p_vals)
                #         max_p = np.max(p_vals)

                #     # Dynamic Ceiling Calculation
                #     if mean_p >= 0.90: req_alt = 35.0
                #     elif mean_p >= 0.70: req_alt = 28.0
                #     else: req_alt = 22.0
                    
                #     # A. CONFIRMATION CHECK
                #     if len(p_vals) > 0 and (current_alt <= req_alt and max_p >= peak_tresh):
                        
                #         # --- DISTINCT VICTIM CHECK (Teleportation) ---
                #         my_pos = drone_positions[d_idx][:2]
                #         dists = np.linalg.norm(victims[:, :2] - my_pos, axis=1)
                #         closest_id = np.argmin(dists)
                #         closest_dist = dists[closest_id]
                        
                #         if closest_dist < 50.0:
                #             if closest_id not in confirmed_ids:
                #                 print(f"[t={t}] UAV{d_idx} CONFIRMED DISTINCT victim #{closest_id}!")
                #                 confirmed_ids.add(closest_id)
                                
                #                 # TELEPORT VICTIM
                #                 victims[closest_id] = np.array([-9999.0, -9999.0])
                                
                #                 # Clear Map
                #                 belief_map = clear_confirmed_region_2d(belief_map, XX, YY, my_pos, v_drift)
                #                 dx = XX - my_pos[0]; dy = YY - my_pos[1]
                #                 belief_map[(dx**2 + dy**2) < 50.0**2] *= 0.05
                                
                #                 # Reset Drone
                #                 tr["active"] = False; tr["confirmed"] = True
                #                 drone_modes[d_idx] = "explore"; vz_des = 2.0
                #             else:
                #                 # Re-detection (Already found)
                #                 belief_map = clear_confirmed_region_2d(belief_map, XX, YY, my_pos, v_drift)
                #                 tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 2.0
                #         else:
                #             # False Positive
                #             print(f"[t={t}] UAV{d_idx} False Positive (No victim < 50m). Clearing map.")
                #             belief_map = clear_confirmed_region_2d(belief_map, XX, YY, my_pos, v_drift)
                #             tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 2.0

                #     # B. TIMEOUT LOGIC (Precise)
                #     # If we are low enough to see, but haven't confirmed after 10s, abort.
                #     if (
                #         (current_alt <= req_alt) 
                #         and (t - tr["hover_start"]) > 10.0
                #     ):
                #         print(f"[t={t}] UAV{d_idx} hover-confirm timeout (low alt, low conf) → aborting.")
                #         tr["active"] = False; drone_modes[d_idx] = "explore"
                #         vz_des = 2.0
                    
                #     # C. HARD TIMEOUT (Safety)
                #     # If we get stuck in this mode for any reason (e.g. stuck high up)
                #     elif (t - tr["hover_start"]) > 20.0:
                #         tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 2.0


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

                    # 2. Rasterize Cone
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

                        # Success -> Hover (Lowered threshold slightly to encourage confirmation attempts)
                        if (frac_high >= rho_th or mean_p >= 0.25 or max_p >= peak_tresh):
                            tr["phase"] = "hover_confirm"
                            tr["fail_timer"] = 0.0
                            tr["hover_start"] = t # Reset start time for next phase
                            tr["pos"] = drone_positions[d_idx][:2].copy() # Anchor
                            tr["time"] = t
                            continue

                        # Descent Logic
                        if max_p > 0.20 and drone_positions[d_idx][2] > MIN_ALT + 0.5:
                            vz_des = -1.5 # Descend
                        
                        # Timeout/Failure Logic
                        z = drone_positions[d_idx][2]
                        
                        # [FIX 1] Relaxed Low-Altitude Abort
                        # Only abort if we are low AND have very weak signal for > 10 seconds
                        if z <= MIN_ALT + 2.0:
                            tr.setdefault("fail_timer", 0.0)
                            if max_p < 0.3: # Only increment if signal is weak
                                tr["fail_timer"] += dt
                            else:
                                tr["fail_timer"] = 0.0 # Reset if we see something good

                            if tr["fail_timer"] > 10.0:
                                print(f"UAV{d_idx} Low Signal at Low Alt. Clearing area.")
                                dx = XX - tr["pos_at_detection"][0]
                                dy = YY - tr["pos_at_detection"][1]
                                belief_map[(dx**2 + dy**2) < 40.0**2] = 0.0
                                tr["active"] = False; drone_modes[d_idx] = "explore"
                                vz_des = 1.5
                                continue
                        
                        # [FIX 2] Extended General Timeout
                        if (t - tr["cone_start"]) >= (cone_search_timeout + 15.0) and max_p < 0.40:
                            tr["active"] = False; drone_modes[d_idx] = "explore"
                            vz_des = 1.0
                            continue

                    # 4. Motion: IPP constrained to Cone
                    soft_cone_constraint = region_poly.buffer(15.0)
                    
                    vx_ipp, vy_ipp, vz_ipp, _, _, _ = plan_velocity_ipp_3D(
                        drone_positions[d_idx], 
                        drone_vels[d_idx],
                        belief_map, inside_mask,
                        (x_min, y_min), grid_resolution,
                        soft_poly, v_wind, 
                        constraint_poly=soft_cone_constraint,
                        step_length=20.0,
                        fov_angle=theta_FOV, v_max=v_max, 
                        altitude_candidates=[drone_positions[d_idx][2]], 
                        pred_depth=2, E_scale=E_scale_track,
                        lam=0.5,       
                        buffer=buffer
                    )

                    vx_des = vx_ipp + v_drift[0] * 0.8
                    vy_des = vy_ipp + v_drift[1] * 0.8
                    if 'vz_des' not in locals(): vz_des = vz_ipp


                # --- PHASE: HOVER CONFIRM ---
                elif tr["phase"] == "hover_confirm":
                    det_pos = tr["pos"]
                    det_time = tr["time"]
                    if "hover_start" not in tr: tr["hover_start"] = t

                    # 1. Get Visibility Window & Local Belief
                    (r_s, c_s), r_m, _ = get_view_window(drone_positions[d_idx])
                    local_belief = belief_map[r_s, c_s]
                    
                    # 2. Stabilized Targeting (Visual Lock Logic)
                    hotspot_mask_local = (local_belief > confirm_pconf)
                    
                    theoretical_center = det_pos + v_drift * (t - det_time)
                    raw_target = theoretical_center # Default to model
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

                        if np.linalg.norm(visual_centroid - theoretical_center) < 20.0:
                            has_visual_lock = True
                            raw_target = visual_centroid

                    # Low Pass Filter
                    if "target_smooth" not in tr or tr["target_smooth"] is None:
                        tr["target_smooth"] = raw_target

                    alpha_pos = 0.4
                    tr["target_smooth"] = (1 - alpha_pos) * tr["target_smooth"] + alpha_pos * raw_target
                    target_xy = tr["target_smooth"]

                    # 3. Control (P-Controller to chase target)
                    err_vec = target_xy - drone_positions[d_idx][:2]
                    dist_err = np.linalg.norm(err_vec)

                    # [FIX 3] Fixed Control Logic
                    # Old code STOPPED if dist > 20. This was backwards.
                    # We want to chase if distance > small_tolerance (e.g. 0.5m)
                    if dist_err > 0.5:
                        speed = np.clip(dist_err * 0.8, 0.5, v_max) # Increased gain slightly
                        vx_cmd, vy_cmd = (err_vec / (dist_err + 1e-6)) * speed
                    else:
                        vx_cmd, vy_cmd = 0.0, 0.0
                    
                    vx_des = vx_cmd + v_drift[0]
                    vy_des = vy_cmd + v_drift[1]

                    # Vertical Logic
                    current_alt = drone_positions[d_idx][2]
                    # Allow descent if we are reasonably close (< 5m horizontal error)
                    if current_alt > 20.0:
                        if dist_err < 5.0:
                            vz_des = -1.5 
                        else:
                            vz_des = 0.0 # Wait to align horizontally
                    else:
                         vz_des = 0.0

                    # 4. Confirmation Stats & Logic
                    p_vals = local_belief[hotspot_mask_local]
                    req_alt = 20.0
                    mean_p = 0.0
                    max_p = 0.0
                    
                    if len(p_vals) > 0:
                        mean_p = np.mean(p_vals)
                        max_p = np.max(p_vals)

                    # if mean_p >= 0.90: req_alt = 35.0
                    # elif mean_p >= 0.70: req_alt = 28.0

                    # Works well now without the dynamic ceiling, which was causing some issues. We can re-tune if needed.
                    
                    # A. CONFIRMATION CHECK
                    if len(p_vals) > 0 and (current_alt <= req_alt and max_p >= peak_tresh):
                        
                        my_pos = drone_positions[d_idx][:2]
                        dists = np.linalg.norm(victims[:, :2] - my_pos, axis=1)
                        closest_id = np.argmin(dists)
                        closest_dist = dists[closest_id]
                        
                        if closest_dist < 50.0:
                            if closest_id not in confirmed_ids:
                                print(f"[t={t}] UAV{d_idx} CONFIRMED DISTINCT victim #{closest_id}!")
                                confirmed_ids.add(closest_id)
                                victims[closest_id] = np.array([-9999.0, -9999.0])
                                
                                belief_map = clear_confirmed_region_2d(belief_map, XX, YY, my_pos, v_drift)
                                dx = XX - my_pos[0]; dy = YY - my_pos[1]
                                belief_map[(dx**2 + dy**2) < 50.0**2] *= 0.05
                                
                                tr["active"] = False; tr["confirmed"] = True
                                drone_modes[d_idx] = "explore"; vz_des = 2.0
                            else:
                                belief_map = clear_confirmed_region_2d(belief_map, XX, YY, my_pos, v_drift)
                                tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 2.0
                        else:
                            print(f"[t={t}] UAV{d_idx} False Positive (No victim < 50m). Clearing map.")
                            belief_map = clear_confirmed_region_2d(belief_map, XX, YY, my_pos, v_drift)
                            tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 2.0

                    # B. TIMEOUT LOGIC
                    # [FIX 4] Extended Timeout
                    # Increased from 10.0 to 20.0 to give time for descent
                    if (t - tr["hover_start"]) > 20.0:
                        # Only abort if we really failed to confirm
                         if current_alt <= req_alt:
                            print(f"[t={t}] UAV{d_idx} hover-confirm timeout (low alt, low conf) → aborting.")
                            tr["active"] = False; drone_modes[d_idx] = "explore"
                            vz_des = 2.0
                         # Safety break if we are stuck high up for too long (e.g. 40s)
                         elif (t - tr["hover_start"]) > 40.0:
                            tr["active"] = False; drone_modes[d_idx] = "explore"; vz_des = 2.0

                else:
                    vx_des, vy_des, vz_des = 0.0, 0.0, 0.0

            # --- Fallback ---
            else:
                vx_des, vy_des, vz_des, *_ = plan_velocity_ipp_3D(
                    drone_positions[d_idx], 
                    drone_vels[d_idx],
                    planning_belief,    # <--- [FIX] Fallback also uses scratchpad
                    inside_mask,
                    (x_min, y_min), grid_resolution,
                    soft_poly, v_wind,
                    constraint_poly=None,
                    step_length=40.0,
                    fov_angle=theta_FOV, v_max=v_max, n_directions=16,
                    altitude_candidates=[30, 50, 70], pred_depth=pred_depth, E_scale=E_scale,
                    lam=0.5,        
                    buffer=buffer
                )
            
            commanded_vels[d_idx] = [vx_des, vy_des, vz_des]

          

            # 1. Calculate the requested jump from current velocity to desired
            v_current = drone_vels[d_idx][:2]
            v_target = np.array([vx_des, vy_des])
            
            delta_v = v_target - v_current
            dist_v = np.linalg.norm(delta_v)
            
            # 2. Universal Acceleration Limit (2.0 m/s^2)
            max_accel = 2.0 
            max_change = max_accel * dt_step
            
            if dist_v > max_change:
                # Clamp the change vector magnitude
                ratio = max_change / dist_v
                delta_v = delta_v * ratio
                
            # 3. Apply Horizontal Velocity
            drone_vels[d_idx][:2] = v_current + delta_v
            
            # 4. Vertical Smoothing (Limit vertical accel to 1.0 m/s^2)
            z_diff = vz_des - drone_vels[d_idx][2]
            drone_vels[d_idx][2] += np.clip(z_diff, -1.0, 1.0)
            
    
        # ------------------------------------------
        # 7. Safety Clamp, Collision Avoidance & Integration
        # ------------------------------------------
        
        # A. Calculate Repulsion Forces (Inter-UAV Collision Avoidance)
        # -------------------------------------------------------------
        # We accumulate repulsion vectors first so the order of drones doesn't matter.
        repulsion_vels = np.zeros((num_drones, 3))
        safe_separation = 15.0  # [m] Minimum comfortable distance
        repulsion_gain = 2.0    # Strength of the "push"

        for i in range(num_drones):
            for j in range(i + 1, num_drones):
                # Vector from J to I
                diff_vec = drone_positions[i] - drone_positions[j]
                dist = np.linalg.norm(diff_vec)
                
                # If too close, push them apart
                if dist < safe_separation and dist > 0.1:
                    # Normalized direction * magnitude proportional to violation
                    push_vec = (diff_vec / dist) * (safe_separation - dist) * repulsion_gain
                    
                    # Apply equal and opposite push
                    repulsion_vels[i] += push_vec  # Push I away from J
                    repulsion_vels[j] -= push_vec  # Push J away from I

        # B. Apply Forces & Integrate
        # -------------------------------------------------------------
        for d_idx in range(num_drones):
            
            # 1. Apply Repulsion to desired velocity
            # We treat this as a high-priority "reflex" adjustment
            drone_vels[d_idx] += repulsion_vels[d_idx]

            # 2. Predicted Next Position
            next_pos = drone_positions[d_idx] + drone_vels[d_idx] * dt_step
            
            # --- FAST GEOFENCE CHECK (Optimized) ---
            # Check the mask first. If True, we skip the slow Shapely math.
            c_idx = int(round((next_pos[0] - x_min) / grid_resolution))
            r_idx = int(round((next_pos[1] - y_min) / grid_resolution))
            
            is_safe = False
            if 0 <= r_idx < rows and 0 <= c_idx < cols:
                if inside_mask[r_idx, c_idx]:
                    is_safe = True
            
            # 3. Geofence Correction (Only run slow math if 'is_safe' is False)
            if not is_safe:
                point_next = Point(next_pos[0], next_pos[1])
                if not soft_poly.contains(point_next):
                    # Project onto boundary (The slow part)
                    nearest = np.array(safe_poly.exterior.interpolate(
                        safe_poly.exterior.project(point_next)
                    ).coords[0])
                    
                    direction_vec = nearest - drone_positions[d_idx][:2]
                    norm = np.linalg.norm(direction_vec)
                    
                    if norm > 1e-3:
                        unit_dir = direction_vec / norm
                        v_headwind_comp = np.dot(v_wind[:2], unit_dir)
                        v_g_allowed = 20.0 + v_headwind_comp 
                        speed_limit = max(2.0, min(20.0, v_g_allowed)) 
                        
                        corr_speed = min(norm / dt_step, speed_limit)
                        drone_vels[d_idx][:2] = unit_dir * corr_speed
                    else:
                        drone_vels[d_idx][:2] = [0.0, 0.0]

            # 4. Final Speed Clamp (Wind Aware)
            v_g = drone_vels[d_idx][:2]
            v_g_mag = np.linalg.norm(v_g)
            if v_g_mag > 1e-3:
                unit_vg = v_g / v_g_mag
                v_headwind_comp = np.dot(v_wind[:2], unit_vg)
                v_g_limit = 20.0 + v_headwind_comp
                if v_g_mag > v_g_limit:
                    drone_vels[d_idx][:2] = unit_vg * max(2.0, v_g_limit)

            # 5. Advance Position
            drone_positions[d_idx] += drone_vels[d_idx] * dt_step

    
        if viz:
            if t % 1 == 0: 
                display_data = belief_map.copy()
                display_data[~inside_mask] = np.nan
                im_display.set_data(display_data)

                for d_idx in range(num_drones):
                    drone_plots[d_idx].set_data(drone_positions[d_idx][0], drone_positions[d_idx][1])
                    tr = track[d_idx]
                    if tr["active"] and tr["pos"] is not None:
                        region_poly, _, _ = tracking_region(t, tr["pos"], tr["time"], v_drift)
                        if region_poly:
                            xR, yR = region_poly.exterior.xy
                            cone_lines[d_idx].set_data(xR, yR)
                            cone_fills[d_idx].set_xy(np.column_stack([xR, yR]))
                    else:
                        cone_lines[d_idx].set_data([], [])
                        cone_fills[d_idx].set_xy(np.zeros((3,2)))

                victim_plot.set_offsets(victims[:, :2])
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(0.0001)

                frame_count += 1
                if time.time() - start_time >= 1.0:
                    print(f"FPS: {frame_count}")
                    frame_count = 0; start_time = time.time()

        # ------------------------------------------
        # 7. Check Success [CHANGE 3]
        # ------------------------------------------
        # ... inside loop ...
        if len(confirmed_ids) >= n_victims:
            success_flag = True
            final_time = t
            break # Exit loop

    # [REPLACE OLD RETURNS WITH THIS]
    return {
        "success": len(confirmed_ids) >= n_victims,
        "time": t,
        "energy_list": energy_consumed.tolist(),
        "victims_found": len(confirmed_ids),
        "total_victims": n_victims,
        "entropy_history": entropy_history
    }
        
        


if __name__ == "__main__":
    import pandas as pd
    
    
    # Change this to True to verify the code works visually
    DEBUG_VISUAL = True  
    
    if DEBUG_VISUAL:
        print("Running Single Debug Simulation (Visual)...")
        # Run 1 trial: Map=500m, 4 Drones, 3 Victims, Mixed Altitude
        success, duration, _ = run_single_sim(
            num_drones=2, n_victims=10, map_s=500, max_time=600, 
            mixed_alt=True, seed=42, viz=True
        )
        print(f"Debug Result: Success={success}, Time={duration}s")
        exit() # Stop here so we don't start the 1-hour batch run immediately


    # --- 2. BATCH MODE (RUN THIS FOR DATA COLLECTION) ---
    TRIALS = 10
    MAX_TIME = 600
    
    CONFIGS = [
        (500,  [2, 4, 6, 8, 10]),
        (2000, [10, 14, 18, 22, 26]),
        (5000, [70, 80, 90, 100, 110])
    ]
    VICTIM_LEVELS = [3, 5, 10]
    ALT_MODES = [False, True]

    results = []
    print(f"--- STARTING BATCH SIMULATION ---")

    import os, json
    # ... (Previous Configs) ...

    # --- [CHANGE] Folder Setup ---
    base_dir = "large_test_NEW"
    json_dir = os.path.join(base_dir, "json")
    csv_file = os.path.join(base_dir, "experiment_results.csv")
    
    # Create directories
    os.makedirs(json_dir, exist_ok=True)
    
    # Initialize CSV if missing
    if not os.path.exists(csv_file):
        headers = ["Map", "Drones", "Victims", "Mode", "Success", "Time", "Avg_Energy", "Victims_Found", "Seed"]
        pd.DataFrame(columns=headers).to_csv(csv_file, index=False)

    print(f"--- STARTING BATCH SIMULATION ---")
    print(f"Results Directory: {base_dir}/")

    for map_s, drone_counts in CONFIGS:
        # ... (Loops continue) ...
        for n_drones in drone_counts:
            for n_victims in VICTIM_LEVELS:
                # if n_victims > n_drones: continue 

                for mixed in ALT_MODES:
                    mode_str = "Mixed" if mixed else "Uniform"
                    print(f"Running: Map {map_s} | Drones {n_drones} | Vic {n_victims} | {mode_str}")
                    
                    for i in range(TRIALS):
                        seed = np.random.randint(0, 1000000)
                        try:
                            # Run Simulation
                            data = run_single_sim(
                                n_drones, n_victims, map_s, MAX_TIME, mixed, seed, viz=False
                            )
                            
                            # 1. Update CSV (Calculate Avg Energy here)
                            avg_energy = np.mean(data["energy_list"])
                            
                            row = {
                                "Map": map_s, "Drones": n_drones, "Victims": n_victims,
                                "Mode": mode_str, "Success": data["success"], 
                                "Time": data["time"], "Avg_Energy": avg_energy,
                                "Victims_Found": data["victims_found"], 
                                "Seed": seed
                            }
                            # Save to CSV
                            pd.DataFrame([row]).to_csv(csv_file, mode='a', header=False, index=False)
                            
                            # 2. Save JSON (Remove Energy List)
                            json_data = data.copy()
                            
                            # [CHANGE] Remove huge energy list to save space
                            if "energy_list" in json_data:
                                del json_data["energy_list"]
                                
                            # Add metadata
                            json_data["config"] = {
                                "map": map_s, "drones": n_drones, 
                                "victims": n_victims, "mode": mode_str, "seed": seed
                            }
                            
                            log_name = f"log_{map_s}m_n{n_drones}_v{n_victims}_{mode_str}_{seed}.json"
                            log_path = os.path.join(json_dir, log_name)
                            
                            with open(log_path, 'w') as f:
                                json.dump(json_data, f, indent=4)
                                
                        except Exception as e:
                            print(f"Error: {e}")

    pd.DataFrame(results).to_csv(csv_file, index=False)
    print("Done.")