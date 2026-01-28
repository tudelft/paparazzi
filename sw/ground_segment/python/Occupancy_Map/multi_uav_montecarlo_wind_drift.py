import numpy as np
import random
import sys
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, Point
import pymap3d as pm
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter
from shapely.affinity import rotate, translate
import pandas as pd

# --- Paparazzi setup ---
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))
sys.path.append(PPRZ_HOME + "/sw/ext/pprzlink/lib/v1.0/python")


lat0, lon0, alt0 = 52.1681551, 4.4126468, 0.0
xml_file = os.path.expanduser("~/paparazzi2/paparazzi/conf/flight_plans/SDB/sdb_rotwing_EHVB.xml")
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
buffer_val = 30.0
safe_poly = soft_poly.buffer(-buffer_val) if buffer_val > 0 else soft_poly

# Precompute Grid globally once
x_min, y_min, x_max, y_max = soft_poly.bounds
grid_resolution = 10.0
grid_x = np.arange(x_min, x_max + grid_resolution, grid_resolution)
grid_y = np.arange(y_min, y_max + grid_resolution, grid_resolution)
XX, YY = np.meshgrid(grid_x, grid_y)
grid_points_all = np.column_stack([XX.ravel(), YY.ravel()])
inside_mask_full = np.array([soft_poly.contains(Point(p)) for p in grid_points_all])
grid_points = grid_points_all[inside_mask_full]

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
# 1. HELPER FUNCTIONS 
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
    

def energy_of_path(path, v_wind_local):
    """
    Calculates total energy (Joules) for a path using a specific wind vector.
    Removed 'global v_wind' to support Monte Carlo iterations.
    """
    total_energy = 0.0
    v_climb, v_desc = 2.0, 3.0
    P_climb = get_vsqp_power(0.0) # Hover/Climb power (1751 W)

    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i+1]
        vec_g = (p1[:2] - p0[:2])
        dist_xy = np.linalg.norm(vec_g)
        dz = p1[2] - p0[2]

        if dist_xy > 1e-3:
            unit_vec = vec_g / (dist_xy + 1e-6)
            
            # Use local wind vector to find allowed ground speed
            v_headwind_comp = np.dot(v_wind_local[:2], unit_vec)
            vg_allowed = max(2.0, min(20.0, 20.0 + v_headwind_comp))
            
            v_ground_vec = unit_vec * vg_allowed
            
            # Resulting airspeed vector determines power draw
            v_air_vec = v_ground_vec - v_wind_local[:2]
            v_air_mag = np.linalg.norm(v_air_vec)
            
            # Power draw is based on sensed Airspeed
            P_segment = get_vsqp_power(v_air_mag)
            
            t_segment = dist_xy / vg_allowed
            E_horiz = P_segment * t_segment
        else:
            E_horiz = 0.0

        # Vertical Energy logic remains unchanged
        if dz > 0:   
            E_vert = P_climb * (dz / v_climb)
        elif dz < 0: 
            E_vert = P_climb * (-dz / v_desc) * 0.5
        else:        
            E_vert = 0.0
        
        total_energy += (E_horiz + E_vert)
        
    return total_energy


# # ==============================================================================
# # 2. THE PLANNER 
# # ==============================================================================

def plan_velocity_ipp_3D(drone_pos, drone_vel, belief, grid_points, soft_poly,
                         v_wind_local,  # ADDED: Explicit wind vector for MC runs
                         constraint_poly=None,
                         fov_angle=theta_FOV, v_max=20.0, n_directions=16,
                         step_length=40.0, altitude_candidates=[50],
                         E_scale=100.0, lam=0.4, buffer=0.0, pred_depth=3):
    
    # 1. Constraint Logic (remains unchanged)
    if constraint_poly is not None:
        valid_area = soft_poly.intersection(constraint_poly).buffer(0)
    else:
        valid_area = Polygon(soft_poly)
        if buffer > 0.0: valid_area = valid_area.buffer(-buffer)

    if valid_area.is_empty:
        return 0.0, 0.0, 0.0, [drone_pos, drone_pos], None, 0.0

    # 2. Motion Primitives (remains unchanged)
    cx, cy, cz = drone_pos
    vx, vy, _ = drone_vel
    current_speed = np.linalg.norm([vx, vy])

    if current_speed > 2.0:
        current_heading = np.arctan2(vy, vx)
        angles = current_heading + np.linspace(-np.pi/3, np.pi/3, n_directions)
    else:
        angles = np.linspace(0, 2*np.pi, n_directions, endpoint=False)

    # 3. Generate Candidate Trajectories
    candidates = []
    for a in angles:
        dx, dy = np.cos(a), np.sin(a)
        for alt in altitude_candidates:
            trajectory = [np.array([cx, cy, cz])]
            valid_traj = True
            for k in range(pred_depth):
                prev = trajectory[-1]
                nx, ny, nz = prev[0] + dx * step_length, prev[1] + dy * step_length, alt
                if not valid_area.contains(Point(nx, ny)):
                    valid_traj = False; break
                trajectory.append(np.array([nx, ny, nz]))
            if valid_traj: candidates.append(trajectory)

    # 4. Evaluation Loop (Matches J = I - lambda * E)
    best_J, best_path, best_mask = -np.inf, None, None

    for path in candidates:
        I_p, vis_mask = expected_info_gain(path, belief, grid_points, fov_angle, pred_depth)
        
        # PASS LOCAL WIND TO ENERGY MODEL
        E_p = energy_of_path(path, v_wind_local)
        
        J = I_p - (lam * (E_p / E_scale))
        
        if J > best_J:
            best_J, best_path, best_mask = J, path, vis_mask

    if best_path is None:
        return 0.0, 0.0, 0.0, [drone_pos, drone_pos], None, 0.0

    # 5. Output Velocity Calculation (Wind-Aware Fix using local wind)
    p0, p1 = best_path[0], best_path[1]
    vec = p1 - p0
    dist = np.linalg.norm(vec)
    unit_vec = vec / (dist + 1e-6)

    # Calculate final ground speed command using local wind
    v_headwind_comp = np.dot(v_wind_local[:2], unit_vec[:2])
    v_g_allowed = 20.0 + v_headwind_comp 
    v_g_final = max(2.0, min(20.0, v_g_allowed)) 

    travel_time = max(dist / v_g_final, 0.1)
    vx_des, vy_des, vz_des = vec[0] / travel_time, vec[1] / travel_time, vec[2] / travel_time

    return float(vx_des), float(vy_des), float(vz_des), best_path, best_mask, best_J



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


def get_random_vector(magnitude):
    """Generates a vector with fixed magnitude but random direction."""
    theta = np.random.uniform(0, 2 * np.pi)
    return np.array([magnitude * np.cos(theta), magnitude * np.sin(theta)])

def get_random_victims(poly, n):
    """Generates n random points inside the geofence."""
    victims = []
    minx, miny, maxx, maxy = poly.bounds
    while len(victims) < n:
        p = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
        if poly.contains(p):
            victims.append([p.x, p.y])
    return np.array(victims)

def get_random_points_in_poly(poly, n):
    """Generates n random points strictly inside a polygon."""
    points = []
    minx, miny, maxx, maxy = poly.bounds
    while len(points) < n:
        p = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
        if poly.contains(p):
            points.append([p.x, p.y])
    return np.array(points)



### Under here, we have the sim loop:

def run_single_sim(num_drones, num_victims, severity_settings, seed):

    # # # Storage for energy history
    energy_time = []
    energy_history = [np.empty((0,)) for _ in range(num_drones)]

    # # -----------------------------
    # # SIMULATION LOOP 
    # # -----------------------------
    max_dheading = np.deg2rad(10)  # max heading change per second
    # buffer = 30.0 # safe buffer for polygon

    # # Initialize UAV states and tracking flags
    drone_modes = ["explore"] * num_drones   # 'explore' or 'track'
    # detection_active = [False]*num_drones
    # detection_confirmed = [False]*num_drones
    detection_pos = [None]*num_drones
    # detection_time = [None]*num_drones
    # tracker_phase = [None]*num_drones
    cooldown_until = np.zeros(num_drones)
    COOLDOWN_TIME = 30  # seconds of detection lockout
    # cone_artists = []

    # # --- Tracking timing parameters ---
    initial_lock_duration = 8.0    # seconds tracking cone center before IPP
    cone_search_timeout   = 30.0   # max time doing IPP inside cone before abort
    # HOVER_CONFIRM_TIME = 10.0  # seconds at low altitude before abort if not confirmed
    dt = dt_step           # simulation time step [s]
    MIN_ALT = 20.0
    DESCENT_STEP = 3.0


    # Set seeds for reproducibility
    np.random.seed(seed)
    random.seed(seed)
    
    # 1. Constants and Environment setup
    E_scale = 100.0
    E_scale_track = 40.0    
    E_max = 100.0
    dt = dt_step
    pred_depth = 3
    
    wind_mag, drift_mag = severity_settings
    v_wind = get_random_vector(wind_mag)
    v_drift = get_random_vector(drift_mag)

    # 2. Reset belief for this specific run
    belief = 0.5 * np.ones(len(grid_points))
    initial_entropy = np.sum(cell_entropy_map(belief))
    
    # 3. Randomize Victims and Drones
    victims_pos = get_random_points_in_poly(soft_poly, num_victims)
    victims = np.array(victims_pos, dtype=float) 
    
    start_coords = get_random_points_in_poly(safe_poly, num_drones)
    drone_positions = [np.array([c[0], c[1], 50.0]) for c in start_coords]
    uav_nominal_altitudes = np.ones(num_drones) * 50.0

    drone_start_coords = [p.copy() for p in drone_positions]
    victim_start_coords = victims_pos.copy()
    
    drone_vels = [np.zeros(3) for _ in range(num_drones)]
    drone_modes = ['explore' for _ in range(num_drones)]
    energy_consumed_joules = np.zeros(num_drones)
    E_rem = np.ones(num_drones) * E_max
    
    # 4. Initialize Tracking and Stats 
    # stats = {"confirmed": 0, "aborted": 0}
 
    stats = {"confirmed_ids": set(), "aborted": 0}
    cooldown_until = np.zeros(num_drones)
    detection_event = [False] * num_drones
    

    track = []
    for _ in range(num_drones):
        track.append({
            "active": False,
            "confirmed": False,
            "pos": None,            
            "time": None,           
            "phase": None,
            "lock_start": None,     
            "cone_start": None,
            "pos_at_detection": None,
            "fail_timer": 0.0,
            "target_smooth": None
        })
        

    for t in range(600):
        # ------------------------------------------
        # 0. Dynamic mapping: propagate belief (drift model)
        # ------------------------------------------
        belief = propagate_belief_vectorized(belief, grid_points, v_drift, dt_step, sigma=1.0)
        # ------------------------------------------
        #  Victim motion (drift)
        # ------------------------------------------
        victims[:, :2] += v_drift * dt_step
        # ------------------------------------------
        #  Victim signal (altitude-dependent footprint)
        # ------------------------------------------
        victim_signal_full = np.zeros(len(grid_points_all), dtype=float)
        # use the lowest UAV altitude (most sensitive footprint)
        min_alt = min(pos[2] for pos in drone_positions)

        for v in victims:
            vsig = victim_signal_at_alt(grid_points_all, v, min_alt)
            victim_signal_full = np.maximum(victim_signal_full, vsig)

        victim_signal = victim_signal_full[inside_mask_full]

        # ------------------------------------------
        #  FOV overlap counting with altitude-dependent p_hit
        # ------------------------------------------
        # update_weight = np.zeros(len(grid_points), dtype=float)
        update_weight = np.zeros(len(belief), dtype=float)


        for d_idx in range(num_drones):
            vis_mask_all, p_hit_all = visible_cells_at(drone_positions[d_idx], grid_points_all)

            vis_mask_inside = vis_mask_all[inside_mask_full]       # mask over grid_points
            p_hit_inside = p_hit_all[inside_mask_full]             # probabilities over grid_points
            update_weight[vis_mask_inside] += p_hit_inside[vis_mask_inside]

        # cap at 1.0 so multiple UAVs don't exceed "certain" detection
        update_weight = np.clip(update_weight, 0.0, 1.0)

        #  Bayesian mapping update 
        # ------------------------------------------

        # Precompute which grid cells contain the victim(s)
        victim_cell_mask = np.zeros(len(grid_points), dtype=bool)
        for v in victims:
            d2 = np.sum((grid_points - v[:2])**2, axis=1)
            # victim_cell_mask |= (d2 <= cell_size**2)   # victim occupies its nearest cell
            victim_cell_mask |= (d2 == np.min(d2))


        for d_idx in range(num_drones):

            drone_pos = drone_positions[d_idx]

            # Visibility on the belief grid (not full grid)
            vis_mask, p_hit = visible_cells_at(drone_pos, grid_points)

            # Extract priors for visible cells
            p_prior = belief[vis_mask]
            if p_prior.size == 0:
                continue

            # --- Which visible cells truly contain a victim? ---
            visible_victim_mask = vis_mask & victim_cell_mask
            visible_victim_mask_local = victim_cell_mask[vis_mask]   # for q-aligned indexing

            # --- DETECTION PROBABILITY & EVENT (Bug #3 Fix) ---
            if np.any(visible_victim_mask):
                # True victim inside FOV → boost probability to avoid false negatives
                base_p_det = np.max(p_hit[visible_victim_mask])
                p_detect = min(1.0, base_p_det + 0.4)   # strong signal when victim is actually visible
            else:
                # No victim in FOV → only false positives possible
                p_detect = P_false

            # Binary detection event
            detection_flag = (np.random.rand() < p_detect)
            detection_event[d_idx] = detection_flag


            # Extract q = p_hit for visible cells, aligned to vis_mask
            q = p_hit[vis_mask]

            # --- BAYES UPDATE ---
            if detection_flag:
                # HIT update
                num = p_prior * q
                den = p_prior * q + (1 - p_prior) * P_false
                den = np.maximum(den, 1e-12)
                p_post = num / den

            else:
                # NO-HIT update (softened to avoid victim collapse)
                num = p_prior * (1 - q)
                den = 1 - p_prior * q
                den = np.maximum(den, 1e-12)
                p_post = num / den

                # Prevent blue collapse when victim is actually there
                p_post = np.maximum(p_post, 0.7 * p_prior)

            # Clip posterior and write back
            p_post = np.clip(p_post, 1e-6, 1 - 1e-6)
            belief[vis_mask] = p_post

        # ------------------------------------------
        # Centralized detection & tracker assignment (multi-UAV)
        # ------------------------------------------

        # Build a belief map that ignores ALL active cones
        belief_for_detection = belief.copy()

        # ---------------------------------------------------------------
        # Build detection belief that EXCLUDES all active cones
        # ---------------------------------------------------------------
        in_active_cone = active_cone_mask(
            t_now=t,
            grid_points=grid_points,
            track=track,
            v_drift=v_drift
        )

        belief_for_detection = belief.copy()
        belief_for_detection[in_active_cone] = 0.0


        # ---------------------------------------------------------------
        # Select exploration UAVs eligible for new tasks
        # ---------------------------------------------------------------
        eligible_explorers = [
            d_idx
            for d_idx, mode in enumerate(drone_modes)
            if mode == "explore" and t >= cooldown_until[d_idx]
        ]

        # ---------------------------------------------------------------
        # FOV-based physical detection trigger
        # ---------------------------------------------------------------
        NEW_DETECTION = False
        detection_uav = None
        detection_pos = None

        # Check all eligible explorers
        for d_idx in eligible_explorers:

            # Must have had a physical sensor detection this timestep
            if not detection_event[d_idx]:
                continue

            # Victim must be physically inside the FOV of this UAV
            vis_mask, _ = visible_cells_at(drone_positions[d_idx], grid_points)
            if not np.any(victim_cell_mask[vis_mask]):
                continue

            # ---- TRUE DETECTION ----
            NEW_DETECTION = True
            detection_uav = d_idx

            # Choose position = MAP cell within FOV intersection
            vis_indices = np.where(vis_mask)[0]
            local_idx = vis_indices[np.argmax(belief[vis_mask])]
            detection_pos = grid_points[local_idx].copy()

            break   # Only first real detection matters


        # ---------------------------------------------------------------
        # If no explorers OR no true real detection → skip
        # ---------------------------------------------------------------
        if (not NEW_DETECTION) or detection_uav is None:
            pass
            # IMPORTANT: do NOT fall back to belief-based detection
        else:

            # For convenience
            det_point = Point(detection_pos[0], detection_pos[1])

            # -----------------------------------------------------------
            #  Duplicate detection suppression
            # -----------------------------------------------------------
            duplicate_thresh = 100.0
            is_duplicate = False

            for tr_exist in track:
                if (not tr_exist["active"]
                    or tr_exist["pos"] is None
                    or tr_exist["time"] is None):
                    continue

                region_poly_exist, _, radius_R = tracking_region(
                    t_now=t,
                    detection_pos=tr_exist["pos"],
                    detection_time=tr_exist["time"],
                    v_drift=v_drift
                )

                if region_poly_exist is None or region_poly_exist.is_empty:
                    continue

                if region_poly_exist.buffer(10.0).contains(det_point):
                    is_duplicate = True
                    break

                d_center = np.linalg.norm(detection_pos - tr_exist["pos"])
                if d_center < max(duplicate_thresh, radius_R * 0.8):
                    is_duplicate = True
                    break

            if is_duplicate:
                print(f"[t={t}] Skipping detection at {detection_pos} — overlaps existing region.")
                pass

            else:
                # -------------------------------------------------------
                # Assign nearest explorer to track
                # -------------------------------------------------------
                print(f"[t={t}] TRUE DETECTION at {detection_pos}")

                best_d, best_cost = None, np.inf

                for d_idx in eligible_explorers:
                    r = detection_pos - drone_positions[d_idx][:2]
                    dist = np.linalg.norm(r)
                    r_hat = r / (dist + 1e-6)
                    Deff = dist + gamma_wind * abs(np.dot(v_wind[:2], r_hat))

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

                    print(f"[t={t}] Assigning UAV{best_d} to track region.")


        # ------------------------------------------
        #  Plan next velocities
        # ------------------------------------------
        for d_idx in range(num_drones):
            tr = track[d_idx]

            # --- A. EXPLORATION MODE ---
            if drone_modes[d_idx] == "explore":
                vx_des, vy_des, vz_des, *_ = plan_velocity_ipp_3D(
                    drone_positions[d_idx], 
                    drone_vels[d_idx],      # <--- Pass Velocity
                    belief, grid_points, soft_poly, v_wind_local=v_wind,
                    constraint_poly=None,   # <--- No Constraint
                    step_length=40.0,       # Large steps
                    fov_angle=theta_FOV, v_max=v_max, n_directions=16,
                    altitude_candidates=[50], pred_depth=pred_depth, E_scale=E_scale,   # Matches Table 5.1
                    lam=0.5,        
                    buffer=buffer_val
                )
                # Maintain nominal altitude
                vz_des = np.clip(uav_nominal_altitudes[d_idx] - drone_positions[d_idx][2], -1.0, 1.0)


            elif drone_modes[d_idx] == "track" and tr["active"] and not tr["confirmed"]:

                # --- NEW: ABORT IF TARGET IS GONE ---
                # Check if the belief at our target has been cleared by someone else
                if tr["pos"] is not None:
                    # Find belief value at target position
                    # (Quick nearest-neighbor check)
                    dists = np.linalg.norm(grid_points - tr["pos"], axis=1)
                    nearest_idx = np.argmin(dists)
                    
                    # If belief is effectively zero, someone else cleared it.
                    if belief[nearest_idx] < 0.05:
                        print(f"[t={t}] UAV{d_idx} target cleared by peer → Aborting track.")
                        tr["active"] = False; tr["phase"] = None
                        tr["pos"] = None; tr["time"] = None
                        drone_modes[d_idx] = "explore"
                        continue

                if tr["phase"] == "to_detection":
                    # Fix: Track the drifting target, not the static detection point
                    t_elapsed = t - tr["time"]
                    raw_target = tr["pos"] + v_drift * t_elapsed
                    
                    # --- GEOFENCE CLAMP ---
                    # If victim drifted out of bounds, target the nearest valid point on the boundary.
                    # This prevents the drone from getting stuck fighting the geofence constraint.
                    target_point = Point(raw_target[0], raw_target[1])
                    
                    if not safe_poly.contains(target_point):
                        # Find projection on the boundary
                        nearest_pt = safe_poly.exterior.interpolate(safe_poly.exterior.project(target_point))
                        target_now = np.array([nearest_pt.x, nearest_pt.y])
                    else:
                        target_now = raw_target
                    
                    vec = target_now - drone_positions[d_idx][:2]
                    dist = np.linalg.norm(vec)
                    print(f"[t={t}] UAV{d_idx} distance to moving target={dist:.2f} m")

                    # Braking logic: Slow down as we get closer
                    if dist > 40.0: 
                        dir_unit = vec / (dist + 1e-6)
                        # Linear braking: 20m/s at distance, slowing to 2m/s at 40m
                        speed = np.clip((dist - 40.0) * 0.5, 5.0, v_max)
                        vx_des, vy_des = dir_unit * speed
                        vz_des = 0.0
                    else:
                        # ARRIVED
                        tr["phase"] = "initial_lock"
                        tr["lock_start"] = t
                        print(f"[t={t}] UAV{d_idx} arrived → starting initial lock on cone.")
                        # Match drift velocity immediately to stop relative motion
                        vx_des, vy_des = v_drift[0], v_drift[1]
                        vz_des = 0.0


                elif tr["phase"] == "initial_lock":
                    det_pos = tr["pos"]
                    det_time = tr["time"]

                    # Follow cone center for a short time (stabilization)
                    region_poly, center_R, radius_R = tracking_region(
                        t_now=t,
                        detection_pos=det_pos,
                        detection_time=det_time,
                        v_drift=v_drift
                    )

                    # Move toward the drifting cone center
                    vec_center = center_R - drone_positions[d_idx][:2]
                    dist_center = np.linalg.norm(vec_center)

                    if dist_center > 5.0:
                        dir_unit = vec_center / (dist_center + 1e-6)
                        vx_des = dir_unit[0] * 5.0
                        vy_des = dir_unit[1] * 5.0
                    else:
                        vx_des = vy_des = 0.0

                    vz_des = 0.0  # hold altitude during initial lock

                    # After initial_lock_duration, switch to IPP inside cone
                    if (t - tr["lock_start"]) >= initial_lock_duration:
                        tr["phase"] = "cone_tracking"
                        tr["cone_start"] = t
                        print(f"[t={t}] UAV{d_idx} finished initial lock → starting cone IPP.")



                elif tr["phase"] == "cone_tracking":
                    det_pos = tr["pos"]
                    det_time = tr["time"]

                    # Update Cone Geometry
                    region_poly, center_R, radius_R = tracking_region(
                        t_now=t, detection_pos=det_pos, detection_time=det_time, v_drift=v_drift
                    )

                    # Validity Check (Fixes "Drift Out of Bounds" Issue)
                    valid_search_area = soft_poly.intersection(region_poly)
                    
                    if valid_search_area.is_empty:
                        print(f"[t={t}] UAV{d_idx} tracking cone drifted out of bounds. Aborting.")
                        tr["active"] = False; tr["confirmed"] = False; tr["phase"] = None
                        tr["pos"] = None; tr["time"] = None
                        drone_modes[d_idx] = "explore"
                        vx_des, vy_des, vz_des = 0.0, 0.0, 0.0
                        continue

                    # Apply IPP *inside* cone
                    if region_poly is not None and not region_poly.is_empty:
                        in_R_mask = np.array([region_poly.contains(Point(p)) for p in grid_points])
                        
                        if np.any(in_R_mask):
                            # Extract probabilities inside cone
                            p_vals = belief[in_R_mask]
                            frac_high = np.mean(p_vals > confirm_pconf)
                            mean_p = np.mean(p_vals)
                            max_p = np.max(p_vals)

                            print(f"[t={t}] UAV{d_idx} cone stats: ρ={frac_high:.3f}, mean_p={mean_p:.3f}, max_p={max_p:.3f}")

                            # --- SUCCESS: Transition to Confirmation ---
                            if (frac_high >= rho_th or mean_p >= 0.30 or max_p >= peak_tresh):
                                tr["phase"] = "hover_confirm"
                                tr["fail_timer"] = 0.0
                                
                                #  Anchor Track (Fixes "Fly Away" Bug)
                                tr["pos"] = drone_positions[d_idx][:2].copy()
                                tr["time"] = t
                                
                                print(f"[t={t}] UAV{d_idx} entering hover-confirmation (ρ={frac_high:.2f}, mean={mean_p:.2f}). Anchoring track here.")
                                continue

                            # --- ALTITUDE DESCENT LOGIC ---
                            if max_p > 0.20 and (drone_positions[d_idx][2] > MIN_ALT + 0.5):
                                current_alt = drone_positions[d_idx][2]
                                target_alt = max(MIN_ALT, current_alt - DESCENT_STEP)
                                vz_des = np.clip(target_alt - current_alt, -1.5, 0.0)
                                print(f"[t={t}] UAV{d_idx} descending for better certainty: alt={current_alt:.1f} → {target_alt:.1f}")

                            # --- FAILURE: Minimum Altitude  ---
                            z = drone_positions[d_idx][2]
                            if z <= MIN_ALT + 0.5:
                                tr["fail_timer"] += dt
                                if tr["fail_timer"] > 8.0:
                                    print(f"[t={t}] UAV{d_idx} BAD DETECTION — no victim found. Returning to IPP.")
                                    stats["aborted"] += 1
                                    # Clear hotspot
                                    if tr["pos_at_detection"] is not None:
                                        dp = tr["pos_at_detection"]
                                        # dists = np.linalg.norm(grid_points_all - dp, axis=1)
                                        dists = np.linalg.norm(grid_points - dp, axis=1)      
                                        belief[dists < 40.0] = 0.0
                                        print(f"[t={t}] Cleared false hotspot near {dp}")
                                    
                                    # Reset
                                    tr["active"] = False; tr["confirmed"] = False; tr["phase"] = None
                                    tr["pos"] = None; tr["time"] = None; tr["pos_at_detection"] = None
                                    tr["fail_timer"] = 0.0
                                    drone_modes[d_idx] = "explore"
                                    
                                    vx_des, vy_des, vz_des = 0.0, 0.0, 1.5
                                    continue

                            # --- FAILURE: Timeout ---
                            if (t - tr["cone_start"]) >= cone_search_timeout and max_p < 0.50:
                                print(f"[t={t}] UAV{d_idx} cone IPP timeout (max_p={max_p:.2f}). Abort.")
                                tr["active"] = False; tr["confirmed"] = False; tr["phase"] = None
                                tr["pos"] = None; tr["time"] = None; tr["pos_at_detection"] = None
                                tr["fail_timer"] = 0.0
                                drone_modes[d_idx] = "explore"
                                vz_des = 1.0 # Climb out
                                continue

                        else:
                            # Cone exists but no grid points inside
                            if (t - tr["cone_start"]) >= 5.0:
                                print(f"[t={t}] UAV{d_idx} empty cone (no grid points). Aborting.")
                                tr["active"] = False; tr["phase"] = None
                                drone_modes[d_idx] = "explore"
                                continue

                    else:
                        print(f"[t={t}] UAV{d_idx} lost cone geometry → abort tracking")
                        tr["active"] = False; tr["phase"] = None
                        drone_modes[d_idx] = "explore"
                        continue

                    # =============================================================
                    #  MOTION LOGIC 
                    # =============================================================
                    t_elapsed = t - det_time
                    drift_center = det_pos + v_drift * t_elapsed
                    
                    # Pass 'drone_vels[d_idx]' and 'constraint_poly'
                    # BUFFER THE CONE: Allow the drone to be 10m outside without panicking/aborting.
                    # This creates a "Soft Wall" so it curves back in smoothly.
                    soft_cone_constraint = region_poly.buffer(10.0) 
                    
                    vx_ipp, vy_ipp, vz_ipp, best_path, _, _ = plan_velocity_ipp_3D(
                        drone_positions[d_idx], 
                        drone_vels[d_idx],
                        belief, grid_points, soft_poly, v_wind_local=v_wind,
                        constraint_poly=soft_cone_constraint, # <--- USE BUFFERED POLYGON
                        step_length=20.0,
                        fov_angle=theta_FOV, v_max=v_max, 
                        altitude_candidates=[drone_positions[d_idx][2]], 
                        pred_depth=2, E_scale=E_scale_track,
                        lam=0.5,       
                        buffer=buffer_val
                    )

                    # Feedforward
                    vx_des = vx_ipp + v_drift[0] * 0.8
                    vy_des = vy_ipp + v_drift[1] * 0.8
                    
                    # Use IPP vertical velocity ONLY if your descent logic didn't set one
                    if 'vz_des' not in locals():
                        vz_des = vz_ipp


                elif tr["phase"] == "hover_confirm":
                    det_pos = tr["pos"]
                    det_time = tr["time"]

                    # --- Initialize hover timer ---
                    if "hover_start" not in tr or tr["hover_start"] is None:
                        tr["hover_start"] = t

                    # --- GET VISIBILITY FIRST ---
                    vis_mask_inside, _ = visible_cells_at(drone_positions[d_idx], grid_points)
                    
                    # Update cone region (for stats)
                    region_poly, center_R, radius_R = tracking_region(
                            t_now=t,
                            detection_pos=tr["pos"],
                            detection_time=tr["time"],
                            v_drift=v_drift
                    )

                    # --- STABILIZED TARGETING (With Gating) ---
                    hotspot_mask = (belief > confirm_pconf) & vis_mask_inside
                    
                    has_visual_lock = False
                    raw_target = None

                    # Calculate "Theoretical" position (Anchor)
                    t_elapsed = t - det_time
                    theoretical_center = det_pos + v_drift * t_elapsed

                    # Calculate Visual Candidate
                    if np.sum(hotspot_mask) > 3:
                        coords = grid_points[hotspot_mask]
                        probs = belief[hotspot_mask]
                        # Peak Chasing (Weighted Centroid)
                        weights = probs ** 2
                        visual_centroid = np.average(coords, axis=0, weights=weights)
                        
                        # --- GATING / TRUST CHECK ---
                        # Only trust visual targets within 20m of the expected location.
                        # This prevents chasing "ghosts" (old signals) far away.
                        divergence = np.linalg.norm(visual_centroid - theoretical_center)
                        
                        if divergence < 20.0:
                            has_visual_lock = True
                            raw_target = visual_centroid
                        else:
                            # print(f"[t={t}] Gating: Rejecting outlier {divergence:.1f}m away.")
                            raw_target = theoretical_center
                            has_visual_lock = False 
                    else:
                        # No signal? Fallback to drift model
                        raw_target = theoretical_center

                    #  Filter Initialization (Snap to Target)
                    if "target_smooth" not in tr or tr["target_smooth"] is None:
                        tr["target_smooth"] = raw_target  # <--- Snap immediately

                    # Apply Low-Pass Filter
                    alpha = 0.4
                    tr["target_smooth"] = alpha * tr["target_smooth"] + (1 - alpha) * raw_target
                    target_xy = tr["target_smooth"]

                    # --- VELOCITY CONTROL (Feedforward + P-Control) ---
                    err_vec = target_xy - drone_positions[d_idx][:2]
                    dist_err = np.linalg.norm(err_vec)

                    # P-Controller (Aggressive Gain)
                    if dist_err > 0.2:
                        speed = np.clip(dist_err * 1.5, 0.5, v_max) 
                        dir_unit = err_vec / (dist_err + 1e-6)
                        vx_cmd = dir_unit[0] * speed
                        vy_cmd = dir_unit[1] * speed
                    else:
                        vx_cmd, vy_cmd = 0.0, 0.0
                    
                    # Feedforward Term
                    vx_des = vx_cmd + v_drift[0]
                    vy_des = vy_cmd + v_drift[1]

                    # Vertical: Coupled Descent Logic with GRADUATED THRESHOLD
                    current_alt = drone_positions[d_idx][2]
                    
                    # High up (80m) -> Allow 10m error. Low down (20m) -> Allow 2m error.
                    allowed_err = np.interp(current_alt, [20.0, 80.0], [2.0, 10.0])
                    
                    if has_visual_lock and dist_err > allowed_err:
                        vz_des = 0.0  # Too far off, fix horizontal first
                    elif current_alt > 20.0:
                        vz_des = -1.5 # Safe to descend
                    else:
                        vz_des = 0.0  # Hold at min altitude

                    # --- CALCULATE STATS ---
                    region_mask_inside = np.array([
                        region_poly.contains(Point(p)) for p in grid_points
                    ]) if (region_poly is not None and not region_poly.is_empty) else np.zeros(len(grid_points), dtype=bool)

                    hover_mask = vis_mask_inside & region_mask_inside
                    n_cells = int(np.sum(hover_mask))

                    frac_high = 0.0
                    mean_p = 0.0
                    max_p = 0.0
                    p_vals = np.array([])

                    if n_cells > 0:
                        p_vals = belief[hover_mask]
                        frac_high = np.mean(p_vals > confirm_pconf)
                        mean_p = np.mean(p_vals)
                        max_p = np.max(p_vals)

                    # --- ABORT CONDITION (Signal Loss) ---
                    if current_alt < 70.0 and mean_p < 0.20 and max_p < 0.40:
                        print(f"[t={t}] UAV{d_idx} signal lost (mean={mean_p:.2f}, max={max_p:.2f}) → ABORT.")
                        stats["aborted"] += 1
                        tr["active"] = False; tr["confirmed"] = False; tr["phase"] = None
                        tr["pos"] = None; tr["time"] = None; tr["hover_start"] = None
                        drone_modes[d_idx] = "explore"
                        
                        nominal_alt = uav_nominal_altitudes[d_idx]
                        vz_des = np.clip(nominal_alt - current_alt, 0.5, 2.0)
                        vx_des, vy_des = 0.0, 0.0
                        continue

                    # ---  DEBUG & CONFIRM ---
                    print(
                        f"[DEBUG hover t={t}] alt={current_alt:.1f} err={dist_err:.1f}m "
                        f"n={n_cells} ρ={frac_high:.2f} mean={mean_p:.2f} "
                        f"(Thresh: ρ={rho_th}, mean={mean_thresh})"
                    )

                    # Dynamic Confirmation Ceiling
                    if mean_p >= 0.90:
                        required_confirmation_alt = 35.0
                    elif mean_p >= 0.70:
                        required_confirmation_alt = 28.0
                    else:
                        required_confirmation_alt = 22.0
                        
                    alt_ready = (current_alt <= required_confirmation_alt)

              

                    # --- Inside the hover_confirm phase of run_single_sim ---
                    if (alt_ready and n_cells > 0 and (frac_high >= rho_th or mean_p >= mean_thresh or max_p >= peak_tresh)):
                        
                        #Find ID of the closest ground-truth victim
                        dists_to_victims = np.linalg.norm(victims[:, :2] - det_pos, axis=1)
                        closest_id = np.argmin(dists_to_victims)
                        
                        # Association Check: Is the detection within the 50m FOV radius?
                        if dists_to_victims[closest_id] < 50.0:
                            if closest_id not in stats["confirmed_ids"]:
                                stats["confirmed_ids"].add(closest_id)
                                print(f"[t={t}] UAV{d_idx} CONFIRMED NEW distinct victim #{closest_id}")
                                
                                # --- MISSION SUCCESS TRICK ---
                                # Physically move the found victim far away or set its signal to zero 
                                # so other drones don't keep "detecting" it.
                                victims[closest_id] = np.array([-9999.0, -9999.0])
                            else:
                                print(f"[t={t}] UAV{d_idx} re-confirmed already found victim #{closest_id}. Ignoring for stats.")
                        else:
                            print(f"[t={t}] UAV{d_idx} False Positive: No victim within 50m of belief hotspot.")

                        # Clear the Belief Map for everyone
                        belief = clear_confirmed_region(belief, grid_points, det_pos, v_drift)
                        # ... rest of your clearing/mode reset logic ...
                        diff = grid_points - det_pos
                        mask = (diff[:, 0]**2 + diff[:, 1]**2) <= 50.0**2
                        belief[mask] *= 0.05
                        print(f"[t={t}] Fully cleared victim region.")

                        # Reset drone to exploration
                        tr["active"] = False; tr["confirmed"] = True; tr["phase"] = None
                        tr["pos"] = None; tr["time"] = None; tr["hover_start"] = None
                        drone_modes[d_idx] = "explore"

                        nominal_alt = uav_nominal_altitudes[d_idx]
                        vz_des = np.clip(nominal_alt - current_alt, -2.0, 2.0)

                    # ---  TIMEOUT ---
                    if (
                        alt_ready
                        and tr.get("hover_start") is not None 
                        and (t - tr["hover_start"]) > 10.0
                    ):
                        print(f"[t={t}] UAV{d_idx} hover-confirm timeout → aborting.")
                        stats["aborted"] += 1
                        
                        tr["active"] = False; tr["confirmed"] = False; tr["phase"] = None
                        tr["pos"] = None; tr["time"] = None; tr["hover_start"] = None
                        drone_modes[d_idx] = "explore"
                        nominal_alt = uav_nominal_altitudes[d_idx]
                        vz_des = np.clip(nominal_alt - current_alt, -2.0, 2.0)
                        vx_des, vy_des = 0.0, 0.0
                        continue



                else:
                    # Some unexpected phase; just do nothing
                    vx_des = vy_des = vz_des = 0.0

            # --- Fallback: any other mode → regular IPP ---
            else:
                vx_des, vy_des, vz_des, *_ = plan_velocity_ipp_3D(
                    drone_positions[d_idx], belief, grid_points, soft_poly, v_wind_local=v_wind,
                    fov_angle=theta_FOV, v_max=v_max, n_directions=16, step_length=40.0,
                    buffer=buffer_val, pred_depth=pred_depth, E_scale=E_scale,   # Matches Table 5.1
                    lam=0.5       
    
                )

            # --- MOTION SMOOTHING (Physics-Compliant) ---
            
            if drone_modes[d_idx] == "track":
                # ACCELERATION LIMITER
                # Instead of snapping instantly (infinite G-force), we clamp the 
                # change in velocity to max_accel. This smooths the "jumps" 
                # but allows braking/reversing without turning (Holonomic).
                
                v_current = drone_vels[d_idx][:2]
                v_target = np.array([vx_des, vy_des])
                
                # Calculate the requested jump
                delta_v = v_target - v_current
                dist_v = np.linalg.norm(delta_v)
                
                # Limit: Max 6 m/s change per second (approx 0.6G braking)

                max_accel = 6.0 
                max_change = max_accel * dt_step
                
                if dist_v > max_change:
                    delta_v = delta_v / dist_v * max_change
                    
                drone_vels[d_idx][:2] = v_current + delta_v
                drone_vels[d_idx][2] = vz_des

            else:
                # Smooth "Cinematic" movement for exploration (Legacy Fixed-Wing style)
                speed = np.linalg.norm([vx_des, vy_des])
                if speed < 1e-3:
                    vx_smooth, vy_smooth = 0.0, 0.0
                else:
                    current_heading = np.arctan2(drone_vels[d_idx][1], drone_vels[d_idx][0])
                    desired_heading = np.arctan2(vy_des, vx_des)
                    
                    delta_heading = (desired_heading - current_heading + np.pi) % (2 * np.pi) - np.pi
                    delta_heading = np.clip(delta_heading, -max_dheading * dt_step, max_dheading * dt_step)
                    
                    new_heading = current_heading + delta_heading
                    vx_smooth = speed * np.cos(new_heading)
                    vy_smooth = speed * np.sin(new_heading)
                    
                drone_vels[d_idx][:2] = [vx_smooth, vy_smooth]
                drone_vels[d_idx][2] = vz_des


        # ------------------------------------------
        # 7. Boundary correction (Wind-Aware)
        # ------------------------------------------
        for d_idx in range(num_drones):
            next_pos = drone_positions[d_idx] + drone_vels[d_idx] * dt_step
            point_next = Point(next_pos[0], next_pos[1])
            
            if not safe_poly.contains(point_next):
                nearest = np.array(safe_poly.exterior.interpolate(
                    safe_poly.exterior.project(point_next)
                ).coords[0])
                
                direction_vec = nearest - drone_positions[d_idx][:2]
                norm = np.linalg.norm(direction_vec)
                
                if norm > 1e-3:
                    unit_dir = direction_vec / norm
                    
                    # --- FIX: Calculate max safe ground speed for this direction ---
                    # Component of wind in the correction direction (negative = headwind)
                    v_headwind_comp = np.dot(v_wind[:2], unit_dir)
                    
                    # Ensure Airspeed <= 20m/s
                    v_g_allowed = 20.0 + v_headwind_comp 
                    
                    # Ground speed cannot exceed the lower of (physical limit) or (wind safety)
                    speed_limit = max(2.0, min(20.0, v_g_allowed)) 
                    
                    vx_corr, vy_corr = unit_dir * min(norm / dt_step, speed_limit)
                else:
                    vx_corr, vy_corr = 0.0, 0.0
                    
                drone_vels[d_idx][:2] = [vx_corr, vy_corr]


        # ------------------------------------------
        # FINAL Safety Clamp
        # ------------------------------------------
        for d_idx in range(num_drones):
            v_g = drone_vels[d_idx][:2]
            v_g_mag = np.linalg.norm(v_g)
            
            if v_g_mag > 1e-3:
                unit_vg = v_g / v_g_mag
                # Component of wind against our intended ground direction0
            
                v_headwind_comp = np.dot(v_wind[:2], unit_vg)
                
                # Max ground speed allowed to keep Airspeed <= 20
                v_g_limit = 20.0 + v_headwind_comp
                
                if v_g_mag > v_g_limit:
                    # Force ground speed to respect the 20m/s airspeed limit
                    drone_vels[d_idx][:2] = unit_vg * max(2.0, v_g_limit)


        # ------------------------------------------
        # Advance UAV positions
        # ------------------------------------------
        for d_idx in range(num_drones):
            drone_positions[d_idx] += drone_vels[d_idx] * dt_step


        if len(stats["confirmed_ids"]) >= num_victims:
            print(f"Mission Success: All {num_victims} DISTINCT victims confirmed.")
            break


        # ==========================================
        # ENERGY CONSUMPTION UPDATE 
        # ==========================================

        battery_capacity = 18000.0 * 60 
        E_scale = E_max / battery_capacity

        for d_idx in range(num_drones):
            #  Get Airspeed Kinematics
            vx, vy, vz = drone_vels[d_idx]
            v_air_vec = np.array([vx, vy]) - v_wind[:2]
            v_air_magnitude = np.linalg.norm(v_air_vec)

            # Base Power Draw from Airspeed (Regime-based)
            P_inst = get_vsqp_power(v_air_magnitude)

            #  Vertical Penalty (Climbing cost vs. Descent savings)
            if vz > 0.1:
                P_inst += 1000.0 * vz  # ~1000W per m/s climb
            elif vz < -0.1:
                P_inst -= 200.0 * abs(vz) # Savings for gravity-assisted descent

            #  Tracking Mode Hover Penalty
            if drone_modes[d_idx] == "track" and track[d_idx]["phase"] == "hover_confirm":
                # Ensure at least Hover Power (1751W) + safety margin while stationary
                P_inst = max(P_inst, 1751.0 * 1.1)

            #  Electronics Floor
            P_inst = max(100.0, P_inst)

            #  Update Total Cumulative Joules ---
            energy_consumed_joules[d_idx] += P_inst * dt_step 

            # --- Update Remaining Battery Percentage ---
            E_rem[d_idx] -= P_inst * dt_step * E_scale
            E_rem[d_idx] = max(0, E_rem[d_idx])

            # --- Log to history (for JSON) ---
            energy_history[d_idx] = np.append(energy_history[d_idx], E_rem[d_idx])

        energy_time.append(t)


    # --- 3. Finalize Metrics ---
    final_entropy = np.sum(cell_entropy_map(belief))
    entropy_reduction_rate = (initial_entropy - final_entropy) / (t + 1)
    
    # --- At the end of run_single_sim ---
    return {
        "seed": seed,
        "num_victims": num_victims,
        "wind_speed": wind_mag,
        "time_elapsed": t,
        "energy_per_drone": energy_consumed_joules.tolist(),
        "entropy_reduction_rate": entropy_reduction_rate,
        "abort_count": stats["aborted"],
        "confirmed_count": len(stats["confirmed_ids"]),
        
        "victim_start_pos": victims_pos.tolist(), 

        "drone_start_pos": [p.tolist() for p in drone_start_coords] 
    }



if __name__ == "__main__":
    import time
    import json
    import os

    # --- Directory and File Setup ---
    results_dir = "campaign_b_results"
    logs_dir = os.path.join(results_dir, "detailed_json_logs")
    os.makedirs(logs_dir, exist_ok=True)
    summary_csv = os.path.join(results_dir, "robustness_summary_log.csv")
    
    # --- Experimental Configuration  ---
    severities = {
        "Low": (2.0, 0.06),
        "Medium": (5.0, 0.18),
        "High": (10.0, 0.30)
    }
    victim_counts = range(1, 11) # 1 to 10 victims
    trials = 25                  # 25 seeds per configuration
    
    for sev_name, settings in severities.items():
        print(f"\n>>> Starting Severity: {sev_name} (Wind: {settings[0]}m/s) <<<")
        for n_vics in victim_counts:
            for trial in range(trials):
                start_wall = time.time()
                
                # Execute simulation seed
                # Ensure run_single_sim returns 'victim_start_pos' and 'drone_start_pos'
                data = run_single_sim(num_drones=3, num_victims=n_vics, severity_settings=settings, seed=trial)
                
                # --- Append to Summary CSV (Trial-by-Trial) ---
                # We save high-level metrics here for easy plotting
                df_row = pd.DataFrame([{
                    "severity": sev_name,
                    "num_victims": n_vics,
                    "trial_seed": trial,
                    "wind_speed": settings[0],
                    "drift_speed": settings[1],
                    "sim_time": data["time_elapsed"],
                    "confirmed": data["confirmed_count"],
                    "aborted": data["abort_count"],
                    "entropy_reduction": data["entropy_reduction_rate"],
                    "avg_energy_consumed": np.mean(data["energy_per_drone"]),
                    "wall_time": time.time() - start_wall
                }])
                
                # Append row to CSV
                df_row.to_csv(summary_csv, mode='a', index=False, header=not os.path.exists(summary_csv))
                
                # --- Save Detailed JSON Log ---
                # We save specific coordinates here to keep the CSV clean
                log_file = os.path.join(logs_dir, f"log_{sev_name}_v{n_vics}_t{trial}.json")
                with open(log_file, 'w') as f:
                    json.dump({
                        "config": {
                            "severity": sev_name, 
                            "victims": n_vics, 
                            "seed": trial,
                            "wind_vec": settings[0],
                            "drift_vec": settings[1]
                        },
                        "results": {
                            "total_sim_time": data["time_elapsed"],
                            "energy_per_uav": data["energy_per_drone"], # Joules
                            "final_confirmed": data["confirmed_count"],
                            "final_aborted": data["abort_count"]
                        },
                        "initial_setup": {
                            "victim_start_positions": data["victim_start_pos"], # Saved as list
                            "uav_start_positions": data["drone_start_pos"]    # Saved as list
                        }
                    }, f, indent=4)
                
                print(f"[{sev_name}] V:{n_vics} | Trial:{trial} | Done: {data['time_elapsed']}s | Confirmed: {data['confirmed_count']}/{n_vics}")

    print(f"\nCampaign B Complete. Results saved in: {results_dir}")

