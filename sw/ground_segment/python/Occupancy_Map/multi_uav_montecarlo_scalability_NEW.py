import numpy as np
import pandas as pd
import time
import os
import json
from shapely.geometry import Polygon, Point
from numba import jit
from matplotlib.path import Path as MplPath

# ==============================================================================
# 1. CORE NUMBA FUNCTIONS (High-Speed Engine)
# ==============================================================================

@jit(nopython=True)
def get_p_hit_numba(h, h_ref=20.0, P0=0.9):
    if h < 1e-3: h = 1e-3
    f_res = 1.0
    if h > h_ref:
        f_res = h_ref / h 
    return P0 * f_res

@jit(nopython=True)
def cell_entropy_numba(p):
    if p < 1e-9: p = 1e-9
    elif p > (1.0 - 1e-9): p = 1.0 - 1e-9
    return -p * np.log2(p) - (1 - p) * np.log2(1 - p)

@jit(nopython=True)
def calc_ig_numba(path_arr, belief_map, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols):
    """
    Fast IG calculator. Returns total Information Gain (float).
    Does NOT modify the map.
    """
    total_IG = 0.0
    pred_belief = belief_map.copy() # Working copy for this path calculation
    
    steps = len(path_arr) - 1
    for i in range(steps):
        p0 = path_arr[i]
        p1 = path_arr[i+1]
        dist = np.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2)
        n_samples = int(np.ceil(dist / 10.0))
        if n_samples < 1: n_samples = 1
        
        dx = (p1[0] - p0[0]) / n_samples
        dy = (p1[1] - p0[1]) / n_samples
        dz = (p1[2] - p0[2]) / n_samples
        
        for k in range(n_samples):
            px = p0[0] + k * dx
            py = p0[1] + k * dy
            pz = p0[2] + k * dz
            
            r_m = pz * fov_tan
            r_cells = int(np.ceil(r_m / grid_res))
            r_sq = r_m * r_m
            
            c_center = int((px - x_min) / grid_res + 0.5)
            r_center = int((py - y_min) / grid_res + 0.5)
            
            r_start = max(0, r_center - r_cells)
            r_end   = min(rows, r_center + r_cells + 1)
            c_start = max(0, c_center - r_cells)
            c_end   = min(cols, c_center + r_cells + 1)
            
            q = get_p_hit_numba(pz)
            
            for r in range(r_start, r_end):
                for c in range(c_start, c_end):
                    if not inside_mask[r, c]: continue
                    
                    dr = (r - r_center) * grid_res
                    dc = (c - c_center) * grid_res
                    if dr*dr + dc*dc <= r_sq:
                        p = pred_belief[r, c]
                        if p < 1e-6 or p > (1.0 - 1e-6): continue
                        
                        # Entropy Before
                        H_prior = cell_entropy_numba(p)
                        
                        # Bayes Update (No Detection)
                        num = p * (1.0 - q)
                        den = 1.0 - p * q
                        if den < 1e-12: den = 1e-12
                        p_post = num / den
                        
                        # Entropy After
                        H_post = cell_entropy_numba(p_post)
                        
                        total_IG += (H_prior - H_post)
                        pred_belief[r, c] = p_post
    return total_IG

@jit(nopython=True)
def mark_path_visited(path_arr, belief_map, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols):
    """
    'Paints' the path onto the scratchpad map with a low value (0.001).
    This tells the next drone: "I am going here, don't look here."
    """
    steps = len(path_arr) - 1
    for i in range(steps):
        p0 = path_arr[i]
        p1 = path_arr[i+1]
        dist = np.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2)
        n_samples = int(np.ceil(dist / 10.0))
        if n_samples < 1: n_samples = 1
        
        dx = (p1[0] - p0[0]) / n_samples
        dy = (p1[1] - p0[1]) / n_samples
        dz = (p1[2] - p0[2]) / n_samples
        
        for k in range(n_samples):
            px = p0[0] + k * dx
            py = p0[1] + k * dy
            pz = p0[2] + k * dz
            
            r_m = pz * fov_tan
            r_cells = int(np.ceil(r_m / grid_res))
            r_sq = r_m * r_m
            
            c_center = int((px - x_min) / grid_res + 0.5)
            r_center = int((py - y_min) / grid_res + 0.5)
            
            r_start = max(0, r_center - r_cells)
            r_end   = min(rows, r_center + r_cells + 1)
            c_start = max(0, c_center - r_cells)
            c_end   = min(cols, c_center + r_cells + 1)
            
            for r in range(r_start, r_end):
                for c in range(c_start, c_end):
                    if not inside_mask[r, c]: continue
                    dr = (r - r_center) * grid_res
                    dc = (c - c_center) * grid_res
                    if dr*dr + dc*dc <= r_sq:
                        belief_map[r, c] = 0.001 # Reserve this cell

# ==============================================================================
# 2. PYTHON HELPERS
# ==============================================================================

def get_vsqp_power(v):
    if v < 12.0: return 1751.0
    elif 12.0 <= v < 17.0: return 154.5 * v - 104.6
    else: return 323.0

def energy_of_path_py(path, v_wind):
    total_energy = 0.0
    P_climb = 1751.0
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i+1]
        vec = p1 - p0
        dist = np.linalg.norm(vec[:2])
        dz = vec[2]
        
        if dist > 1e-3:
            unit_vec = vec[:2] / dist
            v_headwind = np.dot(v_wind[:2], unit_vec)
            v_ground = max(2.0, min(20.0, 20.0 + v_headwind))
            t_seg = dist / v_ground
            
            v_air_vec = (unit_vec * v_ground) - v_wind[:2]
            v_air = np.linalg.norm(v_air_vec)
            total_energy += get_vsqp_power(v_air) * t_seg
        
        if dz > 0: total_energy += (dz / 2.0) * P_climb
        elif dz < 0: total_energy += (-dz / 3.0) * P_climb * 0.5
    return total_energy

# ==============================================================================
# 3. PLANNER WRAPPER
# ==============================================================================

def plan_velocity_ipp_3D_numba(drone_pos, drone_vel, belief_map, inside_mask, origin, 
                               grid_res, soft_poly, v_wind, fov_angle=np.deg2rad(45),
                               step_length=40.0, altitude_candidates=[50], 
                               E_scale=100.0, lam=0.5):
    
    x_min, y_min = origin
    rows, cols = belief_map.shape
    fov_tan = np.tan(fov_angle)
    
    cx, cy, cz = drone_pos
    vx, vy, _ = drone_vel
    
    # 1. Generate Candidates
    current_speed = np.linalg.norm([vx, vy])
    if current_speed > 2.0:
        heading = np.arctan2(vy, vx)
        angles = heading + np.linspace(-np.pi/3, np.pi/3, 12)
    else:
        angles = np.linspace(0, 2*np.pi, 12, endpoint=False)
        
    candidates = []
    for a in angles:
        dx, dy = np.cos(a), np.sin(a)
        for alt in altitude_candidates:
            traj = [np.array([cx, cy, cz])]
            valid = True
            for k in range(3): # Pred depth 3
                prev = traj[-1]
                nx = prev[0] + dx * step_length
                ny = prev[1] + dy * step_length
                if not soft_poly.contains(Point(nx, ny)):
                    valid = False; break
                traj.append(np.array([nx, ny, alt]))
            if valid: candidates.append(traj)
            
    if not candidates:
        return 0.0, 0.0, 0.0, None, -np.inf

    # 2. Evaluate using Numba
    best_J = -np.inf
    best_path = None
    
    for path in candidates:
        path_arr = np.array(path, dtype=np.float64)
        
        # Numba IG
        I_p = calc_ig_numba(path_arr, belief_map, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols)
        
        # Energy
        E_p = energy_of_path_py(path, v_wind)
        
        J = I_p - (lam * (E_p / E_scale))
        if J > best_J:
            best_J = J
            best_path = path_arr # Keep as numpy array for reservation

    if best_path is None:
        return 0.0, 0.0, 0.0, None, -np.inf

    # 3. Calculate Command
    p0, p1 = best_path[0], best_path[1]
    vec = p1 - p0
    dist = np.linalg.norm(vec[:2])
    
    unit_vec = vec[:2] / (dist+1e-6)
    v_headwind = np.dot(v_wind[:2], unit_vec)
    v_ground = max(2.0, min(20.0, 20.0 + v_headwind))
    
    t_travel = max(dist / v_ground, 0.1)
    
    return vec[0]/t_travel, vec[1]/t_travel, vec[2]/t_travel, best_path, best_J

# ==============================================================================
# 4. SIMULATION LOOP (FIXED STARBURST & INIT)
# ==============================================================================

# def run_scalability_numba(map_side_m, num_drones, trial_id):
#     # Setup Map
#     half = map_side_m / 2.0
#     soft_poly = Polygon([(-half, -half), (half, -half), (half, half), (-half, half)])
    
#     grid_res = max(20.0, map_side_m / 200.0) 
#     x_min, y_min, x_max, y_max = soft_poly.bounds
    
#     grid_x = np.arange(x_min, x_max + grid_res, grid_res)
#     grid_y = np.arange(y_min, y_max + grid_res, grid_res)
#     rows, cols = len(grid_y), len(grid_x)
    
#     XX, YY = np.meshgrid(grid_x, grid_y)
    
#     # Generate Masks
#     poly_path = MplPath(list(soft_poly.exterior.coords))
#     points_flat = np.vstack((XX.flatten(), YY.flatten())).T
#     mask_flat = poly_path.contains_points(points_flat)
#     inside_mask = mask_flat.reshape(rows, cols)
    
#     # Init Belief
#     belief_map = 0.5 * np.ones((rows, cols))
#     belief_map[~inside_mask] = 0.5 # Ignore outside
    
#     # Calculate initial entropy
#     p_flat = belief_map[inside_mask]
#     h_start = np.sum(-p_flat * np.log2(p_flat) - (1-p_flat)*np.log2(1-p_flat))
    
#     # Init Drones (Center Launch at 0,0 with small jitter)
#     drone_positions = [np.array([np.random.uniform(-5, 5), np.random.uniform(-5, 5), 50.0]) for _ in range(num_drones)]
#     drone_vels = [np.zeros(3) for _ in range(num_drones)]
#     E_rem = np.ones(num_drones) * 300.0 * 3600 # Joules
    
#     v_wind = np.array([0.0, 0.0])
#     fov_tan = np.tan(np.deg2rad(45))
    
#     max_t = (map_side_m / 20.0) * 10.0
#     if max_t > 3600: max_t = 3600
    
#     t_sim = 0
#     dt = 1.0
#     entropy_history = []
    
#     while t_sim < max_t:
#         # 1. Stats
#         p_flat = belief_map[inside_mask]
#         current_h = np.sum(-p_flat * np.log2(p_flat) - (1-p_flat)*np.log2(1-p_flat))
#         reduction = (1.0 - (current_h / h_start))
#         entropy_history.append(reduction)
        
#         if reduction >= 0.90:
#             return {"status": "SUCCESS", "time": t_sim, "reduction": reduction, "energy": E_rem.tolist()}

#         # 2. Sequential Planning with Reservation
#         planning_belief = belief_map.copy()
#         active_searchers = 0
        
#         for i in range(num_drones):
#             if E_rem[i] <= (300.0*3600 * 0.2):
#                 vec = np.array([0.0,0.0,50.0]) - drone_positions[i]
#                 d = np.linalg.norm(vec)
#                 drone_vels[i] = (vec/(d+1e-6)) * 20.0 if d > 5.0 else np.zeros(3)
#                 continue
            
#             active_searchers += 1
            
#             # --- STARBURST DURATION (5s) ---
#             # 5s * 20m/s = 100m (Safe for 500m map)
#             if t_sim < 5.0:
#                 angle = (2 * np.pi / num_drones) * i
#                 vx = 20.0 * np.cos(angle)
#                 vy = 20.0 * np.sin(angle)
#                 drone_vels[i] = np.array([vx, vy, 0.0])
#                 continue
                
#             # B. Sequential IPP (using Numba Wrapper)
#             vx, vy, vz, best_path, _ = plan_velocity_ipp_3D_numba(
#                 drone_positions[i], drone_vels[i], planning_belief, inside_mask,
#                 (x_min, y_min), grid_res, soft_poly, v_wind
#             )
            
#             # Reservation Step (Numba)
#             if best_path is not None:
#                 mark_path_visited(best_path, planning_belief, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols)
            
#             # Smooth Physics
#             # v_curr = drone_vels[i][:2]
#             # v_targ = np.array([vx, vy])
#             # delta = v_targ - v_curr
#             # if np.linalg.norm(delta) > 2.0:
#             #     delta = delta / np.linalg.norm(delta) * 2.0
            
#             # drone_vels[i] = np.array([v_curr[0]+delta[0], v_curr[1]+delta[1], vz])

#             # --- INSERT THIS BLOCK ---
#             # Realistic Banked Turn (Maintains Speed)
#             v_curr = drone_vels[i][:2]
#             current_speed = np.linalg.norm(v_curr)
            
#             desired_speed = np.linalg.norm([vx, vy])
#             desired_heading = np.arctan2(vy, vx)
            
#             if current_speed < 1.0:
#                 # Linear accel from stop
#                 delta = np.array([vx, vy]) - v_curr
#                 if np.linalg.norm(delta) > 5.0: 
#                     delta = delta / np.linalg.norm(delta) * 5.0
#                 drone_vels[i] = np.array([v_curr[0]+delta[0], v_curr[1]+delta[1], vz])
#             else:
#                 # 1. Heading Update (Turn Rate Limit)
#                 current_heading = np.arctan2(v_curr[1], v_curr[0])
#                 angle_diff = (desired_heading - current_heading + np.pi) % (2*np.pi) - np.pi
                
#                 # Max 15 degrees per second turn rate
#                 max_turn = np.deg2rad(15.0) * dt
#                 clamped_diff = np.clip(angle_diff, -max_turn, max_turn)
#                 new_heading = current_heading + clamped_diff
                
#                 # 2. Speed Update (Longitudinal Accel)
#                 speed_diff = desired_speed - current_speed
#                 clamped_accel = np.clip(speed_diff, -2.0 * dt, 2.0 * dt)
#                 new_speed = current_speed + clamped_accel
                
#                 # 3. Combine
#                 vx_new = new_speed * np.cos(new_heading)
#                 vy_new = new_speed * np.sin(new_heading)
                
#                 drone_vels[i] = np.array([vx_new, vy_new, vz])

#         # 3. Execution (Update Real Map)
#         for i in range(num_drones):
#             drone_positions[i] += drone_vels[i] * dt
            
#             # Energy
#             v_air = np.linalg.norm(drone_vels[i][:2] - v_wind[:2])
#             E_rem[i] -= get_vsqp_power(v_air) * dt
            
#             # Sensor Update
#             pos = drone_positions[i]
#             r_m = pos[2] * fov_tan
#             r_cells = int(np.ceil(r_m / grid_res))
#             c_c = int((pos[0] - x_min)/grid_res + 0.5)
#             r_c = int((pos[1] - y_min)/grid_res + 0.5)
            
#             r_s = max(0, r_c - r_cells); r_e = min(rows, r_c + r_cells + 1)
#             c_s = max(0, c_c - r_cells); c_e = min(cols, c_c + r_cells + 1)
            
#             r_sq = r_m**2
            
#             if r_e > r_s and c_e > c_s:
#                 Y, X = np.ogrid[r_s:r_e, c_s:c_e]
#                 dist_sq = ((Y - r_c)*grid_res)**2 + ((X - c_c)*grid_res)**2
#                 mask = (dist_sq <= r_sq) & inside_mask[r_s:r_e, c_s:c_e]
                
#                 # Bayesian Update (No Hit)
#                 q = get_p_hit_numba(pos[2])
#                 sub_belief = belief_map[r_s:r_e, c_s:c_e]
                
#                 # Vectorized update
#                 p = sub_belief[mask]
#                 p_new = (p * (1.0-q)) / (1.0 - p*q)
#                 sub_belief[mask] = p_new
#                 belief_map[r_s:r_e, c_s:c_e] = sub_belief

#         if active_searchers == 0:
#             return {"status": "BATTERY_EXHAUSTED", "time": t_sim, "reduction": reduction, "energy": E_rem.tolist()}
            
#         t_sim += dt

#     return {"status": "TIMEOUT", "time": max_t, "reduction": reduction, "energy": E_rem.tolist()}

# # ==============================================================================
# # 5. MAIN
# # ==============================================================================
# if __name__ == "__main__":
#     results_dir = "mci_results_scalability_numba_now_good"
#     os.makedirs(results_dir, exist_ok=True)
#     summary_csv = os.path.join(results_dir, "scalability_summary_numba.csv")
    
#     scales = [10000]
#     # swarm_sizes = [1, 3, 5, 7, 10, 15, 25, 35, 50, 75, 100]
#     swarm_sizes = [100, 125, 150]
#     trials = 2 
    
#     print("Starting Corrected Numba Scalability Test (5s Burst)...")
    
#     for s in scales:
#         for n in swarm_sizes:
#             for trial in range(trials):
#                 start = time.time()
#                 data = run_scalability_numba(s, n, trial)
#                 dur = time.time() - start
                
#                 df = pd.DataFrame([{
#                     "map_size": s, "swarm_size": n, "trial": trial,
#                     "status": data["status"], "sim_time": data["time"],
#                     "reduction": data["reduction"], "wall_time": dur
#                 }])
#                 df.to_csv(summary_csv, mode='a', header=not os.path.exists(summary_csv), index=False)
                
#                 print(f"Map: {s}m | N: {n} | {data['status']} in {data['time']:.1f}s (Real: {dur:.2f}s)")



def run_scalability_numba(map_side_m, num_drones, trial_id):
    # ... [KEEP THE SETUP CODE EXACTLY AS BEFORE: Polygon, Grid, Belief, positions, etc.] ...
    half = map_side_m / 2.0
    soft_poly = Polygon([(-half, -half), (half, -half), (half, half), (-half, half)])
    
    grid_res = max(20.0, map_side_m / 200.0) 
    x_min, y_min, x_max, y_max = soft_poly.bounds
    grid_x = np.arange(x_min, x_max + grid_res, grid_res)
    grid_y = np.arange(y_min, y_max + grid_res, grid_res)
    rows, cols = len(grid_y), len(grid_x)
    XX, YY = np.meshgrid(grid_x, grid_y)
    
    poly_path = MplPath(list(soft_poly.exterior.coords))
    points_flat = np.vstack((XX.flatten(), YY.flatten())).T
    mask_flat = poly_path.contains_points(points_flat)
    inside_mask = mask_flat.reshape(rows, cols)
    
    belief_map = 0.5 * np.ones((rows, cols))
    belief_map[~inside_mask] = 0.5 
    p_flat = belief_map[inside_mask]
    h_start = np.sum(-p_flat * np.log2(p_flat) - (1-p_flat)*np.log2(1-p_flat))
    
    drone_positions = [np.array([np.random.uniform(-5, 5), np.random.uniform(-5, 5), 50.0]) for _ in range(num_drones)]
    drone_vels = [np.zeros(3) for _ in range(num_drones)]
    E_rem = np.ones(num_drones) * 300.0 * 3600 
    
    v_wind = np.array([0.0, 0.0]) 
    fov_tan = np.tan(np.deg2rad(45))
    
    max_t = (map_side_m / 20.0) * 10.0
    if max_t > 3600: max_t = 3600
    
    t_sim = 0
    dt = 1.0
    entropy_history = [] # We will return this!
    
    # ... [KEEP THE WHILE LOOP START] ...
    while t_sim < max_t:
        p_flat = belief_map[inside_mask]
        current_h = np.sum(-p_flat * np.log2(p_flat) - (1-p_flat)*np.log2(1-p_flat))
        reduction = (1.0 - (current_h / h_start))
        
        # RECORD HISTORY
        entropy_history.append(float(reduction))
        
        if reduction >= 0.90:
            return {
                "status": "SUCCESS", 
                "time": t_sim, 
                "reduction": reduction, 
                "energy": E_rem.tolist(),
                "curve": entropy_history  # <--- NEW RETURN
            }

        # ... [KEEP PLANNING & PHYSICS LOGIC EXACTLY AS BEFORE] ...
        # (Copy/Paste the block with plan_velocity_ipp_3D_numba and the Banking Physics)
        
        planning_belief = belief_map.copy()
        active_searchers = 0
        
        for i in range(num_drones):
            if E_rem[i] <= (300.0*3600 * 0.2):
                vec = np.array([0.0,0.0,50.0]) - drone_positions[i]
                d = np.linalg.norm(vec)
                drone_vels[i] = (vec/(d+1e-6)) * 20.0 if d > 5.0 else np.zeros(3)
                continue
            
            active_searchers += 1
            
            # Starburst (20 m/s)
            if t_sim < 10.0:
                angle = (2 * np.pi / num_drones) * i
                vx = 20.0 * np.cos(angle)
                vy = 20.0 * np.sin(angle)
                drone_vels[i] = np.array([vx, vy, 0.0])
                continue
                
            vx, vy, vz, best_path, _ = plan_velocity_ipp_3D_numba(
                drone_positions[i], drone_vels[i], planning_belief, inside_mask,
                (x_min, y_min), grid_res, soft_poly, v_wind
            )
            
            if best_path is not None:
                mark_path_visited(best_path, planning_belief, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols)
            
            # Banking Physics
            v_curr = drone_vels[i][:2]
            current_speed = np.linalg.norm(v_curr)
            desired_speed = np.linalg.norm([vx, vy])
            desired_heading = np.arctan2(vy, vx)
            
            if current_speed < 1.0:
                delta = np.array([vx, vy]) - v_curr
                if np.linalg.norm(delta) > 5.0: delta = delta / np.linalg.norm(delta) * 5.0
                drone_vels[i] = np.array([v_curr[0]+delta[0], v_curr[1]+delta[1], vz])
            else:
                current_heading = np.arctan2(v_curr[1], v_curr[0])
                angle_diff = (desired_heading - current_heading + np.pi) % (2*np.pi) - np.pi
                max_turn = np.deg2rad(15.0) * dt
                clamped_diff = np.clip(angle_diff, -max_turn, max_turn)
                new_heading = current_heading + clamped_diff
                
                speed_diff = desired_speed - current_speed
                clamped_accel = np.clip(speed_diff, -2.0 * dt, 2.0 * dt)
                new_speed = current_speed + clamped_accel
                
                drone_vels[i] = np.array([new_speed * np.cos(new_heading), new_speed * np.sin(new_heading), vz])

        # ... [KEEP UPDATE LOOP EXACTLY AS BEFORE] ...
        for i in range(num_drones):
            drone_positions[i] += drone_vels[i] * dt
            v_air = np.linalg.norm(drone_vels[i][:2] - v_wind[:2])
            E_rem[i] -= get_vsqp_power(v_air) * dt
            
            # Sensor Update
            pos = drone_positions[i]
            r_m = pos[2] * fov_tan
            r_cells = int(np.ceil(r_m / grid_res))
            c_c = int((pos[0] - x_min)/grid_res + 0.5)
            r_c = int((pos[1] - y_min)/grid_res + 0.5)
            r_s = max(0, r_c - r_cells); r_e = min(rows, r_c + r_cells + 1)
            c_s = max(0, c_c - r_cells); c_e = min(cols, c_c + r_cells + 1)
            r_sq = r_m**2
            
            if r_e > r_s and c_e > c_s:
                Y, X = np.ogrid[r_s:r_e, c_s:c_e]
                dist_sq = ((Y - r_c)*grid_res)**2 + ((X - c_c)*grid_res)**2
                mask = (dist_sq <= r_sq) & inside_mask[r_s:r_e, c_s:c_e]
                q = get_p_hit_numba(pos[2])
                sub_belief = belief_map[r_s:r_e, c_s:c_e]
                p = sub_belief[mask]
                p_new = (p * (1.0-q)) / (1.0 - p*q)
                sub_belief[mask] = p_new
                belief_map[r_s:r_e, c_s:c_e] = sub_belief

        if active_searchers == 0:
            return {
                "status": "BATTERY_EXHAUSTED", 
                "time": t_sim, 
                "reduction": reduction, 
                "energy": E_rem.tolist(),
                "curve": entropy_history # <--- NEW RETURN
            }
            
        t_sim += dt

    return {
        "status": "TIMEOUT", 
        "time": max_t, 
        "reduction": reduction, 
        "energy": E_rem.tolist(),
        "curve": entropy_history # <--- NEW RETURN
    }

if __name__ == "__main__":
    results_dir = "mci_results_curves"
    os.makedirs(results_dir, exist_ok=True)
    
    # We save curves to a separate JSON because CSVs are bad for arrays
    curve_file = os.path.join(results_dir, "scalability_curves.json")
    
    scales = [500, 2000, 5000, 10000]
    swarm_sizes = [1, 3, 5, 7, 10, 15, 25, 35, 50, 75, 100, 125, 150, 175]
    trials = 3  # Reduced trials for curves (files get big!)
    
    # Data Structure: {map_size: {swarm_size: [ [curve_run1], [curve_run2] ] }}
    all_curves = {}

    print("Starting Scalability Test (With Curve Logging)...")
    
    for s in scales:
        all_curves[s] = {}
        for n in swarm_sizes:
            # Smart Skip: Don't run tiny swarms on massive maps if you want to save time
            if s == 10000 and n < 5: continue 
            
            all_curves[s][n] = []
            
            for trial in range(trials):
                start = time.time()
                data = run_scalability_numba(s, n, trial)
                dur = time.time() - start
                
                # Save the curve
                all_curves[s][n].append(data["curve"])
                
                print(f"Map: {s}m | N: {n} | {data['status']} in {data['time']:.1f}s")
            
            # Save incrementally in case of crash
            with open(curve_file, 'w') as f:
                json.dump(all_curves, f)

    print(f"Done! Curves saved to {curve_file}")