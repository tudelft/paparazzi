import numpy as np
import matplotlib.pyplot as plt
import random
import os
import sys
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, Point
import pymap3d as pm
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter
from shapely.affinity import rotate, translate
import time
import pandas as pd
import numpy as np
import pandas as pd
from shapely.geometry import Polygon, Point
import time
import os
import json
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter

# -----------------------------
# 1. CORE HELPER FUNCTIONS
# -----------------------------

def cell_entropy_map(belief):
    p = np.clip(belief, 1e-9, 1 - 1e-9)
    return -p * np.log2(p) - (1 - p) * np.log2(1 - p)

def get_global_entropy(b):
    return np.sum(cell_entropy_map(b))

def get_vsqp_power(v):
    """Empirical power model: Hover=1751W, Cruise=323W."""
    if v < 12.0: return 1751.0
    elif 12.0 <= v < 17.0: return 154.5 * v - 104.6
    else: return 323.0

def f_res(h, h_ref=20.0):
    """Sensor resolution degradation with altitude."""
    return min(1.0, (h_ref / max(h, 1e-3)))

def P_hit(h, P0=0.9):
    return P0 * f_res(h)

def visible_cells_at(pos_xyz, grid_points, fov_angle=np.deg2rad(45)):
    x, y, h = pos_xyz
    r = h * np.tan(fov_angle)
    dx = grid_points[:, 0] - x
    dy = grid_points[:, 1] - y
    mask = (dx*dx + dy*dy) <= r*r
    p_hit = np.zeros(len(grid_points))
    if np.any(mask):
        p_hit[mask] = P_hit(h)
    return mask, p_hit

def energy_of_path(path):
    cruise_speed = 20.0 
    P_cruise = get_vsqp_power(cruise_speed) 
    P_climb = 1751.0
    total_energy = 0.0
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i+1]
        dist_xy = np.linalg.norm(p1[:2] - p0[:2])
        dz = p1[2] - p0[2]
        total_energy += (dist_xy / cruise_speed) * P_cruise
        if dz > 0: total_energy += (dz / 2.0) * P_climb
        elif dz < 0: total_energy += (-dz / 3.0) * P_climb * 0.5
    return total_energy

def expected_info_gain(path, belief, grid_points, fov_angle, pred_depth=3):
    pred_belief = belief.copy()
    total_IG = 0.0
    for i in range(min(len(path)-1, pred_depth)):
        p0, p1 = path[i], path[i+1]
        pos = (p0 + p1) / 2.0 
        mask, p_hit = visible_cells_at(pos, grid_points, fov_angle)
        if not np.any(mask): continue
        p_prior = pred_belief[mask]
        q = p_hit[mask]
        p_post = (p_prior * (1.0 - q)) / (1.0 - p_prior * q)
        p_post = np.clip(p_post, 1e-9, 1.0 - 1e-9)
        total_IG += np.sum(cell_entropy_map(p_prior) - cell_entropy_map(p_post))
        pred_belief[mask] = p_post
    return total_IG

def plan_velocity_ipp_3D(drone_pos, drone_vel, belief, grid_points, soft_poly, step_length, fov_angle, alt_target, lam=0.5):
    cx, cy, cz = drone_pos
    angles = np.linspace(0, 2*np.pi, 12, endpoint=False)
    candidates = []
    for a in angles:
        target = np.array([cx + np.cos(a)*step_length, cy + np.sin(a)*step_length, alt_target])
        if soft_poly.contains(Point(target[0], target[1])):
            candidates.append([drone_pos, target])
    
    if not candidates: return 0.0, 0.0, 0.0
    
    best_J, best_path = -np.inf, None
    for path in candidates:
        I_p = expected_info_gain(path, belief, grid_points, fov_angle)
        E_p = energy_of_path(path)
        J = I_p - (lam * (E_p / 100.0))
        if J > best_J:
            best_J, best_path = J, path
            
    p0, p1 = best_path[0], best_path[1]
    vec = p1 - p0
    dt = max(np.linalg.norm(vec) / 20.0, 0.1)
    return vec[0]/dt, vec[1]/dt, vec[2]/dt

# -----------------------------
# 2. MONTE CARLO MISSION WRAPPER
# -----------------------------
def run_mission(map_side_m, num_drones, trial_id):
    # ---------------------------------------------------------
    # 1. LOCAL ENVIRONMENT SETUP (Fixes "Not Found" error)
    # ---------------------------------------------------------
    half = map_side_m / 2.0
    # Define the polygon locally for this specific map size
    soft_poly = Polygon([(-half, -half), (half, -half), (half, half), (-half, half)])
    
    # Scale resolution: prevents memory crash on 10km maps
    grid_res = max(10.0, map_side_m / 250.0) 
    grid_x = np.arange(-half, half + grid_res, grid_res)
    grid_y = np.arange(-half, half + grid_res, grid_res)
    XX, YY = np.meshgrid(grid_x, grid_y)
    
    # Generate grid_points locally
    grid_points_all = np.column_stack([XX.ravel(), YY.ravel()])
    inside_mask = np.array([soft_poly.contains(Point(p)) for p in grid_points_all])
    grid_points = grid_points_all[inside_mask]
    
    # Initialize belief for these specific points
    belief = 0.5 * np.ones(len(grid_points))
    h_start = get_global_entropy(belief)
    
    # ---------------------------------------------------------
    # 2. UAV INITIALIZATION
    # ---------------------------------------------------------
    # Randomized deployment +/- 100m around [0,0]
    drone_positions = [np.array([np.random.uniform(-100, 100), 
                                 np.random.uniform(-100, 100), 50.0]) for _ in range(num_drones)]
    drone_vels = [np.zeros(3) for _ in range(num_drones)]
    E_rem = np.ones(num_drones) * 100.0 
    
    # Dynamic time limit based on map scale
    max_t = (map_side_m / 20.0) * 10.0  
    
    t_sim = 0
    dt = 1.0
    entropy_history = []
    fov_angle = np.deg2rad(45)

    # ---------------------------------------------------------
    # 3. MAIN SIMULATION LOOP
    # ---------------------------------------------------------
    while t_sim < max_t:
        current_h = get_global_entropy(belief)
        reduction = (1.0 - (current_h / h_start))
        entropy_history.append(reduction)
        
        # Stop condition: 90% entropy reduction achieved
        if reduction >= 0.90:
            return {"status": "SUCCESS", "time": t_sim, "reduction": reduction, 
                    "energy_per_uav": E_rem.tolist(), "h_series": entropy_history}

        active_searchers = 0
        step_len = max(40.0, map_side_m / 50.0)

        for i in range(num_drones):
            # A. RETURN TO BASE (RTB) LOGIC (20% Battery)
            if E_rem[i] <= 20.0:
                vec_to_base = np.array([0.0, 0.0, 50.0]) - drone_positions[i]
                dist = np.linalg.norm(vec_to_base)
                drone_vels[i] = (vec_to_base / (dist + 1e-6)) * 20.0 if dist > 2.0 else np.zeros(3)
            else:
                active_searchers += 1
                # B. PLANNING (Pass local grid_points and soft_poly)
                vx, vy, vz = plan_velocity_ipp_3D(
                    drone_positions[i], drone_vels[i], belief, 
                    grid_points, soft_poly, # <--- CORRECTLY PASSED HERE
                    step_len, fov_angle, 50.0
                )
                drone_vels[i] = np.array([vx, vy, vz])
                
                # C. SENSOR UPDATE (Pass local grid_points)
                mask, p_hit = visible_cells_at(drone_positions[i], grid_points, fov_angle)
                if np.any(mask):
                    p, q = belief[mask], p_hit[mask]
                    # Bayesian "No-Hit" update
                    belief[mask] = np.clip((p*(1-q))/(1-p*q), 1e-9, 1-1e-9)

            # D. PHYSICS & ENERGY DRAIN
            drone_positions[i] += drone_vels[i] * dt
            p_inst = get_vsqp_power(np.linalg.norm(drone_vels[i][:2]))
            E_rem[i] -= p_inst / 10800.0 # Standard drain rate
            
        # Stop condition: No searchers left (all in RTB mode)
        if active_searchers == 0:
            return {"status": "FLEET_RTB", "time": t_sim, "reduction": reduction, 
                    "energy_per_uav": E_rem.tolist(), "h_series": entropy_history}
        
        t_sim += dt

    return {"status": "TIMEOUT", "time": max_t, "reduction": reduction, 
            "energy_per_uav": E_rem.tolist(), "h_series": entropy_history}
# -----------------------------
# 3. MASTER CONTROLLER
# -----------------------------
if __name__ == "__main__":
    results_dir = "mc_results_scaling"
    logs_dir = os.path.join(results_dir, "detailed_time_series")
    os.makedirs(logs_dir, exist_ok=True)
    summary_csv = os.path.join(results_dir, "scalability_summary_log.csv")
    
    # scales = [500, 2000, 5000, 10000]
    # swarm_sizes = [1, 3, 5, 7, 10, 15, 25, 35, 50, 75, 100]
    # trials = 50
    scales = [10000]
    swarm_sizes = [1, 3, 5, 7, 10, 15, 25, 35, 50, 75, 100]
    trials = 10
    
    for s in scales:
        for n in swarm_sizes:
            for trial in range(trials):
                start_wall = time.time()
                data = run_mission(s, n, trial)
                
                # Append to Summary CSV
                df_row = pd.DataFrame([{
                    "map_size": s, "swarm_size": n, "trial": trial, "status": data["status"],
                    "sim_time": data["time"], "final_reduction": data["reduction"],
                    "avg_battery": np.mean(data["energy_per_uav"]), "wall_time": time.time() - start_wall
                }])
                df_row.to_csv(summary_csv, mode='a', index=False, header=not os.path.exists(summary_csv))
                
                # Save Detailed JSON
                log_file = os.path.join(logs_dir, f"log_s{s}_n{n}_t{trial}.json")
                with open(log_file, 'w') as f:
                    json.dump({"h_history": data["h_series"], "energy": data["energy_per_uav"]}, f)
                
                print(f"S:{s}m | N:{n} | T:{trial} | {data['status']} ({int(data['reduction']*100)}%)")