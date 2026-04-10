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
    total_IG = 0.0
    pred_belief = belief_map.copy() 
    
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
                        
                        H_prior = cell_entropy_numba(p)
                        num = p * (1.0 - q)
                        den = 1.0 - p * q
                        if den < 1e-12: den = 1e-12
                        p_post = num / den
                        
                        H_post = cell_entropy_numba(p_post)
                        total_IG += (H_prior - H_post)
                        pred_belief[r, c] = p_post
    return total_IG

@jit(nopython=True)
def mark_path_visited(path_arr, belief_map, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols):
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
                        belief_map[r, c] = 0.001 

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
            for k in range(3): 
                prev = traj[-1]
                nx = prev[0] + dx * step_length
                ny = prev[1] + dy * step_length
                if not soft_poly.contains(Point(nx, ny)):
                    valid = False; break
                traj.append(np.array([nx, ny, alt]))
            if valid: candidates.append(traj)
            
    if not candidates: return 0.0, 0.0, 0.0, None, -np.inf

    best_J = -np.inf
    best_path = None
    
    for path in candidates:
        path_arr = np.array(path, dtype=np.float64)
        I_p = calc_ig_numba(path_arr, belief_map, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols)
        E_p = energy_of_path_py(path, v_wind)
        
        J = I_p - (lam * (E_p / E_scale))
        if J > best_J:
            best_J = J
            best_path = path_arr 

    if best_path is None: return 0.0, 0.0, 0.0, None, -np.inf

    p0, p1 = best_path[0], best_path[1]
    vec = p1 - p0
    dist = np.linalg.norm(vec[:2])
    
    unit_vec = vec[:2] / (dist+1e-6)
    v_headwind = np.dot(v_wind[:2], unit_vec)
    v_ground = max(2.0, min(20.0, 20.0 + v_headwind))
    
    t_travel = max(dist / v_ground, 0.1)
    return vec[0]/t_travel, vec[1]/t_travel, vec[2]/t_travel, best_path, best_J

# ==============================================================================
# 4. SIMULATION LOOP (BASELINE COMPARISON READY)
# ==============================================================================

# def run_baseline_comparison_numba(map_side_m, num_drones, trial_id, strategy="proposed", wind_mag=6.0, viz=False, lam_val=0.5):
def run_baseline_comparison_numba(map_side_m, num_drones, trial_id, strategy="proposed", wind_mag=6.0, wind_angle_deg=290.0, viz=False, lam_val=0.5): #for the wind direction script
    np.random.seed(trial_id)
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
    
    # Implement Wind
    # v_wind = np.array([wind_mag, 0.0]) 
    # Implement Directional Wind, for the optimal wind scripyt
    rad = np.deg2rad(wind_angle_deg)
    v_wind = np.array([wind_mag * np.cos(rad), wind_mag * np.sin(rad)])
    fov_tan = np.tan(np.deg2rad(45))
    
    max_t = 1500
    t_sim = 0
    dt = 1.0
    entropy_history = [] 

    # --- STRATEGY SETUP ---
    lam_val = 0.5 if strategy == "proposed" else 0.0 #(uncomment for second "if...main" block)
    
    lawnmower_wps = []
    if strategy == "baseline_b":
        corridor_height = map_side_m / num_drones
        sweep_spacing = 100.0 
        for d_idx in range(num_drones):
            wps = []
            y_start = -half + (d_idx * corridor_height) + (sweep_spacing / 2)
            y_end = y_start + corridor_height
            curr_y = y_start
            going_right = True
            while curr_y < y_end:
                if going_right:
                    wps.append(np.array([-half + 30.0, curr_y, 50.0]))
                    wps.append(np.array([half - 30.0, curr_y, 50.0]))
                else:
                    wps.append(np.array([half - 30.0, curr_y, 50.0]))
                    wps.append(np.array([-half + 30.0, curr_y, 50.0]))
                going_right = not going_right
                curr_y += sweep_spacing
            lawnmower_wps.append(wps)
            
    current_wp_idx = np.zeros(num_drones, dtype=int)

    # --- VISUALIZATION SETUP ---
    drone_history = [[pos.copy()] for pos in drone_positions]
    if viz:
        import matplotlib.pyplot as plt
        plt.ion() # Turn on interactive mode
        fig, ax = plt.subplots(figsize=(8, 8))
    
    # --- MAIN LOOP ---
    while t_sim < max_t:
        p_flat = belief_map[inside_mask]
        current_h = np.sum(-p_flat * np.log2(p_flat) - (1-p_flat)*np.log2(1-p_flat))
        
        # Guard against zero division if map is completely empty
        reduction = 0.0 if h_start == 0 else (1.0 - (current_h / h_start))
        entropy_history.append(float(reduction))
        
        # Calculate battery pct remaining
        batt_pcts = (E_rem / (300.0 * 3600)) * 100.0
        
        # STOPPING CONDITION: 80% Coverage
        if reduction >= 0.8:
            return {
                "status": "SUCCESS", 
                "time": t_sim, 
                "reduction": reduction, 
                "energy": E_rem.tolist(),
                "battery_pct_remaining": batt_pcts.tolist(),
                "curve": entropy_history
            }

        planning_belief = belief_map.copy()
        active_searchers = 0
        
        for i in range(num_drones):
            # Battery Dead limit (20%)
            if E_rem[i] <= (300.0*3600 * 0.2):
                vec = np.array([0.0,0.0,50.0]) - drone_positions[i]
                d = np.linalg.norm(vec)
                drone_vels[i] = (vec/(d+1e-6)) * 20.0 if d > 5.0 else np.zeros(3)
                continue
            
            active_searchers += 1
            
            # --- BASELINE B: Lawnmower Logic ---
         
            if strategy == "baseline_b":
                wps = lawnmower_wps[i]
                
                # 1. Advance the waypoint if we are close enough
                if current_wp_idx[i] < len(wps):
                    target = wps[current_wp_idx[i]]
                    dist = np.linalg.norm(target[:2] - drone_positions[i][:2])
                    if dist < 20.0:
                        current_wp_idx[i] += 1
                
                # 2. Infinite Patrol Logic: If we hit the end, reverse the route!
                if current_wp_idx[i] >= len(wps):
                    lawnmower_wps[i].reverse()
                    current_wp_idx[i] = 1 # Head to the next point in the reversed list
                
                # 3. Calculate velocity towards the active waypoint
                target = lawnmower_wps[i][current_wp_idx[i]]
                vec = target - drone_positions[i]
                dist = np.linalg.norm(vec[:2])
                
                unit = vec / (dist + 1e-6)
                v_headwind = np.dot(v_wind[:2], unit[:2])
                speed = max(2.0, min(20.0, 20.0 + v_headwind))
                vx, vy = unit[:2] * speed
                vz = 0.0

            # --- PROPOSED & BASELINE A: IPP Logic ---
            # else:
            #     # Starburst (First 10s)
            #     if t_sim < 10.0:
            #         angle = (2 * np.pi / num_drones) * i
            #         vx = 20.0 * np.cos(angle)
            #         vy = 20.0 * np.sin(angle)
            #         vz = 0.0
            else:
                # Uses dynamic lam_val (0.5 or 0.0)
                vx, vy, vz, best_path, _ = plan_velocity_ipp_3D_numba(
                    drone_positions[i], drone_vels[i], planning_belief, inside_mask,
                    (x_min, y_min), grid_res, soft_poly, v_wind, lam=lam_val
                )
                
                if best_path is not None:
                    mark_path_visited(best_path, planning_belief, inside_mask, x_min, y_min, grid_res, fov_tan, rows, cols)
            
            # Banking Physics
            # --- KINEMATICS & PHYSICS ---
            # if strategy == "baseline_b":
            # --- UNIFIED VSQP KINEMATICS & PHYSICS ---
            v_curr = drone_vels[i][:2]
            current_speed = np.linalg.norm(v_curr)
            
            # 1. Determine Desired State
            desired_speed = np.linalg.norm([vx, vy])
            if desired_speed < 0.1:
                desired_heading = np.arctan2(v_curr[1], v_curr[0]) # Keep facing forward if stopping
            else:
                desired_heading = np.arctan2(vy, vx)
                
            current_heading = np.arctan2(v_curr[1], v_curr[0])
            angle_diff = (desired_heading - current_heading + np.pi) % (2*np.pi) - np.pi
            
            # 2. Flight Regime Turning Constraints
            if current_speed < 12.0:
                # Quadrotor Mode: Can yaw sharply to face the new waypoint
                max_turn = np.deg2rad(90.0) * dt
            else:
                # Fixed-Wing Mode: Aerodynamic banking limits
                max_turn = np.deg2rad(15.0) * dt
                
                # 3. Cornering Deceleration Logic
                # If flying fast but commanded to make a sharp turn (>20 deg),
                # the drone MUST brake to drop into quadrotor mode to make the corner.
                if abs(angle_diff) > np.deg2rad(20.0):
                    desired_speed = min(desired_speed, 10.0) 

            # 4. Apply Kinematics (Max Accel 6.0 m/s^2 from Thesis Table 5.1)
            clamped_diff = np.clip(angle_diff, -max_turn, max_turn)
            new_heading = current_heading + clamped_diff
            
            speed_diff = desired_speed - current_speed
            clamped_accel = np.clip(speed_diff, -6.0 * dt, 6.0 * dt)
            new_speed = current_speed + clamped_accel
            
            drone_vels[i] = np.array([new_speed * np.cos(new_heading), new_speed * np.sin(new_heading), vz])

            # --- DEBUG PRINT FOR VELOCITY VECTORS ---
            # Removed the trial_id == 0 check so it always prints!
            # if strategy == "baseline_b" and int(t_sim) % 5 == 0: 
            #     v_des = np.array([vx, vy])
            #     v_actual = drone_vels[i][:2]
            #     print(f"[t={t_sim:.0f}s] UAV {i} | "
            #             f"v_desired: [{v_des[0]:.1f}, {v_des[1]:.1f}] | "
            #             f"v_actual: [{v_actual[0]:.1f}, {v_actual[1]:.1f}] | "
            #             f"Speed: {current_speed:.1f} m/s")
                    
        # State Execution (Update Real Map)
        for i in range(num_drones):
            drone_positions[i] += drone_vels[i] * dt
            
            # Energy consumption against wind
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
            batt_pcts = (E_rem / (300.0 * 3600)) * 100.0
            return {
                "status": "BATTERY_EXHAUSTED", 
                "time": t_sim, 
                "reduction": reduction, 
                "energy": E_rem.tolist(),
                "battery_pct_remaining": batt_pcts.tolist(),
                "curve": entropy_history 
            }
        
        # Update flight history for the trails
        for i in range(num_drones):
            drone_history[i].append(drone_positions[i].copy())

        # --- REAL-TIME MATPLOTLIB VISUALIZER ---
        if viz and int(t_sim) % 5 == 0:  # Update plot every 5 simulation seconds to keep it fast
            ax.clear()
            
            # 1. Draw Map Boundary
            x, y = soft_poly.exterior.xy
            ax.plot(x, y, color='#666666', linestyle='dashed', linewidth=2)
            
            # 2. Draw Lawnmower Grid (if applicable)
            if strategy == "baseline_b":
                for wps in lawnmower_wps:
                    wp_arr = np.array(wps)
                    ax.plot(wp_arr[:,0], wp_arr[:,1], color='gray', linestyle=':', alpha=0.5)
                    
            # 3. Draw Drones and Trails
            colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
            for i in range(num_drones):
                c = colors[i % len(colors)]
                hist = np.array(drone_history[i])
                
                # Draw Trail
                ax.plot(hist[:,0], hist[:,1], color=c, alpha=0.6, linewidth=2)
                # Draw Drone
                ax.scatter(drone_positions[i][0], drone_positions[i][1], color=c, s=100, zorder=5, edgecolors='black')
                # Optional: Draw Wind Vector on Drone 0
                if i == 0:
                    ax.arrow(0, half-200, wind_mag*10, 0, head_width=30, head_length=40, fc='cyan', ec='cyan')
                    ax.text(0, half-150, f"Wind: {wind_mag} m/s", color='cyan', fontsize=10, ha='center')

            ax.set_xlim(-half - 100, half + 100)
            ax.set_ylim(-half - 100, half + 100)
            ax.set_xlabel("X (m)", color='black')   
            ax.set_ylabel("Y (m)", color='black')
            ax.set_aspect('equal')
            ax.set_facecolor("#ffffff")
            fig.patch.set_facecolor('#ffffff')
            ax.tick_params(colors='black')
            
            ax.set_title(f"Baseline| Time: 228s | Entropy Reduced: 80%", color='black', fontsize=14)
            plt.pause(0.01)
            
        t_sim += dt

    batt_pcts = (E_rem / (300.0 * 3600)) * 100.0
    return {
        "status": "TIMEOUT", 
        "time": max_t, 
        "reduction": reduction, 
        "energy": E_rem.tolist(),
        "battery_pct_remaining": batt_pcts.tolist(),
        "curve": entropy_history 
    }



if __name__ == "__main__":
    print("Launching Visualizer for Lawnmower (Baseline B)...")
    
    # Run a 500m map so you can watch them complete the route quickly
    data = run_baseline_comparison_numba(
        map_side_m=1000, 
        num_drones=5, 
        trial_id=0, 
        strategy="baseline_b", 
        wind_mag=6.0, 
        viz=True    # <--- TURNS ON MATPLOTLIB
    )
    
    print(f"\nFinal Status: {data['status']}")
    print(f"Time: {data['time']}s")
    print(f"Entropy Reduced: {(data['reduction']*100):.1f}%")
    
    # Keep window open at the end
    import matplotlib.pyplot as plt
    plt.ioff()
    plt.show()



# if __name__ == "__main__":
#     results_dir = "phase2_lambda_sweep_290deg"
#     os.makedirs(results_dir, exist_ok=True)
    
#     curve_file = os.path.join(results_dir, "lambda_curves.json")
#     summary_csv = os.path.join(results_dir, "lambda_summary.csv")
    
#     map_size = 1000
#     n_drones = 5
#     wind_mag = 6.0
#     trials = 10
    
#     # 1. First we test the Lawnmower Baseline
#     # 2. Then we test the Proposed Method across all these Lambda values
#     # lambda_values = [0.0, 0.01, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0]
#     lambda_values = [100.0, 10000.0, 1000000.0] # For the wind direction script, to show that high lambda values just make it ignore energy and act like baseline_a
    
#     # Create our test queue (Baseline B + all Lambda variations)
#     test_queue = [{"strategy": "baseline_b", "lam": 0.0, "name": "baseline_b"}]
#     for l in lambda_values:
#         test_queue.append({"strategy": "proposed", "lam": l, "name": f"proposed_lam_{l}"})

#     all_curves = {}

#     print(f"--- Starting Phase II Lambda Sweep ---")
#     print(f"Map: {map_size}m | Drones: {n_drones} | Wind: {wind_mag}m/s")
    
#     for test in test_queue:
#         strat_name = test["name"]
#         print(f"\n>>> Running: {strat_name.upper()} <<<")
#         all_curves[strat_name] = []
        
#         for trial in range(trials):
#             seed = np.random.randint(0, 1000000)
#             start_time = time.time()
            
#             # Pass both the strategy and the specific lambda value!
#             data = run_baseline_comparison_numba(
#                 map_size, n_drones, trial_id=seed, 
#                 strategy=test["strategy"], wind_mag=wind_mag, lam_val=test["lam"]
#             )
            
#             wall_time = time.time() - start_time
#             all_curves[strat_name].append(data["curve"])
            
#             # Print feedback
#             avg_batt = np.mean(data["battery_pct_remaining"])
#             red_pct = data['reduction'] * 100
#             print(f"[{strat_name}] Trial {trial+1}/{trials} | {data['status']} | Time: {data['time']}s | Reduced: {red_pct:.1f}% | Avg Batt Remaining: {avg_batt:.1f}%")
            
#             # Save to CSV
#             df = pd.DataFrame([{
#                 "Condition": strat_name, "Strategy": test["strategy"], "Lambda": test["lam"], 
#                 "Map": map_size, "Trial": trial, "Status": data["status"], 
#                 "Sim_Time_To_90": data["time"], "Entropy_Reduced_Pct": red_pct, 
#                 "Avg_Battery_Remaining": avg_batt, "Wall_Time": wall_time
#             }])
#             df.to_csv(summary_csv, mode='a', header=not os.path.exists(summary_csv), index=False)
            
#         # Save curves incrementally
#         with open(curve_file, 'w') as f:
#             json.dump(all_curves, f)

#     print(f"\nLambda Parameter Sweep Complete! Results in '{results_dir}' directory.")