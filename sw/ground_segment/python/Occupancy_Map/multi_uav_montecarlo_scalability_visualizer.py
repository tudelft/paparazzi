import pandas as pd
import matplotlib.pyplot as plt
import os
import json
import numpy as np

# --- CONFIGURATION ---
RESULTS_DIR = "mc_results_scaling"
SUMMARY_CSV = os.path.join(RESULTS_DIR, "scalability_summary_log.csv")
LOGS_DIR = os.path.join(RESULTS_DIR, "detailed_time_series")

# 1. LOAD SUMMARY DATA
if not os.path.exists(SUMMARY_CSV):
    print(f"Error: {SUMMARY_CSV} not found.")
else:
    df = pd.read_csv(SUMMARY_CSV)
    
    # --- PLOT 1: SEARCH PERFORMANCE (TIME TO SUCCESS) ---
    # We only plot the 'SUCCESS' status missions to see the true efficiency
    success_df = df[df['status'] == 'SUCCESS']
    
    plt.figure(figsize=(12, 7))
    for scale in sorted(df['map_size'].unique()):
        subset = success_df[success_df['map_size'] == scale]
        # Aggregate trials: Mean and Standard Deviation for error bars
        stats = subset.groupby('swarm_size')['sim_time'].agg(['mean', 'std']).reset_index()
        
        plt.errorbar(stats['swarm_size'], stats['mean'], yerr=stats['std'], 
                     marker='o', capsize=5, linewidth=2, label=f'Area: {scale}m x {scale}m')

    plt.title("Search Efficiency: Mean Time to 90% Coverage", fontsize=14)
    plt.xlabel("Swarm Size (Number of UAVs)", fontsize=12)
    plt.ylabel("Mission Time [s]", fontsize=12)
    plt.yscale('log') # Log scale helps visualize 500m and 10km on one plot
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.legend()
    plt.savefig("mission_time_scaling.png")
    
    # --- CORRECTED PLOT 2: SUCCESS RATE ---
    plt.figure(figsize=(12, 7))
    success_rate = df.groupby(['map_size', 'swarm_size'])['status'].apply(
        lambda x: (x == 'SUCCESS').mean() * 100
    ).reset_index()

    for scale in sorted(success_rate['map_size'].unique()):
        subset = success_rate[success_rate['map_size'] == scale]
        
        # Convert Series to numpy arrays to avoid the indexing error
        x_data = subset['swarm_size'].values
        y_data = subset['status'].values
        
        plt.plot(x_data, y_data, marker='s', label=f'Area: {scale}m')
        
    plt.title("Search Completeness: Success Probability vs Swarm Size", fontsize=14)
    plt.xlabel("Swarm Size", fontsize=12)
    plt.ylabel("Success Rate [%]", fontsize=12)
    plt.ylim(-5, 105)
    plt.legend()
    plt.grid(True)
    plt.savefig("success_probability.png")

# --- PLOT 3: ENTROPY REDUCTION CURVES (FROM JSON) ---
# Pick a specific map size to see how "fast" different swarms clear it
TARGET_SCALE = 10000 
SWARM_SIZES_TO_PLOT = [1, 3, 5, 7, 10, 15, 25, 35, 50, 75, 100]

plt.figure(figsize=(12, 7))

if os.path.exists(LOGS_DIR):
    for n in SWARM_SIZES_TO_PLOT:
        curves = []
        # Find all JSON logs for this specific config
        files = [f for f in os.listdir(LOGS_DIR) if f.startswith(f"log_s{TARGET_SCALE}_n{n}_")]
        
        for f in files:
            with open(os.path.join(LOGS_DIR, f), 'r') as jf:
                log_data = json.load(jf)
                curves.append(log_data['h_history'])
        
        if curves:
            # Align and average the time-series curves
            max_len = max(len(c) for c in curves)
            padded = [c + [c[-1]] * (max_len - len(c)) for c in curves]
            mean_curve = np.mean(padded, axis=0)
            
            plt.plot(mean_curve * 100, label=f'Swarm Size: {n}', linewidth=2)

    plt.title(f"Area Coverage Rate over Time ({TARGET_SCALE}m Map)", fontsize=14)
    plt.xlabel("Simulation Time [s]", fontsize=12)
    plt.ylabel("Global Entropy Reduction [%]", fontsize=12)
    plt.legend()
    plt.grid(True)
    plt.savefig("entropy_reduction_curves_10000.png")

print("Visualization Complete. Files generated: mission_time_scaling.png, success_probability.png, entropy_reduction_curves.png")