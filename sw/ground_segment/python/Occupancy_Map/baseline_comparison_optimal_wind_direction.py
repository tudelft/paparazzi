import numpy as np
import pandas as pd
import time
import os
import matplotlib.pyplot as plt

# Import your simulation function!
from baseline_comparison import run_baseline_comparison_numba

if __name__ == "__main__":
    results_dir = "phase2_wind_direction_high_wind"
    os.makedirs(results_dir, exist_ok=True)
    
    map_size = 1000
    n_drones = 5
    wind_mag = 6.0
    trials = 1 # 5 trials per angle is plenty for a baseline test
    
    # Sweep from 0 to 360 degrees in 30 degree increments
    angles_to_test = np.arange(0, 361, 5)
    
    results = []

    print(f"--- Starting Baseline B Wind Direction Optimization ---")
    
    for angle in angles_to_test:
        print(f"\n>>> Testing Baseline B at Wind Angle: {angle}° <<<")
        batt_scores = []
        coverage_scores = []
        
        for trial in range(trials):
            seed = np.random.randint(0, 1000000)
            data = run_baseline_comparison_numba(
                map_size, n_drones, trial_id=seed, 
                strategy="baseline_b", wind_mag=wind_mag, wind_angle_deg=angle
            )
            
            batt_scores.append(np.mean(data["battery_pct_remaining"]))
            coverage_scores.append(data['reduction'] * 100)
            
        avg_batt = np.mean(batt_scores)
        avg_cov = np.mean(coverage_scores)
        print(f"Result: {avg_cov:.1f}% Coverage | {avg_batt:.1f}% Battery Remaining")
        
        results.append({
            "Wind_Angle_Deg": angle,
            "Avg_Battery": avg_batt,
            "Avg_Coverage": avg_cov
        })

    # Save to CSV
    df = pd.DataFrame(results)
    df.to_csv(os.path.join(results_dir, "baseline_wind_angles.csv"), index=False)
    # --- Generate the Proof Plot ---
    plt.style.use('ggplot')
    plt.figure(figsize=(10, 5))
    
    # FIX: Convert Pandas columns to pure Numpy Arrays
    x_angles = df['Wind_Angle_Deg'].to_numpy()
    y_battery = df['Avg_Battery'].to_numpy()
    
    plt.plot(x_angles, y_battery, marker='o', color='tab:red', linewidth=2, markersize=8)
    
    # Highlight the maximum point
    best_idx = df['Avg_Battery'].idxmax()
    worst_idx = df['Avg_Battery'].idxmin()
    best_angle = df.loc[best_idx, 'Wind_Angle_Deg']
    best_batt = df.loc[best_idx, 'Avg_Battery']
    worst_angle = df.loc[worst_idx, 'Wind_Angle_Deg']
    worst_batt = df.loc[worst_idx, 'Avg_Battery']

    plt.annotate(f"Optimal Condition\n({best_angle}°, {best_batt:.1f}%)", 
                 xy=(best_angle, best_batt), xytext=(best_angle, best_batt - 5),
                 arrowprops=dict(facecolor='black', shrink=0.05),
                 fontsize=10, fontweight='bold', ha='center')
    plt.annotate(f"Worst Condition\n({worst_angle}°, {worst_batt:.1f}%)",
                    xy=(worst_angle, worst_batt), xytext=(worst_angle, worst_batt + 5),
                    arrowprops=dict(facecolor='black', shrink=0.05),
                    fontsize=10, fontweight='bold', ha='center')

    plt.title('Baseline B Efficiency vs. Ambient Wind Direction (6.0 m/s)', fontweight='bold')
    plt.xlabel('Wind Direction (Degrees)', fontweight='bold')
    plt.ylabel('Average Battery Remaining (%)', fontweight='bold')
    plt.xticks(np.arange(0, 361, 30))
    plt.ylim(df['Avg_Battery'].min() - 5, df['Avg_Battery'].max() + 10)
    
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'baseline_wind_optimization.png'), dpi=300)
    print("\nTest Complete! Saved data and plot to 'phase2_wind_direction' folder.")