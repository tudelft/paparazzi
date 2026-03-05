import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. CONFIGURATION
# ---------------------------------------------------------
CSV_PATHS = [
    "/home/shivesh/paparazzi2/mci_results_2000m_final/mci_lookup_table.csv",
    "/home/shivesh/paparazzi2/mci_results_2000_second_m_final/mci_lookup_table2.csv"
]
OUTPUT_CSV = "swarm_sizing_lookup_table.csv"
OUTPUT_PLOT = "swarm_performance_curves.png"

# ---------------------------------------------------------
# 2. DATA LOADING
# ---------------------------------------------------------
def load_data(paths):
    dfs = []
    for p in paths:
        if os.path.exists(p):
            try:
                df = pd.read_csv(p)
                df.columns = df.columns.str.strip()
                dfs.append(df)
                print(f"[INFO] Loaded {len(df)} runs from {p}")
            except Exception as e:
                print(f"[WARN] Error loading {p}: {e}")
    if not dfs: return pd.DataFrame()
    return pd.concat(dfs, ignore_index=True)

# ---------------------------------------------------------
# 3. ANALYSIS ENGINE
# ---------------------------------------------------------
def analyze_results(df):
    if df.empty: return pd.DataFrame()

    # --- METRIC 1: Strict Success (Find 100% of victims) ---
    df['success_strict'] = (df['victims_found'] >= df['num_victims']).astype(int)
    
    # --- METRIC 2: Relaxed Success (Find >= 80% of victims) ---
    # Useful for seeing "good enough" performance
    df['pct_found'] = df['victims_found'] / df['num_victims']
    df['success_relaxed'] = (df['pct_found'] >= 0.80).astype(int)

    group_cols = ['map_size', 'severity', 'num_victims', 'swarm_size', 'composition']
    
    summary = df.groupby(group_cols).agg(
        trials=('trial_seed', 'count'),
        avg_pct_found=('pct_found', 'mean'),
        strict_success_rate=('success_strict', 'mean'),
        relaxed_success_rate=('success_relaxed', 'mean'), # Probability of finding >= 80%
        avg_time=('sim_time', 'mean')
    ).reset_index()

    return summary

# ---------------------------------------------------------
# 4. GENERATE RECOMMENDATIONS
# ---------------------------------------------------------
def get_best_config(summary_df, target_confidence=0.90, metric='strict_success_rate'):
    """
    Finds minimum swarm size for a specific success metric.
    """
    recommendations = []
    scenarios = summary_df[['map_size', 'severity', 'num_victims']].drop_duplicates()
    
    for _, row in scenarios.iterrows():
        ms, sev, nv = row['map_size'], row['severity'], row['num_victims']
        
        # Isolate this scenario
        data = summary_df[
            (summary_df['map_size'] == ms) &
            (summary_df['severity'] == sev) &
            (summary_df['num_victims'] == nv)
        ]
        
        # Check if any config met the confidence target
        valid = data[data[metric] >= target_confidence]
        
        if not valid.empty:
            best = valid.loc[valid['swarm_size'].idxmin()]
            rec_size = best['swarm_size']
            conf = best[metric]
            note = "Met Target"
        else:
            # Fallback: Best available
            best = data.loc[data[metric].idxmax()]
            rec_size = f"{best['swarm_size']} (Unreliable)" 
            conf = best[metric]
            note = "Target Not Met"

        recommendations.append({
            "Victims": nv,
            "Rec_Swarm": rec_size,
            "Comp": best['composition'],
            "Confidence": f"{conf:.0%}",
            "Avg_Found": f"{best['avg_pct_found']:.0%}", # Show avg % found to debug
            "Status": note
        })
        
    return pd.DataFrame(recommendations)

# ---------------------------------------------------------
# 5. PLOTTING (Robust)
# ---------------------------------------------------------
def plot_performance(summary_df):
    plt.figure(figsize=(10, 6))
    
    # Plotting "Average Percentage of Victims Found" vs Swarm Size
    # This is a smoother curve than binary success rate
    data = summary_df[summary_df['severity'] == 'Low'].copy()
    data.sort_values(by='swarm_size', inplace=True)
    
    victim_counts = sorted(data['num_victims'].unique())
    colors = plt.cm.viridis(np.linspace(0, 1, len(victim_counts)))
    
    for idx, vc in enumerate(victim_counts):
        subset = data[data['num_victims'] == vc]
        
        # If multiple compositions exist, take the best performing one for the plot
        # to show potential capability
        best_lines = subset.groupby('swarm_size')['avg_pct_found'].max().reset_index()
        
        # CONVERT TO NUMPY ARRAYS to fix the plotting error
        x = best_lines['swarm_size'].values
        y = best_lines['avg_pct_found'].values
        
        plt.plot(x, y, marker='o', linewidth=2, label=f"{vc} Victims", color=colors[idx])

    plt.axhline(0.8, color='gray', linestyle='--', alpha=0.5, label='80% Found Threshold')
    plt.axhline(1.0, color='r', linestyle='--', alpha=0.3, label='Perfect Score')
    
    plt.title("Average Victim Discovery Rate by Swarm Size")
    plt.xlabel("Swarm Size")
    plt.ylabel("Average % of Victims Found")
    plt.ylim(0, 1.1)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT)
    print(f"[INFO] Performance plot saved to {OUTPUT_PLOT}")

# ---------------------------------------------------------
# 6. MAIN
# ---------------------------------------------------------
if __name__ == "__main__":
    df = load_data(CSV_PATHS)
    if not df.empty:
        summary = analyze_results(df)
        
        print("\n--- [RELAXED] SWARM SIZING (Find >= 80% of Victims with 90% Confidence) ---")
        rec_relaxed = get_best_config(summary, target_confidence=0.90, metric='relaxed_success_rate')
        print(rec_relaxed.to_string(index=False))

        print("\n--- [STRICT] SWARM SIZING (Find 100% of Victims with 90% Confidence) ---")
        rec_strict = get_best_config(summary, target_confidence=0.90, metric='strict_success_rate')
        print(rec_strict.to_string(index=False))
        
        plot_performance(summary)