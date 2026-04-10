import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import os

# --- CONFIGURATION ---
CSV_FILE = "mci_results_scalability_numba_now_good/scalability_summary_numba.csv"
JSON_FILE = "mci_results_curves/scalability_curves.json"
OUTPUT_DIR = "scalability_plots_styled"

# --- STYLE SETTINGS ---
COLORS = {500: "#440154", 2000: "#31688e", 5000: "#35b779", 10000: "#fde725"}
MARKERS = {500: "o", 2000: "s", 5000: "^", 10000: "D"}

def load_data():
    if not os.path.exists(CSV_FILE):
        print(f"[ERROR] CSV not found: {CSV_FILE}")
        return pd.DataFrame(), {}
    
    df = pd.read_csv(CSV_FILE)
    
    curves = {}
    if os.path.exists(JSON_FILE):
        with open(JSON_FILE, 'r') as f:
            curves = json.load(f)
            
    return df, curves

def plot_metrics_dashboard(df):
    if df.empty: return
    print("Generating Scalability Metrics Dashboard...")
    
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(2, 2)
    
    ax_success = fig.add_subplot(gs[0, 0])
    ax_speedup = fig.add_subplot(gs[0, 1])
    ax_time    = fig.add_subplot(gs[1, 0])
    ax_energy  = fig.add_subplot(gs[1, 1])

    map_sizes = sorted(df['map_size'].unique())

    # --- 1. SUCCESS RATE ---
    summary = df.groupby(['map_size', 'swarm_size']).agg(
        total=('status', 'count'),
        success_count=('status', lambda x: (x == 'SUCCESS').sum())
    ).reset_index()
    summary['rate'] = (summary['success_count'] / summary['total']) * 100.0

    for ms in map_sizes:
        subset = summary[summary['map_size'] == ms].sort_values('swarm_size')
        if subset.empty: continue
        ax_success.plot(subset['swarm_size'].values, subset['rate'].values, 
                        label=f"{ms}m", color=COLORS.get(ms, 'black'), 
                        marker=MARKERS.get(ms, 'o'), linewidth=2.5, markersize=8)

    ax_success.set_title("Mission Success Rate", fontsize=16)
    ax_success.set_ylabel("Success (%)", fontsize=14)
    ax_success.set_ylim(-5, 105)
    ax_success.legend(loc="lower right")
    ax_success.grid(True, linestyle='--', alpha=0.6)

    # --- 2. SPEEDUP FACTOR ---
    success_df = df[df['status'] == 'SUCCESS']
    max_swarm = 0
    if not success_df.empty:
        for ms in map_sizes:
            subset = success_df[success_df['map_size'] == ms]
            if subset.empty: continue
            min_n = subset['swarm_size'].min()
            base_time = subset[subset['swarm_size'] == min_n]['sim_time'].mean()
            
            agg = subset.groupby('swarm_size')['sim_time'].mean().reset_index()
            agg['speedup'] = base_time / agg['sim_time']
            
            x = agg['swarm_size'].values
            y = agg['speedup'].values
            if x.max() > max_swarm: max_swarm = x.max()

            ax_speedup.plot(x, y, label=f"{ms}m", color=COLORS.get(ms, 'black'),
                            marker=MARKERS.get(ms, 'o'), linewidth=2.5)
    
    ax_speedup.plot([1, max(max_swarm, 10)], [1, max(max_swarm, 10)], 'k--', alpha=0.5, label="Ideal")
    ax_speedup.set_title("Swarm Speedup Factor", fontsize=16)
    ax_speedup.set_ylabel("Speedup ($T_{base}/T_N$)", fontsize=14)
    ax_speedup.grid(True, linestyle='--', alpha=0.6)

    # --- 3. TIME TO COMPLETION ---
    if not success_df.empty:
        time_stats = success_df.groupby(['map_size', 'swarm_size'])['sim_time'].agg(['mean', 'std']).reset_index()
        for ms in map_sizes:
            subset = time_stats[time_stats['map_size'] == ms].sort_values('swarm_size')
            if subset.empty: continue
            x = subset['swarm_size'].values
            y = subset['mean'].values
            err = np.nan_to_num(subset['std'].values)
            ax_time.plot(x, y, color=COLORS.get(ms, 'black'), marker=MARKERS.get(ms, 'o'), linewidth=2.5)
            ax_time.fill_between(x, y-err, y+err, color=COLORS.get(ms, 'black'), alpha=0.2)

    ax_time.set_title("Mission Duration (Success Only)", fontsize=16)
    ax_time.set_ylabel("Time (s) - Log Scale", fontsize=14)
    ax_time.set_yscale('log') 
    ax_time.grid(True, which="both", linestyle='--', alpha=0.6)

    # --- 4. ENERGY (SAFE MODE) ---
    if 'energy' in df.columns:
        box_data = []
        labels = []
        for ms in map_sizes:
            subset = df[df['map_size'] == ms]
            energies = []
            for e_str in subset['energy']:
                try:
                    e_list = json.loads(e_str)
                    consumed = [(300.0 * 3600) - e for e in e_list]
                    energies.extend(consumed)
                except: pass
            if energies:
                box_data.append(energies)
                labels.append(f"{ms}m")

        if box_data:
            bplot = ax_energy.boxplot(box_data, labels=labels, patch_artist=True)
            for patch, ms in zip(bplot['boxes'], map_sizes):
                patch.set_facecolor(COLORS.get(ms, 'gray'))
                patch.set_alpha(0.7)
    else:
        ax_energy.text(0.5, 0.5, "Energy Data Not Found in CSV", 
                       ha='center', va='center', fontsize=14, color='gray')

    ax_energy.set_title("Energy Consumed per Drone", fontsize=16)
    ax_energy.set_ylabel("Energy (Joules)", fontsize=14)
    ax_energy.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "Scalability_Metrics_Dashboard.png")
    plt.savefig(path, dpi=300)
    print(f"Saved Dashboard to {path}")

def plot_curves_styled(json_data):
    if not json_data: return
    print("Generating Coverage Curves (Spectral Contrast)...")
    map_keys = sorted([int(k) for k in json_data.keys()])
    fig, axes = plt.subplots(2, 2, figsize=(20, 12))
    axes = axes.flatten()

    for idx, ms in enumerate(map_keys):
        if idx >= 4: break
        ax = axes[idx]
        map_data = json_data[str(ms)]
        swarm_keys = sorted([int(k) for k in map_data.keys()])
        
        # --- COLOR FIX: Using 'nipy_spectral' for MAXIMUM distinction ---
        # This cycles Black -> Purple -> Blue -> Green -> Yellow -> Red
        # Index 0 (N=1) will be Dark Purple/Black
        # Index 1 (N=3) will be Blue
        # Index 2 (N=5) will be Teal/Green
        colors = plt.cm.nipy_spectral(np.linspace(0.05, 0.95, len(swarm_keys)))

        for i, n in enumerate(swarm_keys):
            runs = map_data[str(n)]
            if not runs: continue
            max_len = max(len(r) for r in runs)
            padded = [r + [r[-1]] * (max_len - len(r)) for r in runs]
            avg_curve = np.mean(padded, axis=0) * 100.0
            
            # Lines slightly thicker for visibility
            ax.plot(np.arange(len(avg_curve)), avg_curve, label=f"N={n}", color=colors[i], linewidth=2.5)

        ax.axhline(y=80, color='black', linestyle=':', linewidth=2, alpha=0.8, label="Target")
        ax.set_title(f"Area Coverage ({ms}m Map)", fontsize=16)
        ax.set_ylim(0, 105)
        ax.set_ylabel("Entropy Reduction (%)")
        ax.set_xlabel("Time (s)")
        ax.grid(True, linestyle='--', alpha=0.6)
        if idx == 0: ax.legend(loc='lower right', ncol=2, fontsize=10)

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "Figure_5_1_Coverage_Curves_Spectral.png")
    plt.savefig(path, dpi=300)
    print(f"Saved Curves to {path}")

if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    df, curves = load_data()
    plot_metrics_dashboard(df)
    plot_curves_styled(curves)

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# import json
# import os

# # --- CONFIGURATION ---
# # CSV_FILE = "mci_results_scalability_numba_now_good/scalability_summary_numba.csv"
# # --- CONFIGURATION ---
# # Change this line:
# CSV_FILE = "mci_results_scalability_numba_now_good/scalability_summary_recovered.csv"
# JSON_FILE = "mci_results_curves/scalability_curves.json"
# OUTPUT_DIR = "scalability_plots_individual"

# # --- STYLE SETTINGS ---
# # plt.style.use('ggplot')   #Makes background Grey
# # Map Colors (Metrics)
# COLORS = {500: "#440154", 2000: "#31688e", 5000: "#35b779", 10000: "#fde725"}
# MARKERS = {500: "o", 2000: "s", 5000: "^", 10000: "D"}

# def load_data():
#     if not os.path.exists(CSV_FILE):
#         print(f"[ERROR] CSV not found: {CSV_FILE}")
#         return pd.DataFrame(), {}
    
#     df = pd.read_csv(CSV_FILE)
#     curves = {}
#     if os.path.exists(JSON_FILE):
#         with open(JSON_FILE, 'r') as f:
#             curves = json.load(f)
#     return df, curves

# # =========================================================
# # 1. INDIVIDUAL METRIC PLOTS
# # =========================================================
# def plot_individual_metrics(df):
#     if df.empty: return
#     print("Generating Individual Metric Plots...")
    
#     map_sizes = sorted(df['map_size'].unique())

#     # --- PLOT A: SUCCESS RATE ---
#     plt.figure(figsize=(10, 7))
#     summary = df.groupby(['map_size', 'swarm_size']).agg(
#         total=('status', 'count'),
#         success_count=('status', lambda x: (x == 'SUCCESS').sum())
#     ).reset_index()
#     summary['rate'] = (summary['success_count'] / summary['total']) * 100.0

#     for ms in map_sizes:
#         subset = summary[summary['map_size'] == ms].sort_values('swarm_size')
#         if subset.empty: continue
#         plt.plot(subset['swarm_size'].values, subset['rate'].values, 
#                  label=f"{ms}m", color=COLORS.get(ms, 'black'), 
#                  marker=MARKERS.get(ms, 'o'), linewidth=2.5, markersize=8)

#     plt.title("Mission Success Rate", fontsize=16)
#     plt.ylabel("Success (%)", fontsize=14)
#     plt.xlabel("Swarm Size ($N$)", fontsize=14)
#     plt.ylim(-5, 105)
#     plt.legend(title="Map Scale", loc="lower right", fontsize=12)
#     plt.grid(True, linestyle='--', alpha=0.6)
#     plt.tight_layout()
#     plt.savefig(os.path.join(OUTPUT_DIR, "Scalability_Success.png"), dpi=300)
#     plt.close()

#     # --- PLOT B: SPEEDUP FACTOR ---
#     plt.figure(figsize=(10, 7))
#     success_df = df[df['status'] == 'SUCCESS']
#     max_swarm = 0
#     if not success_df.empty:
#         for ms in map_sizes:
#             subset = success_df[success_df['map_size'] == ms]
#             if subset.empty: continue
#             min_n = subset['swarm_size'].min()
#             base_time = subset[subset['swarm_size'] == min_n]['sim_time'].mean()
            
#             agg = subset.groupby('swarm_size')['sim_time'].mean().reset_index()
#             agg['speedup'] = base_time / agg['sim_time']
            
#             x = agg['swarm_size'].values
#             y = agg['speedup'].values
#             if x.max() > max_swarm: max_swarm = x.max()

#             plt.plot(x, y, label=f"{ms}m", color=COLORS.get(ms, 'black'),
#                      marker=MARKERS.get(ms, 'o'), linewidth=2.5)
    
#     plt.plot([1, max(max_swarm, 10)], [1, max(max_swarm, 10)], 'k--', alpha=0.5, label="Ideal Linear")
#     plt.title("Swarm Speedup Factor", fontsize=16)
#     plt.ylabel("Speedup ($T_{base}/T_N$)", fontsize=14)
#     plt.xlabel("Swarm Size ($N$)", fontsize=14)
#     plt.legend(fontsize=12)
#     plt.grid(True, linestyle='--', alpha=0.6)
#     plt.tight_layout()
#     plt.savefig(os.path.join(OUTPUT_DIR, "Scalability_Speedup.png"), dpi=300)
#     plt.close()

#     # --- PLOT C: MISSION DURATION ---
#     plt.figure(figsize=(10, 7))
#     if not success_df.empty:
#         time_stats = success_df.groupby(['map_size', 'swarm_size'])['sim_time'].agg(['mean', 'std']).reset_index()
#         for ms in map_sizes:
#             subset = time_stats[time_stats['map_size'] == ms].sort_values('swarm_size')
#             if subset.empty: continue
#             x = subset['swarm_size'].values
#             y = subset['mean'].values
#             err = np.nan_to_num(subset['std'].values)
#             plt.plot(x, y, label=f"{ms}m", color=COLORS.get(ms, 'black'), 
#                      marker=MARKERS.get(ms, 'o'), linewidth=2.5)
#             plt.fill_between(x, y-err, y+err, color=COLORS.get(ms, 'black'), alpha=0.2)

#     plt.title("Mission Duration (Success Only)", fontsize=16)
#     plt.ylabel("Time (s) - Log Scale", fontsize=14)
#     plt.xlabel("Swarm Size ($N$)", fontsize=14)
#     plt.yscale('log')
#     plt.legend(title="Map Scale", fontsize=12)
#     plt.grid(True, which="both", linestyle='--', alpha=0.6)
#     plt.tight_layout()
#     plt.savefig(os.path.join(OUTPUT_DIR, "Scalability_Time.png"), dpi=300)
#     plt.close()

#     # --- PLOT D: ENERGY ---
#     plt.figure(figsize=(10, 7))
#     has_energy = False
#     if 'energy' in df.columns:
#         box_data = []
#         labels = []
#         for ms in map_sizes:
#             subset = df[df['map_size'] == ms]
#             energies = []
#             for e_str in subset['energy']:
#                 try:
#                     e_list = json.loads(e_str)
#                     consumed = [(300.0 * 3600) - e for e in e_list]
#                     energies.extend(consumed)
#                 except: pass
#             if energies:
#                 box_data.append(energies)
#                 labels.append(f"{ms}m")

#         if box_data:
#             has_energy = True
#             bplot = plt.boxplot(box_data, labels=labels, patch_artist=True)
#             for patch, ms in zip(bplot['boxes'], map_sizes):
#                 patch.set_facecolor(COLORS.get(ms, 'gray'))
#                 patch.set_alpha(0.7)
    
#     if not has_energy:
#         plt.text(0.5, 0.5, "Energy Data Not Found in CSV", 
#                  ha='center', va='center', fontsize=14, color='gray')

#     plt.title("Energy Consumed per Drone", fontsize=16)
#     plt.ylabel("Energy (Joules)", fontsize=14)
#     plt.xlabel("Map Scale", fontsize=14)
#     plt.grid(True, linestyle='--', alpha=0.6)
#     plt.tight_layout()
#     plt.savefig(os.path.join(OUTPUT_DIR, "Scalability_Energy.png"), dpi=300)
#     plt.close()

# # =========================================================
# # 2. INDIVIDUAL COVERAGE CURVES (Spectral)
# # =========================================================
# def plot_individual_curves(json_data):
#     if not json_data: return
#     print("Generating Individual Coverage Curves...")
    
#     map_keys = sorted([int(k) for k in json_data.keys()])

#     for ms in map_keys:
#         plt.figure(figsize=(10, 7))
        
#         map_data = json_data[str(ms)]
#         swarm_keys = sorted([int(k) for k in map_data.keys()])
        
#         # Using 'nipy_spectral' for distinct separation
#         colors = plt.cm.nipy_spectral(np.linspace(0.05, 0.95, len(swarm_keys)))

#         for i, n in enumerate(swarm_keys):
#             runs = map_data[str(n)]
#             if not runs: continue
            
#             # Pad and Average
#             max_len = max(len(r) for r in runs)
#             padded = [r + [r[-1]] * (max_len - len(r)) for r in runs]
#             avg_curve = np.mean(padded, axis=0) * 100.0
            
#             plt.plot(np.arange(len(avg_curve)), avg_curve, label=f"N={n}", 
#                      color=colors[i], linewidth=2.5)

#         plt.axhline(y=80, color='black', linestyle=':', linewidth=2, alpha=0.8, label="Target 80%)")
        
#         plt.title(f"Area Coverage Rate ({ms}m Map)", fontsize=16)
#         plt.ylabel("Entropy Reduction (%)", fontsize=14)
#         plt.xlabel("Simulation Time (s)", fontsize=14)
#         plt.ylim(0, 105)
#         plt.grid(True, linestyle='--', alpha=0.6)
#         plt.legend(loc='lower right', ncol=2, fontsize=10)
        
#         plt.tight_layout()
#         filename = f"Coverage_Curve_{ms}m.png"
#         plt.savefig(os.path.join(OUTPUT_DIR, filename), dpi=300)
#         print(f"Saved {filename}")
#         plt.close()

# # =========================================================
# # MAIN
# # =========================================================
# if __name__ == "__main__":
#     os.makedirs(OUTPUT_DIR, exist_ok=True)
#     df, curves = load_data()
    
#     plot_individual_metrics(df)
#     plot_individual_curves(curves)
    
#     print(f"\n[DONE] All separate figures saved in '{OUTPUT_DIR}'")
