import os
import glob
import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIGURATION ---
# Path to your JSON logs directory
LOGS_DIR = "/home/shivesh/paparazzi2/campaign_b_results/detailed_json_logs"

def load_data_from_jsons(logs_dir):
    json_files = glob.glob(os.path.join(logs_dir, "*.json"))
    
    if not json_files:
        print(f"CRITICAL ERROR: No JSON files found in {logs_dir}")
        return pd.DataFrame()

    data_list = []
    print(f"Found {len(json_files)} logs. Processing...")

    for filepath in json_files:
        try:
            with open(filepath, 'r') as f:
                entry = json.load(f)
                
            config = entry["config"]
            results = entry["results"]
            
            severity = config["severity"]
            num_victims = int(config["victims"])
            trial_seed = int(config["seed"])
            
            sim_time = results["total_sim_time"]
            confirmed = int(results["final_confirmed"])
            
            # --- EXTRACT ABORT COUNT ---
            aborted = int(results.get("final_aborted", 0))
            
            # Energy Calculation
            energy_per_uav_list = results["energy_per_uav"]
            total_swarm_energy = sum(energy_per_uav_list)
            
            is_success = (confirmed == num_victims)
            
            if confirmed > 0:
                energy_per_victim = total_swarm_energy / confirmed
            else:
                energy_per_victim = np.nan 

            data_list.append({
                "severity": severity,
                "num_victims": num_victims,
                "trial_seed": trial_seed,
                "sim_time": sim_time,
                "confirmed": confirmed,
                "aborted": aborted,   # <--- Added this field
                "is_success": is_success,
                "total_swarm_energy": total_swarm_energy,
                "energy_per_victim": energy_per_victim
            })

        except Exception as e:
            print(f"Skipping corrupt file {filepath}: {e}")

    return pd.DataFrame(data_list)


def plot_results(df):
    if df.empty:
        return

    # plt.style.use('ggplot')
    
    # 2x2 Grid: Success, Aborts, Time, Energy
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(2, 2)
    
    ax_success = fig.add_subplot(gs[0, 0])
    ax_aborts  = fig.add_subplot(gs[0, 1])  # <--- New Plot
    ax_time    = fig.add_subplot(gs[1, 0])
    ax_energy  = fig.add_subplot(gs[1, 1])

    severities = ["Low", "Medium", "High"]
    colors = {"Low": "#440154", "Medium": "#21918c", "High": "#fde725"} 
    markers = {"Low": "o", "Medium": "s", "High": "^"}

    # -------------------------------------------------------
    # 1. SUCCESS RATE (Top Left)
    # -------------------------------------------------------
    for sev in severities:
        subset = df[df["severity"] == sev]
        if subset.empty: continue
        
        stats = subset.groupby("num_victims")["is_success"].mean().reset_index()
        
        # Explicit numpy conversion
        x_val = stats["num_victims"].values
        y_val = stats["is_success"].values * 100
        
        ax_success.plot(
            x_val, 
            y_val, 
            label=sev,
            color=colors[sev],
            marker=markers[sev],
            linewidth=2.5
        )

    ax_success.set_title("Mission Success Rate", fontsize=16)
    ax_success.set_ylabel("Success (%)", fontsize=14)
    ax_success.set_ylim(-5, 105)
    ax_success.legend(title="Severity", loc="lower left", fontsize=12)
    ax_success.grid(True, linestyle='--', alpha=0.6)

    # -------------------------------------------------------
    # 2. ABORTS vs SEVERITY (Top Right) - NEW
    # -------------------------------------------------------
    abort_data = []
    abort_labels = []
    
    for sev in severities:
        subset = df[df["severity"] == sev]
        if not subset.empty:
            abort_data.append(subset["aborted"].values)
            abort_labels.append(sev)
        else:
            abort_data.append([])
            abort_labels.append(sev)
            
    bplot_ab = ax_aborts.boxplot(
        abort_data,
        labels=abort_labels,
        patch_artist=True,
        medianprops=dict(color="black", linewidth=2),
        flierprops=dict(marker='x', markeredgecolor='black', markersize=6)
    )
    
    for patch, sev in zip(bplot_ab['boxes'], severities):
        patch.set_facecolor(colors[sev])
        patch.set_alpha(0.7)
        
    ax_aborts.set_title("Tracking Instability (Aborts per Mission)", fontsize=16)
    ax_aborts.set_ylabel("Number of Aborts (Count)", fontsize=14)
    ax_aborts.set_xlabel("Severity Level", fontsize=14)
    ax_aborts.grid(True, linestyle='--', alpha=0.6)

    # -------------------------------------------------------
    # 3. TIME TO COMPLETION (Bottom Left) - SUCCESS ONLY
    # -------------------------------------------------------
    for sev in severities:
        subset = df[(df["severity"] == sev) & (df["is_success"] == True)]
        if subset.empty: continue

        stats = subset.groupby("num_victims")["sim_time"].agg(['mean', 'std']).reset_index()
        
        x_val = stats["num_victims"].values
        y_mean = stats["mean"].values
        y_std = stats["std"].values
        
        ax_time.plot(
            x_val, 
            y_mean, 
            color=colors[sev],
            marker=markers[sev],
            linewidth=2.5,
            markersize=8
        )
        
        ax_time.fill_between(
            x_val,
            y_mean - y_std,
            y_mean + y_std,
            color=colors[sev],
            alpha=0.2
        )

    ax_time.set_title("Mission Duration (Successful Runs Only)", fontsize=16)
    ax_time.set_ylabel("Time (s)", fontsize=14)
    ax_time.set_xlabel("Number of Victims", fontsize=14)
    ax_time.grid(True, linestyle='--', alpha=0.6)

    # -------------------------------------------------------
    # 4. ENERGY COST (Bottom Right) - SUCCESS ONLY, LOG SCALE
    # -------------------------------------------------------
    box_data = []
    labels = []
    
    for sev in severities:
        subset = df[df["severity"] == sev].dropna(subset=["energy_per_victim"])
        if not subset.empty:
            box_data.append(subset["energy_per_victim"].values)
            labels.append(sev)
        else:
            box_data.append([])
            labels.append(sev)

    bplot = ax_energy.boxplot(
        box_data, 
        labels=labels, 
        patch_artist=True,
        medianprops=dict(color="black", linewidth=2),
        flierprops=dict(marker='o', markerfacecolor='white', markersize=5, linestyle='none')
    )

    for patch, sev in zip(bplot['boxes'], severities):
        patch.set_facecolor(colors[sev])
        patch.set_alpha(0.7)

    ax_energy.set_title("Total Swarm Energy per Victim\n(Successful Finds Only)", fontsize=16)
    ax_energy.set_ylabel("Energy (Joules) - Log Scale", fontsize=14)
    ax_energy.set_xlabel("Severity Level", fontsize=14)
    ax_energy.set_yscale('log')
    ax_energy.grid(True, which="major", linestyle='--', alpha=0.8)
    ax_energy.grid(True, which="minor", linestyle=':', alpha=0.4)

    plt.tight_layout()
    output_path = "campaign_b_json_analysis_with_aborts.png"
    plt.savefig(output_path, dpi=300)
    print(f"Success! Plot saved to {output_path}")
    plt.show()

# --- MAIN ---
if __name__ == "__main__":
    df = load_data_from_jsons(LOGS_DIR)
    
    if not df.empty:
        df.to_csv("compiled_json_data_with_aborts.csv", index=False)
        print("Compiled data saved to compiled_json_data_with_aborts.csv")
        plot_results(df)