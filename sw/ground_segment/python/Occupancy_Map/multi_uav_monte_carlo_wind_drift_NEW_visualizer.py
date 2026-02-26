import os
import glob
import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIGURATION ---
# Path to your NEW JSON logs directory
LOGS_DIR = "wind_drift_severity_experiment_new/json"

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
            
            # --- Extract Config Data ---
            severity = config["severity"]
            num_victims = int(config["victims"])
            trial_seed = int(config["seed"])
            
            # --- Extract Results Data ---
            sim_time = results["total_sim_time"]
            confirmed = int(results["final_confirmed"])
            aborted = int(results.get("final_aborted", 0))
            is_success = results.get("success", (confirmed == num_victims))

            data_list.append({
                "severity": severity,
                "num_victims": num_victims,
                "trial_seed": trial_seed,
                "sim_time": sim_time,
                "confirmed": confirmed,
                "aborted": aborted,
                "is_success": is_success
            })

        except Exception as e:
            print(f"Skipping corrupt file {filepath}: {e}")

    return pd.DataFrame(data_list)


def plot_results(df):
    if df.empty:
        return

    # 1x3 Grid: Success, Aborts, Time
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    ax_success = axes[0]
    ax_aborts  = axes[1]
    ax_time    = axes[2]

    severities = ["Low", "Medium", "High"]
    colors = {"Low": "#440154", "Medium": "#21918c", "High": "#fde725"} 
    markers = {"Low": "o", "Medium": "s", "High": "^"}

    # -------------------------------------------------------
    # 1. SUCCESS RATE (Left) - FIGURE A
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

    ax_success.set_title("A) Mission Success Rate", fontsize=16, fontweight='bold')
    ax_success.set_ylabel("Success (%)", fontsize=14)
    ax_success.set_xlabel("Number of Victims", fontsize=14)
    ax_success.set_ylim(-5, 105)
    ax_success.legend(title="Severity", loc="lower left", fontsize=12)
    ax_success.grid(True, linestyle='--', alpha=0.6)

    # -------------------------------------------------------
    # 2. ABORTS vs SEVERITY (Middle) - FIGURE B
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
        
    ax_aborts.set_title("B) Tracking Instability (Aborts per Mission)", fontsize=16, fontweight='bold')
    ax_aborts.set_ylabel("Number of Aborts (Count)", fontsize=14)
    ax_aborts.set_xlabel("Severity Level", fontsize=14)
    ax_aborts.grid(True, linestyle='--', alpha=0.6)

    # -------------------------------------------------------
    # 3. TIME TO COMPLETION (Right) - FIGURE C
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

    ax_time.set_title("C) Mission Duration (Successful Runs Only)", fontsize=16, fontweight='bold')
    ax_time.set_ylabel("Time (s)", fontsize=14)
    ax_time.set_xlabel("Number of Victims", fontsize=14)
    ax_time.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    output_path = "visualizer_output_new_run_labeled.png"
    plt.savefig(output_path, dpi=300)
    print(f"Success! Plot saved to {output_path}")
    plt.show()

# --- MAIN ---
if __name__ == "__main__":
    df = load_data_from_jsons(LOGS_DIR)
    
    if not df.empty:
        df.to_csv("compiled_json_data_new.csv", index=False)
        print("Compiled data saved to compiled_json_data_new.csv")
        plot_results(df)