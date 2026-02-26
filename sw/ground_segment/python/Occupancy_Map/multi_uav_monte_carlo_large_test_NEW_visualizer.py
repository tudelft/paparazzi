import os
import glob
import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# --- CONFIGURATION ---
LOGS_DIR = "large_test_NEW/json"  # Directory containing the new large test JSON logs
TARGET_RECALL = 0.9  # We define "Success" as finding at least 90% of victims on average

def load_large_test_data(logs_dir):
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
                
            config = entry.get("config", {})
            
            # --- Extract Config Data ---
            map_size = int(config.get("map", 0))
            num_drones = int(config.get("drones", 0))
            trial_seed = int(config.get("seed", 0))
            # Extract Mode (Fallback to "Uniform" if missing)
            mode = config.get("mode", "Uniform") 
            
            # --- Extract Results Data (Top-Level Keys) ---
            num_victims = int(entry.get("total_victims", config.get("victims", 0)))
            confirmed = int(entry.get("victims_found", 0))
            
            # --- Calculate Recall (Percentage Found) ---
            recall = confirmed / num_victims if num_victims > 0 else 0.0

            data_list.append({
                "map_size": map_size,
                "num_drones": num_drones,
                "num_victims": num_victims,
                "mode": mode,
                "trial_seed": trial_seed,
                "confirmed": confirmed,
                "recall": recall
            })

        except Exception as e:
            print(f"Skipping corrupt file {filepath}: {e}")

    return pd.DataFrame(data_list)

def generate_lookup_tables(df):
    """
    Calculates the MINIMUM swarm size required to hit the TARGET_RECALL
    for every combination of (map_size) and (num_victims), split by Mode.
    """
    tables = {}
    modes = df['mode'].unique()
    
    for mode in modes:
        df_mode = df[df['mode'] == mode]
        
        # 1. Calculate average recall for every configuration in this mode
        stats = df_mode.groupby(['map_size', 'num_victims', 'num_drones'])['recall'].mean().reset_index()
        
        # 2. Filter only configurations that meet the target recall
        successful_configs = stats[stats['recall'] >= TARGET_RECALL]
        
        # 3. Find the minimum drone count for each map/victim combo
        min_drones = successful_configs.groupby(['map_size', 'num_victims'])['num_drones'].min().reset_index()
        
        # 4. Pivot into a 2D Lookup Table (Rows: Map Size, Cols: Victims)
        if not min_drones.empty:
            tables[mode] = min_drones.pivot(index='map_size', columns='num_victims', values='num_drones')
        else:
            tables[mode] = pd.DataFrame()
            
    return tables

def plot_results(df, lookup_tables):
    if df.empty:
        return

    fig = plt.figure(figsize=(22, 8))
    # Adjust width ratios so the heatmaps share space evenly
    gs = fig.add_gridspec(1, 3, width_ratios=[1.2, 1, 1])
    
    ax_recall = fig.add_subplot(gs[0, 0])
    ax_heat_uni = fig.add_subplot(gs[0, 1])
    ax_heat_mix = fig.add_subplot(gs[0, 2])

    unique_maps = sorted(df['map_size'].unique())
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(unique_maps)))

    # -------------------------------------------------------
    # 1. RECALL vs SWARM SIZE (Plot A)
    # -------------------------------------------------------
    for idx, m_size in enumerate(unique_maps):
        # Plot Uniform (Solid Line, Circles)
        sub_uni = df[(df["map_size"] == m_size) & (df["mode"] == "Uniform")]
        if not sub_uni.empty:
            stats_uni = sub_uni.groupby("num_drones")["recall"].mean().reset_index()
            ax_recall.plot(
                stats_uni["num_drones"].values, 
                (stats_uni["recall"] * 100).values, 
                label=f"{m_size}m (Uniform)",
                color=colors[idx],
                linestyle='-',
                marker='o',
                linewidth=2.5
            )
            
        # Plot Mixed (Dashed Line, Squares)
        sub_mix = df[(df["map_size"] == m_size) & (df["mode"] == "Mixed")]
        if not sub_mix.empty:
            stats_mix = sub_mix.groupby("num_drones")["recall"].mean().reset_index()
            ax_recall.plot(
                stats_mix["num_drones"].values, 
                (stats_mix["recall"] * 100).values, 
                label=f"{m_size}m (Mixed)",
                color=colors[idx],
                linestyle='--',
                marker='s',
                linewidth=2.5
            )

    ax_recall.set_title("A) Victim Discovery Rate (Recall)", fontsize=16, fontweight='bold')
    ax_recall.set_ylabel("Average Victims Found (%)", fontsize=14)
    ax_recall.set_xlabel("Swarm Size (Number of UAVs)", fontsize=14)
    ax_recall.axhline(y=TARGET_RECALL*100, color='r', linestyle=':', alpha=0.8, label=f'>= {int(TARGET_RECALL*100)}% Target')
    ax_recall.legend(title="Map Size & Strategy", fontsize=11, loc='lower right')
    ax_recall.grid(True, linestyle='--', alpha=0.6)
    ax_recall.set_ylim(-5, 105)

    # -------------------------------------------------------
    # Calculate Global Min/Max for Shared Heatmap Color Scale
    # -------------------------------------------------------
    all_values = []
    for mode, tbl in lookup_tables.items():
        if not tbl.empty:
            all_values.extend(tbl.values.flatten())
    
    # Filter out NaNs
    all_values = [v for v in all_values if not np.isnan(v)]
    vmin = min(all_values) if all_values else 1
    vmax = max(all_values) if all_values else 10

    # -------------------------------------------------------
    # 2. THE LOOKUP TABLE HEATMAP - UNIFORM (Plot B)
    # -------------------------------------------------------
    tbl_uni = lookup_tables.get("Uniform", pd.DataFrame())
    if not tbl_uni.empty:
        sns.heatmap(
            tbl_uni, annot=True, fmt="g", cmap="YlGnBu", 
            vmin=vmin, vmax=vmax, cbar_kws={'label': 'Required Swarm Size'},
            ax=ax_heat_uni, linewidths=1, linecolor='black',
            annot_kws={"size": 16, "weight": "bold"}  # <--- ADDED HERE
        )
        ax_heat_uni.set_title(f"B) Required Swarm Size (Uniform)\nTarget: >= {int(TARGET_RECALL*100)}% Recall", fontsize=16, fontweight='bold')
        ax_heat_uni.set_ylabel("Map Edge Length (m)", fontsize=14)
        ax_heat_uni.set_xlabel("Number of Victims", fontsize=14)
        ax_heat_uni.invert_yaxis()
    else:
        ax_heat_uni.text(0.5, 0.5, "Insufficient Uniform Data", ha='center', va='center')
        ax_heat_uni.set_title("B) Required Swarm Size (Uniform)", fontsize=16, fontweight='bold')

    # -------------------------------------------------------
    # 3. THE LOOKUP TABLE HEATMAP - MIXED (Plot C)
    # -------------------------------------------------------
    tbl_mix = lookup_tables.get("Mixed", pd.DataFrame())
    if not tbl_mix.empty:
        sns.heatmap(
            tbl_mix, annot=True, fmt="g", cmap="YlGnBu", 
            vmin=vmin, vmax=vmax, cbar_kws={'label': 'Required Swarm Size'},
            ax=ax_heat_mix, linewidths=1, linecolor='black',
            annot_kws={"size": 16, "weight": "bold"}  # <--- ADDED HERE
        )
        ax_heat_mix.set_title(f"C) Required Swarm Size (Mixed)\nTarget: >= {int(TARGET_RECALL*100)}% Recall", fontsize=16, fontweight='bold')
        ax_heat_mix.set_ylabel("", fontsize=14) # Hide Y label to keep it clean (shared with B)
        ax_heat_mix.set_xlabel("Number of Victims", fontsize=14)
        ax_heat_mix.invert_yaxis()
    else:
        ax_heat_mix.text(0.5, 0.5, "Insufficient Mixed Data", ha='center', va='center')
        ax_heat_mix.set_title("C) Required Swarm Size (Mixed)", fontsize=16, fontweight='bold')

    plt.tight_layout()
    output_path = "large_test_swarm_sizing_split_modes.png"
    plt.savefig(output_path, dpi=300)
    print(f"\nSuccess! Visualizations saved to {output_path}")
    plt.show()

# --- MAIN ---
if __name__ == "__main__":
    df = load_large_test_data(LOGS_DIR)
    
    if not df.empty:
        df.to_csv("compiled_large_test_data_recall_split.csv", index=False)
        print("Compiled raw data saved to compiled_large_test_data_recall_split.csv")
        
        lookup_tables = generate_lookup_tables(df)
        
        for mode in ["Uniform", "Mixed"]:
            tbl = lookup_tables.get(mode, pd.DataFrame())
            if not tbl.empty:
                print(f"\n=== MINIMUM SWARM SIZE LOOKUP TABLE ({mode.upper()}) ===")
                print(tbl.to_string())
                print("========================================================\n")
        
        plot_results(df, lookup_tables)




# import os
# import glob
# import json
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# import seaborn as sns

# # --- CONFIGURATION ---
# LOGS_DIR = "large_test_NEW/json"  # Directory containing the new large test JSON logs

# def load_large_test_data(logs_dir):
#     json_files = glob.glob(os.path.join(logs_dir, "*.json"))
    
#     if not json_files:
#         print(f"CRITICAL ERROR: No JSON files found in {logs_dir}")
#         return pd.DataFrame()

#     data_list = []
#     print(f"Found {len(json_files)} logs. Processing for Variance Diagnostics...")

#     for filepath in json_files:
#         try:
#             with open(filepath, 'r') as f:
#                 entry = json.load(f)
                
#             config = entry.get("config", {})
            
#             # --- Extract Config Data ---
#             map_size = int(config.get("map", 0))
#             num_drones = int(config.get("drones", 0))
#             trial_seed = int(config.get("seed", 0))
#             mode = config.get("mode", "Uniform") 
            
#             # --- Extract Results Data ---
#             num_victims = int(entry.get("total_victims", config.get("victims", 0)))
#             confirmed = int(entry.get("victims_found", 0))
            
#             # --- Calculate Recall (Percentage Found) ---
#             recall = confirmed / num_victims if num_victims > 0 else 0.0

#             data_list.append({
#                 "map_size": map_size,
#                 "num_drones": num_drones,
#                 "num_victims": num_victims,
#                 "mode": mode,
#                 "trial_seed": trial_seed,
#                 "recall": recall
#             })

#         except Exception as e:
#             print(f"Skipping corrupt file {filepath}: {e}")

#     return pd.DataFrame(data_list)


# def plot_variance_diagnostics(df):
#     if df.empty:
#         return

#     # Automatically find the largest map in the dataset to focus the diagnosis
#     max_map = df['map_size'].max()
#     df_large = df[df['map_size'] == max_map]

#     if df_large.empty:
#         print(f"No data found for the largest map ({max_map}m).")
#         return

#     fig = plt.figure(figsize=(20, 8))
#     gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.2])
    
#     ax_std = fig.add_subplot(gs[0, 0])
#     ax_box = fig.add_subplot(gs[0, 1])

#     # -------------------------------------------------------
#     # 1. Standard Deviation of Recall (Focusing on sparse victims)
#     # -------------------------------------------------------
#     # We only care about 5, 10, and 15 victims for this plot
#     target_victims = [5, 10, 15]
#     colors = {5: '#d7191c', 10: '#fdae61', 15: '#2c7bb6'}
    
#     for vics in target_victims:
#         sub_uni = df_large[(df_large['num_victims'] == vics) & (df_large['mode'] == 'Uniform')]
#         if not sub_uni.empty:
#             stats_uni = sub_uni.groupby('num_drones')['recall'].std().reset_index()
#             ax_std.plot(stats_uni['num_drones'].values, (stats_uni['recall'] * 100).values, 
#                         label=f'{vics} Victims (Uniform)', color=colors[vics], linestyle='-', marker='o', linewidth=2)

#         sub_mix = df_large[(df_large['num_victims'] == vics) & (df_large['mode'] == 'Mixed')]
#         if not sub_mix.empty:
#             stats_mix = sub_mix.groupby('num_drones')['recall'].std().reset_index()
#             ax_std.plot(stats_mix['num_drones'].values, (stats_mix['recall'] * 100).values, 
#                         label=f'{vics} Victims (Mixed)', color=colors[vics], linestyle='--', marker='s', linewidth=2)

#     ax_std.set_title(f"A) Volatility of Search on {max_map}m Map\n(Standard Deviation of Recall)", fontsize=16, fontweight='bold')
#     ax_std.set_ylabel("Standard Deviation of Recall (%)", fontsize=14)
#     ax_std.set_xlabel("Swarm Size (Number of UAVs)", fontsize=14)
#     ax_std.legend(title="Scenario & Strategy", fontsize=11)
#     ax_std.grid(True, linestyle='--', alpha=0.6)

#     # -------------------------------------------------------
#     # 2. Boxplot of Raw Runs: The "Boom or Bust" check
#     # -------------------------------------------------------
#     # Filter for the absolute hardest scenario: Largest map, exactly 5 victims
#     df_hardest = df_large[df_large['num_victims'] == 5].copy()
    
#     if not df_hardest.empty:
#         # Convert recall to percentage for the boxplot
#         df_hardest['recall_pct'] = df_hardest['recall'] * 100
        
#         sns.boxplot(
#             data=df_hardest, 
#             x='num_drones', 
#             y='recall_pct', 
#             hue='mode', 
#             ax=ax_box, 
#             palette={'Uniform': '#ff9999', 'Mixed': '#99ccff'},
#             boxprops=dict(alpha=0.6),
#             showfliers=False # We will overlay the actual points instead
#         )
        
#         # Overlay the raw data points so we can literally see the "Lucky Spawns"
#         sns.stripplot(
#             data=df_hardest, 
#             x='num_drones', 
#             y='recall_pct', 
#             hue='mode', 
#             ax=ax_box,
#             dodge=True, 
#             alpha=0.7, 
#             linewidth=1,
#             palette={'Uniform': '#cc0000', 'Mixed': '#0055cc'}
#         )
        
#         # Fix the legend so it doesn't duplicate
#         handles, labels = ax_box.get_legend_handles_labels()
#         ax_box.legend(handles[:2], ['Uniform', 'Mixed'], title="Strategy", loc='upper left', fontsize=12)

#         ax_box.set_title(f"B) Raw Run Distribution: The \"Boom or Bust\" Effect\n({max_map}m Map, 5 Victims)", fontsize=16, fontweight='bold')
#         ax_box.set_ylabel("Raw Recall for Individual Trials (%)", fontsize=14)
#         ax_box.set_xlabel("Swarm Size (Number of UAVs)", fontsize=14)
#         ax_box.grid(True, linestyle='--', alpha=0.6)
#     else:
#         ax_box.text(0.5, 0.5, "No data for 5 victims on the largest map.", ha='center', va='center')

#     plt.tight_layout()
#     output_path = "large_test_variance_diagnostic.png"
#     plt.savefig(output_path, dpi=300)
#     print(f"\nDiagnostic plot saved to {output_path}")
#     plt.show()

# # --- MAIN ---
# if __name__ == "__main__":
#     df = load_large_test_data(LOGS_DIR)
    
#     if not df.empty:
#         plot_variance_diagnostics(df)