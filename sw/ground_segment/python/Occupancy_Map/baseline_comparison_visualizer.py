import pandas as pd
import matplotlib.pyplot as plt
import os

# --- 1. Load the Data ---
csv_file = "phase2_lambda_sweep_290deg/lambda_summary.csv"
if not os.path.exists(csv_file):
    print(f"Error: {csv_file} not found!")
    exit()

df = pd.read_csv(csv_file)

# Calculate the mean metrics for each Strategy/Lambda combo
summary_df = df.groupby(['Condition', 'Strategy', 'Lambda']).agg({
    'Entropy_Reduced_Pct': 'mean',
    'Avg_Battery_Remaining': 'mean',
    'Sim_Time_To_90': 'mean'
}).reset_index()

# Separate Baseline B from the Lambda sweep
df_baseline = summary_df[summary_df['Strategy'] == 'baseline_b']
df_lambda = summary_df[summary_df['Strategy'] == 'proposed'].copy()

# Sort lambda values numerically so the X-axis plots in order
df_lambda['Lambda'] = pd.to_numeric(df_lambda['Lambda'])
df_lambda = df_lambda.sort_values('Lambda')
df_lambda['Lambda_str'] = df_lambda['Lambda'].astype(str)

base_red = df_baseline['Entropy_Reduced_Pct'].values[0]
base_batt = df_baseline['Avg_Battery_Remaining'].values[0]

# --- Extract as pure Numpy Arrays to fix Matplotlib ValueError ---
x_labels = df_lambda['Lambda_str'].to_numpy()
y_coverage = df_lambda['Entropy_Reduced_Pct'].to_numpy()
y_battery = df_lambda['Avg_Battery_Remaining'].to_numpy()

# --- 2. Plot 1: Dual Axis Sensitivity Line Plot ---
plt.style.use('ggplot')
fig, ax1 = plt.subplots(figsize=(10, 6))

color1 = 'tab:blue'
ax1.set_xlabel('Energy Penalty Weight ($\lambda$)', fontsize=12, fontweight='bold')
ax1.set_ylabel('Entropy Reduced (%)', color=color1, fontsize=12, fontweight='bold')
# Pass the numpy arrays here
ax1.plot(x_labels, y_coverage, marker='o', markersize=8, color=color1, linewidth=2, label='Proposed (Coverage)')
ax1.axhline(y=base_red, color=color1, linestyle='--', alpha=0.6, label='Baseline B (Coverage)')
ax1.tick_params(axis='y', labelcolor=color1)
ax1.set_ylim(30, 105)

ax2 = ax1.twinx()
color2 = 'tab:red'
ax2.set_ylabel('Avg Battery Remaining (%)', color=color2, fontsize=12, fontweight='bold')
# Pass the numpy arrays here
ax2.plot(x_labels, y_battery, marker='s', markersize=8, color=color2, linewidth=2, label='Proposed (Battery)')
ax2.axhline(y=base_batt, color=color2, linestyle='--', alpha=0.6, label='Baseline B (Battery)')
ax2.tick_params(axis='y', labelcolor=color2)
ax2.set_ylim(40, 100)

# Combine legends
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2)

fig.suptitle('Entropy Reduction vs. Battery Conservation (290 deg wind angle)', fontsize=14, fontweight='bold')
fig.tight_layout()
plt.savefig('phase2_lambda_sweep_290deg/lambda_sensitivity.png', bbox_inches='tight', dpi=300)
print("Saved Sensitivity Line Plot!")

# --- 3. Plot 2: Pareto Frontier Scatter Plot ---
plt.figure(figsize=(10, 6))

# Plot all Lambda variations using numpy arrays
plt.scatter(y_battery, y_coverage, c='blue', s=100, label='Proposed ($\lambda$ variations)')

# Add text labels next to the dots
for i, row in df_lambda.iterrows():
    plt.annotate(f"$\lambda$={row['Lambda_str']}", (row['Avg_Battery_Remaining'] + 0.3, row['Entropy_Reduced_Pct'] + 0.5), fontsize=10, fontweight='bold')

# Plot Baseline B
plt.scatter([base_batt], [base_red], c='red', marker='X', s=200, label='Baseline B')

# Draw the 80% Success Line
plt.axhline(80, color='gray', linestyle=':', label='80% Success Threshold')

plt.title('Performance Frontier: Battery Remaining vs. Coverage', fontsize=14, fontweight='bold')
plt.xlabel('Avg Battery Remaining (%)', fontsize=12, fontweight='bold')
plt.ylabel('Entropy Reduced (%)', fontsize=12, fontweight='bold')
plt.legend(loc='lower left')
plt.grid(True)
plt.savefig('phase2_lambda_sweep_290deg/pareto_frontier.png', bbox_inches='tight', dpi=300)
print("Saved Pareto Frontier Plot!")


# import pandas as pd
# import matplotlib.pyplot as plt
# import os

# # --- 1. Load the Data ---
# csv_file = "phase2_lambda_sweep_310deg/lambda_summary.csv"
# if not os.path.exists(csv_file):
#     print(f"Error: {csv_file} not found!")
#     exit()

# df = pd.read_csv(csv_file)

# # --- FILTER FOR SUCCESSFUL TRIALS ONLY ---
# # This removes any rows where the trial ended in a TIMEOUT
# df = df[df['Status'] == 'SUCCESS']

# if df.empty:
#     print("Error: No successful trials found in the dataset!")
#     exit()
# # -----------------------------------------

# # Calculate the mean metrics for each Strategy/Lambda combo
# summary_df = df.groupby(['Condition', 'Strategy', 'Lambda']).agg({
#     'Entropy_Reduced_Pct': 'mean',
#     'Avg_Battery_Remaining': 'mean',
#     'Sim_Time_To_90': 'mean'
# }).reset_index()

# # Separate Baseline B from the Lambda sweep
# df_baseline = summary_df[summary_df['Strategy'] == 'baseline_b']
# df_lambda = summary_df[summary_df['Strategy'] == 'proposed'].copy()

# # Sort lambda values numerically so the X-axis plots in order
# df_lambda['Lambda'] = pd.to_numeric(df_lambda['Lambda'])
# df_lambda = df_lambda.sort_values('Lambda')
# df_lambda['Lambda_str'] = df_lambda['Lambda'].astype(str)

# base_red = df_baseline['Entropy_Reduced_Pct'].values[0] if not df_baseline.empty else 0
# base_batt = df_baseline['Avg_Battery_Remaining'].values[0] if not df_baseline.empty else 0

# # --- Extract as pure Numpy Arrays to fix Matplotlib ValueError ---
# x_labels = df_lambda['Lambda_str'].to_numpy()
# y_coverage = df_lambda['Entropy_Reduced_Pct'].to_numpy()
# y_battery = df_lambda['Avg_Battery_Remaining'].to_numpy()

# # Ensure output directory exists
# os.makedirs('phase2_lambda_sweep_310deg', exist_ok=True)

# # --- 2. Plot 1: Dual Axis Sensitivity Line Plot ---
# plt.style.use('ggplot')
# fig, ax1 = plt.subplots(figsize=(10, 6))

# color1 = 'tab:blue'
# ax1.set_xlabel('Energy Penalty Weight ($\lambda$)', fontsize=12, fontweight='bold')
# ax1.set_ylabel('Entropy Reduced (%)', color=color1, fontsize=12, fontweight='bold')
# # Pass the numpy arrays here
# ax1.plot(x_labels, y_coverage, marker='o', markersize=8, color=color1, linewidth=2, label='Proposed (Coverage)')

# if not df_baseline.empty:
#     ax1.axhline(y=base_red, color=color1, linestyle='--', alpha=0.6, label='Baseline B (Coverage)')

# ax1.tick_params(axis='y', labelcolor=color1)
# ax1.set_ylim(30, 105)

# ax2 = ax1.twinx()
# color2 = 'tab:red'
# ax2.set_ylabel('Avg Battery Remaining (%)', color=color2, fontsize=12, fontweight='bold')
# # Pass the numpy arrays here
# ax2.plot(x_labels, y_battery, marker='s', markersize=8, color=color2, linewidth=2, label='Proposed (Battery)')

# if not df_baseline.empty:
#     ax2.axhline(y=base_batt, color=color2, linestyle='--', alpha=0.6, label='Baseline B (Battery)')

# ax2.tick_params(axis='y', labelcolor=color2)
# ax2.set_ylim(40, 100)

# # Combine legends
# lines_1, labels_1 = ax1.get_legend_handles_labels()
# lines_2, labels_2 = ax2.get_legend_handles_labels()
# ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2)

# fig.suptitle('Sensitivity Analysis: Entropy Reduction vs. Battery Conservation\n(Successful Trials Only)', fontsize=14, fontweight='bold')
# fig.tight_layout()
# plt.savefig('phase2_lambda_sweep_310deg/lambda_sensitivity_success.png', bbox_inches='tight', dpi=300)
# print("Saved Sensitivity Line Plot!")

# # --- 3. Plot 2: Pareto Frontier Scatter Plot ---
# plt.figure(figsize=(10, 6))

# # Plot all Lambda variations using numpy arrays
# plt.scatter(y_battery, y_coverage, c='blue', s=100, label='Proposed ($\lambda$ variations)')

# # Add text labels next to the dots
# for i, row in df_lambda.iterrows():
#     plt.annotate(f"$\lambda$={row['Lambda_str']}", (row['Avg_Battery_Remaining'] + 0.3, row['Entropy_Reduced_Pct'] + 0.5), fontsize=10, fontweight='bold')

# # Plot Baseline B
# if not df_baseline.empty:
#     plt.scatter([base_batt], [base_red], c='red', marker='X', s=200, label='Baseline B')

# # Draw the 80% Success Line (Updated from 90% based on your previous messages)
# plt.axhline(80, color='gray', linestyle=':', label='80% Success Threshold')

# plt.title('Battery Remaining vs. Coverage\n(Successful Trials Only and 310 deg wind angle)', fontsize=14, fontweight='bold')
# plt.xlabel('Avg Battery Remaining (%)', fontsize=12, fontweight='bold')
# plt.ylabel('Entropy Reduced (%)', fontsize=12, fontweight='bold')
# plt.legend(loc='lower left')
# plt.grid(True)
# plt.savefig('phase2_lambda_sweep_310deg/pareto_frontier_succes.png', bbox_inches='tight', dpi=300)
# print("Saved Pareto Frontier Plot!")