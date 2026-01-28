import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Load the data
try:
    df1 = pd.read_csv('mc_results_final3.csv')
    df2 = pd.read_csv('mc_results_final4.csv')
    df = pd.concat([df1, df2], ignore_index=True)
except Exception as e:
    print(f"Error loading files: {e}")
    df = pd.DataFrame() 

if not df.empty:
    # 1. Data Preprocessing
    df['Energy_Used_kJ'] = df['Energy_Used_J'] / 1000.0
    
    # 2. Setup Visualization Styling
    sns.set_theme(style="whitegrid")
    
    # --- PLOT 1: FLIGHT REGIME DYNAMICS ---
    plt.figure(figsize=(10, 6))
    sns.regplot(data=df, x='Pct_Cruise', y='Energy_Used_kJ', scatter_kws={'alpha':0.6}, line_kws={'color':'red'})
    plt.title('Flight Regime Dynamics: Impact of Cruise Time on Energy')
    plt.xlabel('Percentage of Time in Cruise Mode (%)')
    plt.ylabel('Total Energy Consumed (kJ)')
    plt.savefig('flight_regime_dynamics.png')
    plt.close()

    # --- PLOT 2: ENVIRONMENTAL IMPACT ---
    plt.figure(figsize=(10, 6))
    sns.regplot(data=df, x='Wind_Speed', y='Energy_Used_kJ', scatter_kws={'alpha':0.6}, line_kws={'color':'orange'})
    plt.title('Environmental Impact: Wind Speed vs Energy Consumption')
    plt.xlabel('Wind Speed (m/s)')
    plt.ylabel('Total Energy Consumed (kJ)')
    plt.savefig('wind_impact.png')
    plt.close()

    # --- PLOT 3: SCALABILITY ---
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    sns.boxplot(data=df, x='Drones', y='Time', hue='Victims', ax=axes[0], palette="Set2")
    axes[0].set_title('Scalability: Search Time vs Swarm Size')
    axes[0].set_ylabel('Time to Confirmation (s)')
    
    sns.boxplot(data=df, x='Drones', y='Energy_Used_kJ', hue='Victims', ax=axes[1], palette="Set2")
    axes[1].set_title('Scalability: Total Swarm Energy vs Swarm Size')
    axes[1].set_ylabel('Total Energy (kJ)')
    plt.savefig('scalability_analysis.png')
    plt.close()

    # --- PLOT 4: PARETO FRONT ---
    plt.figure(figsize=(10, 6))
    sns.scatterplot(data=df, x='Time', y='Energy_Used_kJ', hue='Drones', style='Victims', palette="deep", s=100)
    plt.title('Pareto Front: Time vs Energy Trade-off')
    plt.xlabel('Time to Confirmation (s)')
    plt.ylabel('Total Energy (kJ)')
    plt.grid(True)
    plt.savefig('pareto_front.png')
    plt.close()

    # --- PLOT 5: FAILURE ANALYSIS ---
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    # We add hue='Success' and legend=False to fix the FutureWarning
    sns.boxplot(data=df, x='Success', y='Wind_Speed', hue='Success', legend=False, ax=axes[0], palette="coolwarm")
    axes[0].set_title('Wind Speed distribution by Outcome')
    
    sns.boxplot(data=df, x='Success', y='Drift_Speed', hue='Success', legend=False, ax=axes[1], palette="coolwarm")
    axes[1].set_title('Drift Speed distribution by Outcome')
    
    sns.boxplot(data=df, x='Success', y='Total_Aborts', hue='Success', legend=False, ax=axes[2], palette="coolwarm")
    axes[2].set_title('Planner Aborts by Outcome')
    plt.savefig('failure_analysis.png')
    plt.close()

    # --- PLOT 6: SPECIFIC ENERGY EFFICIENCY ---
    df['Specific_Energy'] = df['Energy_Used_J'] / (df['Entropy_Reduction'] + 1e-6)
    plt.figure(figsize=(10, 6))
    sns.boxplot(data=df, x='Drones', y='Specific_Energy', hue='Drones', legend=False, palette="viridis")
    plt.title('Swarm Efficiency: Specific Energy Cost')
    plt.ylabel('Joules per Bit of Information (J/bit)')
    plt.xlabel('Swarm Size')
    plt.yscale('log') 
    plt.savefig('specific_energy.png')
    plt.close()

    # --- PLOT 7: CONFLICT DENSITY (ROBUST FIX) ---
    # We calculate the mean and standard deviation manually to bypass the crash
    agg_df = df.groupby('Drones')['Total_Aborts'].agg(['mean', 'std']).reset_index()
    
    # Extract pure numpy arrays to ensure no Pandas Series are passed to Matplotlib
    x_vals = agg_df['Drones'].to_numpy()
    y_vals = agg_df['mean'].to_numpy()
    y_err = agg_df['std'].to_numpy()

    plt.figure(figsize=(10, 6))
    
    # Plot Mean Line
    plt.plot(x_vals, y_vals, marker='o', linewidth=2.5, color='#4c72b0', label='Mean Aborts')
    
    # Plot Confidence/Variance Band (Mean +/- Std Dev)
    plt.fill_between(x_vals, y_vals - y_err, y_vals + y_err, alpha=0.2, color='#4c72b0', label='Std Dev')
    
    plt.title('Conflict Density Analysis: Abortions vs Number of UAVs')
    plt.xlabel('Number of Drones')
    plt.ylabel('Average Abortions Triggered')
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend()
    plt.savefig('conflict_density.png')
    plt.close()

    # --- STATISTICAL SUMMARY ---
    print("--- DATASET SUMMARY ---")
    print(f"Total Runs: {len(df)}")
    print(f"Success Rate: {df['Success'].mean()*100:.1f}%")
    print(f"Mean Transition %: {df['Pct_Transition'].mean():.4f}")
    
    print("\n--- CORRELATIONS (Pearson, Success Cases Only) ---")
    success_df = df[df['Success'] == True]
    print(success_df[['Energy_Used_kJ', 'Time', 'Pct_Cruise', 'Wind_Speed', 'Total_Aborts', 'Drones']].corr())
    
    print("\n--- FAILURE ANALYSIS (Mean Values) ---")
    print(df.groupby('Success')[['Wind_Speed', 'Drift_Speed', 'Total_Aborts', 'Energy_Used_kJ']].mean())

else:
    print("No data found.")