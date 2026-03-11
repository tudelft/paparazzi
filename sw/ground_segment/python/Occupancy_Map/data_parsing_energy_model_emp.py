# import numpy as np
# import matplotlib.pyplot as plt
# import os

# script_dir = os.path.dirname(os.path.abspath(__file__))
# log_file = os.path.abspath(os.path.join(script_dir, '../../../../var/logs/26_02_23__11_35_33.data'))

# energy_time = []
# power_w = []

# fp_time = []
# speed_ms = []

# # 1. Parse the log file
# with open(log_file, 'r') as f:
#     for line in f:
#         parts = line.strip().split()
#         if len(parts) < 4:
#             continue
            
#         try:
#             timestamp = float(parts[0])
#             msg_type = parts[2]
            
#             # Extract instantaneous power (Voltage * Current)
#             if msg_type == 'ENERGY':
#                 voltage = float(parts[4])
#                 current = float(parts[5])
#                 power = voltage * current
                
#                 energy_time.append(timestamp)
#                 power_w.append(power)
                
#             # Extract horizontal velocity
#             elif msg_type == 'ROTORCRAFT_FP':
#                 veast = float(parts[6]) / 524288.0
#                 vnorth = float(parts[7]) / 524288.0
#                 speed = np.sqrt(veast**2 + vnorth**2)
                
#                 fp_time.append(timestamp)
#                 speed_ms.append(speed)
                
#         except (ValueError, IndexError):
#             continue

# # 2. Synchronize timestamps
# aligned_speed = np.interp(energy_time, fp_time, speed_ms)

# # 3. Apply your custom filters
# filtered_speed = []
# filtered_power = []

# for s, p in zip(aligned_speed, power_w):
#     # Keep only data where speed is up to 15 m/s and power is at least 20 W
#     if s <= 15.0 and p >= 30.0:
#         filtered_speed.append(s)
#         filtered_power.append(p)

# # 4. Plot the results
# plt.figure(figsize=(10, 6))
# plt.scatter(filtered_speed, filtered_power, alpha=0.7, c='crimson', edgecolors='k')

# # Formatting the plot
# plt.title('Filtered: Power Consumed vs. Airspeed\n(Airspeed $\leq$ 15 m/s, Power $\geq$ 20 W)', fontsize=14)
# plt.xlabel('Horizontal Speed (Proxy for Airspeed) [m/s]', fontsize=12)
# plt.ylabel('Electrical Power $P_{elec}$ [W]', fontsize=12)
# plt.grid(True, linestyle='--', alpha=0.7)

# # Show data point reduction info
# textstr = f'Original Data Points: {len(power_w)}\nFiltered Points: {len(filtered_speed)}'
# plt.gca().text(0.05, 0.95, textstr, transform=plt.gca().transAxes, fontsize=11,
#                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

# plt.tight_layout()
# plt.savefig('energy_vs_airspeed_filtered.png')


import numpy as np
import matplotlib.pyplot as plt
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
log_file = os.path.abspath(os.path.join(script_dir, '../../../../var/logs/26_03_06__16_15_17.data'))

# energy_time = []
# power_w = []

# fp_time = []
# speed_ms = []
# vup_ms = []

# with open(log_file, 'r') as f:
#     for line in f:
#         parts = line.strip().split()
#         if len(parts) < 4: continue
            
#         try:
#             timestamp = float(parts[0])
#             msg_type = parts[2]
            
#             if msg_type == 'ENERGY':
#                 voltage = float(parts[4])
#                 current = float(parts[5])
#                 power = voltage * current
#                 energy_time.append(timestamp)
#                 power_w.append(power)
                
#             elif msg_type == 'ROTORCRAFT_FP':
#                 veast = float(parts[6]) / 524288.0
#                 vnorth = float(parts[7]) / 524288.0
#                 vup = float(parts[8]) / 524288.0
#                 speed = np.sqrt(veast**2 + vnorth**2)
                
#                 fp_time.append(timestamp)
#                 speed_ms.append(speed)
#                 vup_ms.append(vup)
                
#         except (ValueError, IndexError):
#             continue

# # Convert to numpy arrays for vector math
# time_arr = np.array(fp_time)
# speed_arr = np.array(speed_ms)

# # 1. UPSAMPLING Power
# upsampled_power = np.interp(time_arr, energy_time, power_w)

# # 2. CALCULATE HORIZONTAL ACCELERATION (Delta V / Delta T)
# # np.gradient handles the boundaries and calculating the rate of change
# dt = np.gradient(time_arr)
# accel_arr = np.gradient(speed_arr) / dt 

# filtered_speed = []
# filtered_power = []

# for s, p, vz, a in zip(speed_arr, upsampled_power, vup_ms, accel_arr):
#     # THE STRICT EQUILIBRIUM FILTER:
#     # 1. Speed <= 15 m/s
#     # 2. Power >= 30 W
#     # 3. Vertical Velocity essentially zero (no climbing/descending)
#     # 4. Horizontal Acceleration essentially zero (steady cruise only!)
#     if s <= 20.0 and p >= 40.0 and abs(vz) <= 0.2 and abs(a) <= 5.0:
#         filtered_speed.append(s)
#         filtered_power.append(p)

# # Plot the physically correct results
# plt.figure(figsize=(10, 6))
# plt.scatter(filtered_speed, filtered_power, alpha=0.5, c='darkorchid', edgecolors='k', s=25)

# plt.title('Strict Equilibrium Power vs. Airspeed\n(Accel $\leq 2.0 \ m/s^2$, $V_z \leq 0.5 \ m/s$, Power $\geq 30$ W)', fontsize=13)
# plt.xlabel('Horizontal Speed [m/s]', fontsize=12)
# plt.ylabel('Electrical Power $P_{elec}$ [W]', fontsize=12)
# plt.grid(True, linestyle='--', alpha=0.7)

# textstr = f'Strict Steady-State Points: {len(filtered_speed)}'
# plt.gca().text(0.05, 0.95, textstr, transform=plt.gca().transAxes, fontsize=11,
#                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

# plt.tight_layout()
# plt.savefig('energy_vs_airspeed_strict_equilibrium.png')


# log_file = '26_03_06__16_15_17.data'

energy_time = []
power_w = []

fp_time = []
speed_ms = []
vup_ms = []

# 1. Parse the log file
with open(log_file, 'r') as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) < 4: continue
            
        try:
            timestamp = float(parts[0])
            msg_type = parts[2]
            
            if msg_type == 'ENERGY':
                voltage = float(parts[4])
                current = float(parts[5])
                power = voltage * current
                energy_time.append(timestamp)
                power_w.append(power)
                
            elif msg_type == 'ROTORCRAFT_FP':
                veast = float(parts[6]) / 524288.0
                vnorth = float(parts[7]) / 524288.0
                vup = float(parts[8]) / 524288.0
                speed = np.sqrt(veast**2 + vnorth**2)
                
                fp_time.append(timestamp)
                speed_ms.append(speed)
                vup_ms.append(vup)
                
        except (ValueError, IndexError):
            continue

time_arr = np.array(fp_time)
speed_arr = np.array(speed_ms)

# 2. UPSAMPLE POWER
upsampled_power = np.interp(time_arr, energy_time, power_w)

# 3. SMOOTH THE SPEED DATA (Moving Average to remove sensor noise)
window_size = 10
smoothed_speed = np.convolve(speed_arr, np.ones(window_size)/window_size, mode='same')

# 4. CALCULATE HORIZONTAL ACCELERATION on the SMOOTHED speed
dt = np.gradient(time_arr)
accel_arr = np.gradient(smoothed_speed) / dt 

filtered_speed = []
filtered_power = []

# 5. APPLY STRICT EQUILIBRIUM FILTERS
for s, p, vz, a in zip(speed_arr, upsampled_power, vup_ms, accel_arr):
    if s <= 20.0 and p >= 60.0 and abs(vz) <= 0.2 and abs(a) <= 0.2:
        filtered_speed.append(s)
        filtered_power.append(p)

# Convert to numpy arrays for polynomial fitting
x_data = np.array(filtered_speed)
y_data = np.array(filtered_power)

# 6. FIT A 2ND DEGREE POLYNOMIAL
# Returns coefficients [a, b, c] for equation: y = a*x^2 + b*x + c
coeffs = np.polyfit(x_data, y_data, 2)
p_func = np.poly1d(coeffs)

# Generate smooth points for plotting the fitted line
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = p_func(x_fit)

# 7. PLOT THE RESULTS
plt.figure(figsize=(10, 6))

# Scatter plot of the underlying clean data points
plt.scatter(x_data, y_data, alpha=0.3, c='teal', edgecolors='k', s=30, label='Empirical Flight Data')

# Plot the 2nd degree polynomial curve on top
eq_str = f'$P(v) = {coeffs[0]:.3f}v^2 {coeffs[1]:+.3f}v {coeffs[2]:+.3f}$'
plt.plot(x_fit, y_fit, color='crimson', linewidth=3, label=f'2nd Degree Fit\n{eq_str}')

# Formatting
plt.title('Empirical Aerodynamic Power Curve\n(2nd Degree Polynomial Fit)', fontsize=14)
plt.xlabel('Horizontal Airspeed [m/s]', fontsize=12)
plt.ylabel('Electrical Power $P_{elec}$ [W]', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc='lower right', fontsize=12)

plt.tight_layout()
plt.show()