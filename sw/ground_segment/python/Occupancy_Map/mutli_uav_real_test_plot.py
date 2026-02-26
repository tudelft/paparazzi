import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 1. Construct the path to the log file ---
# Script location: paparazzi/sw/ground_segment/python/Occupancy_Map/
# Log location:    paparazzi/var/logs/
script_dir = os.path.dirname(os.path.abspath(__file__))
filename = os.path.abspath(os.path.join(script_dir, '../../../../var/logs/26_02_23__11_35_33.data'))

# --- 2. Find t_max_east for AC 123 strictly between 960s and 1020s ---
max_east_123 = -float('inf')
t_max_east_123 = 960

with open(filename, 'r') as f:
    for line in f:
        if 'ROTORCRAFT_FP' in line:
            parts = line.split()
            ac_id = parts[1]
            t = float(parts[0])
            z = float(parts[5]) / 256.0
            
            # Look for max east ONLY within 960 to 1020 seconds and when Z >= 0
            if ac_id == '123' and 960 <= t <= 1020 and z >= 0:
                east = float(parts[3]) / 256.0
                if east > max_east_123:
                    max_east_123 = east
                    t_max_east_123 = t

print(f"Within 960s-1020s, AC 123 max east was {max_east_123}m at t={t_max_east_123}s")

# --- 3. Extract the data points based on our custom constraints ---
data = {'121': {'x': [], 'y': [], 'z': []}, 
        '123': {'x': [], 'y': [], 'z': []},
        '219': {'x': [], 'y': [], 'z': []}}

with open(filename, 'r') as f:
    for line in f:
        if 'ROTORCRAFT_FP' in line:
            parts = line.split()
            ac_id = parts[1]
            
            if ac_id not in data:
                continue
                
            t = float(parts[0])
            z = float(parts[5]) / 256.0
            
            # Filter out negative Z positions universally
            if z < 0:
                continue
                
            # Time constraints per aircraft
            if ac_id in ['121', '219'] and 1018 <= t <= 1030:
                data[ac_id]['x'].append(float(parts[3]) / 256.0)
                data[ac_id]['y'].append(float(parts[4]) / 256.0)
                data[ac_id]['z'].append(z)
            elif ac_id == '123' and 1018 <= t <= 1030:
                data[ac_id]['x'].append(float(parts[3]) / 256.0)
                data[ac_id]['y'].append(float(parts[4]) / 256.0)
                data[ac_id]['z'].append(z)

# --- 4. Plot the 3D Trajectory ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

for ac_id, coords in data.items():
    if coords['x']:  # Only plot if there is data
        ax.plot(coords['x'], coords['y'], coords['z'], label=f'AC {ac_id}', alpha=0.8)

ax.set_xlabel('East (m)')
ax.set_ylabel('North (m)')
ax.set_zlabel('Up (m)')
# ax.set_title(f'3D Trajectory (121/219: 980-1022s | 123: 980-{t_max_east_123:.1f}s | Z >= 0m)')
ax.set_title(f'3D Trajectory (970-1018s)')
ax.legend()

plt.show()