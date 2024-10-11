import sys
import os
import time
import numpy as np
sys.path.append(os.getcwd())
try:
    import pySdsFalcon as sdk  # Import all of the pybind11 cpp functions
except ImportError:
    print("Failed to import pySdsFalcon.pyd (or pySdsFalcon.so on Linux). Make sure that SdsFalcon.dll and pySdsFalcon.pyd are in the same folder as this file")

version = sdk.getVersion()
print(version.getString())

falconManager = sdk.createFalconManager()

falconManager.registerMessageCallback(lambda message: print(message.message))

def pose_6dof_callback(pose):
    print(f'X: {pose.x:.2f} Y: {pose.y:.2f} Z: {pose.z:.2f} Qw: {pose.qw:.2f} Qx: {pose.qx:.2f}  Qy: {pose.qy:.2f}  Qz: {pose.qz:.2f} ')

falconManager.registerPose6DofCallback(lambda col: pose_6dof_callback(col))

net_config = sdk.NetworkConfig()
net_config.sensor_id = 162
net_config.sensor_network_name = 'Ethernet 2'

net_success = falconManager.initializeNetwork(net_config)
if not net_success:
    print('Network setup failed')
else:
    print('Network setup success')

config = sdk.Config6Dof()

b0 = sdk.Beacon()
b0.id = 1089
b0.x = 0.025
b0.y = 0.0
b0.z = 0.025

b1 = sdk.Beacon()
b1.id = 1102
b1.x = 0.025
b1.y = 0.0
b1.z = -0.025

b2 = sdk.Beacon()
b2.id = 1101
b2.x = -0.025
b2.y = 0.0
b2.z = -0.025

b3 = sdk.Beacon()
b3.id = 1096
b3.x = -0.025
b3.y = 0.0
b3.z = 0.025

config.map = [b0, b1, b2, b3]

init_success = falconManager.initialize6Dof(config)
if not init_success:
    print('Initialization failed')
else:
    print('Initialization success')

falconManager.setTrackingMode(sdk.TrackingMode.Sixdof)

time.sleep(10)

falconManager.setTrackingMode(sdk.TrackingMode.TrackingOFF)
