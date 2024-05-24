import sys
import os
import time
import numpy as np
sys.path.append(os.getcwd())
try:
    import pySdsDroneSdk as sdk  # Import all of the pybind11 cpp functions
except ImportError:
    print("Failed to import pySdsDroneSdk.pyd (or pySdsDroneSdk.so on Linux). Make sure that SdsDroneSdk.dll and pySdsDroneSdk.pyd are in the same folder as this file")

version = sdk.getVersion()
print(version.getString())

droneManager = sdk.createDroneTrackingManager()

droneManager.registerMessageCallback(lambda message: print(message.message))

def relative_angle_callback(col):
    for ra in col:
        print(f'id: {ra.id} ang_x: {ra.x_angle * 180.0 / np.pi:.2f} ang_z: {ra.z_angle * 180.0 / np.pi:.2f}')

droneManager.registerRelativeAngleCallback(lambda col: relative_angle_callback(col))

net_config = sdk.NetworkConfig()
net_config.sensor_id = 54
net_config.sensor_network_name = 'Ethernet'

net_success = droneManager.initializeNetwork(net_config)
if not net_success:
    print('Network setup failed')
else:
    print('Network setup success')

config = sdk.ConfigRelativeAngle()

init_success = droneManager.initializeRelativeAngle(config)
if not init_success:
    print('Initialization failed')
else:
    print('Initialization success')

droneManager.setTrackingMode(sdk.TrackingMode.RelativeAngle)

time.sleep(10)

droneManager.setTrackingMode(sdk.TrackingMode.TrackingOFF)
