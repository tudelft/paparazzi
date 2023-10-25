import pymap3d
import numpy as np


lat_drone = 1
lon_drone = 1
alt_drone = 1

pitch_drone = 1
roll_drone = 1
yaw_drone = 1

# STEP 1: Geodetic to ECEF coordinates drone 

ECEF_drone = pymap3d.geodetic2ecef(lat_drone, lon_drone, alt_drone, ell=None, deg=False) # returns X, Y, Z

# STEP 2: Define rotation matrix (pitch, roll, yaw)

def rotation_matrix_yaw_pitch_roll(yaw, pitch, roll):

    # Yaw (rotation around z-axis)
    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Pitch (rotation around y-axis)
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Roll (rotation around x-axis)
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R_combined = R_yaw @ R_pitch @ R_roll

    return R_combined # should output the new x, y, z to be introduced into rel_to_ECEF 


# STEP 3: convert ArUco's relative coordinates (x, y, z) into ECEF coordinates

def rel_to_ECEF(X,x,Y,y,Z,z):
    X_0 = X + x
    Y_0 = Y + y 
    Z_0 = Z + z 

    return X_0, Y_0, Z_0 #ECEF of the aruco marker

# STEP 3: ArUco ECEF coordinates into Geodetic (long and lat)

geo_aruco = pymap3d.ecef2geodetic(X_0, Y_0, Z_0, ell=None, deg=False) # returns lat, long, alt of Aruco Marker

#STEP 4: Take the difference between geodetic measurements to find location of aruco in geodetic plane

def aruco_NED(lat_drone, lon_drone, alt_drone, lat_aruco, lon_aruco, alt_aruco):
    delta_phi = lat_aruco - lat_drone
    delta_lambda = lon_aruco - lon_drone
    delta_h = alt_aruco - alt_drone

    return delta_phi, delta_lambda, delta_h


# STEP 5: Defining North East Down from difference in latitudes, longitudes, and altitudes (aruco_NED function)

def geodetic_to_NED_aruco(delta_phi, delta_lambda, delta_h, lat_drone):
    N = delta_phi
    E = delta_lambda*np.cos(lat_drone)
    D = -delta_h

    return N, E, D


# The 