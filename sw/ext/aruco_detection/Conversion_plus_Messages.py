import numpy as np
import pymap3d
import sys
import math

# --------- Ivybus Specific ---------- #
sys.path.append("/home/sergio/paparazzi/sw/ext/pprzlink/lib/v2.0/python/")

from ivy.std_api import *
import pprzlink.ivy
import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message

pitch_values = None                   # Global variable to store Ivybus received pitch values
roll_values = None                    # Global variable to store Ivybus received roll values
yaw_values = None                     # Global variable to store Ivybus received yaw values
lat_values = None
long_values = None
alt_values = None
N_drone_values = None
E_drone_values = None
D_drone_values = None

conversion_attitude = 0.0139882
conversion_geodetic = 0.0000001
conversion_NED = 0.0039063

                                  # FUNCTIONS -> IVYBUS MESSAGES #
# ------------------------------------------------------------------------------------------------------- #
# --------- Bind to Drone Attitude and Location Message --------- # 
def attitude_callback(ac_id, pprzMsg):
    global pitch_values
    global roll_values
    global yaw_values
    
    pitch = pprzMsg['theta']
    roll  = pprzMsg['phi']
    yaw   = pprzMsg['psi']
    pitch_values = pitch
    roll_values  = roll
    yaw_values   = yaw

def geodetic_callback(ac_id, pprzMsg):
    global lat_values
    global long_values
    global alt_values
    
    lat = pprzMsg['lat']
    long = pprzMsg['lon']
    alt = pprzMsg['alt']
    lat_values = lat
    long_values  = long
    alt_values   = alt

def NED_drone_callback(ac_id, pprzMsg):
    global N_drone_values 
    global E_drone_values
    global D_drone_values

    N_drone = pprzMsg['north']
    E_drone = pprzMsg['east']
    D_drone = pprzMsg['up']  
    N_drone_values = N_drone
    E_drone_values = E_drone
    D_drone_values = D_drone
    


    # --------- Get Attitude and Location Values --------- # 
def get_attitude_values():
    global pitch_values
    global roll_values
    global yaw_values
    return pitch_values, roll_values, yaw_values

def get_geodetic_values():
    global lat_values
    global long_values
    global alt_values
    return lat_values, long_values, alt_values

def get_NED_drone_values():
    global N_drone_values
    global E_drone_values
    global D_drone_values
    return N_drone_values, E_drone_values, D_drone_values
                                  # FUNCTIONS -> NED COORDINATES CONVERSION #
# ------------------------------------------------------------------------------------------------------- #
def ned_conversion(pitch, roll, yaw, aruco_position):
    # --------- Rotation Matrix (Rotation around Z-Axis/Yaw) ------- # 
    RX= np.array([
                 [np.cos(yaw), -np.sin(yaw), 0],
                 [np.sin(yaw), np.cos(yaw),  0],
                 [          0,           0,  1]])

    # --------- Rotation Matrix (Rotation around Y-Axis/Pitch) ------- # 
    RY = np.array([
                  [ np.cos(pitch),  0, np.sin(pitch)],
                  [             0,  1,             0],
                  [-np.sin(pitch),  0, np.cos(pitch)]])

    # --------- Rotation Matrix (Rotation around X-Axis/Roll) ------- # 
    RZ = np.array([
                  [1,            0,             0],
                  [0, np.cos(roll), -np.sin(roll)],
                  [0, np.sin(roll),  np.cos(roll)]])

    # --------- Rotation Matrix ------- # 
    R = RZ @ RY @ RX 

    # --------- Obtain NED Coordinates ------- # 
    ned_vector = np.dot(R, aruco_position)
    north, east, down = ned_vector.squeeze()

    return north, east, down



                                 # FUNCTION -> MOVE WAYPOINT and asks which AIRFRAME is being used #
# ------------------------------------------------------------------------------------------------------- #
# def move_waypoint(ac_id, wp_id, NORTH, EAST, DOWN):
#     msg = message.PprzMessage("ground", "MOVE_WAYPOINT")
#     msg['ac_id'] = ac_id
#     msg['wp_id'] = wp_id
#     msg['x'] = NORTH
#     msg['y'] = EAST
#     msg['z'] = DOWN
#     print("Sending message: %s" % msg)
#     ivy.send(msg)
#     time.sleep(0.5)

# ac_id = input("What Aicraft ID is it being used: ")



                                      # Ivybus INITIALISATION #
# ------------------------------------------------------------------------------------------------------- #
# --------- Create Ivy Interface --------- # 
ivy = pprzlink.ivy.IvyMessagesInterface(agent_name="ArucoMarker", start_ivy=False, ivy_bus="127.255.255.255:2010")

# --------- Start Ivy Interface --------- # 
ivy.start()

# --------- Subscribe to Ivy Messages --------- # 
ivy.subscribe(attitude_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))
ivy.subscribe(geodetic_callback, message.PprzMessage("telemetry", "GPS_INT"))
ivy.subscribe(NED_drone_callback, message.PprzMessage("telemetry", "ROTORCRAFT_FP"))

                                            # RUN MAIN LOOP #
# ------------------------------------------------------------------------------------------------------- #

pitch, roll, yaw = get_attitude_values()
lat, long, alt = get_geodetic_values()
N_drone, E_drone, D_drone = get_NED_drone_values()

while pitch is None and lat is None and N_drone is None:
    pitch, roll, yaw = get_attitude_values()
    lat, long, alt = get_geodetic_values()
    N_drone, E_drone, D_drone = get_NED_drone_values()


while True:

    # Obtaining pitch, roll, yaw, and latitude, longitude, altitutde and North, East, Down from the Drone

    pitch, roll, yaw = get_attitude_values()

    pitch = float(pitch)
    roll  = float(roll)
    yaw   = float(yaw)

    pitch = pitch*conversion_attitude
    roll = roll*conversion_attitude
    yaw = yaw*conversion_attitude

    pitch = math.radians(pitch)
    roll  = math.radians(roll)
    yaw   = math.radians(yaw)

    lat, long, alt = get_geodetic_values()

    lat = float(lat)
    long = float(long)
    alt = float(alt)

    lat = lat*conversion_geodetic
    long = long*conversion_geodetic
    alt = alt*conversion_geodetic

    N_drone, E_drone, D_drone = get_NED_drone_values()

    N_drone = float(N_drone)
    E_drone = float(E_drone)
    D_drone = float(D_drone)

    N_drone = N_drone*conversion_NED
    E_drone = E_drone*conversion_NED
    D_drone = D_drone*conversion_NED

    print("Pitch: ", pitch)
    print("Roll: ", roll)
    print("Yaw: ", yaw)
    print("")
    print("Latitude: ", lat)
    print("Longitude: ", long)
    print("Altitude ", alt)
    print("")
    print("North Drone: ", N_drone)
    print("East Drone: ", E_drone)
    print("Down Drone: ", D_drone)
    print("")
    
    # Definition of Aruco Marker Location (Camera Axis)
    X = 10
    Y = 10
    Z = 13

    # Definition of Aruco Marker Location (Body Axis)
    X_B = Y
    Y_B = -X
    Z_B = Z

    aruco_position = np.array([[X_B], [Y_B], [Z_B]])

    # Aruco Marker Location in NED Axis
    NORTH_aruco, EAST_aruco, DOWN_aruco = ned_conversion(pitch, roll, yaw, aruco_position)

    # Negative in D_drone is because the message sends up instead of down

    NORTH = N_drone + NORTH_aruco
    EAST = E_drone + EAST_aruco
    DOWN = -D_drone + DOWN_aruco

    # Aurco Marker Total Location in Geodetic Form (this is the waypoint message!) 
    # This is the Multiplication part that you already did and should output Lat, Long, Alt

    print("Message to be sent to drone")
    print('Latitude: ', lat_aruco)
    print('Longitude: ', long_aruco)
    print("Altitude: ", alt_aruco)


