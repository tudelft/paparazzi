#!/usr/bin/env python3
"""
paparazzi_bridge.py

- Dynamic multi-UAV discovery (ac_id)
- Reads INS + GPS_INT + EXTERNAL_POSE from Paparazzi (PPRZLink)
- Feeds states to SwarmPlanner (multi_uav_new_runtime.py) --> neglect this!
- Sends GUIDED_SETPOINT_NED velocity setpoints
- Per-UAV logs
"""

import os, sys, time, math, csv, threading
from collections import defaultdict
import numpy as np
import pymap3d as pm

# ---------------- Paparazzi / PPRZLink ----------------
PPRZ_HOME = os.getenv(
    "PAPARAZZI_HOME",
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
)
sys.path.append(os.path.join(PPRZ_HOME, "sw/ext/pprzlink/lib/v1.0/python"))

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# ---------------- ENU reference ----------------
LAT0, LON0, ALT0 = 51.990634, 4.376789, 0.0

# ---------------- INS scales ----------------
POS_SCALE = 0.0039063
VEL_SCALE = 0.0000019

# ---------------- Parameters ----------------
LOG_DIR = "logs_bridge"
os.makedirs(LOG_DIR, exist_ok=True)

UAVS = {}
LOCK = threading.Lock()

def ensure_uav(ac_id):
    with LOCK:
        if ac_id in UAVS:
            return UAVS[ac_id]

        st = {"x": None,"y": None,"z": None,"vx":0.0,"vy":0.0,"vz":0.0,"heading":None}
        cmd_prev = np.zeros(3)

        lf = open(os.path.join(LOG_DIR, f"uav_{ac_id:02d}.csv"), "w", newline="")
        w = csv.writer(lf)
        w.writerow(["t","x","y","z","vx","vy","vz","vx_raw","vy_raw","vz_raw","vx_cmd","vy_cmd","vz_cmd"])
        lf.flush()

        UAVS[ac_id] = {"state": st, "cmd_prev": cmd_prev, "log_f": lf, "log_w": w}
        print(f"[BRIDGE] Registered AC{ac_id}")
        return UAVS[ac_id]

def on_gps_int(ac_id, msg):
    ac_id = int(ac_id)
    uav = ensure_uav(ac_id)

    lat = float(msg["lat"]) * 1e-7
    lon = float(msg["lon"]) * 1e-7
    alt = float(msg["alt"]) / 100.0

    x, y, z = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)
    with LOCK:
        uav["state"]["x"] = x
        uav["state"]["y"] = y
        uav["state"]["z"] = z

def on_ins(ac_id, msg):
    ac_id = int(ac_id)
    uav = ensure_uav(ac_id)

    ins_x  = float(msg["ins_x"])
    ins_y  = float(msg["ins_y"])
    ins_z  = float(msg["ins_z"])
    ins_xd = float(msg["ins_xd"])
    ins_yd = float(msg["ins_yd"])
    ins_zd = float(msg["ins_zd"])

    north_m = ins_x * POS_SCALE
    east_m  = ins_y * POS_SCALE
    up_m    = -ins_z * POS_SCALE

    north_v = ins_xd * VEL_SCALE
    east_v  = ins_yd * VEL_SCALE
    up_v    = -ins_zd * VEL_SCALE

    heading = math.atan2(east_v, north_v) if (abs(east_v)>1e-6 or abs(north_v)>1e-6) else None
    heading_deg = math.degrees(heading) if heading is not None else float('nan')

    with LOCK:
        st = uav["state"]
        st["x"], st["y"], st["z"] = east_m, north_m, up_m
        st["vx"], st["vy"], st["vz"] = east_v, north_v, up_v
        st["heading"] = heading

    print(f"INS -> AC{ac_id}: x={east_m:.2f}, y={north_m:.2f}, z={up_m:.2f}")

# --- NEW HANDLER FOR MOCAP DATA ---
def on_external_pose(ac_id, msg):
    ac_id = int(ac_id)
    uav = ensure_uav(ac_id)

    # Extract coordinates directly in meters (No POS_SCALE needed)
    x = float(msg["enu_x"])
    y = float(msg["enu_y"])
    z = float(msg["enu_z"])

    with LOCK:
        uav["state"]["x"] = x
        uav["state"]["y"] = y
        uav["state"]["z"] = z

    # Print the output exactly as requested
    print(f"EXTERNAL_POSE -> AC{ac_id}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

def send_guided(interface, ac_id, vx_enu, vy_enu, vz_enu=0.0):
    vx_ned = vy_enu
    vy_ned = vx_enu
    vz_ned = -vz_enu

    msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
    msg['ac_id'] = int(ac_id)
    msg['flags'] = np.packbits([0,0,0,0,0,1,1,0], bitorder='little')[0].astype(np.uint8)
    msg['x'] = float(vx_ned)
    msg['y'] = float(vy_ned)
    msg['z'] = float(vz_ned)
    msg['yaw'] = 0.0

    interface.send(msg, ac_id=int(ac_id))
    print(f"[SEND] AC{ac_id}: vx_enu={vx_enu:.2f}, vy_enu={vy_enu:.2f}, vz_enu={vz_enu:.2f}")

def on_pprz_msg(ac_id, msg):
    # Route to the correct functions based on the message name
    if msg.name == "GPS_INT":
        on_gps_int(ac_id, msg)
    elif msg.name in ["INS", "INS_EKF2"]:
        on_ins(ac_id, msg)
    elif msg.name == "EXTERNAL_POSE":
        on_external_pose(ac_id, msg)

def manual_test(interface):
    while True:
        send_guided(interface, 121, 10.0, 0.0, 0.0)
        time.sleep(0.2) 

def main():
    ivy_bus = os.getenv("PPRZ_IVY_BUS", "127.255.255.255:2010")
    interface = IvyMessagesInterface("multi_uav_bridge", ivy_bus=ivy_bus)
    
    # We only need to subscribe to our router function now!
    interface.subscribe(on_pprz_msg)     

    interface.start()

    # Commented out to prevent terminal flooding while you test the Mocap read
    # print("[BRIDGE] Running manual velocity test.")
    # manual_test(interface)

    print("[BRIDGE] Listening for Ivy messages. Move the cardboard to test! Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        interface.stop()
        with LOCK:
            for uav in UAVS.values():
                uav["log_f"].close()

if __name__ == "__main__":
    main()

# """
# paparazzi_bridge.py

# - Dynamic multi-UAV discovery (ac_id)
# - Reads INS + GPS_INT from Paparazzi (PPRZLink)
# - Feeds states to SwarmPlanner (multi_uav_new_runtime.py) --> neglect this!
# - Sends GUIDED_SETPOINT_NED velocity setpoints
# - Per-UAV logs
# """

# import os, sys, time, math, csv, threading
# from collections import defaultdict
# import numpy as np
# import pymap3d as pm

# # ---------------- Paparazzi / PPRZLink ----------------
# PPRZ_HOME = os.getenv(
#     "PAPARAZZI_HOME",
#     os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
# )
# sys.path.append(os.path.join(PPRZ_HOME, "sw/ext/pprzlink/lib/v1.0/python"))

# from pprzlink.ivy import IvyMessagesInterface
# from pprzlink.message import PprzMessage

# # ---------------- Planner wrapper ----------------
# # from multi_uav_new_runtime import SwarmPlanner

# # ---------------- ENU reference ----------------
# # LAT0, LON0, ALT0 = 52.1681551, 4.4126468, 0.0
# # cyberzoo:
# LAT0, LON0, ALT0 = 51.990634, 4.376789, 0.0

# # ---------------- INS scales ----------------
# POS_SCALE = 0.0039063
# VEL_SCALE = 0.0000019

# # ---------------- Parameters ----------------
# PLANNER_DT = 0.2       # 5 Hz
# SMOOTHING_ON = True
# MAX_ACCEL = 1.0        # m/s^2 slew limit
# ALPHA_LP = 0.3         # low-pass filter
# MAX_CMD_SPEED = 2.0

# LOG_DIR = "logs_bridge"
# os.makedirs(LOG_DIR, exist_ok=True)

# UAVS = {}
# LOCK = threading.Lock()

# def ensure_uav(ac_id):
#     with LOCK:
#         if ac_id in UAVS:
#             return UAVS[ac_id]

#         st = {"x": None,"y": None,"z": None,"vx":0.0,"vy":0.0,"vz":0.0,"heading":None}
#         cmd_prev = np.zeros(3)

#         lf = open(os.path.join(LOG_DIR, f"uav_{ac_id:02d}.csv"), "w", newline="")
#         w = csv.writer(lf)
#         w.writerow(["t","x","y","z","vx","vy","vz","vx_raw","vy_raw","vz_raw","vx_cmd","vy_cmd","vz_cmd"])
#         lf.flush()

#         UAVS[ac_id] = {"state": st, "cmd_prev": cmd_prev, "log_f": lf, "log_w": w}
#         print(f"[BRIDGE] Registered AC{ac_id}")
#         return UAVS[ac_id]

# def on_gps_int(ac_id, msg):
#     if msg.name != "GPS_INT": return
#     ac_id = int(ac_id)
#     uav = ensure_uav(ac_id)

#     lat = float(msg["lat"]) * 1e-7
#     lon = float(msg["lon"]) * 1e-7
#     alt = float(msg["alt"]) / 100.0

#     x, y, z = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)
#     with LOCK:
#         uav["state"]["x"] = x
#         uav["state"]["y"] = y
#         uav["state"]["z"] = z

# def on_ins(ac_id, msg):
#     if msg.name != "INS":
#         return

#     ac_id = int(ac_id)
#     uav = ensure_uav(ac_id)

#     ins_x  = float(msg["ins_x"])
#     ins_y  = float(msg["ins_y"])
#     ins_z  = float(msg["ins_z"])
#     ins_xd = float(msg["ins_xd"])
#     ins_yd = float(msg["ins_yd"])
#     ins_zd = float(msg["ins_zd"])

#     north_m = ins_x * POS_SCALE
#     east_m  = ins_y * POS_SCALE
#     up_m    = -ins_z * POS_SCALE

#     north_v = ins_xd * VEL_SCALE
#     east_v  = ins_yd * VEL_SCALE
#     up_v    = -ins_zd * VEL_SCALE

#     heading = math.atan2(east_v, north_v) if (abs(east_v)>1e-6 or abs(north_v)>1e-6) else None
#     heading_deg = math.degrees(heading) if heading is not None else float('nan')

#     # --- store ---
#     with LOCK:
#         st = uav["state"]
#         st["x"], st["y"], st["z"] = east_m, north_m, up_m
#         st["vx"], st["vy"], st["vz"] = east_v, north_v, up_v
#         st["heading"] = heading

#     # --- DEBUG PRINT ---
#     print(f"INS -> AC{ac_id}: x={east_m:.2f}, y={north_m:.2f}, z={up_m:.2f}, "
#           f"vx={east_v:.2f}, vy={north_v:.2f}, vz={up_v:.2f}, "
#           f"heading={heading_deg:.1f}°")

# def send_guided(interface, ac_id, vx_enu, vy_enu, vz_enu=0.0):
#     """
#     Multi-UAV safe wrapper to send GUIDED_SETPOINT_NED velocity commands
#     using ENU input (vx, vy, vz) from the planner.
#     """

#     # Convert ENU → NED (North-East-Down)
#     # ENU X (East)  -> NED Y (East)
#     # ENU Y (North) -> NED X (North)
#     # ENU Z (Up)    -> NED Z (Down, so negative)
#     vx_ned = vy_enu
#     vy_ned = vx_enu
#     vz_ned = -vz_enu

#     msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
#     msg['ac_id'] = int(ac_id)
    
#     # Flags: Bit 5 (XY as vel) and Bit 6 (Z as vel) are set to 1
#     # bitorder='little' means index 5 is Bit 5, index 6 is Bit 6
#     msg['flags'] = np.packbits([0,0,0,0,0,1,1,0], bitorder='little')[0].astype(np.uint8)

#     msg['x'] = float(vx_ned)
#     msg['y'] = float(vy_ned)
#     msg['z'] = float(vz_ned)
#     msg['yaw'] = 0.0

#     interface.send(msg, ac_id=int(ac_id))
    
#     # Added vz_enu and vz_ned to the print statement for full visibility
#     print(f"[SEND] AC{ac_id}: vx_enu={vx_enu:.2f}, vy_enu={vy_enu:.2f}, vz_enu={vz_enu:.2f} | "
#           f"vx_ned={vx_ned:.2f}, vy_ned={vy_ned:.2f}, vz_ned={vz_ned:.2f}")


# def on_pprz_msg(ac_id, msg):

#     # Only print message names for messages we care about
#     if msg.name in ["INS", "INS_EKF2", "GPS_INT"]:
#         print(f"[RECV] AC{ac_id} -> {msg.name}")

#     # Forward messages to the proper handlers:
#     if msg.name == "GPS_INT":
#         on_gps_int(ac_id, msg)

#     elif msg.name == "INS":
#         on_ins(ac_id, msg)

#     elif msg.name == "INS_EKF2":
#         # Many setups use INS_EKF2 as better replacement → treat like INS
#         on_ins(ac_id, msg)



# def manual_test(interface):
 
#     while True:
#         # send_guided(interface, 44,  1.0,  5.0, 0.0)
#         send_guided(interface, 121, 10.0, 0.0, 0.0)

#         # print("Sent test commands: AC44 -> (1,5), AC46 -> (-1,-5)")
#         time.sleep(0.2)   # 5 Hz


# def main():
#     ivy_bus = os.getenv("PPRZ_IVY_BUS", "127.255.255.255:2010")
#     interface = IvyMessagesInterface("multi_uav_bridge", ivy_bus=ivy_bus)
#     interface.subscribe(on_pprz_msg)     # debug
#     interface.subscribe(on_gps_int)
#     interface.subscribe(on_ins)
#     interface.subscribe(lambda ac_id, msg: on_ins(ac_id, msg) if msg.name=="INS_EKF2" else None)


#     interface.start()

#     # th = threading.Thread(target=planner_thread, args=(interface,), daemon=True)
#     # th.start()
#     print("[BRIDGE] Running manual velocity test.")
#     manual_test(interface)


#     print("[BRIDGE] Running. Ctrl+C to stop.")
#     try:
#         while True:
#             time.sleep(1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         interface.stop()
#         with LOCK:
#             for uav in UAVS.values():
#                 uav["log_f"].close()

# if __name__ == "__main__":
#     main()
