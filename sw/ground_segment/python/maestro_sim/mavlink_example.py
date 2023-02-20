import time
from pymavlink import mavutil
import math

import keyboard  # using module keyboard

# Start a connection listening on a UDP port
master = mavutil.mavlink_connection('udpin:192.168.56.1:14550')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Once connected, use 'master' to get and send messages

# Arm
# master.arducopter_arm() or:
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

mode_id = 0 #master.mode_mapping()[mavutil.mavlink.MAV_MODE_GUIDED_ARMED]
master.mav.set_mode_send(
    master.target_system,
    128+8,
    mode_id)

# bordeaux
time_in_usec = (int(time.time() * 1000000))
# master.mav.set_gps_global_origin_send(
#     master.target_system,
#     int(449e6),
#     int(-6e6),
#     194,
#     0)

# valkenburg
master.mav.set_gps_global_origin_send(
    master.target_system,
    int(521680913),
    int(44123255),
    45,
    0)

print("origin set")

time.sleep(2)

q = [0,0,0,0]

# bordeaux
# master.mav.set_home_position_send(
#     master.target_system,
#     int(4495e5),
#     int(-6e6),
#     220,
#     0,
#     0,
#     0,
#     q,
#     0,
#     0,
#     0,
#     time_in_usec)

# valkenburg
master.mav.set_home_position_send(
    master.target_system,
    int(521680913),
    int(44123255),
    45,
    0,
    0,
    0,
    q,
    0,
    0,
    0,
    time_in_usec)

print("home set")

print("takeoff in 2 secs!!")

time.sleep(2)

# Takeoff
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 5)

time.sleep(7)

print("go to start")

# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_LAND,
#     0,
#     0, 0, 0, 0, 0, 0, 0)

# time.sleep(10)

# exit()

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1, #frame
    0b0001111111011111, #typemask only position
    0, #x
    0, #y
    -10, #z (down)
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0)

time.sleep(10)

print("go south")

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1,
    0b0001111111011111, #typemask only position
    -10,
    0,
    -10,
    0,
    0,
    0,
    0,
    0,
    0,
    10/180*3.14,
    0)

time.sleep(10)

print("go back")

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1, #frame
    0b0001111111011111, #typemask only position
    0, #x
    0, #y
    -10, #z (down)
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0)

time.sleep(10)

print("start circle")

t0 = time.time()
t = time.time()

while t-t0 < 20:

    t = time.time()
    w_c = 0.5 # rad/s
    amplitude = 5 #meter
    xpos = amplitude*math.sin(t*w_c)
    xdpos = w_c*amplitude*math.cos(t*w_c)
    xddpos = -w_c*w_c*amplitude*math.sin(t*w_c)
    ypos = amplitude*math.cos(t*w_c)
    ydpos = -w_c*amplitude*math.sin(t*w_c)
    yddpos = -w_c*w_c*amplitude*math.cos(t*w_c)

    master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1,
    0,
    xpos,
    ypos,
    -20,
    xdpos,
    ydpos,
    0,
    xddpos,
    yddpos,
    0,
    10/180*3.14,
    0)

    time.sleep(0.1)

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1, #frame
    0, #typemask
    0, #x
    0, #y
    -10, #z (down)
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0)

print("hover")

time.sleep(5)

print("land")

# Land
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)

exit()
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)
