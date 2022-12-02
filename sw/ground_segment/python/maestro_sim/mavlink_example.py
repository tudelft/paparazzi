import time
from pymavlink import mavutil
import math

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

time_in_usec = (int(time.time() * 1000000))
master.mav.set_gps_global_origin_send(
    master.target_system,
    int(449e6),
    int(-6e6),
    194,
    0)

time.sleep(5)

q = [0,0,0,0]

master.mav.set_home_position_send(
    master.target_system,
    int(4495e5),
    int(-6e6),
    220,
    0,
    0,
    0,
    q,
    0,
    0,
    0,
    time_in_usec)

time.sleep(5)

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1,
    0,
    0,
    0,
    -20,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0)

time.sleep(1)

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1,
    0,
    0,
    10,
    -20,
    0,
    0,
    0,
    0,
    0,
    0,
    90/180*3.14,
    0)

time.sleep(1)

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    1,
    0,
    0,
    10,
    -20,
    0,
    0,
    0,
    0,
    0,
    0,
    90/180*3.14,
    0)

while True:

    t = time.time()
    w_c = 0.5 # rad/s
    amplitude = 10 #meter
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
    90/180*3.14,
    0)

    time.sleep(0.002)

while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)
