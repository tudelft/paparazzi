import serial
from time import perf_counter

import sys
from os import path, getenv
from time import time, sleep
import numpy as np
from collections import deque
import argparse

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

## parse args
#parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
#parser.add_argument('-ac', action='append', nargs=2, metavar=('rigid_id','ac_id'), help='pair of rigid body and A/C id (multiple possible)')
#parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
#parser.add_argument('-s', '--server', dest='server', default="127.0.0.1", help="NatNet server IP address")
#parser.add_argument('-m', '--multicast_addr', dest='multicast', default="239.255.42.99", help="NatNet server multicast address")
#parser.add_argument('-dp', '--data_port', dest='data_port', type=int, default=1511, help="NatNet server data socket UDP port")
#parser.add_argument('-cp', '--command_port', dest='command_port', type=int, default=1510, help="NatNet server command socket UDP port")
#parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
#parser.add_argument('-f', '--freq', dest='freq', default=10, type=int, help="transmit frequency")
#parser.add_argument('-gr', '--ground_ref', dest='ground_ref', action='store_true', help="also send the GROUND_REF message")
#parser.add_argument('-vs', '--vel_samples', dest='vel_samples', default=4, type=int, help="amount of samples to compute velocity (should be greater than 2)")
#parser.add_argument('-rg', '--remote_gps', dest='rgl_msg', action='store_true', help="use the old REMOTE_GPS_LOCAL message")
#parser.add_argument('-sm', '--small', dest='small_msg', action='store_true', help="enable the EXTERNAL_POSE_SMALL message instead of the full")
#parser.add_argument('-o', '--old_natnet', dest='old_natnet', action='store_true', help="Change the NatNet version to 2.9")
#parser.add_argument('-zf', '--z_forward', dest='z_forward', action='store_true', help="Z-axis as forward")
#args = parser.parse_args()

## start ivy interface
#if args.ivy_bus is not None:
#    ivy = IvyMessagesInterface("natnet2ivy", ivy_bus=args.ivy_bus)
#else:
#    ivy = IvyMessagesInterface("natnet2ivy")
ivy = IvyMessagesInterface("arduino2ivy")

# make sure the 'COM#' is set according the Windows Device Manager
ser = serial.Serial('/dev/ttyUSB1', 9600)

estm_deck_flat = 0
estm_deck_up   = 0

for i in range(150):
    line = ser.readline()   # read a byte
    if line:
        string = line.decode()  # convert the byte string to a unicode string
        #num = int(string) # convert the unicode string to an int
        print("new_string: ")
        print(string)
        
        if string.find("deck flat") != -1:
           estm_deck_flat = float(string[28:]) #dummy
           
        if string.find("start roll") != -1:
           estm_deck_up = float(string[36:]) #dummy
        
        msg = PprzMessage("datalink", "SHIP_DATA")
        msg['estm_deck_flat'] = estm_deck_flat #dummy
        msg['estm_deck_up']   = estm_deck_up #dummy
        ivy.send(msg)
        print("ivy msg send")
        
ser.close()



