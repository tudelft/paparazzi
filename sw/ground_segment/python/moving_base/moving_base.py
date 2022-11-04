#!/usr/bin/env python3
#
# Copyright (C) 2017 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

'''
Moving base simulator
'''

# too many things here
from __future__ import print_function
from cmath import sin
from http.client import PRECONDITION_REQUIRED
import sys
import numpy as np
import json
from time import sleep
from os import path, getenv
import time
import pymap3d as pm
import math as m

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
#PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '~/paparazzi'))) #TEST
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

from scipy import linalg as la

import geopy.distance

BASE_MOVING = True
BASE_WAVES = False

CYBERZOO = False
ROLL_COMPENSATION_ANALYSIS = False

class UAV:
    def __init__(self, ac_id):
        self.initialized = False
        self.id = ac_id
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.timeout = 0

class Base:
    def __init__(self, freq=10., use_ground_ref=False, ignore_geo_fence=False, verbose=False):
        self.step = 1. / freq
        self.use_ground_ref = use_ground_ref
        self.enabled = True # run sim by default
        self.verbose = verbose
        self.ids = [203] # originaly 14, but did not work, did not match A/C in Paparazzi
        self.uavs = [UAV(i) for i in self.ids]
        self.time = time.mktime(time.gmtime())

        if not CYBERZOO:
            # Outdoor configuration
            if BASE_MOVING:
                 self.speed = 1.5 # m/s
            else:
                self.speed = 0.0 # m/s
            self.course = 150 # deg (moving directon of platform)
            self.heading = 150 # deg (orientation of platform)
            self.alt0 = 43 # ref_alt0 is already about 46m in simulator

            print("lat0,long0,alt0 position = Valkenburg")
            self.lat0 = 52.168391 #deg
            self.lon0 = 4.413100 #deg

            #print("lat0,long0,alt0 position = iMAV")
            #self.lat0 = 52.170591 #deg
            #self.lon0 = 4.417030 #deg

        else:
            # Cyberzoo configuration
            if BASE_MOVING:
                 self.speed = 0.2 # m/s
            else:
                self.speed = 0.0 # m/s
            self.course = 90 # deg (moving directon of platform)
            self.heading = 180 # deg (orientation of platform)
            self.alt0 = 45 # ref_alt0 is already about 46m in simulator

            print("lat0,long0,alt0 position = Cyberzoo")
            self.lat0 = 51.9906114 #deg
            self.lon0 = 4.3767600 #deg

        self.lat = self.lat0
        self.lon = self.lon0
        self.alt = self.alt0

        self.lat_prev = self.lat
        self.lon_prev = self.lon
        self.alt_prev = self.alt

        self.loop_counter = 0

        self.bufLon = [0,0,0,0,0,0,0,0,0,0] # length 10
        self.bufLat = [0,0,0,0,0,0,0,0,0,0] # length 10
        self.bufAlt = [0,0,0,0,0,0,0,0,0,0] # length 10

        
        self.bufRoll_vel = [0] * (5) # store last 5 datapoints
        self.bufRoll_min = [0] * (5) # store last 10 waves
        self.bufRoll_max = [0] * (5) # store last 10 waves
        self.lastRoll_vel_ENU = [0] * (3)
        self.Roll_amplitude = 0
        self.Roll_compensation = 0

        self.LLA_compensated = [0,0,0]


        self.estmCourse = 0
        self.estmSpeed = 0


        # Start IVY interface
        self._interface = IvyMessagesInterface("Moving Base")

        # bind to GPS_INT message
        def ins_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "GPS_INT":
                uav = self.uavs[self.ids.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                uav.lat[0] = float(msg['lat']) / 1e7
                uav.lon[1] = float(msg['lon']) / 1e7
                uav.alt[2] = float(msg['alt']) / 100
                uav.timeout = 0
                uav.initialized = True
                print("GPS_INT msg received\n") #TEST
        if not self.use_ground_ref:
            self._interface.subscribe(ins_cb, PprzMessage("telemetry", "INS"))

        # bind to GROUND_REF message
        def ground_ref_cb(ground_id, msg):
            ac_id = int(msg['ac_id'])
            if ac_id in self.ids:
                uav = self.uavs[self.ids.index(ac_id)]
                # X and V in NED
                uav.X[0] = float(msg['pos'][1])
                uav.X[1] = float(msg['pos'][0])
                uav.X[2] = -float(msg['pos'][2])
                uav.V[0] = float(msg['speed'][1])
                uav.V[1] = float(msg['speed'][0])
                uav.V[2] = -float(msg['speed'][2])
                uav.timeout = 0
                uav.initialized = True
                print("GPS_INT msg received\n") #TEST
        if self.use_ground_ref:
            self._interface.subscribe(ground_ref_cb, PprzMessage("ground", "GROUND_REF"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    # Adds the NED coordinates to the GROUND_REF to get new LLA coordinate of ship
    def move_base(self, north, east, down=0):
        # [lat,lon,h] = ned2geodetic(xNorth,yEast,zDown,lat0,lon0,h0,spheroid) 
        # transforms the local north-east-down (NED) Cartesian coordinates specified by xNorth, yEast, and zDown 
        # to the geodetic coordinates specified by lat, lon, and h. 
        # Specify the origin of the local NED system with the geodetic coordinates lat0, lon0, and h0. 
        # Each coordinate input argument must match the others in size or be scalar. 
        # Specify spheroid as the reference spheroid for the geodetic coordinates. 
        out = pm.ned2geodetic(north, east, down, self.lat, self.lon, self.alt) 
        self.lat = out[0]
        self.lon = out[1]
        self.alt = self.alt+down
        if BASE_MOVING:
            print("loop counter: " + str(self.loop_counter))
        print(out)
    
    def EstimateSeastate(self):
        distances = [0] * 9
        for i in range(0,9):
            distances[i] = geopy.distance.geodesic((self.bufLat[i], self.bufLon[i]), (self.bufLat[i+1], self.bufLon[i+1])).m
        #print(distances)

    def RollCompensatedPosition(self):
        # rotation matrix
        c, s = np.cos(m.radians(-90+self.heading)), np.sin(m.radians(-90+self.heading)) # cosine and sine elements
        RotShip = np.array([[c, -s, 0], [s, c, 0], [0,0,1]])
        print("Rotation_matrix")
        print(RotShip)
    
        ENU_shift_ship = pm.geodetic2enu(self.lat, self.lon, self.alt, self.lat_prev, self.lon_prev, self.alt_prev) 
        freq_matrix = np.array([[1/self.step,0,0],[0,1/self.step,0], [0,0,1/self.step]])
        ENU_vel_ship = np.matmul(freq_matrix, ENU_shift_ship)
        print("ENU_vel_ship", ENU_vel_ship) # position shift relative to previous coordinate

        # rotation calculation
        XYZ_vel_ship = RotShip.dot(ENU_vel_ship)
        print("XYZ_vel_ship", XYZ_vel_ship) # velocity in ship coordinate frame
        Roll_vel_ship = XYZ_vel_ship[1]
        Forw_vel_ship = XYZ_vel_ship[0]

        self.bufRoll_vel.pop(len(self.bufRoll_vel)-1)
        self.bufRoll_vel.insert(0,Roll_vel_ship)

        derrivative_1 = self.bufRoll_vel[0] - self.bufRoll_vel[1]
        derrivative_2 = self.bufRoll_vel[1] - self.bufRoll_vel[2]

        if (derrivative_1 < 0 and derrivative_2 > 0):
            # Roll max (to right)
            roll_max = self.bufRoll_vel[1]
            self.bufRoll_max.pop(len(self.bufRoll_max)-1)
            self.bufRoll_max.insert(0,roll_max)             # add maximum 
            self.Roll_compensation = -self.Roll_amplitude/2 # reset compensation
        if (derrivative_1 > 0 and derrivative_2 < 0):
            # Roll min (to left)
            roll_min = self.bufRoll_vel[1]
            self.bufRoll_min.pop(len(self.bufRoll_min)-1)
            self.bufRoll_min.insert(0,roll_min)
            self.Roll_compensation = self.Roll_amplitude/2 # reset compensation

        self.Roll_compensation = self.Roll_compensation + self.step*self.bufRoll_vel[0]

        print("compensation: ", self.Roll_compensation)

        ENU_pos_ship = pm.geodetic2enu(self.lat, self.lon, self.alt, self.lat0, self.lon0, self.alt0) 
        self.LLA_compensated  = pm.enu2geodetic(ENU_pos_ship[0], ENU_pos_ship[1]-self.Roll_compensation, ENU_pos_ship[2], self.lat0, self.lon0, self.alt0)
        print("LLA_pos_ship \n", [self.lat, self.lon, self.alt]) # 
        print("LLA_pos_compensated \n", self.LLA_compensated ) #

        
        avg_Roll_min = np.average(self.bufRoll_min)
        avg_Roll_max = np.average(self.bufRoll_max)
        self.Roll_amplitude = avg_Roll_max - avg_Roll_min
        print("Roll_amplitude", self.Roll_amplitude)

        self.lat_prev = self.lat
        self.lon_prev = self.lon
        self.alt_prev = self.alt

    def send_pos(self):
        '''
        Send position of base sation
        '''
        ready = True
        for uav in self.uavs:
            if not uav.initialized:
                if self.verbose:
                    print("Waiting for state of rotorcraft ", uav.id)
                    sys.stdout.flush()
                ready = False
            if uav.timeout > 0.5:
                if self.verbose:
                    print("TIMEOUT ERROR: The state msg of rotorcraft ", uav.id, ": stopped")
                    sys.stdout.flush()
                ready = False

        # if not ready:
        #     return


        if self.verbose:
            E = 0 #dummy value (TODO: ERROR SHOULD BE CALCULATED BY RTK)
            print("Error distances: " + str(E).replace('[','').replace(']',''))
            sys.stdout.flush()

        for ac in self.uavs:
            # send ship location/speed information to drone
            msg = PprzMessage("datalink", "TARGET_POS")
            msg['ac_id'] = ac.id
            msg['target_id'] = ac.id
            msg['lat'] = int(self.lat * 1e7)
            #msg['lat'] = int(self.LLA_compensated[0] * 1e7)
            msg['lon'] = int(self.lon * 1e7)
            #msg['lon'] = int(self.LLA_compensated[1] * 1e7)
            msg['alt'] = int(self.alt *1000)
            msg['speed'] = self.speed
            msg['climb'] = 0
            msg['course'] = self.course
            msg['heading'] = self.heading
            msg['itow'] = 10000000  # very high, so the system does not think there is a time-out
            self._interface.send(msg)

            self.bufLon.pop(9)
            self.bufLon.insert(0,self.lon)
            self.bufLat.pop(9)
            self.bufLat.insert(0,self.lat)
            self.bufAlt.pop(9)
            self.bufAlt.insert(0,self.alt)

            if ROLL_COMPENSATION_ANALYSIS:
                self.RollCompensatedPosition()

    def run(self):
        try:

            # WAVE SETTINGS to simulate sea-state
            amplitude = 0.5 #m (movement = amp*2 !!)
            period = 8 #sec (wave period)
            timeStepsWave = np.arange(0, period, self.step)
            sineWave = amplitude*np.sin(2*np.pi*timeStepsWave*(1/period))
            cosnWave = amplitude*np.cos(2*np.pi*timeStepsWave*(1/period)+np.pi)
            wave_loop_idx_counter = 1

            # The main loop
            print("\nmoving base loop started!\n")
            while True:
                # TODO: make better frequency managing
                sleep(self.step)

                if ROLL_COMPENSATION_ANALYSIS:
                    self.EstimateSeastate() # TESTING

                for uav in self.uavs:
                    uav.timeout = uav.timeout + self.step

                if BASE_WAVES:
                    print("wave: " + str(sineWave[wave_loop_idx_counter]))


                # Send base (ship) position
                if self.enabled:
                    wave_pertubation = (sineWave[wave_loop_idx_counter] - sineWave[wave_loop_idx_counter-1])*BASE_WAVES
                    wave_pertubation_heigth = (cosnWave[wave_loop_idx_counter] - cosnWave[wave_loop_idx_counter-1])*BASE_WAVES
                    

                    dn = self.speed*m.cos(self.course/180.0*m.pi)*BASE_MOVING # derrivative North?
                    de = self.speed*m.sin(self.course/180.0*m.pi)*BASE_MOVING # derrivative East?
                    self.move_base(
                        self.step*dn+wave_pertubation,  # addition to North
                        self.step*de,                   # addition to East  -wave_pertubation
                        wave_pertubation_heigth)        # addition to Heigth (down)
                    self.send_pos()
                    self.loop_counter = self.loop_counter+1
                    print("course: ", self.course)
                
                if not CYBERZOO and self.loop_counter > 500: # change direction
                    #if self.course >= 180: 
                    #    self.course = self.course - 180
                    #else: 
                    #    self.course = self.course + 180
                    #if self.heading >= 180: 
                    #    self.heading = self.heading - 180
                    #else: 
                    #    self.heading = self.heading + 180
                    self.speed = self.speed*-1
                    self.loop_counter = 0

                if CYBERZOO and self.loop_counter > 90*(0.5/abs(self.speed+0.0001)):
                    self.speed = self.speed*-1
                    self.loop_counter = 0
                
                wave_loop_idx_counter = wave_loop_idx_counter+1
                if wave_loop_idx_counter == timeStepsWave.size -1:
                    wave_loop_idx_counter = 1
                    print("wave_loop_reset")

        except KeyboardInterrupt:
            self.stop()


if __name__ == '__main__':
    import argparse

    base = Base()
    base.run()

