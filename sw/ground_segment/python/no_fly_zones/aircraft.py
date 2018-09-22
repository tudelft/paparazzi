#!/usr/bin/env python
#
# Copyright (C) 2018 Freek van Tienen <freek.v.tienen@gmail.com>
#               2018 Dennis Wijngaarden
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
"""
Mission communication through datalinks
"""

from os import path, getenv
from sets import Set
import sys
import logging
import threading
import time

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface
from pprz_math import geodetic

class Aircraft(object):
    
    def __init__(self, ac_id, ivy_interface = None):
        assert ac_id <> None

        # Initialize the variables
        self.ac_id = ac_id
        self.got_ltp = False
        self.ltp_def = None
        self.lla = geodetic.LlaCoor_f(0.0, 0.0, 0.0)
        self.course = 0.0
        self.ground_speed = 0.0
        self.airspeed = 0.0
        self.lock = threading.Lock()

        # Create a new ivy interface if needed
        if ivy_interface == None:
            self.ivy_interface = IvyMessagesInterface("Aircraft NFZ")
        else:
            self.ivy_interface = ivy_interface

        self.ivy_interface.subscribe(self.aircraft_ins_ref_cb, PprzMessage("telemetry", "INS_REF"))
        self.ivy_interface.subscribe(self.aircraft_navigation_ref_cb, PprzMessage("telemetry", "NAVIGATION_REF"))
        self.ivy_interface.subscribe(self.ground_flight_param_cb, PprzMessage("ground", "FLIGHT_PARAM"))

    def aircraft_ins_ref_cb(self, ac_id, msg):
        """
        When a INS_REF message is received from the AC
        """
        # Check if it is for our aircraft
        if self.ac_id != ac_id:
            return
        
        try:
            self.lock.acquire()
            # Get the LTP definition
            self.ltp_def = geodetic.EcefCoor_i(int(msg['ecef_x0']), int(msg['ecef_y0']), int(msg['ecef_z0'])).to_float().to_ltp_def()
            self.got_ltp = True
            logging.debug("Got LTP from aircraft")
        finally:
            self.lock.release()

    def aircraft_navigation_ref_cb(self, ac_id, msg):
        """
        When a NAVIGATION_REF message is received from the AC
        """
        # Check if it is for our aircraft
        if self.ac_id != ac_id:
            return
        
        try:
            self.lock.acquire()
            # Get the LTP definition
            self.ltp_def = geodetic.UtmCoor_i(int(msg['utm_north'])*100, int(msg['utm_east'])*100, int(float(msg['ground_alt'])*1000), int(msg['utm_zone'])).to_float().to_lla().to_ltp_def()
            self.got_ltp = True
            logging.debug("Got LTP from aircraft")
        finally:
            self.lock.release()

    def ground_flight_param_cb(self, ac_id, msg):
        """
        When a INS_REF message is received from the AC
        """
        # Check if it is for our aircraft
        if self.ac_id != int(msg['ac_id']):
            return
        
        try:
            self.lock.acquire()
            
            self.lla = geodetic.LlaCoor_f(float(msg['lat']), float(msg['long']), float(msg['alt']))
            self.course = float(msg['course'])
            self.ground_speed = float(msg['speed'])
            self.airspeed = float(msg['airspeed'])
            
            logging.debug("Got the Flight parameters [%f,%f,%f, %f, %f, %f]",
                float(msg['lat']), float(msg['long']), float(msg['alt']),
                float(msg['course']), float(msg['speed']), float(msg['airspeed']))
        finally:
            self.lock.release()

    def get_ltp_def(self):
        """
        Return the LTP of the aircraft
        """
        ltp_def = None
        try:
            self.lock.acquire()
            ltp_def = self.ltp_def
        finally:
            self.lock.release()
        return ltp_def
