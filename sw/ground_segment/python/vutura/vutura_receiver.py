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

import sys
from os import path, getenv
from sets import Set
import threading
import socket
import numpy as np
import time
import logging

from nanomsg import Socket, PUB, SUB, SUB_SUBSCRIBE
import messages_pb2

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

# Vutura receiver over tcp
class VuturaReceiver(threading.Thread):
    def __init__(self):
        """
        Initialize the Vutura Receiver Thread
        """
        threading.Thread.__init__(self)
        
        self.name = "VuturaReceiver"
        self.events = Set([])
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        
        self.sock = Socket(SUB)
        #self.sock.connect('ipc:///tmp/traffic_info_0'.encode('utf-8'))
        self.sock.connect('tcp://10.11.0.1:8340'.encode('utf-8'))
        self.sock.set_string_option(SUB, SUB_SUBSCRIBE, ''.encode('utf-8'))
        
        
    def run(self):
        """
        Vutura packet Receiver
        """
        
        while not self.stop_event.is_set():
            
            # Reveive and parse Traffic message
            msg = self.sock.recv()
            traffic_info = messages_pb2.TrafficInfo()
            traffic_info.ParseFromString(msg)
            if (not "flight|" in traffic_info.aircraft_id):
                continue
            # create a new event
            new_t = VuturaEvent(traffic_info)
            
           
            
            # Check if already existed then update, else add
            try:
                self.lock.acquire()
                found = False
                for t in self.events:
                    if t == new_t:
                        t.update(new_t)
                        found = True
                    
                if not found:
                    self.events.add(new_t)
            finally:
                self.lock.release()
                
            # Check if events need to be removed
            try:
                self.lock.acquire()
                events_copy = self.events.copy()
                for t in events_copy:
                    if t.timed_out():
                        self.events.discard(t)
            finally:
                self.lock.release()
                
    def get_events(self):
        """
        Get the current events in a thread safe way
        """
        events = set([])
        try:
            self.lock.acquire()
            events = self.events.copy()
        finally:
            self.lock.release()
        return events
    
    def stop(self):
        """
        Thread safe stopping the Vutura receiver
        """
        self.stop_event.set()
        self.sock.close()
                        

# Vutura evenet object received through traffic packet
class VuturaEvent(object):
    TIMEOUT = 20.
    FT_TO_M = 0.3048
    
    def __init__(self, vutura_packet):
        """
        Create new event from received vutura data
        """
        self.aircraft_id = vutura_packet.aircraft_id
        self.recorded_time = vutura_packet.recorded_time * 1e-3
        self.lat = vutura_packet.lat * 1e-7
        self.lon = vutura_packet.lon * 1e-7
        self.alt = vutura_packet.alt * VuturaEvent.FT_TO_M * 1e-3
        self.groundspeed = vutura_packet.groundspeed * 1e-3
        self.heading = vutura_packet.heading
        
    def update(self, other):
        """
        Updare current object with a new object
        """
        assert self.aircraft_id == other.aircraft_id
        self.recorded_time = other.recorded_time
        self.lat = other.lat
        self.lon = other.lon
        self.alt = other.alt
        self.groundspeed = other.groundspeed
        self.heading = other.heading
        
    def timed_out(self):
        """
        Whether we haven't received an update for a while
        """
        return((time.time() - self.recorded_time) > VuturaEvent.TIMEOUT)
        
    def get_aircraft_id(self):
        """
        Return the aircraft id of the vutura event
        """
        return str(self.aircraft_id)
        
    def get_recorded_time(self):
        """
        Return the recorded time of the vutura event
        """
        return self.recorded_time
    
    def get_lat(self):
        """
        Return the lat of the vutura event
        """
        return self.lat
    
    def get_lon(self):
        """
        Return the lon of the vutura event
        """
        return self.lon
    
    def get_alt(self):
        """
        Return the alt of the vutura event
        """
        return self.alt
    
    def get_groundspeed(self):
        """
        Return the groundspeed of the vutura event
        """
        return self.groundspeed
    
    def get_heading(self):
        """
        Return the heading of the vutura event
        """
        return self.heading
        
    def __eq__(self, other):
        """
        Equivalent if the aircraft id is similar
        """
        return self.aircraft_id == other.aircraft_id
            