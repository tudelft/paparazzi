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

import messages_pb2

import sys
from os import path, getenv
import time
import numpy as np
from vutura_receiver import VuturaEvent, VuturaReceiver
import logging

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

class VuturaVisualizer(object):
    def  __init__(self, ivy_interface = None):
        """
        Initialize Vututa visualizer
        """
        self.id_number_map = {}
        
        if ivy_interface == None:
            self.ivy_interface = IvyMessagesInterface("Aircraft NFZ")
        else:
            self.ivy_interface = ivy_interface
        
    def __delete__(self):
        self.stop()
        
    def stop(self):
        self.ivy_interface.stop()
    
    def visualize(self, vutura_events):
        """
        Visualize the vutura events
        """
        remove_events = self.id_number_map.copy()
        
        # Go through the vutura events
        for ev in vutura_events:
            # Check timeout
            if (ev.timed_out()):
                continue
            try: # If already exists
                del remove_events[ev.get_aircraft_id()]
                self.draw_event(ev)
            except: # If new
                self.draw_event(ev)
            
        # Delete events that are not in the list anymore
        for ev in remove_events:
            self.delete_event(ev.get_aircraft_id())
            
    def draw_event(self, ev):
        """
        Draw an vutura event
        """
        logging.debug("test")
        shape_id = 255
        try:
            shape_id = self.id_number_map[ev.get_aircraft_id()]
        except:
            for i in range(256):
                values = []
                for key, value in self.id_number_map.items():
                    values.append(value)
                if (values.count == 0):   
                    shape_id = i
                    self.id_number_map[ev.get_aircraft_id()] = i
        latd_old = ev.get_lat()
        lond_old = ev.get_lon()
        radius = wgs_84_radius(latd_old)
        
        hdg = np.radians(ev.get_heading())
        groundspeed = ev.get_groundspeed()
        v_e = groundspeed * np.sin(hdg)
        v_n = groundspeed * np.cos(hdg)
        dt = time.time() - ev.get_recorded_time()
        
        d_p_e = v_e * dt
        d_p_n = v_n * dt
        
        d_latd = np.degrees(np.arcsin(d_p_n / radius))
        d_lond = np.degrees(np.arcsin(d_p_e / (radius * np.cos(np.radians(latd_old)))))
        
        latd = latd_old + d_latd
        lond = lond_old + d_lond
        

        self.draw_shape(shape_id, latd, lond)
    
    def delete_event(self, aircraft_id):
        """
        Delete a drawn event
        """
        self.delete_shape(self.id_number_map[aircraft_id])
        del self.id_number_map[aircraft_id]
        
        
    def draw_shape(self, shape_id, latd, lond):
        """
        Draw or update shape on ground station
        """
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = shape_id
        msg['linecolor'] = "orange"
        msg['fillcolor'] = "orange"
        msg['opacity'] = 1
        msg['shape'] = 0 # Circle
        msg['status'] = 0 # Create
        msg['latarr'] = [int(latd * 1e7), 0]
        msg['lonarr'] = [int(lond * 1e7), 0]
        msg['radius'] = 25
        self.ivy_interface.send(msg)
    
    def delete_shape(self, shape_id):
        """
        Delete a shape on the ground station
        """
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = shape_id
        msg['status'] = 1 # Delete
        self.ivy_interface.send(msg)
        
def wgs_84_radius(latd):
    coslat = np.cos(np.radians(latd))
    sinlat = np.sin(np.radians(latd))

    a = 6378137.0
    b = 6356752.314245
    an = a * a * coslat
    bn = b * b * sinlat
    ad = a * coslat
    bd = b * sinlat
    anan = an * an
    bnbn = bn * bn
    adad = ad * ad
    bdbd = bd * bd

    return np.sqrt((anan + bnbn) / (adad + bdbd))


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    
    # start receiving vutura data
    vutura_receiver = VuturaReceiver()
    vutura_receiver.start()
    
    # Ceeate new visualizer
    visualizer = VuturaVisualizer()
    
    try:
        while True:
            events = vutura_receiver.get_events()
            visualizer.visualize(events)
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        visualizer.stop()
        vutura_receiver.stop()
        
        