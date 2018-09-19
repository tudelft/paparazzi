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
Visualize asterix events in the ground station
"""

from os import path, getenv
import time
import sys
from asterix_receiver import AsterixEvent, AsterixReceiver
from sets import Set

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

class AsterixVisualizer(object):
    def __init__(self):
        """
        Initialize the asterix visualizer
        """
        self.event_ids = Set([]) # IDs that are currently drawn on the ground station
        self.ivy_interface = IvyMessagesInterface("Asterix Visualizer")

    def __delete__(self):
        self.stop()

    def stop(self):
        self.ivy_interface.stop()
        
    def visualize(self, asterix_events):
        """
        Visualize the asterix events
        """
        remove_events = self.event_ids.copy()

        # Go through the asterix events
        for ev in asterix_events:
            remove_events.discard(ev.id)

            # Update or draw the event
            self.draw_event(ev)

        # Delete the asterix events that are not in the list anymore
        for ev in remove_events:
            self.delete_event(ev)

    def draw_event(self, ev):
        """
        Draw an asterix event
        """
        self.event_ids.add(ev.id)

        ev_type = ev.get_type()
        if ev_type == AsterixEvent.OTHER_AIR_TRAFFIC:
            self.draw_shape(ev.id, ev.get_lla(), ev.get_radius(), "orange", "yellow")
        elif ev_type == AsterixEvent.LOCALIZED_WEATHER:
            self.draw_shape(ev.id, ev.get_lla(), ev.get_radius(), "orange", "yellow")
        elif ev_type == AsterixEvent.MIGRATORY_BIRD:
            self.draw_shape(ev.id, ev.get_lla(), ev.get_radius(), "orange", "orange")
        else:
            self.draw_shape(ev.id, ev.get_lla(), ev.get_radius(), "orange", "orange")

    def delete_event(self, ev):
        """
        Delete a drawn event
        """
        self.delete_shape(id)
        self.event_ids.discard(ev.id)

    def draw_shape(self, id, lla, radius, line_color, fill_color):
        """
        Draw or update a shape on the ground station
        """
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 32 + id
        msg['linecolor'] = line_color
        msg['fillcolor'] = fill_color
        msg['opacity'] = 1
        msg['shape'] = 0 # Circle
        msg['status'] = 0 # Create
        msg['latarr'] = [lla.to_int(), 0]
        msg['lonarr'] = [lla.to_int(), 0]
        msg['radius'] = radius
        self.ivy_interface.send(msg)

    def delete_shape(self, id):
        """
        Delete a shape on the ground station
        """
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 32 + id
        msg['status'] = 1 # Delete
        self.ivy_interface.send(msg)

if __name__ == '__main__':
    # Start receiving asterix packets
    asterix_receiver = AsterixReceiver()
    asterix_receiver.start()

    # Create a new visualizer
    visualizer = AsterixVisualizer()

    # Visualize the asterix packets in the ground station every second
    try:
        while True:
            events = asterix_receiver.get_events()
            visualizer.visualize(events)
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        visualizer.stop()
        asterix_receiver.stop()
        