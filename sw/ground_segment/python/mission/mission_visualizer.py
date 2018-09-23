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
Mission visualizer
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


class MissionVisualizer(object):

    def __init__(self, ivy_interface = None, ltp_def = None):
        """
        Initialize the Mission visualizer
        """
        # Initialize the variables
        self.ltp_def = ltp_def
        self.elem_ids = Set([])

        # Create a new ivy interface if needed
        if ivy_interface == None:
            self.ivy_interface = IvyMessagesInterface("Mission Visualizer")
        else:
            self.ivy_interface = ivy_interface

    def visualize(self, mission_elements):
        """
        Visualize the Mission elements
        """
        assert self.ltp_def != None
        logging.debug("Visualizing mission elements")
        remove_elements = self.elem_ids.copy()

        # Sort the mission elements by id
        mission_elements = sorted(mission_elements, key=lambda elem: elem.id)

        # Go through the mission elements
        prev_elem = None
        for elem in mission_elements:
            remove_elements.discard(elem.id)

            # Update or draw the element
            self.draw_element(elem, prev_elem)
            prev_elem = elem

        # Delete the mission elements that are not in the list anymore
        for elem_id in remove_elements:
            self.delete_element(elem_id)

    def draw_element(self, elem, prev_elem = None):
        """
        Draw a mission element
        """
        self.elem_ids.add(elem.id)

        # Only draw line if we have a previous point
        if prev_elem <> None:
            logging.info('Drawing line from %d to %d', prev_elem.id, elem.id)
            self.draw_line(elem.id, prev_elem.wp, elem.wp)

    def delete_element(self, elem_id):
        """
        Delete a drawn mission element
        """
        self.delete_line(elem_id)
        self.elem_ids.discard(elem_id)

    def draw_line(self, id, enu_from, enu_to):
        """
        Draw a line from lla_from to lla_to
        """
        # Convert enu to lla using ltp_def
        lla_from = enu_from.to_ecef(self.ltp_def).to_lla().to_int()
        lla_to = enu_to.to_ecef(self.ltp_def).to_lla().to_int()

        # Draw the shape
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 512 + id
        msg['linecolor'] = "white"
        msg['fillcolor'] = "blue"
        msg['opacity'] = 1
        msg['shape'] = 2 # Line
        msg['status'] = 0
        msg['latarr'] = [lla_from.lat, lla_to.lat] # e-7 deg
        msg['lonarr'] = [lla_from.lon, lla_to.lon] # e-7 deg
        msg['radius'] = 500
        self.ivy_interface.send(msg)

    def delete_line(self, id):
        """
        Delete a line on the ground station
        """
        # Delete the line
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 512 + id
        msg['status'] = 1 # Delete
        self.ivy_interface.send(msg)