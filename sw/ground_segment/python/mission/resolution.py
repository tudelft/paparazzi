#!/usr/bin/env python
#
# Copyright (C)     2018 Dennis Wijngaarden
#                    2018 Freek van Tienen <freek.v.tienen@gmail.com>
#               
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

from os import path, getenv
import sys

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface
from pprz_math import geodetic

import traffic_scenario

class RealtimeResolution(object):
    def __init__(self, circular_zones):
        self.realtime_scenario = traffic_scenario.TrafficScenario(circular_zones)
        
    def init_realtime(self):
        """
        Initialize the real time resolutions
        """
        self.realtime_scenario.init_SSD_plot()
        
    def run_realtime(self, tla, wind, detection_margin, airspeed):
        """
        run the realtime_resolutions
        """
        self.realtime_scenario.update_traffic_scenario(aircraft, ReceiverThread)
        if self.traffic_scenario.Traffic.ntraf > 1:
            try:
                self.realtime_scenario.detect_conflicts(tla, wind, detection_margin, airspeed)
                self.realtime_Scenario.plot_SSD() 
            except: # All errors to overcome pyclipper error UnboundLocalError: # When simulated aircraft are too far
                pass # Do nothing
        
        
