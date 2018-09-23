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

import numpy as np

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
    def __init__(self, circular_zones, ltp_def):
        self.realtime_scenario = traffic_scenario.TrafficScenario(circular_zones, ltp_def)
        
    def init_realtime(self):
        """
        Initialize the real time resolutions
        """
        self.realtime_scenario.init_SSD_plot()
        
    def run_realtime(self, tla, wind, detection_margin, airspeed, aircraft, traffic_events):
        """
        run the realtime_resolutions
        """
        self.realtime_scenario.update_traffic_scenario(aircraft, traffic_events)
        
        if self.realtime_scenario.Traffic.ntraf > 1:
#            try:
            self.realtime_scenario.detect_conflicts(tla, wind, detection_margin, airspeed)
            self.realtime_scenario.plot_SSD() 
#            except: # All errors to overcome pyclipper error UnboundLocalError: # When simulated aircraft are too far
#                pass # Do nothing
                
class ResolutionFinder(object):
    def __init__(self, circular_zones, ltp_def):
        self.extrapolated_scenario = traffic_scenario.ExtrapolatedScenario(circular_zones, ltp_def)
    
    def resolution_on_leg(self, from_point_enu, to_point_enu, groundspeed, margin):
        dx = to_point_enu.x - from_point_enu.x
        dy = to_point_enu.y - from_point_enu.y
        hdg = np.rad2deg(np.arctan2(dx, dy)) % 360.            
        
        #abs_speed, speed_vector = self.ground_velocity_from_commanded_airspeed(hdg, self.commanded_airspeed)
        
        self.traffic_scenario_extrapolated.update_traffic_scenario(speed_vector, groundspeed, hdg, self.wind, from_point_enu, 0., self.receiver_thread, types)
        time_to_arrive_at_point = self.time_to_arrive_at_point(from_point, to_point, self.commanded_airspeed)

        
        reduced_tla = self.min_avoid_leg_time
        looping = True        
        
        while looping:
            abs_speed, speed_vector = self.ground_velocity_from_commanded_airspeed(hdg, self.commanded_airspeed)
            
            self.traffic_scenario_extrapolated.update_traffic_scenario(speed_vector, abs_speed, hdg, self.wind, from_point_enu, dt, self.receiver_thread, types)
            resolutions = self.traffic_scenario_extrapolated.detect_conflicts(reduced_tla + self.reduced_tla_margin, self.wind, resolution_margin)
            
            if (resolutions[0] == 'free'):
                return 'free'
            
            if (resolutions[0] == 'nosol'):
                print ("nosol from begin in resolution function")
                return 'nosol'                        
            
            if (len(resolutions[1]) < 30):
                n_i = len(resolutions[1])
            else:
                n_i = 30
                
            resolution_points = []
            distances = []
            rightleftsolution = {'right' : False, 'left' : False}
            
            for i in range(n_i-1):
                if ((rightleftsolution['right'] == True) and (rightleftsolution['left'] == True)):
                    break
                
                hdg_res = np.rad2deg(np.arctan2(resolutions[1][i], resolutions[2][i])) % 360.
                UAV_speed = self.ground_velocity_from_commanded_airspeed(hdg_res, self.commanded_airspeed)[1]
                    
                res_dir = self.CheckRelativeHeadingDirection(hdg, hdg_res)
                
                if (res_dir == 'right' and not rightleftsolution['right']):
                    pass
                elif (res_dir == 'left' and not rightleftsolution['left']):
                    pass
                else:
                    continue
                
                new_from_point = (from_point[0] + UAV_speed[0]*reduced_tla, from_point[1] + UAV_speed[1]*reduced_tla)
                
                linestring_to = geometry.LineString([from_point, new_from_point])
                linestring_from = geometry.LineString([new_from_point, next_point])
                
                if ((self.soft_geofence.polygon.contains(linestring_to) == False) and (self.soft_geofence.polygon.contains(linestring_from) == False)):
                    continue
                
                for Zone in range(len(self.Zones)):
                    if (linestring_to.intersects(self.Zones[Zone].circle['geo']) and (linestring_from.intersects(self.Zones[Zone].circle['geo']))):
                        continue
                    
                new_from_point_enu = geodetic.EnuCoor_f(new_from_point[0], new_from_point[1], self.altitude)
                
                dx_target = to_point[0] - new_from_point[0]
                dy_target = to_point[1] - new_from_point[1]
                
                hdg_target = np.rad2deg(np.arctan2(dx_target, dy_target))
                
                abs_speed, speed_vector = self.ground_velocity_from_commanded_airspeed(hdg_target, self.commanded_airspeed)
                
                self.traffic_scenario_extrapolated.update_traffic_scenario(speed_vector, abs_speed, hdg_target, self.wind, new_from_point_enu, dt + reduced_tla, self.receiver_thread, types = types)
                tla_target = min(self.time_to_arrive_at_point(new_from_point, to_point, self.commanded_airspeed), self.max_tla) # look later
                
                conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla_target, self.wind, resolution_margin) # mayve detection margin good as well
                
                if (conflict[0] == 'free'):
                    
                    if (res_dir == 'right'):
                        rightleftsolution['right'] = True
                    else:
                        rightleftsolution['left'] = True
                    resolution_points.append(new_from_point)
                    distance = np.sqrt((new_from_point[0] - from_point[0])**2 + (new_from_point[1] - from_point[1])**2) + np.sqrt((to_point[0] - new_from_point[0])**2 + (to_point[1] - new_from_point[1])**2)
                    distances.append(distance)
                    
            if len(resolution_points) > 0:
                resolution_point = resolution_points[np.argmin(distances)]
                looping = False
                return resolution_point
            else:
                if (reduced_tla <= time_to_arrive_at_point + self.extended_tla):
                    reduced_tla = reduced_tla + self.reduced_tla_step
                else:
                    looping = False
                    return 'nosol'    
        
    
    
        
        
