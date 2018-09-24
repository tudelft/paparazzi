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
import shapely.geometry as geometry

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
        
    def run_realtime(self, tla, wind, detection_margin, airspeed, aircraft, traffic_events, groundspeed):
        """
        run the realtime_resolutions
        """
        self.realtime_scenario.update_traffic_scenario(aircraft, traffic_events, groundspeed)
        
        if self.realtime_scenario.Traffic.ntraf > 1:
            self.realtime_scenario.detect_conflicts(tla, wind, detection_margin, groundspeed)
            self.realtime_scenario.plot_SSD() 
                
class ResolutionFinder(object):
    def __init__(self, circular_zones, ref_utm_i, ltp_def):
        self.extrapolated_scenario = traffic_scenario.ExtrapolatedScenario(circular_zones, ref_utm_i, ltp_def)
        self.conflict_counter = 0
            
    def resolution_on_leg(self, from_point_enu, to_point_enu, groundspeed, margin, aircraft, traffic_events, wind, altitude, geofence, zones, max_tla, conflict_counter_th, avoidance_time_th, hdg_diff_th, avoid_dist_min):
        dx = to_point_enu.x - from_point_enu.x
        dy = to_point_enu.y - from_point_enu.y
        hdg = np.rad2deg(np.arctan2(dx, dy)) % 360.            
        
        self.extrapolated_scenario.update_traffic_scenario(aircraft, traffic_events, hdg, 0., groundspeed)
        t_arrive = time_to_arrive_at_point(from_point_enu, to_point_enu, groundspeed)
        resolutions = self.extrapolated_scenario.detect_conflicts(min(max_tla, t_arrive), wind, margin)
        
        if ((resolutions[0] == 'conflict') or (resolutions[0] == 'nosol')):
            if ((self.conflict_counter < conflict_counter_th) or (resolutions[0] == 'nosol')):
                self.conflict_counter = self.conflict_counter + 1
                return 'free'
        if (resolutions[0] == 'free'):
            self.conflict_counter = 0
            return 'free'
        
        if ((self.conflict_counter >= conflict_counter_th) and (resolutions[0] != 'nosol') and (t_arrive > avoidance_time_th)):
            #find resolutions
            resolution_points = []
            distances = []
            half_leg_distance = enu_distance(from_point_enu, to_point_enu) / 2.
            avoid_dist = min(half_leg_distance, avoid_dist_min)
            first_loop = True
            for i in range(len(resolutions[1])):
                # stop when not within thresholds
                x_res = resolutions[1][i]
                y_res = resolutions[2][i]
                hdg_res = np.rad2deg(np.arctan2(x_res, y_res)) % 360.
                hdg_diff = calc_absolute_hdg_diff(hdg, hdg_res)
                if (hdg_diff > hdg_diff_th):
                    if (first_loop == True):
                        if (checkeheadingdirection(hdg, hdg_res) == 'right'):
                            hdg_res = (hdg + 30.) % 360.
                        else:
                            hdg_res = (hdg - 30.) % 360.
                    else:
                        break
                
                first_loop = False
                
                resolution_dist = avoid_dist
                new_from_point_enu = geodetic.EnuCoor_f(from_point_enu.x + np.sin(np.deg2rad(hdg_res)) * resolution_dist, from_point_enu.y + np.cos(np.deg2rad(hdg_res)) * resolution_dist, altitude)
                
                linestring_to = geometry.LineString([(from_point_enu.x, from_point_enu.y), (new_from_point_enu.x, new_from_point_enu.y)])
                linestring_from = geometry.LineString([(new_from_point_enu.x, new_from_point_enu.y), (to_point_enu.x, to_point_enu.y)])
                    
                geofence_polygon = geometry.Polygon(enu_lst_to_polygon(geofence))
                    
                if ((geofence_polygon.contains(linestring_to) == False) or (geofence_polygon.contains(linestring_from) == False)):
                    # Non valid solution
                    continue
        
                for zone in zones:
                    zone_polygon = geometry.Polygon(enu_lst_to_polygon(zone.enu_points))
                    if (linestring_to.intersects(zone_polygon) or (linestring_from.intersects(zone_polygon))):
                        # Non valid solution
                        continue
                
                dx_target = to_point_enu.x - new_from_point_enu.x
                dy_target = to_point_enu.y - new_from_point_enu.y
                
                resolution_points.append(new_from_point_enu)
                distance = enu_distance(new_from_point_enu, from_point_enu) + enu_distance(new_from_point_enu, to_point_enu)
                distances.append(distance)
                    
            if len(resolution_points) > 0:
                resolution_point = resolution_points[-1]
                return resolution_point
            else:
                return 'nosol' 
        else:
            return 'free'
        
def check_area_conflicts(location, radius, time, ltp_def, margin, traffic_events):
    free_circle = geometry.Point((location.x, location.y)).buffer(radius)
    for traffic_event in traffic_events:
        pos_traf_lla = traffic_event.get_lla()
        pos_traf_enu = pos_traf_lla.to_enu(ltp_def)
        start_pos_traf = (pos_traf_enu.x, pos_traf_enu.y)
        Vx = traffic_event.get_gspeed()['east']
        Vy = traffic_event.get_gspeed()['north']
        end_pos_traffic = (start_pos_traf[0] + time * Vx, start_pos_traf[1] + time * Vy)
        traf_line_obstacle = geometry.LineString([start_pos_traf, end_pos_traffic])#.buffer(margin)
        if traf_line_obstacle.intersects(free_circle):
            return False
    return True
                    
def time_to_arrive_at_point(from_point_enu, to_point_enu, groundspeed):
    dist = np.sqrt((from_point_enu.x - to_point_enu.x) ** 2 + (from_point_enu.y - to_point_enu.y) ** 2)
    V = groundspeed
    if V != 0:
        time = dist/V
    else:
        time = 5.*60.
    return time
        
def calc_absolute_hdg_diff(hdg1, hdg2):
    diff = 180. - abs(180. - abs(hdg1 - hdg2) % 360.)
    return diff
        
def enu_distance(enu1, enu2):
    distance = np.sqrt((enu1.x - enu2.x) ** 2 + (enu1.y - enu2.y) ** 2)
    return distance
    
def enu_lst_to_polygon(enu_lst):
    coords = []
    for enu in enu_lst:
        coords.append((enu.x, enu.y))
    return coords
    
def checkeheadingdirection(hdg_current, hdg_new):
    hdg_diff = hdg_new - hdg_current
    
    if ((hdg_diff > 0) and (hdg_diff < 180)):
        return 'right'
    
    if ((hdg_diff > 0) and (hdg_diff > 180)):
        return 'left'
        
    if ((hdg_diff < 0) and (hdg_diff > -180)):
        return 'left'
    #else
    return 'right'