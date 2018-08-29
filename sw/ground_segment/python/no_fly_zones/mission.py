# -*- coding: utf-8 -*-
"""
Created on Wed Jul 25 22:25:41 2018

@author: dennis
"""

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import traffic_scenario
import copy
import resolution_functions as res_func
import route_functions as route_func
import coordinate_transformations as coord_trans
import traffic_server
import asterix_receiver
import static_nfz
import avoidance_gui as gui
import shapely.geometry as geometry


# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
#from settings_xml_parse import PaparazziACSettings
from pprz_math import geodetic
from time import sleep

class Mission(object):
    def __init__(self, ac_id, commanded_airspeed, Zones, Transit_points, alt, UAV, interface, soft_geofence, flightplan, platform_type):
        self.ac_id = ac_id
        self.commanded_airspeed = commanded_airspeed
        self.platform_type = platform_type
        self.Zones = Zones
        self.Transit_points = Transit_points
        self.soft_geofence = soft_geofence
        self.flightplan = flightplan
        self._interface = interface
        self.UAV = UAV
        self.receiver_thread = asterix_receiver.ReceiverThread(1, "Receiver", self.UAV.ref_utm_i, self._interface)
        self.receiver_thread.start()
        self.altitude = alt
        self.wind = {'east' : 0., 'north' : 0., 'up' : 0., 'airspeed' : 0.}
        #self.initial_route = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes([], self.Transit_points[1:-1], (self.Transit_points[0]['east'], self.Transit_points[0]['north']), (self.Transit_points[-1]['east'], self.Transit_points[-1]['north'])))
        self.initial_route = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes_cz(self.Zones, self.Transit_points[1:-1], (self.Transit_points[0]['east'], self.Transit_points[0]['north']), (self.Transit_points[-1]['east'], self.Transit_points[-1]['north'])))
        self.initial_route_in_legs = route_func.route_break_in_legs(self.Transit_points, self.initial_route)
        self.route = self.initial_route
        self.route_in_legs = self.initial_route_in_legs
        self.current_block = 0
        self.resolution_in_progress = np.zeros(len(self.Transit_points)-1 , dtype = bool)
        self.change_current_index = (False, None)
        
        self.indices = []
        self.messages = []
        self.current_message_id = 1
        self.assign_initial_indices()
        self.mission_comm = Mission_comm(self.ac_id, self._interface)
        
        self.mission_comm.parse_initial_mission(self)
        
        self.current_index_change = False
        self.set_current_index = 10
        
        
        self.leg_status = self.init_leg_status()
        print("leg_status: ", self.leg_status)
        
        self.detection_margin = 20. #[m]
        self.resolution_margin = 30. #[m]
        self.reduced_resolution_margin = 25. #[m]
        self.reduced_tla_step = 2. #[s] minimal reduced tla to be selected
        self.extended_tla = 10. #[s]
        self.reduced_tla_margin = 10. #[s]
        
        self.circular_zones = static_nfz.get_circular_zones(Zones, self.UAV.ref_utm_i)
        self.draw_circular_static_nfzs()
        self.traffic_scenario = traffic_scenario.traffic_scenario(self.UAV, 'HDG', self.circular_zones)
        self.traffic_scenario_extrapolated = traffic_scenario.traffic_scenario_extrapolated(self.UAV, 'HDG', self.circular_zones)
        
        self.tla = 60.
        
        self.gui = gui.Avoidance_GUI(self.detection_margin, self.resolution_margin, self.commanded_airspeed, self.reduced_tla_step, self.extended_tla, self.reduced_tla_margin, self.reduced_resolution_margin)
        
        def WIND_cb(ac_id, msg):
            if ac_id == self.ac_id:
                self.wind['east'] = float(msg['east'])
                self.wind['north'] = float(msg['north'])
                self.wind['up'] = float(msg['up'])
                self.wind['airspeed'] = float(msg['airspeed'])
        self._interface.subscribe(WIND_cb, PprzMessage("telemetry", "WIND_INFO_RET"))
        
        def NAVIGATION_cb(ac_id, msg):
            if ac_id == self.ac_id:
                self.current_block = int(msg['cur_block'])
        if self.platform_type == 'fw':
            self._interface.subscribe(NAVIGATION_cb, PprzMessage("telemetry", "NAVIGATION"))
            
        def ROTORCRAFT_NAV_STATUS_cb(ac_id, msg):
            if ac_id == self.ac_id:
                self.current_block = int(msg['cur_block'])
        if self.platform_type == 'rc':
            self._interface.subscribe(ROTORCRAFT_NAV_STATUS_cb, PprzMessage("telemetry", "ROTORCRAFT_NAV_STATUS"))
        
        self.run_mission()
        
    def reinit(self):
        self.initial_route = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes_cz(self.Zones, self.Transit_points[1:-1], (self.Transit_points[0]['east'], self.Transit_points[0]['north']), (self.Transit_points[-1]['east'], self.Transit_points[-1]['north'])))
        self.initial_route_in_legs = route_func.route_break_in_legs(self.Transit_points, self.initial_route)
        self.route = self.initial_route
        self.route_in_legs = self.initial_route_in_legs
        self.current_block = 0
        self.resolution_in_progress = np.zeros(len(self.Transit_points)-1 , dtype = bool)
        self.change_current_index = (False, None)
        
        self.indices = []
        self.messages = []
        self.current_message_id = 1
        self.assign_initial_indices()
        self.mission_comm = Mission_comm(self.ac_id, self._interface)
        
        self.mission_comm.parse_initial_mission(self)
        
        self.leg_status = self.init_leg_status()
        print("leg_status: ", self.leg_status)
        
        self.gui.resend = False
    
    def get_gui(self):
        self.detection_margin = self.gui.w_detm_slider.get()
        self.resolution_margin = self.gui.w_resm_slider.get()
        self.reduced_resolution_margin = self.gui.w_reduced_resolution_margin_slider.get()
        self.commanded_airspeed = self.gui.w_cas_slider.get()
        self.reduced_tla_step = self.gui.w_reduced_tla_step_slider.get()
        self.extended_tla = self.gui.w_extended_tla_slider.get()
        self.reduced_tla_margin = self.gui.w_reduced_tla_margin_slider.get()
        if self.gui.resend:
            self.reinit()
            
    def init_leg_status(self):
        leg_status = []
        for i in range(len(self.indices)-1):
            if(self.indices[i]/10 == self.indices[i+1]/10):
                leg_status.append('static_avoid')
            else:
                leg_status.append('to_target')
        leg_status.append('to_target')
        return leg_status
   
        
    def assign_initial_indices(self):
        for i in range(len(self.route_in_legs)):
            for j in range(len(self.route_in_legs[i])-1):
                self.indices.append(10*(i+1) + j)
                self.messages.append(self.current_message_id)
                self.current_message_id = self.current_message_id + 1
                if self.current_message_id == 255:
                    self.current_message_id = 1
        
    def get_current_index(self):
        return self.mission_comm.indices_confirmed[0]
        
    def draw_circular_static_nfzs(self):
        for i in range(len(self.circular_zones)):
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = i
            msg['linecolor'] = "orange"
            msg['fillcolor'] = "blue"
            msg['opacity'] = 0
            msg['shape'] = 0
            msg['status'] = 0
            msg['latarr'] = [self.circular_zones[i][2].lat, 0] # e-7 deg
            msg['lonarr'] = [self.circular_zones[i][2].lon, 0] # e-7 deg
            msg['radius'] = self.circular_zones[i][0]
            msg['text'] = str(i)
            self._interface.send(msg)
        
    
    ################################
    # Dynamic avoidance algotithms #
    ################################    
    
    def time_to_arrive_at_next_point(self, UAV):
        index = self.get_current_index
        target = self.route[1:][self.indices == index]
        x = UAV.P[0]
        y = UAV.P[1]
        t_x = target[0]
        t_y = target[1]
        dist = np.sqrt((x-t_x)**2 + (y-t_y)**2)
        V = np.sqrt(UAV.V[0]**2 + UAV.V[1]**2)
        if V != 0:
            time = dist/V
        else:
            time = 5.*60.
        return time
        
    def time_to_arrive_at_point(self, from_point, to_point, traffic_scenario):
        dist = np.sqrt((from_point[0]-to_point[0])**2 + (from_point[1]-to_point[1])**2)
        V = traffic_scenario.UAV_speed
        if V !=0:
            time = dist/V
        else:
            time = 5.*60.
        return time
    
    def check_collision_to_point(self, from_point, to_point, velocity, dt):
        hdg_to_target = np.rad2deg(np.arctan2(to_point[0] - from_point[0], to_point[1] - from_point[1]))
        if hdg_to_target < 0:
            hdg_to_target = hdg_to_target + 360.
            
        from_point_enu = geodetic.EnuCoor_f(from_point[0], from_point[1], self.altitude)
        
        self.traffic_scenario_extrapolated.update_traffic_scenario(velocity, hdg_to_target, self.wind, from_point_enu, dt, self.receiver_thread)

        tla = self.time_to_arrive_at_point(from_point, to_point, self.traffic_scenario_extrapolated)
        conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla, self.wind, self.detection_margin)
        return conflict[0], tla
        
    def check_collision_on_current_leg(self):
        from_point = (self.UAV.P[0], self.UAV.P[1])
        current_index = self.get_current_index()
        to_point = self.route_in_legs[current_index/10 - 1][-1]
        velocity = np.sqrt(self.UAV.V[0]**2 + self.UAV.V[1]**2)
        collision = self.check_collision_to_point(from_point, to_point, velocity, 0.)
        if collision[0] == 'conflict':
            resolution_point = self.resolution_on_leg(from_point, to_point, 0.)
            if resolution_point != 'nosol':
                self.resolution_in_progress[current_index/10 - 1] = True
                return resolution_point
        else:    
            return 'no conflict'

    def resolution_on_leg(self, from_point, to_point, dt, types = "both"):
        print("start resolution on leg function")
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        hdg = np.rad2deg(np.arctan2(dx, dy))
        if hdg < 0:
            hdg = hdg + 360.
            
        
        from_point_enu = geodetic.EnuCoor_f(from_point[0], from_point[1], self.altitude)
        
        self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg, self.wind, from_point_enu, dt, self.receiver_thread, types = "both")
        tla = self.time_to_arrive_at_point(from_point, to_point, self.traffic_scenario_extrapolated)
        
        reduced_tla = self.reduced_tla_step
        looping = True
        reduce_resolution_margin = False        
        
        while looping:
            self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg, self.wind, from_point_enu, dt, self.receiver_thread, types = "both")
            print("entering looping with t= ", reduced_tla)
            if reduce_resolution_margin:
                resolutions = self.traffic_scenario_extrapolated.detect_conflicts(reduced_tla + self.reduced_tla_margin, self.wind, self.reduced_resolution_margin)
            else:
                resolutions = self.traffic_scenario_extrapolated.detect_conflicts(reduced_tla + self.reduced_tla_margin, self.wind, self.resolution_margin)
            
            if ((resolutions[0] == 'nosol') or (resolutions[0] =='free')):
                if reduce_resolution_margin == False:
                    print("reducing margin because current calculation point within margin")
                    reduce_resolution_margin = True
                    reduced_tla = self.reduced_tla_step
                    resolutions = self.traffic_scenario_extrapolated.detect_conflicts(reduced_tla + self.reduced_tla_margin, self.wind, self.reduced_resolution_margin)
                    if ((resolutions[0] == 'nosol') or (resolutions[0] =='free')):
                        return 'nosol'
                else:
                    print('To close to obstacle to calculate resolution')
                    return 'nosol'       
                    
            
            if (len(resolutions[1]) < 30):
                n_i = len(resolutions[1])
            else:
                n_i = 30
                
            resolution_points = []
            distances = []
            rightleftsolution = {'right' : False, 'left' : False}
            
            for i in range(n_i-1):
                if (rightleftsolution['right'] and rightleftsolution['left']):
                    print ('Left and Right solution found')
                    sys.stdout.flush()
                    break
                
                hdg_res = np.arctan2(resolutions[1][i], resolutions[2][i])
                UAV_speed = (self.commanded_airspeed * np.sin(hdg_res) + self.wind['east'], self.commanded_airspeed * np.cos(hdg_res) + self.wind['north'])
                hdg_res = np.rad2deg(hdg_res)
                if hdg_res < 0:
                    hdg_res = hdg_res + 360.
                    
                res_dir = self.CheckRelativeHeadingDirection(hdg, hdg_res)
                
                if (res_dir == 'right' and not rightleftsolution['right']):
                    pass
                elif (res_dir == 'left' and not rightleftsolution['left']):
                    pass
                else:
                    continue

                new_from_point = (from_point[0] + UAV_speed[0]*reduced_tla, from_point[1] + UAV_speed[1]*reduced_tla)
                new_from_point_geom = geometry.Point(new_from_point[0], new_from_point[1])
                if (new_from_point_geom.within(self.soft_geofence.polygon) == False):
                    continue
                new_from_point_enu = geodetic.EnuCoor_f(new_from_point[0], new_from_point[1], self.altitude)
                
                dx_target = to_point[0] - new_from_point[0]
                dy_target = to_point[1] - new_from_point[1]
                
                hdg_target = np.rad2deg(np.arctan2(dx_target, dy_target))
                
                
                
                self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg_target, self.wind, new_from_point_enu, dt + reduced_tla, self.receiver_thread, types = types)
                tla_target = self.time_to_arrive_at_point(new_from_point, to_point, self.traffic_scenario_extrapolated)
                
                if reduce_resolution_margin:
                    conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla_target, self.wind, self.reduced_resolution_margin)
                else:
                    conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla_target, self.wind, self.resolution_margin)
                
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
                if (reduced_tla <= tla + self.extended_tla):
                    reduced_tla = reduced_tla + self.reduced_tla_step
                else:
                    if not reduce_resolution_margin:
                        print("Reducing margin for resolution")
                        reduce_resolution_margin = True;
                        reduced_tla = self.reduced_tla_step
                    else:
                        looping = False
                        print("resolution_on_leg: no solutions found on leg")
                        return 'nosol'
            
        
    def CheckRelativeHeadingDirection(self, hdg_current, hdg_new):
        hdg_diff = hdg_new - hdg_current
        
        if ((hdg_diff > 0) and (hdg_diff < 180)):
            return 'right'
        
        if ((hdg_diff > 0) and (hdg_diff > 180)):
            return 'left'
            
        if ((hdg_diff < 0) and (hdg_diff > -180)):
            return 'left'
        #else
        return 'right'
        
    def process_current_leg(self):
        # Check if direct route to target is free
        from_point = (self.UAV.P[0], self.UAV.P[1])
        from_point_2 = (from_point[0] + 2 * self.UAV.V[0], from_point[1] + 2 * self.UAV.V[1]) # from point extrapolated by 2 seconds
        current_index = self.get_current_index() # current index send over datalink
        try:
            current_i = self.indices.index(current_index)
        except ValueError:
            print("process_current_leg: not synchronus with live flight")
            return
        leg_i = current_index / 10 - 1
        to_point = self.route_in_legs[current_index/10 - 1][-1]
        hdg_to_target = np.rad2deg(np.arctan2(to_point[0] - from_point[0], to_point[1] - from_point[1]))
        if hdg_to_target < 0:
            hdg_to_target = hdg_to_target + 360.
            
        from_point_enu = geodetic.EnuCoor_f(from_point[0], from_point[1], self.altitude)
        
        try:
            if(self.indices[current_i+1] == ((current_index / 10) + 1)*10):
                self.resolution_in_progress[leg_i] = False
        except IndexError:
            self.resolution_in_progress[leg_i] = False
            
        # Use detection margin if no resolution in progress on current leg
        if self.resolution_in_progress[leg_i] == False:
            self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg_to_target, self.wind, from_point_enu, 0., self.receiver_thread)
            tla = self.time_to_arrive_at_point(from_point, to_point, self.traffic_scenario_extrapolated)
            self.tla = tla
            conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla, self.wind, self.detection_margin)
            if conflict[0] == 'conflict':
                resolution_point = self.resolution_on_leg(from_point_2, to_point, 0.)
                if resolution_point != 'nosol':
                    self.resolution_in_progress[leg_i] = True
                    self.route = self.route[:current_i] + [from_point_2, resolution_point] + self.route[current_i+1:]
                    #self.route_in_legs = route_func.route_break_in_legs(self.Transit_points, self.route)
                    self.indices = self.indices[:current_i+1] + [current_index + 1] + self.indices[current_i+1:]
                    self.messages = self.messages[:current_i] + [self.current_message_id, self.current_message_id + 1] + self.messages[current_i+1:]
                    self.current_message_id = self.current_message_id + 2
                    if self.current_message_id >= 254:
                        self.current_message_id = 1
            
        else:
            # Check for other collisions on current leg even if already in avoidance mode
            
            avoidance_point = self.route[current_i]

            hdg_to_avoidance = np.rad2deg(np.arctan2(avoidance_point[0] - from_point[0], avoidance_point[1] - from_point[1]))
            if hdg_to_avoidance < 0:
                hdg_to_avoidance = hdg_to_avoidance + 360.
                
            from_point_enu = geodetic.EnuCoor_f(from_point[0], from_point[1], self.altitude)
            
            self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg_to_avoidance, self.wind, from_point_enu, 0., self.receiver_thread)
            tla = self.time_to_arrive_at_point(from_point, avoidance_point, self.traffic_scenario_extrapolated)
            self.tla = tla
            conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla, self.wind, self.detection_margin)
            
            if conflict[0] == 'conflict':
                resolution_point = self.resolution_on_leg(from_point_2, to_point, 0.)
                if resolution_point != 'nosol':
                    self.route = self.route[:current_i] + [from_point_2, resolution_point] + self.route[current_i+2:]
                    #self.route_in_legs = route_func.route_break_in_legs(self.Transit_points, self.route)
                    #self.indices = self.indices[:current_i+1] + [current_index + 1] + self.indices[current_i+1:]
                    self.messages = self.messages[:current_i] + [self.current_message_id, self.current_message_id + 1] + self.messages[current_i+2:]
                    self.current_message_id = self.current_message_id + 2
                    if self.current_message_id >= 254:
                        self.current_message_id = 1
            
            # check if route is free towards target point with resolution margin
            self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg_to_target, self.wind, from_point_enu, 0., self.receiver_thread)
            tla = self.time_to_arrive_at_point(from_point, to_point, self.traffic_scenario_extrapolated)
            conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla, self.wind, self.resolution_margin)
            if ((conflict[0] == 'free') and tla > 0):
                self.change_current_index = (True, current_index / 10 * 10)
                print('free of conflict to target')
                # directly go to target again
                
                next_leg_i = leg_i + 2
                try:
                    begin_leg_i = self.indices.index((leg_i+1)*10)
                except ValueError:
                    print("process_current_leg: begin")
                    return
                    
                self.resolution_in_progress[leg_i] = False
                    
                final_leg = False
                try:
                    next_leg_i = self.indices.index(next_leg_i*10) # leg exists
                except ValueError:
                    final_leg = True
                    
                if not final_leg:    
                    self.route = self.route[:begin_leg_i] + [from_point_2] + self.route[next_leg_i:]
                    self.indices = self.indices[:begin_leg_i+1] + self.indices[next_leg_i:]
                    self.messages = self.messages[:begin_leg_i] + [self.current_message_id] + self.messages[next_leg_i:]
                    self.current_message_id = self.current_message_id + 1
                    if self.current_message_id >= 254:
                        self.current_message_id = 1
                        
                else:
                    self.route = self.route[:begin_leg_i] + [from_point_2, to_point]
                    self.indices = self.indices[:begin_leg_i+1]
                    self.messages = self.messages[:begin_leg_i] + [self.current_message_id]
                    self.current_message_id = self.current_message_id + 1
                    if self.current_message_id >= 254:
                        self.current_message_id = 1
    
    def process_leg(self, leg_i, dt, index = 'start', from_point = 'start'):
        if index == 'start':
            index = (leg_i+1)*10        
        
        indices_i = self.indices.index(index)
        indices_i_start_leg = self.indices.index(index/10*10)
        
        final_leg = False
        try:
            indices_i_next_leg = self.indices.index((index+10)/10*10)
        except ValueError:
            final_leg = True
        
        if from_point == 'start':
            from_point = self.route[indices_i]
        
        target_point = self.route_in_legs[leg_i][-1]
        hdg_to_target = np.rad2deg(np.arctan2(target_point[0] - from_point[0], target_point[1] - from_point[1]))
        if hdg_to_target < 0:
            hdg_to_target = hdg_to_target + 360.
        
        next_point = self.route[indices_i+1]
        hdg_to_next = np.rad2deg(np.arctan2(next_point[0] - from_point[0], next_point[1] - from_point[1]))
        if hdg_to_next < 0:
            hdg_to_next = hdg_to_next + 360.
        
        from_point_enu = geodetic.EnuCoor_f(from_point[0], from_point[1], self.altitude)
        
        # Use detection margin towards next point if no dynamix avoidance in progress
        if ((self.leg_status[indices_i] == 'to_target') or (self.leg_status[indices_i] == 'static_avoid')):
            
            self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg_to_next, self.wind, from_point_enu, dt, self.receiver_thread, types = "dynamic")
            tla = self.time_to_arrive_at_point(from_point, next_point, self.traffic_scenario_extrapolated)
            self.tla = tla
            conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla, self.wind, self.detection_margin)
            if conflict[0] == 'conflict':
                resolution_point = self.resolution_on_leg(from_point, target_point, dt, types="dynamic")
                    
                if resolution_point != 'nosol':
                    # check if only static avoidance has to be applied
                    print("resolution_point found")
                
                    if (self.check_intersecting_static_nfzs(from_point, resolution_point)):
                        print("resolution_point_ intersecting static zone: ", resolution_point)
                        new_leg = res_func.leg_return_shortest_leg_cz(self.Zones, from_point, target_point)
                        
                        hdg_to_new_point = np.rad2deg(np.arctan2(new_leg[1][0] - from_point[0], new_leg[1][1] - from_point[1]))
                        if hdg_to_new_point < 0:
                            hdg_to_new_point = hdg_to_next + 360.
                        self.traffic_scenario_extrapolated.update_traffic_scenario(self.commanded_airspeed, hdg_to_new_point, self.wind, from_point_enu, dt, self.receiver_thread, types = "dynamic")
                        conflict = self.traffic_scenario_extrapolated.detect_conflicts(tla, self.wind, self.detection_margin)
                        
                        # if new dynamic avoid needed because on new leg still conflict
                        if conflict[0] == 'conflict':
                            resolution_point = self.resolution_on_leg(from_point, target_point, dt, types="both")
                            if resolution_point != 'nosol':
                                resetted_index = index/10*10
                                self.current_index_change = True
                                self.mission_comm.set_current_index(resetted_index)
                                if final_leg == False:
                                    self.route = self.route[:indices_i_start_leg] + [from_point, resolution_point, target_point] + self.route[indices_i_next_leg+1:]
                                    self.indices = self.indices[:indices_i_start_leg] + [resetted_index, resetted_index + 1] + self.indices[indices_i_next_leg:]
                                    self.leg_status = self.leg_status[:indices_i_start_leg]  + ["dynamic_avoid", "to_target"] + self.leg_status[indices_i_next_leg]
                                    self.messages = self.messages[:indices_i_start_leg] + [self.current_message_id, self.current_message_id + 1] + self.messages[indices_i_next_leg:]
                                    self.current_message_id = self.current_message_id + 2
                                    if self.current_message_id >= 240:
                                        self.current_message_id = 1
                                else:
                                    self.route = self.route[:indices_i_start_leg] + [from_point, resolution_point, target_point]
                                    self.indices = self.indices[:indices_i_start_leg] + [resetted_index, resetted_index + 1]
                                    self.leg_status = self.leg_status[:indices_i_start_leg]  + ["dynamic_avoid", "to_target"]
                                    self.messages = self.messages[:indices_i_start_leg] + [self.current_message_id, self.current_message_id + 1]
                                    self.current_message_id = self.current_message_id + 2
                                    if self.current_message_id >= 240:
                                        self.current_message_id = 1
                                        
                        # if new static avoid leg calculated
                        else:
                            print("valid point resolution_point: ", resolution_point)
                            resetted_index = index/10*10
                            self.mission_comm.set_current_index(resetted_index)
                            new_leg_length = len(new_leg)
                            new_leg_index_range = range(resetted_index, resetted_index + new_leg_length-1)
                            new_leg_status = []
                            for i in range(len(new_leg_index_range)):
                                if i != len(new_leg_index_range):
                                    new_leg_status.append("static_avoid")
                                else:
                                    new_leg_status.append("to_target")
                            new_leg_messages = range(self.current_message_id, self.current_message_id + new_leg_length-1)
                            if final_leg == False:
                                self.route = self.route[:indices_i_start_leg] + new_leg + self.route[indices_i_next_leg+1:]
                                self.indices = self.indices[:indices_i_start_leg] + new_leg_index_range + self.indices[indices_i_next_leg:]
                                self.leg_status = self.leg_status[:indices_i_start_leg]  + ["dynamic_avoid", "to_target"] + self.leg_status[indices_i_next_leg:]
                                self.messages = self.messages[:indices_i_start_leg] + new_leg_messages + self.messages[indices_i_next_leg:]
                                self.current_message_id = self.current_message_id + len(new_leg_messages)
                                if self.current_message_id >= 240:
                                    self.current_message_id = 1
                            else:
                                self.route = self.route[:indices_i_start_leg] + new_leg
                                self.indices = self.indices[:indices_i_start_leg] + new_leg_index_range
                                self.leg_status = self.leg_status[:indices_i_start_leg]  + ["dynamic_avoid", "to_target"]
                                self.messages = self.messages[:indices_i_start_leg] + new_leg_messages
                                self.current_message_id = self.current_message_id + len(new_leg_messages)
                                if self.current_message_id >= 240:
                                    self.current_message_id = 1
                    else:
                         print("resolution_point_ free of static zones: ", resolution_point)
                         resetted_index = index/10*10
                         self.mission_comm.set_current_index(resetted_index)
                         if final_leg == False:
                            self.route = self.route[:indices_i_start_leg] + [from_point, resolution_point, target_point] + self.route[indices_i_next_leg+1:]
                            self.indices = self.indices[:indices_i_start_leg] + [resetted_index, resetted_index + 1] + self.indices[indices_i_next_leg:]
                            self.leg_status = self.leg_status[:indices_i_start_leg]  + ["dynamic_avoid", "to_target"] + self.leg_status[indices_i_next_leg:]
                            self.messages = self.messages[:indices_i_start_leg] + [self.current_message_id, self.current_message_id + 1] + self.messages[indices_i_next_leg:]
                            self.current_message_id = self.current_message_id + 2
                            if self.current_message_id >= 240:
                                self.current_message_id = 1
                         else:
                            self.route = self.route[:indices_i_start_leg] + [from_point, resolution_point, target_point]
                            self.indices = self.indices[:indices_i_start_leg] + [resetted_index, resetted_index + 1]
                            self.leg_status = self.leg_status[:indices_i_start_leg]  + ["dynamic_avoid", "to_target"]
                            self.messages = self.messages[:indices_i_start_leg] + [self.current_message_id, self.current_message_id + 1]
                            self.current_message_id = self.current_message_id + 2
                            if self.current_message_id >= 240:
                                self.current_message_id = 1
    
    
        
                     

    def experiment_run_current_leg(self):
        # Check if direct route to target is free
        from_point = (self.UAV.P[0], self.UAV.P[1])
        from_point_2 = (from_point[0] + 4 * self.UAV.V[0], from_point[1] + 4 * self.UAV.V[1]) # from point extrapolated by 2 seconds
        current_index = self.get_current_index() # current index send over datalink
        try:
            current_i = self.indices.index(current_index)
        except ValueError:
            print("process_current_leg: not synchronus with live flight")
            return
        leg_i = current_index / 10 - 1
        
        self.process_leg(leg_i, 0., index = current_index, from_point = from_point_2)
        
    def check_intersecting_static_nfzs(self, from_point, to_point):
        intersecting = False
        line = geometry.LineString([from_point, to_point])
        for i in range(len(self.Zones)):
            if line.intersects(self.Zones[i].circle['geo']):
                intersecting = True
                return intersecting
        return intersecting
            
        
        
        
    def run_mission(self):
        self.traffic_scenario.init_SSD_plot()
        try:
            while True:
                # First do traffic calculations and visualize
                sleep(0.5)
                try:
                    self.get_gui()
                # If widget closed
                except RuntimeError:
                    pass
                
                #print("indices: ", self.indices)
                #print("messages: ", self.messages)
                #print("route: ", self.route)
                
                self.traffic_scenario.update_traffic_scenario(self.UAV, self.receiver_thread)
                if self.traffic_scenario.Traffic.ntraf > 1:
                    try:
                        self.traffic_scenario.detect_conflicts(self.tla, self.wind, self.detection_margin)
                        self.traffic_scenario.plot_SSD() 
                    except: # All errors to overcome pyclipper error UnboundLocalError: # When simulated aircraft are too far
                        pass # Do nothing
                    if self.current_block == self.flightplan.block_name_lookup['RUN MISSION'].no :
                        #self.process_current_leg()
                        self.experiment_run_current_leg()
                self.mission_comm.parse_missing_messages(self)
                visualization = True
                if visualization:
                    self.mission_comm.visualize_legs(self)
                    visualization = False
                sys.stdout.flush()
                # Afterwards check conclficts and check resolution points

        except KeyboardInterrupt:
            self.stop()
        
class Mission_comm(object):
    def __init__(self, ac_id, interface):
        self.ac_id = ac_id
        self._interface = interface
        self.indices = []
        self.indices_confirmed = [0]
        self.messages = []
        self.messages_confirmed = [0]
        self.set_current_index_flag = False
        self.set_current_index_value = False
        
        def mission_status_cb(ac_id, msg):
            if ac_id == self.ac_id:
                indices_confirmed = msg['index_list']
                if(isinstance(indices_confirmed, basestring)):
                    indices_confirmed = [indices_confirmed]
                messages_confirmed = msg['msg_id_list']
                if(isinstance(messages_confirmed, basestring)):
                    messages_confirmed = [messages_confirmed]
                self.indices_confirmed = []
                self.messages_confirmed = []
                for i in range(len(indices_confirmed)):
                    self.indices_confirmed.append(int(indices_confirmed[i]))
                    self.messages_confirmed.append(int(messages_confirmed[i]))
        self._interface.subscribe(mission_status_cb, PprzMessage("telemetry", "MISSION_STATUS"))
        
    #######################################
    # Message parsing and check functions #
    #######################################
    def parse_initial_mission(self, mission_object):
        self.indices = mission_object.indices
        self.messages = mission_object.messages
        for i in range(len(self.indices)):
            self.parse_ordered_segment(mission_object, self.indices[i])
            self.visualize_leg(mission_object, self.indices[i])
      
    def update_indices_and_messages(self, mission_object):
        if self.indices_confirmed[0] == self.set_current_index_value:
            self.set_current_index_flag = False
        if self.set_current_index_flag:
            current_index = self.set_current_index_value
        else:
            current_index = self.indices_confirmed[0]
            
        try:
            i = mission_object.indices.index(current_index) # Start index list of comm object at currrent index
        except ValueError:
            return # Do nothing
        else:
            self.indices = mission_object.indices[i:]
            self.messages = mission_object.messages[i:]
    
    def set_current_index(self, index):
        self.set_current_index_flag = True
        self.set_current_index_value = index
        
    def parse_missing_messages(self, mission_object):
        self.update_indices_and_messages(mission_object)
        # Check if messages are all parsed or most recent and (re)parse them
        
        # Check if segments must be deleted
        # Make sure not deleting the only element causing the UAV going to standby
        if (len(self.indices_confirmed) > 1):
            for j in range(len(self.indices_confirmed)):
                try:
                    self.indices.index(self.indices_confirmed[j])
                # If segment is not found in current index list and must be deleted 
                except ValueError:
                    if (self.indices_confirmed[0] != 0 and self.indices_confirmed[j] != 0):                
                        self.parse_delete_index(self.indices_confirmed[j])
                else:
                    pass
            
        for i in range(len(self.indices)):
            index = self.indices[i]
            
            # Check if index is in confirmed index list
            try:
                message_index = self.indices_confirmed.index(self.indices[i]) 
            except ValueError: # If index has not correctly been parsed, parse message again
                if self.indices_confirmed[0] != 0: #mission is still running
                    self.parse_ordered_segment(mission_object, index)
            else: # If index has been parsed to the UAV
                if(self.messages_confirmed[message_index] != self.messages[i]): # Check if message id is the most recent one
                    self.parse_ordered_segment(mission_object, index) # If confirmed message is not most recent one, resend
            
        # If start index is lower than current index
        if (mission_object.change_current_index[0] == True):
            index = mission_object.change_current_index[1]
            self.parse_ordered_segment(mission_object, index)
            mission_object.change_current_index = (False, None)
        
    def parse_ordered_segment(self, mission_object, index):
        print("parse message with index ", index)
        i = mission_object.indices.index(index)
        msg = PprzMessage("datalink", "MISSION_SEGMENT_LLA")
        msg['ac_id'] = self.ac_id
        msg['insert'] = 5
        point_enu = geodetic.EnuCoor_f(float(mission_object.route[i][0]), float(mission_object.route[i][1]), float(mission_object.altitude))
        point_lla = coord_trans.enu_to_lla_fw(point_enu, mission_object.UAV.ref_utm_i)
        msg['segment_lat_1'] = point_lla.lat + mission_object.UAV.latlon_error[0]
        msg['segment_lon_1'] = point_lla.lon + mission_object.UAV.latlon_error[1]
        point_enu = geodetic.EnuCoor_f(float(mission_object.route[i+1][0]), float(mission_object.route[i+1][1]), float(mission_object.altitude))
        point_lla = coord_trans.enu_to_lla_fw(point_enu, mission_object.UAV.ref_utm_i)
        msg['segment_lat_2'] = point_lla.lat + mission_object.UAV.latlon_error[0]
        msg['segment_lon_2'] = point_lla.lon + mission_object.UAV.latlon_error[1]
        msg['segment_alt'] = int(mission_object.altitude * 1000.)
        msg['duration'] = 0
        msg['index'] = int(index)
        msg['msg_id'] = int(mission_object.messages[i])
        self._interface.send(msg)
        #self.visualize_leg(mission_object, index)
        
    def parse_delete_index(self, index):
        print("delete message with index ", index)
        msg = PprzMessage("datalink", "MISSION_DELETE_INDEX")
        msg['ac_id'] = self.ac_id
        msg['index'] = index
        self._interface.send(msg)
        
        
    def visualize_leg(self, mission_object, index):
        # if index doesnt exist any more
        try:
            i = mission_object.indices.index(index)
        except ValueError:
            return
        point_enu_prev = geodetic.EnuCoor_f(float(mission_object.route[i][0]), float(mission_object.route[i][1]), 0.)
        point_lla_prev = coord_trans.enu_to_lla_fw(point_enu_prev, mission_object.UAV.ref_utm_i)
        latarr = [point_lla_prev.lat + mission_object.UAV.latlon_error[0]]
        lonarr = [point_lla_prev.lon + mission_object.UAV.latlon_error[1]]
        point_enu = geodetic.EnuCoor_f(float(mission_object.route[i+1][0]), float(mission_object.route[i+1][1]), 0.)
        point_lla = coord_trans.enu_to_lla_fw(point_enu, mission_object.UAV.ref_utm_i) #!!!
        latarr.append(point_lla.lat + mission_object.UAV.latlon_error[0])
        lonarr.append(point_lla.lon + mission_object.UAV.latlon_error[1])
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 64+index
        msg['linecolor'] = "white"
        msg['fillcolor'] = "blue"
        msg['opacity'] = 1
        msg['shape'] = 2
        msg['status'] = 0
        msg['latarr'] = latarr # e-7 deg
        msg['lonarr'] = lonarr # e-7 deg
        msg['radius'] = 500.
        self._interface.send(msg)
        
    def devisualize_leg(self, index):
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 64+index
        msg['linecolor'] = "white"
        msg['fillcolor'] = "blue"
        msg['opacity'] = 1
        msg['shape'] = 2
        msg['status'] = 1
        msg['latarr'] = [0, 0] # e-7 deg
        msg['lonarr'] = [0, 0] # e-7 deg
        msg['radius'] = 500.
        self._interface.send(msg)
        
    def visualize_legs(self, mission_object):
        for index in range(255-64):
            if ((index in self.indices_confirmed) and (index != 0)):
                self.visualize_leg(mission_object, index)
            else:
                self.devisualize_leg(index)