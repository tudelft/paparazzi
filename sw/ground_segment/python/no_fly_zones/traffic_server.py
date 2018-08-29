#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import shapely.geometry as geometry
#import shapely.affinity as affinity
#import shapely.ops as ops
import resolution_functions as res_func
import route_functions as route_func
import mission
import traffic_scenario
import coordinate_transformations as coord_trans
import time
from itertools import islice
import json
import random
import copy
import copyreg
import asterix_receiver
import static_nfz

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
import flightplan_xml_parse
    
# Read static NFZ's from textfile    
    
def create_Zones_fw(filename, ref_utm_i):
    # First identify the pieces of text per NFZ
    startlines = []
    with open(filename, 'r') as f:
        for (i, line) in enumerate(f):
            if 'createnfz' in line.lower():
                startlines.append(i+1)
    f.close()
        
    # Secondly read each piece of text
    Zone_coord = [] #lat, lon
    for i in range(len(startlines)):
        with open(filename) as lines:
            Zone_coord.append(np.genfromtxt(islice(lines, startlines[i], startlines[i]+4), delimiter=','))
            
    # Thirdly create an list containing Zone objects
    Zones = []
    for i in range(len(Zone_coord)):
        latarr = []
        lonarr = []
        xy_arr = []
        for j in range(len(Zone_coord[i])):
            lat = Zone_coord[i][j,0]
            lon = Zone_coord[i][j,1]
            point_lla = geodetic.LlaCoor_i(int(lat*10**7), int(lon*10**7), int(0*1000))
            point_enu = coord_trans.lla_to_enu_fw(point_lla, ref_utm_i)
            x = point_enu.x
            y = point_enu.y
            print("Zone ", i,", point ", j, " x=", x, " y=", y)
            sys.stdout.flush()  
            latarr.append(int(lat*10**7))
            lonarr.append(int(lon*10**7))
            xy_arr.append((x,y))
        Zones.append(Zone(latarr, lonarr, xy_arr, i))
    return Zones
    
def create_Zones_rc(filename, ref_lla_i):
    # First identify the pieces of text per NFZ
    startlines = []
    with open(filename, 'r') as f:
        for (i, line) in enumerate(f):
            if 'createnfz' in line.lower():
                startlines.append(i+1)
    f.close()
        
    # Secondly read each piece of text
    Zone_coord = [] #lat, lon
    for i in range(len(startlines)):
        with open(filename) as lines:
            Zone_coord.append(np.genfromtxt(islice(lines, startlines[i], startlines[i]+4), delimiter=','))
            
    # Thirdly create an list containing Zone objects
    Zones = []
    for i in range(len(Zone_coord)):
        latarr = []
        lonarr = []
        xy_arr = []
        for j in range(len(Zone_coord[i])):
            lat = Zone_coord[i][j,0]
            lon = Zone_coord[i][j,1]
            point_lla = geodetic.LlaCoor_i(int(lat*10**7), int(lon*10**7), int(0*1000))
            point_enu = coord_trans.lla_to_enu_fw(point_lla, ref_utm_i)
            x = point_enu.x
            y = point_enu.y
            print("Zone ", i,", point ", j, " x=", x, " y=", y)
            sys.stdout.flush()  
            latarr.append(int(lat*10**7))
            lonarr.append(int(lon*10**7))
            xy_arr.append((x,y))
        Zones.append(Zone(latarr, lonarr, xy_arr, i))
    return Zones
    
# Read geofences from file
    
def create_geofence(filename, ref_utm_i):
    coordinates = np.genfromtxt(filename, delimiter=',')
    latarr = []
    lonarr = []
    xy_arr = []
    for i in range(len(coordinates)):
        lat = coordinates[i][0]
        lon = coordinates[i][1]
        point_lla = geodetic.LlaCoor_i(int(lat*10**7), int(lon*10**7), int(0*1000))
        point_enu = coord_trans.lla_to_enu_fw(point_lla, ref_utm_i)
        x = point_enu.x
        y = point_enu.y
        print("Geofence ", i, " x=", x, " y=", y)
        sys.stdout.flush()  
        latarr.append(int(lat*10**7))
        lonarr.append(int(lon*10**7))
        xy_arr.append((x,y))
    geofence = Zone(latarr, lonarr, xy_arr, 0)
    return geofence

def create_Transit_points_fw(filename, ref_utm_i):    
    WP_coords_ll = np.genfromtxt(filename, delimiter=',', dtype=('a5', float, float)) #Name, lat, lon
    
    WP_coords_en = []
    for i in range(len(WP_coords_ll)):
       WP_coord_en = {}  # Name, East, North
       WP_coord_en['name'] = WP_coords_ll[i][0] # Append name
       WP_lla = geodetic.LlaCoor_i(int(WP_coords_ll[i][1]*10**7),  int(WP_coords_ll[i][2]*10**7), int(0*1000))
       WP_enu = coord_trans.lla_to_enu_fw(WP_lla, ref_utm_i)
       WP_coord_en['east'] = WP_enu.x # in m
       WP_coord_en['north'] = WP_enu.y # in m
       WP_coords_en.append(WP_coord_en)
    return WP_coords_en    
    
def create_Transit_points_rc(filename, ref_lla_i):    
    WP_coords_ll = np.genfromtxt(filename, delimiter=',', dtype=('a5', float, float)) #Name, lat, lon
    
    WP_coords_en = []
    for i in range(len(WP_coords_ll)):
       WP_coord_en = {}  # Name, East, North
       WP_coord_en['name'] = WP_coords_ll[i][0] # Append name
       WP_lla = geodetic.LlaCoor_i(int(WP_coords_ll[i][1]*10**7),  int(WP_coords_ll[i][2]*10**7), int(0*1000))
       WP_enu = coord_trans.lla_to_enu_rc(WP_lla, ref_lla_i)
       WP_coord_en['east'] = WP_enu.x # in m
       WP_coord_en['north'] = WP_enu.y # in m
       WP_coords_en.append(WP_coord_en)
    return WP_coords_en    
    
def write_mission_file(filename, mission):
    with open(filename, 'w') as status_file:
        status_file.write(json.dumps(mission.__dict__))

# Class containg a NFZ
class Zone(object):
    def __init__(self, latarr, lonarr,  xy_arr, id_zone):
        self.latarr = latarr
        self.lonarr = lonarr
        self.id_zone = id_zone
        self.xy_arr = xy_arr
        self.polygon = geometry.Polygon(xy_arr)
        self.circle = static_nfz.get_circle_geometry(self.xy_arr)
        
    def visualize_Zone(self):
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = int(self.id_zone)
        msg['linecolor'] = "orange"
        msg['fillcolor'] = "yellow"
        msg['opacity'] = 1
        msg['shape'] = 1
        msg['status'] = 0
        msg['latarr'] = self.latarr # e-7 deg
        msg['lonarr'] = self.lonarr # e-7 deg
        msg['radius'] = 500.
        msg['text'] = "Zone" + str(self.id_zone)
        return msg
        
# Class containg a geofence
class Geofence(object):
    def __init__(self, latarr, lonarr,  xy_arr, id_zone):
        self.latarr = latarr
        self.lonarr = lonarr
        self.id_zone = id_zone
        self.xy_arr = xy_arr
        self.polygon = geometry.Polygon(xy_arr)
    
# Class of UAV including relevant state
class ControlledUAV(object):
    def __init__(self, ac_id):
        self.id = ac_id
        self.P = np.zeros(3) # position x, y, z
        self.V = np.zeros(3) # velocity x, y, z
        self.ref_lla_i = geodetic.LlaCoor_i()
        self.ref_utm_i = geodetic.UtmCoor_i()
        self.latlon_error = None
        #self.ahead_vector = geometry.LineString()
        self.hdg = 0.
        #self.current_leg_parse_index = 0
    def __reduce__(self):
        return (self.__class__, (self.ref_lla_i, ), (self.ref_lla_i, ))
            
# Traffic server including message definitions, conflict detection and actions
class TrafficServer(object):
    def __init__(self, ac_id, platform_type, verbose=True, interface=None):
        self.ac_id = ac_id
        self.UAV = ControlledUAV(ac_id)
        self.verbose = verbose
        self._interface = IvyMessagesInterface("Traffic Server")
        self.leg_visualization_index = 0
        self.platform_type = platform_type
        self.wind = {'east' : 0., 'north' : 0., 'up' : 0., 'airspeed' : 0.}
        self.mission = None
        self.flightplan = flightplan_xml_parse.PaparazziACFlightplan(self.ac_id)
        
#        def WIND_cb(ac_id, msg):
#            self.wind['east'] = float(msg['east'])
#            self.wind['north'] = float(msg['north'])
#            self.wind['up'] = float(msg['up'])
#            self.wind['airspeed'] = float(msg['airspeed'])
#        self._interface.subscribe(WIND_cb, PprzMessage("telemetry", "WIND_INFO_RET"))
                
        def GPS_cb(ac_id, msg):
            if ac_id == self.ac_id:
                UAV = self.UAV
                UAV.hdg = float(msg['course'])/10.
                course = UAV.hdg/180.*np.pi # rad
                h_speed = float(msg['speed'])/100. # m/s
                UAV.V[0] = h_speed*np.sin(course)
                UAV.V[1] = h_speed*np.cos(course)
                UAV.V[2] = float(msg['climb'])/100.
                UAV.P[2] = float(msg['alt'])/1000.
        if self.platform_type == 'fw':
            self._interface.subscribe(GPS_cb, PprzMessage("telemetry", "GPS"))
            
        def INS_cb(ac_id, msg):
            if ac_id == self.ac_id:
                UAV = self.UAV
#                pos_unit_coefficient = 0.0039063 # m
#                UAV.P[0] = float(msg['ins_y']) * pos_unit_coefficient
#                UAV.P[1] = float(msg['ins_x']) * pos_unit_coefficient
#                UAV.P[2] = float(msg['ins_z']) * -pos_unit_coefficient
                speed_unit_coeficient = 0.0000019 # m
                UAV.V[0] = float(msg['ins_yd']) * speed_unit_coeficient
                UAV.V[1] = float(msg['ins_xd']) * speed_unit_coeficient
                UAV.V[2] = float(msg['ins_zd']) * -speed_unit_coeficient
                bearing = np.rad2deg(np.arctan2(UAV.V[0], UAV.V[1]))
                if bearing < 0.:
                    bearing = bearing + 360.
                UAV.hdg = bearing
        if self.platform_type == 'rc':
            self._interface.subscribe(INS_cb, PprzMessage("telemetry", "INS"))
            
        def GPS_INT_cb(ac_id, msg):
            if ac_id == self.ac_id:
                UAV = self.UAV
                lat = int(msg['lat']) # 10^7 deg
                lon = int(msg['lon']) # 10^7 deg
                alt = int(msg['alt']) # in mm
                point_lla = geodetic.LlaCoor_i(lat, lon, alt)
                point_enu = coord_trans.lla_to_enu_fw(point_lla, self.UAV.ref_utm_i)
                UAV.P[0] = point_enu.x
                UAV.P[1] = point_enu.y
                UAV.P[2] = point_enu.z
        if self.platform_type == 'rc':
            self._interface.subscribe(GPS_INT_cb, PprzMessage("telemetry", "GPS_INT"))
                
        def nav_cb(ac_id, msg):
            if ac_id == self.ac_id:
                UAV = self.UAV
                UAV.P[0] = float(msg['pos_x'])
                UAV.P[1] = float(msg['pos_y'])
        if self.platform_type == 'fw':
            self._interface.subscribe(nav_cb, PprzMessage("telemetry", "NAVIGATION"))
        
#        def mission_status_cb(ac_id, msg):
#            if ac_id == self.ac_id:
#                indices = list(msg['index_list'])
#                self.mission.indices = []
#                for i in range(len(indices)):
#                    self.mission.indices.append(int(indices[i]))
#        self._interface.subscribe(mission_status_cb, PprzMessage("telemetry", "MISSION_STATUS"))
    
    def create_Zones_fp(self):
        Zones = []
        NFZ_number = 0
        for i in range(len(self.flightplan.sectors.member_list)):
            if "NFZ" in self.flightplan.sectors.member_list[i].name:
                latarr = []
                lonarr = []
                xy_arr = []
                for j in range(len(self.flightplan.sectors.member_list[i].corner_list)):
                    lat = self.flightplan.sectors.member_list[i].corner_list[j].lat
                    lon = self.flightplan.sectors.member_list[i].corner_list[j].lon
                    point_lla = geodetic.LlaCoor_i(int(lat*10**7), int(lon*10**7), int(0*1000))
                    point_enu = coord_trans.lla_to_enu_fw(point_lla, self.UAV.ref_utm_i)
                    x = point_enu.x
                    y = point_enu.y
                    print("Zone ", NFZ_number,", point ", j, " x=", x, " y=", y)
                    sys.stdout.flush()  
                    latarr.append(int(lat*10**7))
                    lonarr.append(int(lon*10**7))
                    xy_arr.append((x,y))
                Zones.append(Zone(latarr, lonarr, xy_arr, NFZ_number))
                print("zone appended: ", NFZ_number)
                NFZ_number = NFZ_number + 1
        return Zones
        
    def create_Transit_points_fp(self):    
        WP_coords_en = []
        for i in range(len(self.flightplan.waypoints.member_list)):
            if "WP" in self.flightplan.waypoints.member_list[i].name:
                WP_coord_en = {}  # Name, East, North
                WP_coord_en['name'] = self.flightplan.waypoints.member_list[i].name
                WP_lla = geodetic.LlaCoor_i(int(self.flightplan.waypoints.member_list[i].lat*10**7),  int(self.flightplan.waypoints.member_list[i].lon*10**7), int(0*1000))
                WP_enu = coord_trans.lla_to_enu_fw(WP_lla, self.UAV.ref_utm_i)
                WP_coord_en['east'] = WP_enu.x # in m
                WP_coord_en['north'] = WP_enu.y # in m
                WP_coords_en.append(WP_coord_en)
        return WP_coords_en    
        
    def create_geofence_fp(self):
        latarr = []
        lonarr = []
        xy_arr = []
        for i in range(len(self.flightplan.sector_name_lookup['SoftBoundary'].corner_list)):
            lat = self.flightplan.sector_name_lookup['SoftBoundary'].corner_list[i].lat
            lon = self.flightplan.sector_name_lookup['SoftBoundary'].corner_list[i].lon
            point_lla = geodetic.LlaCoor_i(int(lat*10**7), int(lon*10**7), int(0*1000))
            point_enu = coord_trans.lla_to_enu_fw(point_lla, self.UAV.ref_utm_i)
            x = point_enu.x
            y = point_enu.y
            print("Geofence ", i, " x=", x, " y=", y)
            latarr.append(int(lat*10**7))
            lonarr.append(int(lon*10**7))
            xy_arr.append((x,y))
        geofence = Geofence(latarr, lonarr, xy_arr, 0)
        return geofence
          
    def visualize_leg(self, leg, index):
        point_enu_prev = geodetic.EnuCoor_f(leg[0][0], leg[0][1], 0.)
        point_lla_prev = coord_trans.enu_to_lla_fw(point_enu_prev, self.UAV.ref_utm_i) #!!!
        latarr = [point_lla_prev.lat + self.UAV.latlon_error[0]]
        lonarr = [point_lla_prev.lon + self.UAV.latlon_error[1]]
        point_enu = geodetic.EnuCoor_f(leg[1][0], leg[1][1], 0.)
        point_lla = coord_trans.enu_to_lla_fw(point_enu, self.UAV.ref_utm_i) #!!!
        latarr.append(point_lla.lat + self.UAV.latlon_error[0])
        lonarr.append(point_lla.lon + self.UAV.latlon_error[1])
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = 64+index
        msg['linecolor'] = "green"
        msg['fillcolor'] = "blue"
        msg['opacity'] = 1
        msg['shape'] = 2
        msg['status'] = 0
        msg['latarr'] = latarr # e-7 deg
        msg['lonarr'] = lonarr # e-7 deg
        msg['radius'] = 500.
        #msg['text'] = "Zone" + str(self.id_zone)
        #self.leg_visualization_index = self.leg_visualization_index + 1
        self._interface.send(msg)
    
    def parse_mission_path_fw(self, leg_points, index, alt):
        # The message can handle a maximum of 5 legs
        number_of_legs = int(len(leg_points)-1)
        number_of_messages = int(np.ceil(number_of_legs/4.))
        for i in range(number_of_messages):
            msg = PprzMessage("datalink", "MISSION_PATH_LLA")
            msg['ac_id'] = self.ac_id
            msg['insert'] = 0
            if len(leg_points)-4*i > 5:
                nb = 5
                for j in range(nb):
                    point_enu = geodetic.EnuCoor_f(float(leg_points[4*i+j][0]), float(leg_points[4*i+j][1]), float(alt))
                    point_lla = coord_trans.enu_to_lla_fw(point_enu, self.UAV.ref_utm_i)
                    msg['point_lat_'+str(j+1)] = point_lla.lat + self.UAV.latlon_error[0]
                    msg['point_lon_'+str(j+1)] = point_lla.lon + self.UAV.latlon_error[1]
            else:
                nb = len(leg_points)-4*i
                for j in range(nb):
                    point_enu = geodetic.EnuCoor_f(float(leg_points[4*i+j][0]), float(leg_points[4*i+j][1]), float(alt))
                    point_lla = coord_trans.enu_to_lla_fw(point_enu, self.UAV.ref_utm_i)
                    msg['point_lat_'+str(j+1)] = point_lla.lat + self.UAV.latlon_error[0]
                    msg['point_lon_'+str(j+1)] = point_lla.lon + self.UAV.latlon_error[1]
                
            msg['path_alt'] = int(alt * 1000.)
            msg['duration'] = 0
            msg['nb'] = nb
            msg['index'] = int(index*10 + 1 + i)
            self._interface.send(msg)
            
    def parse_mission_segment_fw(self, leg_points, index, alt):
        # The message can handle a maximum of 5 legs
        number_of_legs = int(len(leg_points)-1)
        number_of_messages = int(np.ceil(number_of_legs/4.))
        for i in range(number_of_messages):
            msg = PprzMessage("datalink", "MISSION_SEGMENT_LLA")
            msg['ac_id'] = self.ac_id
            msg['insert'] = 5
            for i in range(len(leg_points) - 1):
                point_enu = geodetic.EnuCoor_f(float(leg_points[i][0]), float(leg_points[i][1]), float(alt))
                point_lla = coord_trans.enu_to_lla_fw(point_enu, self.UAV.ref_utm_i)
                msg['segment_lat_1'] = point_lla.lat + self.UAV.latlon_error[0]
                msg['segment_lon_1'] = point_lla.lon + self.UAV.latlon_error[1]
                point_enu = geodetic.EnuCoor_f(float(leg_points[i+1][0]), float(leg_points[i+1][1]), float(alt))
                point_lla = coord_trans.enu_to_lla_fw(point_enu, self.UAV.ref_utm_i)
                msg['segment_lat_2'] = point_lla.lat + self.UAV.latlon_error[0]
                msg['segment_lon_2'] = point_lla.lon + self.UAV.latlon_error[1]
                msg['segment_alt'] = int(alt * 1000.)
                msg['duration'] = 0
                random_number = random.randint(1,100)
                msg['index'] = random_number#int(index*10 + 1 + i)
                print(random_number)
                msg['msg_id'] = int(self.msg_id)
                self.msg_id = self.msg_id+1
                sleep(0.5)
                self._interface.send(msg)
                self.visualize_leg(leg_points[i:i+2], index*10+1+i)
                    
    def check_lat_lon_error(self):
        point_enu = geodetic.EnuCoor_f(0,0,0)
        point_lla = coord_trans.enu_to_lla_fw(point_enu, self.UAV.ref_utm_i)
        self.UAV.latlon_error = [self.UAV.ref_lla_i.lat - point_lla.lat, self.UAV.ref_lla_i.lon - point_lla.lon]
                
    def shutdown(self):
        for i in range(255):
            self.erase_visualize_leg(i)
        if self._interface is not None:
            print("Shutting down ivy interface...")
            self._interface.shutdown()
            self._interface = None
    
    def __del__(self):
        self.shutdown()
            
    def run(self):
        sleep(5.0)
        # The main loop
        # Initialise reference position
        self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(-27.2738966*10.**7), int(151.2900371*10.**7), int(343.8*1000.))
        geodetic.utm_of_lla_i(self.UAV.ref_utm_i, self.UAV.ref_lla_i)
        self.check_lat_lon_error()
        
        # initialise reference UTM for ENU conversion
        Zones = create_Zones_fw('sw/ground_segment/python/no_fly_zones/nfzs_cyberzoo.dat', self.UAV.ref_utm_i)
        Transit_points = create_Transit_points_fw('sw/ground_segment/python/no_fly_zones/transit_waypoints.dat', self.UAV.ref_utm_i)
        
        for i in range(len(Zones)):
            msg = Zones[i].visualize_Zone()
            self._interface.send(msg)
        
        # Perform checks to identify if program has already run in advance and has parsed stuff:
#        if self.mission.indices != ['0']:
#            number_of_segments = int(len(Transit_points)-1) # for a single way
#            
#        else:
        # Route to Joe for the outback mission
        first_legs = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes(Zones, Transit_points[1:2], (Transit_points[0]['east'], Transit_points[0]['north']), (Transit_points[2]['east'], Transit_points[2]['north'])))
        avoidance_routes = route_func.route_calc_avoidance_routes(Zones, Transit_points[3:-1], (Transit_points[2]['east'], Transit_points[2]['north']), (Transit_points[-1]['east'], Transit_points[-1]['north']))
        self.mission.route_to_joe = first_legs[:-1] + res_func.leg_shortest_route(avoidance_routes)
        self.mission.route_to_joe_in_legs = route_func.route_break_in_legs(Transit_points, self.mission.route_to_joe)
        for i in range(len(self.mission.route_to_joe_in_legs)):
            #self.sleep(1.0)
            self.parse_mission_segment_fw(self.mission.route_to_joe_in_legs[i],i, 434.8)
        write_mission_file('sw/ground_segment/python/no_fly_zones/mission_status.dat', self.mission)
            
        Traffic_Scenario = traffic_scenarios.traffic_scenario(self.UAV, traffic_scenarios.traf_list1, self.platform_type, 'HDG')
        Traffic_Scenario.init_SSD_plot()
        start_time = time.time()
            
        try:
            while True:
                tla_to_next_point = self.mission.time_to_arrive_at_next_point(self.UAV, self.mission.get_current_index())
                sleep(0.1)
                stop_time = time.time()
                dt = stop_time-start_time
                start_time = time.time()
                Traffic_Scenario.update_traffic_scenario(dt, self.UAV)
                Traffic_Scenario.detect_conflicts(tla_to_next_point, self.wind)
                Traffic_Scenario.plot_SSD()
                sys.stdout.flush()
                for i in range(len(Traffic_Scenario.traf_list)):
                    msg = Traffic_Scenario.visualize_traffic(i)
                    self._interface.send(msg)
                
        except KeyboardInterrupt:
            self.stop()
            
    def run_cyberzoo(self):
        sleep(5.0)
        # The main loop
        # Initialise reference position
        self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(51.9906310*10.**7), int(4.3767960*10.**7), int(0.*1000.))
        geodetic.utm_of_lla_i(self.UAV.ref_utm_i, self.UAV.ref_lla_i)
        self.check_lat_lon_error()
        
        # initialise reference UTM for ENU conversion
        Zones = create_Zones_fw('sw/ground_segment/python/no_fly_zones/nfzs_cyberzoo.dat', self.UAV.ref_utm_i)
        Transit_points = create_Transit_points_fw('sw/ground_segment/python/no_fly_zones/transit_waypoints_cyberzoo.dat', self.UAV.ref_utm_i)
        
        Traffic_Scenario = traffic_scenarios.traffic_scenario(self.UAV, traffic_scenarios.traf_list2, self.platform_type, 'HDG')
        Traffic_Scenario.init_SSD_plot()
        start_time = time.time()
        
        # Load and visualize No Fly Zones
        for i in range(len(Zones)):
            msg = Zones[i].visualize_Zone()
            self._interface.send(msg)
            
        # Route to Joe for the outback mission
        first_legs = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes(Zones, Transit_points[1:2], (Transit_points[0]['east'], Transit_points[0]['north']), (Transit_points[2]['east'], Transit_points[2]['north'])))
        avoidance_routes = route_func.route_calc_avoidance_routes(Zones, Transit_points[3:-1], (Transit_points[2]['east'], Transit_points[2]['north']), (Transit_points[-1]['east'], Transit_points[-1]['north']))
        self.mission.route_to_joe = first_legs[:-1] + res_func.leg_shortest_route(avoidance_routes)
        self.mission.route_to_joe_in_legs = route_func.route_break_in_legs(Transit_points, self.mission.route_to_joe)
        for i in range(len(self.mission.route_to_joe_in_legs)):
            self.parse_mission_segment_fw(self.mission.route_to_joe_in_legs[i],i, 1.0)
        write_mission_file('sw/ground_segment/python/no_fly_zones/mission_status.dat', self.mission)
        sys.stdout.flush()
        
        try:
            while True:
                #Calculate some mission variables like lookahead time
                tla_to_next_point = self.mission.time_to_arrive_at_next_point(self.UAV, self.mission.get_current_index())          
                
                # First do traffic calculations and visualize
                sleep(0.1)
                stop_time = time.time()
                dt = stop_time-start_time
                start_time = time.time()
                Traffic_Scenario.update_traffic_scenario(dt, self.UAV)
                Traffic_Scenario.detect_conflicts(tla_to_next_point, self.wind)
                Traffic_Scenario.plot_SSD()
                sys.stdout.flush()
                for i in range(len(Traffic_Scenario.traf_list)):
                    msg = Traffic_Scenario.visualize_traffic(i)
                    self._interface.send(msg)
               
               # Afterwards check conclficts and check resolution points
        except KeyboardInterrupt:
            self.stop()

    def run_valkenburg(self):
        sleep(2.0)
        # The main loop
        # Initialise reference position
        self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(52.1682916*10.**7), int(4.4134931*10.**7), int(0.*1000.))
        geodetic.utm_of_lla_i(self.UAV.ref_utm_i, self.UAV.ref_lla_i)
        self.check_lat_lon_error()
        
        # initialise reference UTM for ENU conversion
        Zones = create_Zones_fw('sw/ground_segment/python/no_fly_zones/nfzs_valkenburg.dat', self.UAV.ref_utm_i)
        Transit_points = create_Transit_points_fw('sw/ground_segment/python/no_fly_zones/transit_waypoints_valkenburg.dat', self.UAV.ref_utm_i)
        
        Traffic_Scenario = traffic_scenarios.traffic_scenario(self.UAV, traffic_scenarios.traf_list3, self.platform_type, 'HDG')
        Traffic_Scenario.init_SSD_plot()
        start_time = time.time()
        
        # Load and visualize No Fly Zones
        for i in range(len(Zones)):
            msg = Zones[i].visualize_Zone()
            self._interface.send(msg)
        
        #legs = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes(Zones, Transit_points[2:4], (Transit_points[1]['east'], Transit_points[1]['north']), (Transit_points[4]['east'], Transit_points[4]['north'])))
        legs = res_func.leg_shortest_route(route_func.route_calc_avoidance_routes(Zones, Transit_points[1:2], (Transit_points[1]['east'], Transit_points[1]['north']), (Transit_points[2]['east'], Transit_points[2]['north'])))
        legs = legs[:-1] + (res_func.leg_shortest_route(route_func.route_calc_avoidance_routes(Zones, Transit_points[2:3], (Transit_points[2]['east'], Transit_points[2]['north']), (Transit_points[3]['east'], Transit_points[3]['north']))))
        legs = legs[:-1] + (res_func.leg_shortest_route(route_func.route_calc_avoidance_routes(Zones, Transit_points[3:4], (Transit_points[3]['east'], Transit_points[3]['north']), (Transit_points[4]['east'], Transit_points[4]['north']))))
        self.mission.route_to_joe = legs
        self.mission.route_to_joe_in_legs = route_func.route_break_in_legs(Transit_points, self.mission.route_to_joe)
        for i in range(len(self.mission.route_to_joe_in_legs)):
            self.parse_mission_segment_fw(self.mission.route_to_joe_in_legs[i],i, 70.0)
        write_mission_file('sw/ground_segment/python/no_fly_zones/mission_status.dat', self.mission)
        sys.stdout.flush()
        
        try:
            while True:
                #Calculate some mission variables like lookahead time
                tla_to_next_point = self.mission.time_to_arrive_at_next_point(self.UAV, self.mission.get_current_index())          
                
                # First do traffic calculations and visualize
                sleep(0.1)
                stop_time = time.time()
                dt = stop_time-start_time
                start_time = time.time()
                Traffic_Scenario.update_traffic_scenario(dt, self.UAV)
                Traffic_Scenario.detect_conflicts(tla_to_next_point, self.wind)
                Traffic_Scenario.plot_SSD()
                #mission.mission_check_conflict_resolution(Traffic_Scenario, tla_to_next_point, self.mission, self.mission.get_current_index, self.wind)
                sys.stdout.flush()
                for i in range(len(Traffic_Scenario.traf_list)):
                    msg = Traffic_Scenario.visualize_traffic(i)
                    self._interface.send(msg)
               
               # Afterwards check conclficts and check resolution points
        except KeyboardInterrupt:
            self.stop()
            
    def run_valkenburg_test(self):
        sleep(5.0)
        # The main loop
        # Initialise reference position
        self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(52.1682895*10.**7), int(4.4134879*10.**7), int(0.*1000.))
        geodetic.utm_of_lla_i(self.UAV.ref_utm_i, self.UAV.ref_lla_i)
        self.check_lat_lon_error()
        
        # initialise reference UTM for ENU conversion
        Zones = create_Zones_fw('sw/ground_segment/python/no_fly_zones/nfzs_cyberzoo.dat', self.UAV.ref_utm_i)
        Transit_points = create_Transit_points_fw('sw/ground_segment/python/no_fly_zones/transit_waypoints_valkenburg.dat', self.UAV.ref_utm_i)
        
        
        Traffic_Scenario = traffic_scenarios.traffic_scenario(self.UAV, traffic_scenarios.traf_list4, self.platform_type, 'HDG', self._interface)
        
        # Load and visualize No Fly Zones
        for i in range(len(Zones)):
            msg = Zones[i].visualize_Zone()
            self._interface.send(msg)
        
        self.mission = mission.Mission(self.ac_id, Zones, Transit_points, 70.0, self.UAV, self._interface, Traffic_Scenario, 15.0)
        
    def run_dalby_asterix(self):
        sleep(5.0)
        # The main loop
        # Initialise reference position
        self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(-27.2738966*10.**7), int(151.2900371*10.**7), int(343.8*1000.))
        geodetic.utm_of_lla_i(self.UAV.ref_utm_i, self.UAV.ref_lla_i)
        self.check_lat_lon_error()
        
        # initialise reference UTM for ENU conversion
        Zones = create_Zones_fw('sw/ground_segment/python/no_fly_zones/nfzs_valkenburg.dat', self.UAV.ref_utm_i)
        Transit_points = create_Transit_points_fw('sw/ground_segment/python/no_fly_zones/transit_waypoints.dat', self.UAV.ref_utm_i)
        
        self.mission = mission.Mission(self.ac_id, Zones, Transit_points, 70.0, self.UAV, self._interface)
        
        sys.stdout.flush()
        
#        try:
#            while True:
#                # First do traffic calculations and visualize
#                sleep(0.5)
#                Traffic_Scenario.update_traffic_scenario(self.UAV, receiver_thread)
#                if Traffic_Scenario.Traffic.ntraf > 1:
#                    Traffic_Scenario.detect_conflicts(120., self.wind, 50.)
#                    Traffic_Scenario.plot_SSD()
#               
#                sys.stdout.flush()
#                # Afterwards check conclficts and check resolution points
#        except KeyboardInterrupt:
#            self.stop()
        
    def run_valkenburg_asterix(self):
        sleep(1.0)
        # The main loop
        # Initialise reference position
        #self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(52.1702964*10.**7), int(4.4279714*10.**7), int(0.*1000.))
        self.UAV.ref_lla_i = geodetic.LlaCoor_i(int(self.flightplan.flight_plan.lat0*10.**7), int(self.flightplan.flight_plan.lon0*10.**7), int(0.*1000.))
        geodetic.utm_of_lla_i(self.UAV.ref_utm_i, self.UAV.ref_lla_i)
        self.check_lat_lon_error()
        
        # initialise reference UTM for ENU conversion
        #Zones = create_Zones_fw('sw/ground_segment/python/no_fly_zones/nfzs_valkenburg.dat', self.UAV.ref_utm_i)
        Zones = self.create_Zones_fp()
        #Transit_points = create_Transit_points_fw('sw/ground_segment/python/no_fly_zones/transit_waypoints_valkenburg.dat', self.UAV.ref_utm_i)
        Transit_points = self.create_Transit_points_fp()
        #soft_geofence = create_geofence('sw/ground_segment/python/no_fly_zones/soft_geofence_valkenburg.dat', self.UAV.ref_utm_i)
        soft_geofence = self.create_geofence_fp()
        sys.stdout.flush()
        
        self.mission = mission.Mission(self.ac_id, 15., Zones, Transit_points, 70.0, self.UAV, self._interface, soft_geofence, self.flightplan, self.platform_type)
        
        sys.stdout.flush()
        
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Traffic Server for Avoidance")
    parser.add_argument("-ac", "--ac_id", dest='ac_id', default=0, type=int, help="aircraft ID")
    parser.add_argument("-platform", "--platform_type", dest='platform_type', default='rc', type=str, help="UAV type (fw or rc)")
    args = parser.parse_args()
    
    ac_id = args.ac_id
    platform_type = args.platform_type
    
    ts = TrafficServer(ac_id, platform_type)
    ts.run_valkenburg_asterix()