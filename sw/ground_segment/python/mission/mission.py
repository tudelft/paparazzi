#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import shapely.geometry as geometry

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from pprz_math import geodetic

import static_nfz
import coordinate_transformations as coord_trans

import flightplan_xml_parse

#defines
static_margin = 100. #[m]
altitude = 120.

class Mission(object):
    def __init__(self, ac_id):
        # Load flightplan
        self.ac_id = ac_id
        self._interface = IvyMessagesInterface("Avoid Mission")
        self.flightplan = flightplan_xml_parse.PaparazziACFlightplan(self.ac_id)
        
        # Reference values for locations
        self.ref_lla_i = geodetic.LlaCoor_i(int(self.flightplan.flight_plan.lat0*10.**7), int(self.flightplan.flight_plan.lon0*10.**7), int(0.*1000.))
        self.ref_utm_i = geodetic.UtmCoor_i()
        geodetic.utm_of_lla_i(self.ref_utm_i, self.ref_lla_i)        
        
        # From flightplan
        self.Zones = self.create_Zones_fp()
        self.circular_zones = static_nfz.get_circular_zones(self.Zones, self.ref_utm_i)
        self.Transit_points = self.create_Transit_points_fp()
        self.geofence = self.create_geofence_fp()
        
        # mission elements
        self.altitude = altitude
        self.mission_elements = []
        
        
    
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
                    point_enu = coord_trans.lla_to_enu_fw(point_lla, self.ref_utm_i)
                    x = point_enu.x
                    y = point_enu.y  
                    latarr.append(int(lat*10**7))
                    lonarr.append(int(lon*10**7))
                    xy_arr.append((x,y))
                Zones.append(Zone(latarr, lonarr, xy_arr, NFZ_number))
                NFZ_number = NFZ_number + 1
        return Zones
        
    def create_Transit_points_fp(self):    
        WP_coords_en = []
        for i in range(len(self.flightplan.waypoints.member_list)):
            if "WP" in self.flightplan.waypoints.member_list[i].name:
                WP_coord_en = {}  # Name, East, North
                WP_coord_en['name'] = self.flightplan.waypoints.member_list[i].name
                WP_lla = geodetic.LlaCoor_i(int(self.flightplan.waypoints.member_list[i].lat*10**7),  int(self.flightplan.waypoints.member_list[i].lon*10**7), int(0*1000))
                WP_enu = coord_trans.lla_to_enu_fw(WP_lla, self.ref_utm_i)
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
            point_enu = coord_trans.lla_to_enu_fw(point_lla, self.ref_utm_i)
            x = point_enu.x
            y = point_enu.y
            print("Geofence ", i, " x=", x, " y=", y)
            latarr.append(int(lat*10**7))
            lonarr.append(int(lon*10**7))
            xy_arr.append((x,y))
        geofence = Geofence(latarr, lonarr, xy_arr, 0)
        return geofence
        
    def draw_circular_static_nfzs(self):
        for i in range(len(self.Zones)):
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
            
    def initialize_mission_elements(self):
        for i in range(len(self.Transit_points)):
            identity = 1 + 10 *1 
            self.mission_elements.append(MissionPathElem(self.Transit_points[i].east, self.Transit_points[i].north), self.altitude, identity)
            
    def run_mission(self):
        self.ref_lla_i = geodetic.LlaCoor_i(int(self.flightplan.flight_plan.lat0*10.**7), int(self.flightplan.flight_plan.lon0*10.**7), int(self.flightplan.flight_plan.ground_alt*1000.))
        self.draw_circular_static_nfzs()
        self.initialize_mission_elements()
     
# Class containg a NFZ
class Zone(object):
    def __init__(self, latarr, lonarr,  xy_arr, id_zone):
        self.latarr = latarr
        self.lonarr = lonarr
        self.id_zone = id_zone
        self.xy_arr = xy_arr
        self.polygon = geometry.Polygon(xy_arr)
        self.circle = static_nfz.get_circle_geometry(self.xy_arr)  

# class containing mission path elements
class MissionPathElem(object):
    def __init__(self, e, n, u, wp_i):
        self.wp = geodetic.EnuCoor_f()
        self.wp_id = 0
        
# Class containg a geofence
class Geofence(object):
    def __init__(self, latarr, lonarr,  xy_arr, id_zone):
        self.latarr = latarr
        self.lonarr = lonarr
        self.id_zone = id_zone
        self.xy_arr = xy_arr
        self.polygon = geometry.Polygon(xy_arr)
        
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Mission for Avoidance")
    parser.add_argument("-ac", "--ac_id", dest='ac_id', default=0, type=int, help="aircraft ID")
    args = parser.parse_args()
    
    ac_id = args.ac_id

    mission = Mission(ac_id)
    mission.run_mission()