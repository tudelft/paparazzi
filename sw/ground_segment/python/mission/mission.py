#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import shapely.geometry as geometry
import math
import time
import argparse
import logging

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
import asterix_receiver
import asterix_visualizer
import aircraft
import resolution
import mission_comm
import mission_visualizer

import flightplan_xml_parse

#defines
static_margin = 100. #[m]
dynamic_margin = 150. #[m]
altitude = 120.
airspeed = 23.
max_tla = 60. #[s] change later
wind = {'east': 0., 'north' : 0.}
t_arrive_lock = 30. #10. # [s]

#defines for resolution algoritm
conflict_counter_th = 3 # min number of consecutive detections before avoidance
avoidance_time_th = 30. #10. # [s] min tume before waypoint for which avoidance is not allowed
hdg_diff_th = 30. # max absolute heading change for resolution
avoid_dist_min = 500. # avoid distance, when possible, otherwise half leg length
hdg_margin = 5. # deg

# defines for occupancy check
base_radius = 300.
base_free_time = 30.



class Mission(object):
    def __init__(self, ac_id):
        # Load flightplan
        self.ac_id = ac_id
        self.ivy_interface = IvyMessagesInterface("Avoid Mission")
        self.flightplan = flightplan_xml_parse.PaparazziACFlightplan(self.ac_id)
        
        # Reference values for locations
        self.ref_lla_i = geodetic.LlaCoor_i(int(self.flightplan.flight_plan.lat0*10.**7), int(self.flightplan.flight_plan.lon0*10.**7), int(0.*1000.))
        self.ref_utm_i = geodetic.UtmCoor_i()
        geodetic.utm_of_lla_i(self.ref_utm_i, self.ref_lla_i)  
        self.ltp_def = geodetic.LlaCoor_f(self.flightplan.flight_plan.lat0/180*math.pi, self.flightplan.flight_plan.lon0/180*math.pi, self.flightplan.flight_plan.alt).to_ltp_def()   
        
        # From flightplan
        self.transit_points = self.transit_waypoints_from_fp()
        self.static_nfzs = self.static_nfzs_from_fp()
        self.geofence = self.geofence_from_fp()
        self.circular_zones = static_nfz.get_circular_zones(self.static_nfzs)
        
        # Mission elements
        self.altitude = altitude
        self.mission_elements = self.mission_elem_from_fp()

        # Initialize the mission communication
        self.mission_comm = mission_comm.MissionComm(self.ac_id, self.ivy_interface)
        self.mission_comm.start()
        self.mission_comm.set_mission(self.mission_elements)

        # Initialize the mission visualizer
        self.mission_visualizer = mission_visualizer.MissionVisualizer(self.ivy_interface, self.ltp_def)
        
        # Initialize the asterix receiver
        self.asterix_receiver = asterix_receiver.AsterixReceiver(self.ltp_def)
        self.asterix_receiver.start()

        # Initialize the asterix visualiser
        self.asterix_visualizer = asterix_visualizer.AsterixVisualizer(self.ivy_interface)
        
        # own aircraft
        self.aircraft = aircraft.Aircraft(self.ac_id, self.ivy_interface, self.ltp_def)
        
        # realtime ssd
        self.realtime_ssd = resolution.RealtimeResolution(self.circular_zones, self.ltp_def)
        
        # avoidance variables
        self.avoidance_lock_id = 0 # until what onboard id avoidance is prohibited
        self.resolution_finder = resolution.ResolutionFinder(self.circular_zones, self.ref_utm_i, self.ltp_def)
        
    def static_nfzs_from_fp(self):
        """
        Get the static no fly zones from the flight plan
        """
        zones = []
        for sector in self.flightplan.sectors.member_list:
            if "NFZ" in sector.name:
                enu_points = []
                for point in sector.corner_list:
                    point_enu = geodetic.LlaCoor_f(point.lat/180*math.pi, point.lon/180*math.pi, 0).to_enu(self.ltp_def)
                    enu_points.append(point_enu)
                zones.append(StaticNFZ(sector.name, enu_points))
        return zones
        
    def transit_waypoints_from_fp(self):    
        """
        Get the transit waypoints from the flight plan
        """
        waypoints = []
        for wp in self.flightplan.waypoints.member_list:
            # Select only waypoints from the Main path
            if wp.name[:2] == "FP":
                wp_enu = geodetic.LlaCoor_f(wp.lat/180*math.pi, wp.lon/180*math.pi, self.flightplan.flight_plan.alt).to_enu(self.ltp_def)
                wp_enu.z = self.flightplan.flight_plan.alt - self.flightplan.flight_plan.ground_alt
                wp = TransitWaypoint(wp.name, wp_enu)
                waypoints.append(wp)
        return waypoints    
        
    def geofence_from_fp(self):
        """
        Get the geofence from the flight plan
        """
        enu_points = []
        if self.flightplan.sector_name_lookup['MissionBoundary']:
            for point in self.flightplan.sector_name_lookup['MissionBoundary'].corner_list:
                point_enu = geodetic.LlaCoor_f(point.lat/180*math.pi, point.lon/180*math.pi, 0).to_enu(self.ltp_def)
                enu_points.append(point_enu)
        return enu_points
        
    def draw_circular_static_nfzs(self):
        """
        Draw a cirular NFZ
        """
        id = 0
        for zone in self.circular_zones:
            enu_coor = geodetic.EnuCoor_f(zone[0], zone[1], 0)
            lla_coor = enu_coor.to_ecef(self.ltp_def).to_lla().to_int()
            
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = 0 + id 
            msg['linecolor'] = "red"
            msg['fillcolor'] = "orange"
            msg['opacity'] = 1
            msg['shape'] = 0 # Circle
            msg['status'] = 0 # Create
            msg['latarr'] = [lla_coor.lat, 0] # e-7 deg
            msg['lonarr'] = [lla_coor.lon, 0] # e-7 deg
            msg['radius'] = zone[2]
            self.ivy_interface.send(msg)
            id += 1
            
    def mission_elem_from_fp(self):
        """
        Add the mission flight path
        """
        id = 0
        elems = []
        for point in self.transit_points:
            elem = MissionElement(id, point.enu)
            elems.append(elem)
            id += 100
        return elems
        
    def avoidance(self):
        """
        calling the avoidance function when allowed
        """
        # first sort the offline mission elements
        sorted(self.mission_elements, key=lambda elem: elem.id)
        
        idx = self.mission_comm.get_remote_idx()
        if (idx > len(self.mission_elements) - 1):
            return
        
        if ((self.mission_elements[idx].id > self.avoidance_lock_id) and (idx > 0)): 
            groundspeed = self.aircraft.get_gspeed()
            
            # if groundspeed messages are not coming in
            if groundspeed < 1.0:
                groundspeed = airspeed
            
            from_point_enu = self.aircraft.get_enu()
            to_point_enu = self.mission_elements[idx].wp
            t_arrive = resolution.time_to_arrive_at_point(from_point_enu, to_point_enu, groundspeed)
            if (t_arrive > t_arrive_lock):
                resolution_point = self.resolution_finder.resolution_on_leg(from_point_enu, to_point_enu, self.aircraft.get_gspeed(), dynamic_margin, self.aircraft, self.asterix_receiver.get_events(), wind, self.flightplan.flight_plan.alt - self.flightplan.flight_plan.ground_alt, self.geofence, self.static_nfzs, max_tla, conflict_counter_th, avoidance_time_th, hdg_diff_th, avoid_dist_min)
                if resolution_point == 'free':
                    return
                elif resolution_point == 'nosol':
                    return
                else:
                        self.mission_elements.insert(idx, MissionElement(self.mission_elements[idx-1].id + 2, resolution_point))
                        self.mission_elements.insert(idx, MissionElement(self.mission_elements[idx-1].id + 1, from_point_enu))
    
                        self.avoidance_lock_id = self.mission_elements[idx + 1].id
                        self.mission_comm.set_mission(self.mission_elements)
                
                
    def run_mission(self):
        """
        Main loop
        """
        self.draw_circular_static_nfzs()
        self.realtime_ssd.init_realtime()
        
        while True:
            logging.debug("Mission LOOP")
            
            #Check occupancy of baseif 
            if self.mission_comm.get_remote_idx() == 0:
                start_location = self.mission_elements[0].wp
            
            # Visualize the asterix events and the mission
            self.asterix_visualizer.visualize(self.asterix_receiver.get_events())
            self.mission_visualizer.visualize(self.mission_comm.get_mission())
            
            self.draw_circular_static_nfzs()
            
            # realimte ssd plotter
            groundspeed = self.aircraft.get_gspeed()
            # if no groundspeed message update
            if groundspeed < 1.0:
                groundspeed = airspeed
            self.realtime_ssd.run_realtime(max_tla, wind, dynamic_margin, groundspeed, self.aircraft, self.asterix_receiver.get_events(), groundspeed)
            
            # run avoidance
            self.avoidance()

            # Wait time in main loop
            time.sleep(1)

# Mission element
class MissionElement(object):
    def __init__(self, id, wp):
        self.wp = wp
        self.id = id

# Transit waypoint
class TransitWaypoint(object):
    def __init__(self, name, enu):
        self.name = name
        self.enu = enu

# Static no fly zone  
class StaticNFZ(object):
    def __init__(self, name, enu_points):
        self.name = name
        self.enu_points = enu_points
        
if __name__ == '__main__':
    # Argyment parser
    parser = argparse.ArgumentParser(description="Mission for Avoidance")
    parser.add_argument("-ac", "--ac_id", dest='ac_id', type=int, help="aircraft ID", required = True)
    parser.add_argument("-v", "--verbose", dest="verbose", action="store_true", help="Verbose")
    args = parser.parse_args()
    
    # Enable debugging
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    # Run the mission
    mission = Mission(args.ac_id)
    mission.run_mission()
