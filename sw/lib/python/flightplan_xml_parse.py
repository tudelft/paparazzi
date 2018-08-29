# -*- coding: utf-8 -*-
"""
Created on Sat Aug 18 17:40:08 2018

@author: dennis
"""

from __future__ import absolute_import, print_function

import os
import sys
from lxml import etree

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                                              '../../..')))
class PaparazziFlightPlan:
    "Paparazzi Flightplan Class"
    def __init__(self, name, lon0, lat0, ground_alt, alt):
        self.name = name
        self.lon0 = float(lon0)
        self.lat0 = float(lat0)
        self.ground_alt = float(ground_alt)
        self.alt = float(alt)

class PaparazziWaypoint:
    "Paparazzi Waypoint Class"
    def __init__(self, name, x, y, lon=False, lat=False, alt=False):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.lon = float(lon)
        self.lat = float(lat)
        self.alt = float(alt)
        
class PaparazziWaypoints:
    "Paparazzi Waypoints Class"
    def __init__(self):
        self.member_list = []   

class PaparazziSector:
    "Paparazzi Sector Class"
    def __init__(self, name):
        self.name = name
        self.corner_list = []

class PaparazziSectors:
    "Paparazzi Sectors Class"
    def __init__(self):
        self.member_list = []
        
class PaparazziBlock:
    "Paparazzi Block Class"
    def __init__(self, name, no):
        self.name = name
        self.no = float(no)

class PaparazziBlocks:
    "Paparazzi Blocks class"
    def __init__(self):
        self.member_list = []
                                                                      
# Class for all flightplan elements
class PaparazziACFlightplan:
    "Paparazzi Settings Class"
    ac_id = 0
    waypoints = PaparazziWaypoints()
    wp_lookup = []
    wp_name_lookup = {}
    sectors = PaparazziSectors()
    sector_lookup = []
    sector_name_lookup = {}
    blocks = PaparazziBlocks()
    block_lookup = []
    block_name_lookup = {}
    
    # Takes a string file path for flight plan XML file and
    # returns a flightplan AC object
    
    def __init__(self, ac_id):
        self.ac_id = ac_id
        paparazzi_home = PPRZ_HOME
        paparazzi_conf = os.path.join(paparazzi_home, 'conf')
        conf_xml_path = os.path.join(paparazzi_conf, 'conf.xml')
        conf_tree = etree.parse(conf_xml_path)
        # extract aircraft node from conf.xml file
        ac_node = conf_tree.xpath('/conf/aircraft[@ac_id=%i]' % ac_id)
        if (len(ac_node) != 1):
            print("Aircraft ID %i not found." % ac_id)
            sys.exit(1)

        # save AC name for reference
        self.name = ac_node[0].attrib['name']
        
        # get flightplan.xml file path from var/aircrafts/<ac_name> directory
        flightplan_xml_path = os.path.join(paparazzi_home, 'var/aircrafts/' + self.name + '/flight_plan.xml')
        
        if not os.path.isfile(flightplan_xml_path):
            print("Could not find %s, did you build it?" % flightplan_xml_path)
            sys.exit(1)
        
        parser = etree.XMLParser(recover=True)
        tree = etree.parse(flightplan_xml_path, parser=parser)
        
        the_flight_plan = tree.xpath("/dump/flight_plan")[0]
        try:
            fp_name = the_flight_plan.attrib['name']
            fp_lon0 = the_flight_plan.attrib['lon0']
            fp_lat0 = the_flight_plan.attrib['lat0']
            fp_ground_alt = the_flight_plan.attrib['ground_alt']
            fp_alt = the_flight_plan.attrib['alt']
            self.flight_plan = PaparazziFlightPlan(fp_name, fp_lon0, fp_lat0, fp_ground_alt, fp_alt)
        except:
            print("ERROR: name, lat0, lon0, ground_alt or alt cannot be read from flightplan")
        
        # Read Waypoints
        for the_waypoint in tree.xpath("/dump/flight_plan/waypoints/waypoint"):
            try:
                if 'NAME' in the_waypoint.attrib:
                    waypoint_name = the_waypoint.attrib['NAME']
                else:
                    waypoint_name = the_waypoint.attrib['name']
            except:
                print("Could not read name of waypoint")
                continue
            
            waypoint_x = the_waypoint.attrib['x']
            waypoint_y = the_waypoint.attrib['y']
            try:
                waypoint_lon = the_waypoint.attrib['lon']
            except:
                waypoint_lon=False
            try:
                waypoint_lat = the_waypoint.attrib['lat']
            except:
                waypoint_lat=False
            try:
                waypoint_alt = the_waypoint.attrib['alt']
            except:
                waypoint_alt=False
            
            
            waypoint = PaparazziWaypoint(waypoint_name, waypoint_x, waypoint_y, lon=waypoint_lon, lat=waypoint_lat, alt=waypoint_alt)
            self.waypoints.member_list.append(waypoint)   
            self.wp_lookup.append(waypoint)
            self.wp_name_lookup[waypoint_name] = waypoint
            
        #Read Sectors
        for the_sector in tree.xpath("/dump/flight_plan/sectors/sector"):
            try:
                if 'NAME' in the_sector.attrib:
                    sector_name = the_sector.attrib['NAME']
                else:
                    sector_name = the_sector.attrib['name']
            except:
                print("Could not read name of sector")
                continue
            #print (sector_name)
            sector = PaparazziSector(sector_name)
            for the_corner in tree.xpath("/dump/flight_plan/sectors/sector[@name='" + sector_name + "']/corner"):
                try:
                    corner_name = the_corner.attrib['name']
                except:
                    print("Could not read name of corner")
                    continue
            
                #print (corner_name)
                sector.corner_list.append(self.wp_name_lookup[corner_name])
            
            self.sectors.member_list.append(sector)
            self.sector_lookup.append(sector)
            self.sector_name_lookup[sector_name] = sector
        
        for the_block in tree.xpath("/dump/flight_plan/blocks/block"):
            try:
                block_name = the_block.attrib['name']
            except:
                print("Could not read name of block")
                continue
            try:
                block_no = the_block.attrib['no']
            except:
                print("Could not read the block number")
                
            block = PaparazziBlock(block_name, block_no)
            self.blocks.member_list.append(block)
            self.block_lookup.append(block)
            self.block_name_lookup[block_name] = block
                
#test = PaparazziACFlightplan(1)