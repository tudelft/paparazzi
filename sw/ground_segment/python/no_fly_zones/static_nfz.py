# -*- coding: utf-8 -*-
"""
Created on Sat Aug 11 18:29:57 2018

@author: dennis
"""

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import itertools
import coordinate_transformations as coord_trans
from shapely import geometry

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprz_math import geodetic

def get_circle_around_zone (Zone, ref_utm_i):
    centers = []
    diameters = []
    
    combinations = list(itertools.combinations(range(len(Zone.xy_arr)), 2))
    
    for i in range(len(combinations)):
        first_element = combinations[i][0]
        second_element = combinations[i][1]
        p1 = Zone.xy_arr[first_element] # select xy [2] and then the the first_element_th point
        p2 = Zone.xy_arr[second_element] # select xy [2] and then the the second_element_th point
        center = (((p1[0] + p2[0]) / 2), ((p1[1] + p2[1]) / 2))
        diameter = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        points_in_circle = True
        
        for j in range(len(Zone.xy_arr)):
            if (j != (first_element or second_element)):
                point_in_circle = check_if_point_in_circle(Zone.xy_arr[j], center, diameter)
                if not point_in_circle:
                    points_in_circle = False
        
        if points_in_circle:
            centers.append(center)
            diameters.append(diameter)
            
    min_radius = min(diameters) / 2
    min_center = centers[diameters.index(min(diameters))]
    
    min_center_lla = coord_trans.enu_to_lla_fw(geodetic.EnuCoor_f(min_center[0], min_center[1], 0), ref_utm_i)
    
    return min_radius, min_center, min_center_lla
    
def get_circle_geometry (xy_arr):
    centers = []
    diameters = []
    
    combinations = list(itertools.combinations(range(len(xy_arr)), 2))
    
    for i in range(len(combinations)):
        first_element = combinations[i][0]
        second_element = combinations[i][1]
        p1 = xy_arr[first_element] # select xy [2] and then the the first_element_th point
        p2 = xy_arr[second_element] # select xy [2] and then the the second_element_th point
        center = (((p1[0] + p2[0]) / 2), ((p1[1] + p2[1]) / 2))
        diameter = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        points_in_circle = True
        
        for j in range(len(xy_arr)):
            if (j != (first_element or second_element)):
                point_in_circle = check_if_point_in_circle(xy_arr[j], center, diameter)
                if not point_in_circle:
                    points_in_circle = False
        
        if points_in_circle:
            centers.append(center)
            diameters.append(diameter)
            
    min_radius = min(diameters) / 2
    min_center = centers[diameters.index(min(diameters))]
    
    circle_geometry = geometry.Point(min_center).buffer(min_radius)
    
    circle = {}
    circle['geo'] = circle_geometry
    circle['radius'] = min_radius
    circle['center'] = min_center
    return circle
    
    
    
def check_if_point_in_circle (point, center, diameter):
    radius = diameter/2
    distance_from_center = np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)
    if (radius+0.5 >= distance_from_center):
        in_circle = True
    else:
        in_circle = False
    return in_circle
    
def get_circular_zones (Zones, ref_utm_i):
    circular_zones = []
    for i in range(len(Zones)):
        circular_zone = get_circle_around_zone(Zones[i], ref_utm_i)
        circular_zones.append(circular_zone)
    return circular_zones