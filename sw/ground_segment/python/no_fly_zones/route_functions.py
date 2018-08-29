# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 20:51:48 2018

@author: dennis
"""

import numpy as np
import sys
import shapely.geometry as geometry
import shapely.affinity as affinity
import shapely.ops as ops
from resolution_functions import *
import itertools
        
# Return all possible Waypoint sequences
def route_return_possible_sequences(Transit_points, position, target_pos):
    Transit_coords = [(x['east'], x['north']) for x in Transit_points]
    possible_direct_routes = list(itertools.permutations(Transit_coords, len(Transit_coords)))
    possible_direct_routes = [[position] + list(x) + [target_pos] for x in possible_direct_routes]
    return possible_direct_routes

# Returns all possible legs between all pairs of Transit points
def route_return_possible_legs(Zones, Transit_points, position, target_pos):
    Transit_coords = [(x['east'], x['north']) for x in Transit_points]
    Transit_coords = [position] + Transit_coords + [target_pos]
    direct_legs = list(itertools.permutations(Transit_coords, 2))
    avoidance_legs = [leg_return_shortest_leg(Zones, x[0], x[1]) for x in direct_legs]
    return avoidance_legs

# Calculates possible routes including avoidance waypoints
def route_calc_avoidance_routes(Zones, Transit_points,  position, target_pos):
    possible_direct_routes = route_return_possible_sequences(Transit_points, position, target_pos)
    avoidance_legs = route_return_possible_legs(Zones, Transit_points, position, target_pos)
    possible_avoidance_routes = []
    for i in range(len(possible_direct_routes)):
        avoidance_route = []
        for j in range(len(possible_direct_routes[i])-1):
            for k in range(len(avoidance_legs)):
                if (avoidance_legs[k][0] == possible_direct_routes[i][j]) and (avoidance_legs[k][-1] == possible_direct_routes[i][j+1]):
                    if j==0:
                        avoidance_route = avoidance_route + avoidance_legs[k]
                    else:
                        avoidance_route = avoidance_route + avoidance_legs[k][1:]
                    
                    break
        possible_avoidance_routes.append(avoidance_route)
    return possible_avoidance_routes

# Break a full route into legs between 2 Transit points inclusing avoidance waypoints 
def route_break_in_legs(Transit_points, route):
    breaked_route = []
    route_part = [route[0]]
    for i in range(1,len(route)):
        for j in range(len(Transit_points)):
            if route[i] == (Transit_points[j]['east'], Transit_points[j]['north']):
                route_part.append(route[i])
                breaked_route.append(route_part)
                route_part = [route[i]]
                break
            elif j == len(Transit_points)-1: # If in final j loop
                route_part.append(route[i])
    return breaked_route
    
    
##############################################################################
# Circular Zone functions                                                    #
##############################################################################
    
# Returns all possible legs between all pairs of Transit points
def route_return_possible_legs_cz(Zones, Transit_points, position, target_pos):
    Transit_coords = [(x['east'], x['north']) for x in Transit_points]
    Transit_coords = [position] + Transit_coords + [target_pos]
    direct_legs = list(itertools.permutations(Transit_coords, 2))
    avoidance_legs = [leg_return_shortest_leg_cz(Zones, x[0], x[1]) for x in direct_legs]
    return avoidance_legs

# Calculates possible routes including avoidance waypoints
def route_calc_avoidance_routes_cz(Zones, Transit_points,  position, target_pos):
    possible_direct_routes = route_return_possible_sequences(Transit_points, position, target_pos)
    
    avoidance_legs = route_return_possible_legs_cz(Zones, Transit_points, position, target_pos)
    print("avoidance route: ", avoidance_legs)
    sys.stdout.flush()
    possible_avoidance_routes = []
    for i in range(len(possible_direct_routes)):
        
        avoidance_route = []
        for j in range(len(possible_direct_routes[i])-1):
            for k in range(len(avoidance_legs)):
                if (avoidance_legs[k][0] == possible_direct_routes[i][j]) and (avoidance_legs[k][-1] == possible_direct_routes[i][j+1]):
                    if j==0:
                        avoidance_route = avoidance_route + avoidance_legs[k]
                    else:
                        avoidance_route = avoidance_route + avoidance_legs[k][1:]
                    
                    break
        possible_avoidance_routes.append(avoidance_route)
        
    return possible_avoidance_routes