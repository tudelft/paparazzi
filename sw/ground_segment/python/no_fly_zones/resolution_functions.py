# -*- coding: utf-8 -*-
"""
Created on Thu May 31 11:18:55 2018

@author: dennis
"""

import numpy as np
import sys
import shapely.geometry as geometry
import shapely.affinity as affinity
import shapely.ops as ops

# Function that calculates the Zone that first collides with the UAV for a giiven leg
def leg_calc_first_intersection(Zones, position, target_pos):
    # Check collisions on the leg:
    leg_vector = geometry.LineString([position, target_pos])
        
    first_intersection = {'dist': None, 'Zone': None, 'collision': False} # Initialise dictionary for first intersection
    for i in range(len(Zones)):
        if leg_vector.crosses(Zones[i].polygon):
            intersection_line = Zones[i].polygon.intersection(leg_vector)# leg_vector.intersection(Zones[i].polygon) # MultiPoint
            # Calculate distance per intersectionpoint and search for nearest intersection
            for j in range(len(intersection_line.coords)):
                distance = geometry.LineString(([position[0], position[1]], intersection_line.coords[j])).length
                if first_intersection['dist'] == None or distance < first_intersection['dist']:
                    first_intersection['dist'] = distance
                    first_intersection['Zone'] = Zones[i]
                    first_intersection['collision'] = True
                    
    return first_intersection

class leg_horizontal_resolutiions(object):
    def __init__(self, Zone, Zones, position, target_pos, route):
        self.Zone = Zone
        self.Zones = Zones
        self.position = position
        self.target_pos = target_pos
        self.resolutions = []
        self.route = route
    
    
    def calc_horizontal_res(self):
        dx_leg = self.target_pos[0] - self.position[0]
        dy_leg = self.target_pos[1] - self.position[1]
        azimuth_leg = np.arctan2(dx_leg, dy_leg) # in range -pi, pi
        points_zone = geometry.mapping(self.Zone.polygon)['coordinates'][0]
        
        max_right_hdg = 0
        max_left_hdg = 0
        point_left = None
        point_right = None
        
        for i in range(len(points_zone)-1): #-1 because first and final point are equal
            point = points_zone[i]
            dx_point = point[0] - self.position[0]
            dy_point = point[1] - self.position[1]
            azimuth_point = np.arctan2(dx_point, dy_point)
            d_hdg = np.rad2deg(azimuth_point - azimuth_leg)
            
            # Make sure heading difference is in range -pi, pi
            if d_hdg>180:
                d_hdg = d_hdg-360
            if d_hdg<-180:
                d_hdg = d_hdg+360
            
            # Resolution to the right
            if d_hdg>0 and (d_hdg > max_right_hdg): 
                max_right_hdg = d_hdg
                point_right = points_zone[i]
            
            # Resolution to the left
            if d_hdg<0 and (d_hdg < max_left_hdg): 
                max_left_hdg = d_hdg
                point_left = points_zone[i]
        #Give a horizontal resolution
        new_resolutions = []
        if (point_left not in (self.resolutions or self.route)) and (point_left is not None) and (point_left != self.position):
            self.resolutions.append(point_left)
            new_resolutions.append(point_left)
        if (point_right not in (self.resolutions or self.route)) and (point_right is not None) and (point_right != self.position):
            self.resolutions.append(point_right)
            new_resolutions.append(point_right)
            
            if len(new_resolutions)>0:
                for i in range(len(new_resolutions)):
                    new_collision = leg_calc_first_intersection(self.Zones, self.position, points_zone[i])
                    if new_collision['collision']:
                        self.target_pos = new_resolutions[i]
                        self.calc_horizontal_res()
        return self.resolutions

def leg_calc_horizontal_res(Zone, Zones, position, target_pos, route):
    resolution_object = leg_horizontal_resolutiions(Zone, Zones, position, target_pos, route)
    resolutions = resolution_object.calc_horizontal_res()
    return resolutions
## Calculates resolution
#def leg_give_resolutions(Zone, position, target_pos):
#    conflict_res = leg_calc_intersection_hdg_offset(Zone, position, target_pos)
#    if conflict_res['left_found'] and conflict_res['right_found']:
#        num_res = 2
#        resolution = (conflict_res['left_turning_point'], conflict_res['right_turning_point'])
#    elif conflict_res['left_found'] and not conflict_res['right_found']:
#        resolution = conflict_res['left_turning_point']
#    elif conflict_res['right_found'] and not conflict_res['left_found']:
#        resolution = conflict_res['right_turning_point']
#    else:
#        num_res = 0
#    return num_res, resolution

# Calculate a new route
#def leg_calc_route(Zones, position, target_pos):
#    first_intersection = leg_calc_first_intersection(Zones, position, target_pos)
#        
#    if first_intersection['collision']:
#        investigated_all_routes = False
#    else: 
#        investigated_all_routes = True
#        
#    route = []
#    
#    while not investigated_all_routes:    
#        collision_free = False
#        while not collision_free: 
#            first_intersection = leg_calc_first_intersection(Zones, position, target_pos)
#            resolutions = leg_give_resolutions(Zone, position, target_pos)
#            for i in range(resolutions[0]):


class optimize_leg(object):
    def __init__(self, Zones, position, target_position):
        self.Zones = Zones
        self.position = position
        self.target_position = target_position
        self.routelist = []
        self.route = []
        self.loop_number = 0
    
    def leg_walk_routes(self):
        if len(self.route)>0:
            if self.route[-1] == self.target_position:
                self.route = self.route[:self.loop_number]
        
        self.route.append(self.position)
        first_intersection = leg_calc_first_intersection(self.Zones, self.position, self.target_position)
        if first_intersection['collision']:
            resolutions = leg_calc_horizontal_res(first_intersection['Zone'], self.Zones, self.position, self.target_position, self.route)
            for i in range(len(resolutions)):
                #Keep track of loop number
                if i==0:
                    self.loop_number = self.loop_number + 1
                
                self.position = resolutions[i]
                self.leg_walk_routes()
                
                if i ==len(resolutions)-1:
                    self.loop_number = self.loop_number - 1
                
        else:
            self.route.append(self.target_position)
            self.routelist.append(self.route)
            #print "route: ", self.route
            #sys.stdout.flush()
        return self.routelist

def leg_return_routelist(Zones, position, target_position):
    leg_optimization = optimize_leg(Zones, position, target_position)
    routelist = leg_optimization.leg_walk_routes()
    return routelist
    
def leg_return_shortest_leg(Zones, position, target_position):
    leg_optimization = optimize_leg(Zones, position, target_position)
    routelist = leg_optimization.leg_walk_routes()
    shortest_leg = leg_shortest_route(routelist)
    return shortest_leg
    
def leg_shortest_route(routelist):
    shortest_length = None
    for i in range(len(routelist)):
        route = geometry.LineString(routelist[i])
        length = route.length
        if shortest_length == None or length < shortest_length:
            shortest_length = length
            shortest_route = routelist[i]
    return shortest_route
    
###############################################################################
# Functions for circular no fly Zones                                         #
###############################################################################

# Function that calculates the Zone that first collides with the UAV for a giiven leg
def leg_calc_first_intersection_cz(Zones, position, target_pos):
    # Check collisions on the leg:
    leg_vector = geometry.LineString([position, target_pos])
        
    first_intersection = {'dist': None, 'Zone': None, 'collision': False} # Initialise dictionary for first intersection
    for i in range(len(Zones)):
        if leg_vector.crosses(Zones[i].circle['geo']):
            intersection_line = Zones[i].circle['geo'].intersection(leg_vector)# leg_vector.intersection(Zones[i].circle['geo']) # MultiPoint
            # Calculate distance per intersectionpoint and search for nearest intersection
            for j in range(len(intersection_line.coords)):
                distance = geometry.LineString(([position[0], position[1]], intersection_line.coords[j])).length
                if first_intersection['dist'] == None or distance < first_intersection['dist']:
                    first_intersection['dist'] = distance
                    first_intersection['Zone'] = Zones[i]
                    first_intersection['collision'] = True
                    
    return first_intersection    
    
class leg_horizontal_resolutiions_cz(object):
    def __init__(self, Zone, Zones, position, target_pos, route, from_Zone = None):
        self.Zone = Zone
        self.Zones = Zones
        self.position = position
        self.target_pos = target_pos
        self.resolutions = []
        self.route = route
        self.from_Zone = from_Zone
    
    def calc_horizontal_res(self):
         # Calc 2 resolution points

        dx_center_pos = self.Zone.circle['center'][0] - self.position[0]
        dy_center_pos = self.Zone.circle['center'][1] - self.position[1]
        
        center_length_pos = np.sqrt(dx_center_pos**2 + dy_center_pos**2) 
        
        
        avoid_angle_pos = np.rad2deg(np.arcsin(self.Zone.circle['radius']/center_length_pos))+5.

        center_vector_pos = geometry.LineString([self.position, (self.position[0] + dx_center_pos*100., self.position[1] + dy_center_pos*100.)])
        center_vector_pos_avoid_left = affinity.rotate(center_vector_pos, avoid_angle_pos, origin=self.position)
        center_vector_pos_avoid_right = affinity.rotate(center_vector_pos, -avoid_angle_pos, origin=self.position)
        
        dx_center_target = self.Zone.circle['center'][0] - self.target_pos[0]
        dy_center_target = self.Zone.circle['center'][1] - self.target_pos[1]
        
        center_length_target = np.sqrt(dx_center_target**2 + dy_center_target**2) 
        
        avoid_angle_target = np.rad2deg(np.arcsin(self.Zone.circle['radius']/center_length_target))+5.
        
        center_vector_target = geometry.LineString([self.target_pos, (self.target_pos[0] + dx_center_target*100., self.target_pos[1] + dy_center_target*100.)])
        center_vector_target_avoid_left = affinity.rotate(center_vector_target, avoid_angle_target, origin=self.target_pos)
        center_vector_target_avoid_right = affinity.rotate(center_vector_target, -avoid_angle_target, origin=self.target_pos)

        sys.stdout.flush()        
        
        
        new_resolutions = []
        
        resolution1 = center_vector_pos_avoid_left.intersection(center_vector_target_avoid_right).coords[0]
        resolution2 = center_vector_pos_avoid_right.intersection(center_vector_target_avoid_left).coords[0]
        
        if (self.from_Zone != None):
            
            new_collision1 = leg_calc_first_intersection_cz(self.Zones, self.position, resolution1)
            if new_collision1['collision']:
                if new_collision1['Zone'] != self.from_Zone:
                    new_resolutions.append(resolution1)
            else:
                new_resolutions.append(resolution1)
                
            new_collision2 = leg_calc_first_intersection_cz(self.Zones, self.position, resolution2)
            if new_collision2['collision']:
                if new_collision2['Zone'] != self.from_Zone:
                    new_resolutions.append(resolution2)
            else:
                new_resolutions.append(resolution2)
        
        else:
            #self.resolutions.append(resolution1)
            new_resolutions.append(resolution1)
            #self.resolutions.append(resolution2)
            new_resolutions.append(resolution2)
                
        sys.stdout.flush()
            
        #Give a horizontal resolutions
        if len(new_resolutions)>0:
            for i in range(len(new_resolutions)):
                new_collision = leg_calc_first_intersection_cz(self.Zones, self.position, new_resolutions[i])
                
                if new_collision['collision']:
                    self.Zone = new_collision['Zone']
                    self.target_pos = new_resolutions[i]
                    self.calc_horizontal_res()
                else:
                    
                    self.resolutions.append(new_resolutions[i])
        
        return self.resolutions
    
#class leg_horizontal_resolutiions_cz(object):
#    def __init__(self, Zone, Zones, position, target_pos, route, from_Zone = None):
#        self.Zone = Zone
#        self.Zones = Zones
#        self.position = position
#        self.target_pos = target_pos
#        self.resolutions = []
#        self.route = route
#        self.from_Zone = from_Zone
#    
#    def calc_horizontal_res(self):
#         # Calc 2 resolution points
#        print('from point: ', self.position, ' to point: ', self.target_pos)
#        sys.stdout.flush()
#        #dx_center_pos = self.Zone.circle['center'][0] - self.position[0]
#        #dy_center_pos = self.Zone.circle['center'][1] - self.position[1]
#
#        
#        #center_vector_pos = geometry.LineString([self.position, (dx_center_pos*10., dy_center_pos*10.)])
#        #avoid_angle_pos = self.calc_avoidance_angle(center_vector_pos)+1.
#        #print("avoid angle pos: ", avoid_angle_pos)
#        
#        avoidance_vectors_pos = self.calc_avoidance_angles(self.position, self.target_pos)  
#        
#        center_vector_pos_avoid_left = avoidance_vectors_pos['left']
#        center_vector_pos_avoid_right = avoidance_vectors_pos['right']
#        print("pos_avoid_right: ", np.array(center_vector_pos_avoid_right))
#        print("pos_avoid_left: ", np.array(center_vector_pos_avoid_left))
#        
##        dx_center_target = self.Zone.circle['center'][0] - self.target_pos[0]
##        dy_center_target = self.Zone.circle['center'][1] - self.target_pos[1]
#        
#        avoidance_vectors_target = self.calc_avoidance_angles(self.target_pos, self.position)  
#        
##        center_vector_target = geometry.LineString([self.target_pos, (dx_center_target*10., dy_center_target*10.)])
##        avoid_angle_target = self.calc_avoidance_angle(center_vector_target)+1.    
##        print("avoid angle target: ", avoid_angle_target)
#        
#        
#        
#        center_vector_target_avoid_left = avoidance_vectors_target['left']
#        center_vector_target_avoid_right = avoidance_vectors_target['right']
#        print("target_avoid_left: ", np.array(center_vector_target_avoid_left))
#        print("target_avoid_right: ", np.array(center_vector_target_avoid_right))
#             
#        
#        
#        new_resolutions = []
#        
#        resolution1 = center_vector_pos_avoid_left.intersection(center_vector_target_avoid_right).coords[0]
#        print("resolution 1: ", resolution1)
#        sys.stdout.flush()   
#        resolution2 = center_vector_pos_avoid_right.intersection(center_vector_target_avoid_left).coords[0]
#        
#        
#        if (self.from_Zone != None):
#            
#            new_collision1 = leg_calc_first_intersection_cz(self.Zones, self.position, resolution1)
#            if new_collision1['collision']:
#                if new_collision1['Zone'] != self.from_Zone:
#                    new_resolutions.append(resolution1)
#            else:
#                new_resolutions.append(resolution1)
#                
#            new_collision2 = leg_calc_first_intersection_cz(self.Zones, self.position, resolution2)
#            if new_collision2['collision']:
#                if new_collision2['Zone'] != self.from_Zone:
#                    new_resolutions.append(resolution2)
#            else:
#                new_resolutions.append(resolution2)
#        
#        else:
#            #self.resolutions.append(resolution1)
#            new_resolutions.append(resolution1)
#            #self.resolutions.append(resolution2)
#            new_resolutions.append(resolution2)
#                
#        print('new_reslutions: ', new_resolutions)
#        sys.stdout.flush()
#            
#        #Give a horizontal resolutions
#        if len(new_resolutions)>0:
#            for i in range(len(new_resolutions)):
#                new_collision = leg_calc_first_intersection_cz(self.Zones, self.position, new_resolutions[i])
#                
#                if new_collision['collision']:
#                    print("test1: ", new_collision['Zone'].id_zone)
#                    sys.stdout.flush()
#                    self.Zone = new_collision['Zone']
#                    self.target_pos = new_resolutions[i]
#                    self.calc_horizontal_res()
#                else:
#                    
#                    self.resolutions.append(new_resolutions[i])
#        
#        return self.resolutions
#        
#    def calc_avoidance_angles(self, from_point, to_point, step = 1.):
#        avoidance_vectors = {}        
#        
#        colliding_left = True
#        colliding_right = True
#        path_vector = geometry.LineString([from_point, to_point])
#        avoidance_vector_left = path_vector
#        avoidance_vector_right = path_vector
#        avoidance_angle_left = 0.
#        avoidance_angle_right = 0.
#        
#        while colliding_left:
#            collision_left = leg_calc_first_intersection_cz([self.Zone], self.position, avoidance_vector_left.coords[1])
#            if collision_left['collision']:
#                avoidance_angle_left = avoidance_angle_left + step
#                avoidance_vector_left = affinity.rotate(path_vector, avoidance_angle_left, origin=avoidance_vector_left.coords[0])
#                
#            else:
#                avoidance_angle_left = avoidance_angle_left + 5* step
#                avoidance_vector_left = affinity.rotate(path_vector, avoidance_angle_left, origin=avoidance_vector_left.coords[0])
#                colliding_left = False
#        
#        avoidance_vectors['left'] = avoidance_vector_left
#                
#        while colliding_right:
#            collision_right = leg_calc_first_intersection_cz([self.Zone], self.position, avoidance_vector_right.coords[1])
#            if collision_right['collision']:
#                avoidance_angle_right = avoidance_angle_right + step
#                avoidance_vector_right = affinity.rotate(path_vector, -avoidance_angle_right, origin=avoidance_vector_right.coords[0])
#                
#            else:
#                avoidance_angle_right = avoidance_angle_right + 5* step
#                avoidance_vector_right = affinity.rotate(path_vector, -avoidance_angle_right, origin=avoidance_vector_right.coords[0])
#                colliding_right = False
#                
#                
#        avoidance_vectors['right'] = avoidance_vector_right
#                
#        return avoidance_vectors
                    
                        
            
            
            

def leg_calc_horizontal_res_cz(Zone, Zones, position, target_pos, route, from_Zone = None):
    resolution_object = leg_horizontal_resolutiions_cz(Zone, Zones, position, target_pos, route, from_Zone)
    resolutions = resolution_object.calc_horizontal_res()
    return resolutions
    
class optimize_leg_cz(object):
    def __init__(self, Zones, position, target_position):
        self.Zones = Zones
        self.position = position
        self.target_position = target_position
        self.routelist = []
        self.route = []
        self.loop_number = 0
        self.from_Zone = None
    
    def leg_walk_routes(self):
        
        if len(self.route)>0:
            if self.route[-1] == self.target_position:
                self.route = self.route[:self.loop_number]
        
        self.route.append(self.position)
        first_intersection = leg_calc_first_intersection_cz(self.Zones, self.position, self.target_position)
        if first_intersection['collision']:
            resolutions = leg_calc_horizontal_res_cz(first_intersection['Zone'], self.Zones, self.position, self.target_position, self.route, self.from_Zone)
            print("resolutions", resolutions)
            sys.stdout.flush()
            for i in range(len(resolutions)):
                #Keep track of loop number
                if i==0:
                    self.loop_number = self.loop_number + 1
                
                self.position = resolutions[i]
                self.from_Zone = first_intersection['Zone']
                self.leg_walk_routes()
                
                if i ==len(resolutions)-1:
                    self.loop_number = self.loop_number - 1
                
        else:
            self.route.append(self.target_position)
            self.routelist.append(self.route)
        
        
        return self.routelist
    
def leg_return_routelist_cz(Zones, position, target_position):
    leg_optimization = optimize_leg_cz(Zones, position, target_position)
    routelist = leg_optimization.leg_walk_routes()
    return routelist
    
def leg_return_shortest_leg_cz(Zones, position, target_position):
    leg_optimization = optimize_leg_cz(Zones, position, target_position)
    routelist = leg_optimization.leg_walk_routes()
    shortest_leg = leg_shortest_route(routelist)
    return shortest_leg