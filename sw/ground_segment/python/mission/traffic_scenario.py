# -*- coding: utf-8 -*-
"""
Created on Sun Jun 17 20:50:05 2018

@author: dennis
"""

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import ssd_resolutions
import traffic
import coordinate_transformations as coord_trans
import matplotlib.pyplot as plt
from matplotlib.cbook import get_sample_data
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from PIL import Image
from matplotlib.offsetbox import (OffsetImage, AnnotationBbox)
from time import sleep


# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from pprz_math import geodetic

class traffic_scenario(object):
    def __init__(self, aircraft, circular_zones):
        self.UAV_speed = aircraft.get_ground_speed() # [m/s]
        self.UAV_point_enu = aircraft.get_enu()
        self.UAV_lla = aircraft.get_lla()
        self.UAV_hdg = aircraft.get_course()
        self.Traffic = traffic.Aircraft()
        self.circular_zones = circular_zones
        
    def update_traffic_scenario(self, aircraft, ReceiverThread):
        self.UAV_speed = aircraft.get_ground_speed() # [m/s]
        self.UAV_point_enu = aircraft.get_enu()#geodetic.EnuCoor_f(self.UAV.P[0], self.UAV.P[1], self.UAV.P[2])
        self.UAV_lla = aircraft.get_lla()
        self.UAV_hdg = aircraft.get_course()
        
        self.Traffic = traffic.Aircraft()
        self.Traffic.create(1 , self.UAV_speed, self.UAV_lat, self.UAV_lon, self.UAV_hdg, 'UAV', 0.)
        
        traffic_events = ReceiverThread.get_events()
        
        
        for i in range(len(traffic_events)):
            self.Traffic.create(1, traffic_events[i].get_speed(), traffic_events[i].get_lla().lat, traffic_events[i].get_lla().lon, traffic_events[i].get_course(), 'AC' + str(i+1), traffic_events[i].get_radius())
        
        for j in range(len(self.circular_zones)):
            self.Traffic.create(1, 0, self.circular_zones[j][2].lat * 10. **-7, self.circular_zones[j][2].lon * 10. **-7, 0, 'Zone' + str(j), self.circular_zones[j][0])
        
    def detect_conflicts(self, tla, wind, margin):
        self.asas = ssd_resolutions.Asas(self.Traffic.ntraf, self.UAV_speed, self.strategy, tla, margin)
        self.asas.inconf[0] = True
        ssd_resolutions.constructSSD(self.asas, self.Traffic, self.asas.tla, wind)
        resolutions = ssd_resolutions.calculate_resolution(self.asas, self.Traffic)
        return resolutions
        
    def calc_tcpa(self, enu_other, speed_other_x, speed_other_y, airspeed, UAV_hdg):
        du = speed_other_x - (airspeed * np.sin(np.deg2rad(UAV_hdg)))
        dv = speed_other_y - (airspeed * np.cos(np.deg2rad(UAV_hdg)))
        dx = enu_other.x - self.UAV_point_enu.x
        dy = enu_other.y - self.UAV_point_enu.y
        vrel2 = du * du + dv * dv
        tcpa = -(du * dx + dv * dy) / vrel2
        return tcpa
        
    def init_SSD_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        
    def plot_SSD(self):
        #==============================================================================
        #PLOTTING VARIABLES
        #==============================================================================
        
        #Choose ownship
        i_own = 0
        
        #AC velocity vector
        
        #SSD- CIRCLES OF VMAX AND VMIN
        vmin = self.asas.vmin
        vmax = self.asas.vmax
        N_angle = 180
        
        angles = np.arange(0, 2 * np.pi, 2 * np.pi / N_angle)
        xyc = np.transpose(np.reshape(np.concatenate((np.sin(angles), np.cos(angles))), (2, N_angle)))
        SSD_lst = [list(map(list, np.flipud(xyc * vmax))), list(map(list , xyc * vmin ))]
        
        #Outer circle: vmax
        SSD_outer = np.array(SSD_lst[0])
        x_SSD_outer = np.append(SSD_outer[:,0] , np.array(SSD_outer[0,0]))
        y_SSD_outer = np.append(SSD_outer[:,1] , np.array(SSD_outer[0,1]))
        #Inner circle: vmin
        SSD_inner = np.array(SSD_lst[1])
        x_SSD_inner = np.append(SSD_inner[:,0] , np.array(SSD_inner[0,0]))
        y_SSD_inner = np.append(SSD_inner[:,1] , np.array(SSD_inner[0,1]))
        
        
        #------------------------------------------------------------------------------
        
        #PLOTS
        self.ax.clear()
        self.ax.plot(x_SSD_outer, y_SSD_outer, color = '#404040')
        self.ax.plot(x_SSD_inner, y_SSD_inner, color = '#404040')
        
        if self.asas.ARV_calc[i_own]:
            for j in range(len(self.asas.ARV_calc[i_own])):
                FRV_1 = np.array(self.asas.ARV_calc[i_own][j])
                x_FRV1 = np.append(FRV_1[:,0] , np.array(FRV_1[0,0]))
                y_FRV1 = np.append(FRV_1[:,1] , np.array(FRV_1[0,1]))
                self.ax.plot(x_FRV1, y_FRV1, '-', color = '#000000') #grey       
        
        #Plot conflict zones            
        if self.asas.FRV[i_own]:
            for j in range(len(self.asas.FRV[i_own])):
                FRV_1 = np.array(self.asas.FRV[i_own][j])
                x_FRV1 = np.append(FRV_1[:,0] , np.array(FRV_1[0,0]))
                y_FRV1 = np.append(FRV_1[:,1] , np.array(FRV_1[0,1]))
                self.ax.plot(x_FRV1, y_FRV1, '-', color = '#FF0000') #red
                self.ax.fill(x_FRV1, y_FRV1, color = '#FF0000') #red      
                
        #Plot conflict-free zones        
        if self.asas.ARV[i_own]:
            for j in range(len(self.asas.ARV[i_own])):
                FRV_1 = np.array(self.asas.ARV[i_own][j])
                x_FRV1 = np.append(FRV_1[:,0] , np.array(FRV_1[0,0]))
                y_FRV1 = np.append(FRV_1[:,1] , np.array(FRV_1[0,1]))
                self.ax.plot(x_FRV1, y_FRV1, '-', color = '#C0C0C0')#, alpha = 0.2) #grey       
                self.ax.fill(x_FRV1, y_FRV1, color = '#C0C0C0')#, alpha = 0.2) #grey
        
        #Arrow defining the velocity vector
        vown = self.Traffic.gs[i_own]*0.92 #to get the velocity arrow with the right size
        hdg = np.radians(self.Traffic.hdg[i_own])
        vownx = vown*np.sin(hdg)
        vowny = vown*np.cos(hdg)
        self.ax.arrow(x=0,y=0, dx=vownx, dy=vowny, color = '#00CC00', head_width=1, overhang=0.5, zorder= 10)
        
        #Plot solution point         
        sol_point, = self.ax.plot(self.asas.asase[i_own], self.asas.asasn[i_own], 'd', color = '#000099', label='Solution')
        
        self.ax.axis('equal')
        self.ax.axis('off')
        self.fig.canvas.draw()
        
class traffic_scenario_extrapolated(object):
    def __init__(self, aircraft, circular_zones):
        self.circular_zones = circular_zones
        
    def update_traffic_scenario(self, commanded_airspeed, UAV_hdg, wind, pos_enu, dt, ReceiverThread):
        self.UAV_in_PZ = False
        self.UAV_speed = np.sqrt((commanded_airspeed * np.sin(np.deg2rad(UAV_hdg)) + wind['east'])**2 + (commanded_airspeed * np.cos(np.deg2rad(UAV_hdg)) + wind['north'])**2)
        self.UAV_point_enu = pos_enu
        self.UAV_lla = coord_trans.enu_to_lla_fw(self.UAV_point_enu, self.UAV.ref_utm_i) ## check
        self.UAV_lat = self.UAV_lla.lat*10.**-7
        self.UAV_lon = self.UAV_lla.lon*10.**-7
        self.UAV_hdg = float(UAV_hdg)
        self.Traffic = traffic.Aircraft()
        self.Traffic.create(1 , self.UAV_speed, self.UAV_lat, self.UAV_lon, self.UAV_hdg, 'UAV', 0.)
        
        traffic_events = ReceiverThread.get_events()
        
        for i in range(len(traffic_events)):
            velocity = traffic_events[i].get_ground_speed()
            lla_point_old = traffic_events[i].get_lla()
            hdg = traffic_events[i].get_course()
            radius = traffic_events[i].get_radius()
            
            # Speed components to compute future position
            Vx = velocity * np.sin(np.deg2rad(hdg)) # V in east direction
            Vy = velocity * np.cos(np.deg2rad(hdg)) # V in north direction
            Vz = traffic_events[i].RoC # V in upward direction
            
            # enu_point conversion to new point and to lla
            enu_point_old = coord_trans.lla_to_enu_fw(lla_point_old, self.UAV.ref_utm_i)
            enu_point_new = geodetic.EnuCoor_f(enu_point_old.x + Vx * dt, enu_point_old.y + Vy * dt, enu_point_old.z + Vz * dt)
            lla_point_new = coord_trans.enu_to_lla_fw(enu_point_new, self.UAV.ref_utm_i)
            
            # Create traffic object
            self.Traffic.create(1, velocity, lla_point_new.lat * 10.**-7, lla_point_new.lon * 10.**-7, hdg, 'AC' + str(i+1), radius)
    
        for j in range(len(self.circular_zones)):
                         
            self.Traffic.create(1, 0, self.circular_zones[j][2].lat * 10. **-7, self.circular_zones[j][2].lon * 10. **-7, 0, 'Zone' + str(j), self.circular_zones[j][0])
            
    def detect_conflicts(self, tla, wind, margin):
        self.asas = ssd_resolutions.Asas(self.Traffic.ntraf, self.UAV_speed, self.strategy, tla, margin)
        self.asas.inconf[0] = True
        if ssd_resolutions.constructSSD(self.asas, self.Traffic, self.asas.tla, wind) == 'LoS':
            return ("nosol", 0., 0.)
        
        resolutions = ssd_resolutions.calculate_resolution(self.asas, self.Traffic)
        return resolutions
        
    def calc_tcpa(self, enu_other, speed_other_x, speed_other_y, airspeed, wind, UAV_hdg):
        du = speed_other_x - (airspeed * np.sin(np.deg2rad(UAV_hdg)) + wind['east'])
        dv = speed_other_y - (airspeed * np.cos(np.deg2rad(UAV_hdg)) + wind['north'])
        dx = enu_other.x - self.UAV_point_enu.x
        dy = enu_other.y - self.UAV_point_enu.y
        vrel2 = du * du + dv * dv
        tcpa = -(du * dx + dv * dy) / vrel2
        return tcpa