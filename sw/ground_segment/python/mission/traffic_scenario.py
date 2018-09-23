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

class TrafficScenario(object):
    def __init__(self, circular_zones, ltp_def):
        self.UAV_speed = 0 # [m/s]
        self.UAV_point_enu = 0
        self.UAV_lla = 0
        self.UAV_hdg = 0
        self.Traffic = traffic.Aircraft()
        self.circular_zones = circular_zones
        self.circular_zones_lla = []
        for zone in circular_zones:
            zone_ecef = geodetic.EnuCoor_f(zone[0], zone[1], 0).to_ecef(ltp_def)
            zone_lla = zone_ecef.to_lla()
            self.circular_zones_lla.append(zone_lla)
        
    def update_traffic_scenario(self, aircraft, traffic_events, groundspeed):
        self.UAV_speed = groundspeed # [m/s]
        self.UAV_point_enu = aircraft.get_enu()#geodetic.EnuCoor_f(self.UAV.P[0], self.UAV.P[1], self.UAV.P[2])
        self.UAV_lla = aircraft.get_lla()
        self.UAV_hdg = aircraft.get_course()
        
        self.Traffic = traffic.Aircraft()
        self.Traffic.create(1 , self.UAV_speed, np.rad2deg(self.UAV_lla.lat), np.rad2deg(self.UAV_lla.lon), self.UAV_hdg, 'UAV', 0.)
    
        i=0
        for traffic_event in traffic_events:
            self.Traffic.create(1, traffic_event.get_velocity(), np.rad2deg(traffic_event.get_lla().lat), np.rad2deg(traffic_event.get_lla().lon), traffic_event.get_course(), 'AC' + str(i), traffic_event.get_radius())
            i += i
            
        for j in range(len(self.circular_zones_lla)):
            self.Traffic.create(1, 0., np.rad2deg(self.circular_zones_lla[j].lat), np.rad2deg(self.circular_zones_lla[j].lon), 0, 'Zone' + str(j), self.circular_zones[j][2])
        
    def detect_conflicts(self, tla, wind, margin, airspeed):
        self.asas = ssd_resolutions.Asas(self.Traffic.ntraf, airspeed, 'HDG', tla, margin)
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
        #i_own = Traffic.id.index("KL204")
        
        #AC velocity vector
        v_own = np.array([self.Traffic.gseast[i_own], self.Traffic.gsnorth[i_own]])
        
        
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
        #fig, ax = plt.subplots()
        self.ax.clear()
        #line1, = plt.plot(x_SSD_outer, y_SSD_outer, color = '#404040', label="Velocity limits")
        #self.line1.set_xdata(x_SSD_outer)
        #self.line1.set_ydata(y_SSD_outer)
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
        #plt.savefig('bar.jpg',bbox_inches = 'tight')
        #plt.show()
        
class ExtrapolatedScenario(object):
    def __init__(self, circular_zones, ref_utm_i, ltp_def):
        self.circular_zones = circular_zones
        self.circular_zones_lla = []
        for zone in circular_zones:
            zone_ecef = geodetic.EnuCoor_f(zone[0], zone[1], 0).to_ecef(ltp_def)
            zone_lla = zone_ecef.to_lla()
            self.circular_zones_lla.append(zone_lla)
        self.ref_utm_i = ref_utm_i
        
    def update_traffic_scenario(self, aircraft, traffic_events, hdg, dt, groundspeed):
        self.UAV_speed = groundspeed # [m/s]
        self.UAV_point_enu = aircraft.get_enu()
        self.UAV_lla = aircraft.get_lla() # check
        self.UAV_hdg = aircraft.get_course()
        
        self.Traffic = traffic.Aircraft()
        self.Traffic.create(1 , self.UAV_speed, np.rad2deg(self.UAV_lla.lat), np.rad2deg(self.UAV_lla.lon), self.UAV_hdg, 'UAV', 0.)
        
        i=0
        for traffic_event in traffic_events:
            velocity = traffic_event.get_velocity()
            lla_point = traffic_event.get_lla()
            lla_point_old = geodetic.LlaCoor_i(int(np.rad2deg(lla_point.lat) * 10. ** 7), int(np.rad2deg(lla_point.lon) * 10. ** 7), int(np.rad2deg(lla_point.alt) * 1000.))
            hdg = traffic_event.get_course()
            radius = traffic_event.get_radius()
            
            # Speed components to compute future position
            Vx = velocity * np.sin(np.deg2rad(hdg)) # V in east direction
            Vy = velocity * np.cos(np.deg2rad(hdg)) # V in north direction
            Vz = traffic_event.get_roc() # V in upward direction
            
            # enu_point conversion to new point and to lla
            enu_point_old = coord_trans.lla_to_enu_fw(lla_point_old, self.ref_utm_i)
            enu_point_new = geodetic.EnuCoor_f(enu_point_old.x + Vx * dt, enu_point_old.y + Vy * dt, enu_point_old.z + Vz * dt)
            lla_point_new = coord_trans.enu_to_lla_fw(enu_point_new, self.ref_utm_i)
            
            # Create traffic object
            self.Traffic.create(1, velocity, lla_point_new.lat * 10.**-7, lla_point_new.lon * 10.**-7, hdg, 'AC' + str(i+1), radius)
            i += 1
             
        for j in range(len(self.circular_zones_lla)):
            self.Traffic.create(1, 0., np.rad2deg(self.circular_zones_lla[j].lat), np.rad2deg(self.circular_zones_lla[j].lon), 0, 'Zone' + str(j), self.circular_zones[j][2])
            
    def detect_conflicts(self, tla, wind, margin):
        self.asas = ssd_resolutions.Asas(self.Traffic.ntraf, self.UAV_speed, 'HDG', tla, margin)
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