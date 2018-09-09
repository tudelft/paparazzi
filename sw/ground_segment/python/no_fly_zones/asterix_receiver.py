# -*- coding: utf-8 -*-
"""
Created on Wed Jul  4 13:25:36 2018

@author: dennis
"""

import sys
from os import path, getenv
import threading
import socket
import struct
import asterix
import coordinate_transformations as coord_trans
import numpy as np
import json
import time

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprz_math import geodetic
from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

ft_to_m = 0.3048

exitFlag = False

class ReceiverThread (threading.Thread):

    def __init__(self, threadID, name, ref_utm_i, interface):
        """
        Initialize receiver thread
        :param threadID:
        :param name:
        :param counter:
        :return:
        """
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.ref_utm_i = ref_utm_i
        self.TrkN_lst = np.array([])     
        self.Traffic_object_lst = np.array([])
        self.Active_object_lst = np.array([]) # Active if at least 2 packages have been arrived from the AC (for speed and hdg)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.interface = interface
        self.visualizer = TrafficVisualizer(self.interface)
        self.dt = 0
        self.visualize_time = time.clock()
        self.active_traffic_objects = []
    def run(self):
        """
        Receive asterix packets
        :return:
        """
        print("Starting " + self.name)

        self.sock.bind(('', 45454))
        mreq = struct.pack("=4sl", socket.inet_aton("224.51.105.104"), socket.INADDR_ANY)
        #mreq=socket.inet_aton("224.51.105.104")+socket.inet_aton("10.42.0.1") # If hotspot on
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        
        while True:
            if exitFlag:
                self.name.exit()
            asterix_packet = self.sock.recv(10240)
            parsed = asterix.parse(asterix_packet)
            
            for i in range(len(parsed)):
                # Asterix data of current package under inspection
                TrkN = parsed[i]['I040']['TrkN']['val'] 
                ToT = parsed[i]['I070']['ToT']['val'] # [s]
                Lat = parsed[i]['I105']['Lat']['val'] # [deg]
                Lon = parsed[i]['I105']['Lon']['val'] # [deg]
                Alt = parsed[i]['I130']['Alt']['val'] * ft_to_m # [m]
                RoC = parsed[i]['I220']['RoC']['val'] * ft_to_m # [m]
                
                # Add new aircraft to the traffic list and initialize
                if TrkN not in self.TrkN_lst:
                    self.TrkN_lst = np.append(self.TrkN_lst, TrkN)
                    self.Traffic_object_lst = np.append(self.Traffic_object_lst, ReceivedTraffic(TrkN, ToT, Lat, Lon, Alt, RoC, self.ref_utm_i, self.interface))
                    self.Active_object_lst = np.append(self.Active_object_lst, False) # Object not directly active after initialisation, active after reception of 2 data packages for speed and heading computations
                elif(self.Traffic_object_lst[self.TrkN_lst == TrkN][0].ToT_lst[-1] != ToT):
                    self.Traffic_object_lst[self.TrkN_lst == TrkN][0].update_traffic(ToT, Lat, Lon, Alt, RoC, i) # Updare traffic in Traffic_object_lst
                    if self.Active_object_lst[self.TrkN_lst == TrkN][0] != True:
                        
                        self.Active_object_lst[self.TrkN_lst == TrkN] = True # Set object to actif if not active yet
            
            # Remove old non streamed objects from the traffic object list
            if len(parsed) > 0 :
                current_ToT = parsed[0]['I070']['ToT']['val']
                i = 0
                while i < len(self.Traffic_object_lst):
                    if (((abs(current_ToT - self.Traffic_object_lst[i].ToT_lst[-1])) > 4) or ((time.time() - self.Traffic_object_lst[i].sys_time) > 5)): # Remove object if no data received for 5 seconds (And restart of server)
                        self.TrkN_lst = np.delete(self.TrkN_lst, i)
                        self.Traffic_object_lst = np.delete(self.Traffic_object_lst, i)
                        self.Active_object_lst = np.delete(self.Active_object_lst, i)
                    else:
                        i = i + 1
            
            # Update active traffic list
            self.active_traffic_objects = self.Traffic_object_lst[self.Active_object_lst == True]
            # Visualize traffic on the ground station
            self.visualizer.visualize(self.TrkN_lst, self.Traffic_object_lst, self.Active_object_lst)            
            
            #print('%d. Receiver received = %s' % (self.counter, parsed))
            #print(parsed)
            #print(len(parsed))
            #print(asterix.describe(parsed))

        print("Exiting " + self.name)
        
#    def run(self):
#        """
#        Receive asterix packets
#        :return:
#        """
#        print("Starting " + self.name)
#
#        self.sock.bind(('', 45454))
#        mreq = struct.pack("=4sl", socket.inet_aton("224.51.105.104"), socket.INADDR_ANY)
#        #mreq=socket.inet_aton("224.51.105.104")+socket.inet_aton("10.42.0.1") # If hotspot on
#        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
#        
#        
#        while True:
#            if exitFlag:
#                self.name.exit()
#            parsed = json.loads(self.sock.recv(10240))
#                 
#            # Asterix data of current package under inspection
#            TrkN = parsed['I040']['TrkN']['val'] 
#            ToT = parsed['I070']['ToT']['val'] # [s]
#            Lat = parsed['I105']['Lat']['val'] # [deg]
#            Lon = parsed['I105']['Lon']['val'] # [deg]
#            Alt = parsed['I130']['Alt']['val'] * ft_to_m # [m]
#            RoC = parsed['I220']['RoC']['val'] * ft_to_m # [m]
#                
#
#            # Add new aircraft to the traffic list and initialize
#            if TrkN not in self.TrkN_lst:
#                self.TrkN_lst = np.append(self.TrkN_lst, TrkN)
#                self.Traffic_object_lst = np.append(self.Traffic_object_lst, ReceivedTraffic(TrkN, ToT, Lat, Lon, Alt, RoC, self.ref_utm_i))
#                self.Active_object_lst = np.append(self.Active_object_lst, False) # Object not directly active after initialisation, active after reception of 2 data packages for speed and heading computations
#                
#            elif(self.Traffic_object_lst[self.TrkN_lst == TrkN][0].ToT_lst[-1] != ToT):
#                self.Traffic_object_lst[self.TrkN_lst == TrkN][0].update_traffic(ToT, Lat, Lon, Alt, RoC) # Updare traffic in Traffic_object_lst
#                if self.Active_object_lst[self.TrkN_lst == TrkN][0] != True:
#                    
#                    self.Active_object_lst[self.TrkN_lst == TrkN] = True # Set object to actif if not active yet
#            
#            # Remove old non streamed objects from the traffic object list
#            current_ToT = parsed['I070']['ToT']['val']
#            i = 0
#            while i < len(self.Traffic_object_lst):
#                if ((abs(current_ToT - self.Traffic_object_lst[i].ToT_lst[-1])) > 5): # Remove object if no data received for 5 seconds (And restart of server)
#                    self.TrkN_lst = np.delete(self.TrkN_lst, i)
#                    self.Traffic_object_lst = np.delete(self.Traffic_object_lst, i)
#                    self.Active_object_lst = np.delete(self.Active_object_lst, i)
#                else:
#                    i = i + 1
#                    
#            # Update active traffic list
#            self.active_traffic_objects = self.Traffic_object_lst[self.Active_object_lst == True]
#            
#            # Visualize traffic on the ground station
#            if (self.dt < 0.5):
#                self.dt = time.clock() - self.visualize_time
#            else:
#                self.visualizer.visualize(self.TrkN_lst, self.Traffic_object_lst, self.Active_object_lst) 
#                self.dt = 0
#                self.visualize_time = time.clock()
#                
#            
#            # Visualize traffic on the ground station
#            #print('%d. Receiver received = %s' % (self.counter, parsed))
#            #print(parsed)
#            #print(len(parsed))
#            #print(asterix.describe(parsed))
#            
#
#        print("Exiting " + self.name)
        
        
class ReceivedTraffic(object):
    def __init__(self, TrkN, ToT, Lat, Lon, Alt, RoC, ref_utm_i, interface):
        self.TrkN = TrkN
        self.ToT_lst = [ToT] # Needs at least 2 time values to derive speed and heading
        self.sys_time = time.time()
        self.ref_utm_i = ref_utm_i
        self.lla_lst = [geodetic.LlaCoor_i(int(Lat*10**7), int(Lon*10**7), int((Alt)*10**3))] # Needs at least last 2 coordinates to derive speed and heading
        self.lla_lst_prey = []
        self.enu_lst = [coord_trans.lla_to_enu_fw(self.lla_lst[-1], self.ref_utm_i)]
        self.enu_lst_prey = []
        self.Roc = RoC
        self.gspeed = None
        self.gspeed_prey = None
        self.velocity = None
        self.velocity_prey = None
        self.prey_loitering = False
        self.hdg = None
        self.hdg_prey = None
        self.yaw_rate = None
        self.radius = None
        self.radius_prey = None
        self.store_radius()
        self.interface = interface
        self.visualizer = TrafficVisualizer(self.interface)
        
    
    def update_traffic(self, ToT, Lat, Lon, Alt, RoC, number):
        self.lla_lst = self.lla_lst[-1:] + [geodetic.LlaCoor_i(int(Lat*10**7), int(Lon*10**7), int((Alt)*10**3))]
        self.enu_lst = self.enu_lst[-1:] + [coord_trans.lla_to_enu_fw(self.lla_lst[-1], self.ref_utm_i)]
        self.RoC = RoC
        self.ToT_lst = self.ToT_lst[-1:] + [ToT]
        self.sys_time = time.time()
        dt = (self.ToT_lst[1] - self.ToT_lst[0])
        dx = (self.enu_lst[1].x - self.enu_lst[0].x)
        dy = (self.enu_lst[1].y - self.enu_lst[0].y) 
        self.gspeed = {'east': dx/dt , 'north' : dy/dt }
        self.velocity = np.sqrt(self.gspeed['east']**2 + self.gspeed['north']**2)
        
        
        hdg = np.rad2deg(np.arctan2(self.gspeed['east'], self.gspeed['north'])) % 360.
        if self.hdg != None:
            yaw_rate = hdg - self.hdg #new heading - old heading
            if yaw_rate > 180.:
                yaw_rate = yaw_rate - 360.
            if yaw_rate < -180.:
                yaw_rate = yaw_rate + 360.
        else:
            yaw_rate = None
        self.yaw_rate = yaw_rate
        self.hdg = hdg
        
        # Update the radius if localised weather event with altitude
        if (self.TrkN >= 20000 and self.TrkN < 30000):
            self.store_radius()
        
        # Check if bird of prey is loitering or intercepting and store data
        if ((self.TrkN >= 40000 and self.TrkN < 50000) and self.yaw_rate != None):
            self.Bird_of_prey_detection()
            if (self.prey_loitering == True):
                self.visualizer.visualize_bird_of_prey(number, self.lla_lst_prey[-1], self.radius_prey)
            else:
                self.visualizer.devisualize_bird_of_prey(number)
        
    def store_radius(self):
        # Other Air Traffic
        if (self.TrkN < 20000):
            self.radius = 30. # Mind to set it to 300 for DALBY !!!!!!!!!!
        # Localised Weather Events
        elif (self.TrkN >= 20000 and self.TrkN < 30000): 
            alt = self.lla_lst[-1].alt/1000. # connect to receive messages !!!!!!!
            self.radius = 150. + 150. * alt/3000.
        # Animal Migratory Bird
        elif (self.TrkN >= 30000 and self.TrkN < 40000):
            self.radius = 100.
        # Animal Bird of prey
        else:
            self.radius = 200.
            
    def Bird_of_prey_detection(self):
        if abs(self.yaw_rate) > 2.: # If circling/loitering
            circle_time = 360. / abs(self.yaw_rate) # [s]
            circle_radius = (circle_time * self.velocity) / (2. * np.pi)
            self.radius_prey = circle_radius + self.radius # Radius of loiter circle + radius of NFZ 
            if self.yaw_rate > 0:
                circle_center_x = self.enu_lst[-1].x - circle_radius * np.sin(np.deg2rad(-90 + self.hdg))
                circle_center_y = self.enu_lst[-1].y - circle_radius * np.cos(np.deg2rad(-90 + self.hdg))
            else:
                circle_center_x = self.enu_lst[-1].x + circle_radius * np.sin(np.deg2rad(-90 + self.hdg))
                circle_center_y = self.enu_lst[-1].y + circle_radius * np.cos(np.deg2rad(-90 + self.hdg))
            if (len(self.enu_lst_prey) < 1):
                self.enu_lst_prey.append(geodetic.EnuCoor_f(circle_center_x, circle_center_y, self.enu_lst[-1].z))
                self.lla_lst_prey.append(coord_trans.enu_to_lla_fw(self.enu_lst_prey[-1], self.ref_utm_i))
            else:
                self.enu_lst_prey = self.enu_lst_prey[-1:] + [geodetic.EnuCoor_f(circle_center_x, circle_center_y, self.enu_lst[-1].z)]
                self.lla_lst_prey = self.lla_lst_prey[-1:] + [coord_trans.enu_to_lla_fw(self.enu_lst_prey[-1], self.ref_utm_i)]
                dt = (self.ToT_lst[1] - self.ToT_lst[0])
                dx = (self.enu_lst_prey[1].x - self.enu_lst_prey[0].x)
                dy = (self.enu_lst_prey[1].y - self.enu_lst_prey[0].y) 
                self.gspeed_prey = {'east': dx/dt , 'north' : dy/dt }
                self.hdg_prey = np.rad2deg(np.arctan2(self.gspeed_prey['east'], self.gspeed_prey['north'])) % 360.
                self.velocity_prey = np.sqrt(self.gspeed_prey['east']**2 + self.gspeed_prey['north']**2)
                self.prey_loitering = True
        
        # If bird of prey not loitering, reset circle buffers
        else:
            self.enu_lst_prey = []
            self.lla_lst_prey = []
            self.radius_prey_list = []
            self.prey_loitering = False
        
        
class TrafficVisualizer(object):
    def __init__(self, interface):
        self._interface = interface
        
    def visualize(self, TrkN_lst, Traffic_object_lst, Active_object_lst):
        plotable_traffic = Traffic_object_lst[Active_object_lst == True]
        for i in range(len(plotable_traffic)):
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = int(32 + i)
            msg['linecolor'] = "orange"
            msg['fillcolor'] = "yellow"
            msg['opacity'] = 1
            msg['shape'] = 0
            msg['status'] = 0
            msg['latarr'] = [int(plotable_traffic[i].lla_lst[-1].lat), 0] # e-7 deg
            msg['lonarr'] = [int(plotable_traffic[i].lla_lst[-1].lon), 0] # e-7 deg
            msg['radius'] = plotable_traffic[i].radius
            msg['text'] = str(int(plotable_traffic[i].radius))
            self._interface.send(msg)
        # devisualize non active traffic
        for j in range(40-len(plotable_traffic)):
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = int(32 + len(plotable_traffic) + j)
            msg['linecolor'] = "orange"
            msg['fillcolor'] = "yellow"
            msg['opacity'] = 1
            msg['shape'] = 0
            msg['status'] = 1
            msg['latarr'] = [0, 0] # e-7 deg
            msg['lonarr'] = [0, 0] # e-7 deg
            msg['radius'] = 100.
            self._interface.send(msg)
            # Remove birds as well
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = int(72 + len(plotable_traffic) + j)
            msg['linecolor'] = "orange"
            msg['fillcolor'] = "yellow"
            msg['opacity'] = 1
            msg['shape'] = 0
            msg['status'] = 1
            msg['latarr'] = [0, 0] # e-7 deg
            msg['lonarr'] = [0, 0] # e-7 deg
            msg['radius'] = 100.
            self._interface.send(msg)
            
            
    def visualize_bird_of_prey(self, number, lla, radius):
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = int(72 + number)
        msg['linecolor'] = "orange"
        msg['fillcolor'] = "orange"
        msg['opacity'] = 1
        msg['shape'] = 0
        msg['status'] = 0
        msg['latarr'] = [int(lla.lat), 0] # e-7 deg
        msg['lonarr'] = [int(lla.lon), 0] # e-7 deg
        msg['radius'] = radius
        msg['text'] = "Bird"
        self._interface.send(msg)
        
    def devisualize_bird_of_prey(self, number):
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = int(72 + number)
        msg['linecolor'] = "orange"
        msg['fillcolor'] = "orange"
        msg['opacity'] = 1
        msg['shape'] = 0
        msg['status'] = 0
        msg['latarr'] = [0, 0] # e-7 deg
        msg['lonarr'] = [0, 0] # e-7 deg
        msg['radius'] = 100.
        msg['text'] = "Bird"
        self._interface.send(msg)
            

        
## Create new threads
#ref_lla_i = geodetic.LlaCoor_i(int(-27.2738966*10.**7), int(151.2900371*10.**7), int(343.8*1000.))
#ref_utm_i = geodetic.UtmCoor_i()
#geodetic.utm_of_lla_i(ref_utm_i, ref_lla_i)
#
#receiver_thread = ReceiverThread(1, "Receiver", ref_utm_i, IvyMessagesInterface("Traffic Server"))
#
## Start new Threads
#receiver_thread.start()
#
#print("Exiting Main Thread") 