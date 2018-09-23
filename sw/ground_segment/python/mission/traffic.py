# -*- coding: utf-8 -*-
"""
Created on Sun Jun 17 16:04:41 2018

@author: dennis
"""

import numpy as np
from math import *
import geo


#Conversion variables
nm = 1852 #NM to m
ft = 0.3048 #ft to m

#-----------------------------------------------------------------------------
#Functions adapted from bluesky/traffic/traffic.py
#-----------------------------------------------------------------------------

class Aircraft:
    def __init__(self):
        
        self.id      = []  # identifier (string)
        
        self.ntraf = 0
        
        #Positions
        self.lat = np.empty((0))      # latitude [deg]
        self.lon     = np.empty((0))  # longitude [deg]
        self.alt     = np.empty((0))  # altitude [m]
        self.hdg     = np.empty((0))  # traffic heading [deg]

       # Velocities
        self.gs      = np.empty((0))  # ground speed [m/s]
        self.gsnorth = np.empty((0))  # ground speed [m/s]
        self.gseast  = np.empty((0))  # ground speed [m/s]
        
       # Seperation minima
        self.hsep = np.empty((0)) # horizontal seperation [m]
        
    
    def create(self, n=1, acspd=None, aclat=None, aclon=None, achdg=None, acid=None, achsep=None):
        """ Create multiple random aircraft in a specified area """
        
        self.ntraf += n

        # Aircraft Info
        self.id.append(acid)

        # Positions
        self.lat = np.append(self.lat, aclat)
        self.lon = np.append(self.lon, aclon)
        
        #self.hdg[-n:]  = achdg
        self.hdg = np.append(self.hdg, achdg)
        
        # Velocities
        self.gs = np.append(self.gs, acspd)
        
        # horizontal seperation
        self.hsep = np.append(self.hsep, achsep)
        
        hdgrad = np.radians(achdg)
        
        gsnorth = acspd* np.cos(hdgrad)
        gseast = acspd* np.sin(hdgrad)
        self.gsnorth = np.append(self.gsnorth, gsnorth)
        self.gseast = np.append(self.gseast, gseast)