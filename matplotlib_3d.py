#!/usr/bin/env python

# Import paparazzi stuff
from __future__ import print_function
from hashlib import new
import sys
from turtle import distance
import numpy as np
import json
from time import sleep
from os import path, getenv
import time
import pymap3d as pm
import math as m
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
#PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '~/paparazzi'))) #TEST
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from scipy import linalg as la

# Import matplot stuff
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from spatialmath.base import *

import array


### PAPARAZZI setup
class UAV:
    def __init__(self, ac_id=203):
        self.initialized = False
        self.id = ac_id
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.timeout = 0

ship_lon = 0 
ship_lat = 0
ship_alt = 0
ship_course = 0
ship_heading = 0
off_heading = 0
off_distance = 0
off_heigth = 0

class AMT_msg:
    def __init__(self, ):
        self.rel_des_pos_x = 0
        self.rel_des_pos_y = 0
        self.rel_des_pos_z = 0
        self.rel_target_pos_x = 0
        self.rel_target_pos_y = 0
        self.rel_target_pos_z = 0
        self.start_distance = 0
        self.distance = 0
        self.slope_ref = 0

        self.drone_track_x = array.array('f', [])
        self.drone_track_y = array.array('f', [])
        self.drone_track_z = array.array('f', [])

        self.x_lim = 0
        self.y_lim = 0
        self.z_lim = 0

        self.new_msg = False
        self.initialised = False


        #IVY_interface = IvyMessagesInterface("Moving Base")
        IVY_interface = IvyMessagesInterface("Pygame visual")

        # bind to APPROACH_MOVING_TARGET msgs
        def approach_moving_target_cb(ac_id, msg):
            if msg.name == "APPROACH_MOVING_TARGET":
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                self.rel_des_pos_x = float(msg['ref_pos_x'])# / 1e7
                self.rel_des_pos_y = float(msg['ref_pos_y'])# / 1e7
                self.rel_des_pos_z = float(msg['ref_pos_z'])# / 100

                self.rel_target_pos_x = float(msg['target_pos_x'])# / 1e7
                self.rel_target_pos_y = float(msg['target_pos_y'])# / 1e7
                self.rel_target_pos_z = float(msg['target_pos_z'])# / 100

                self.start_distance = float(msg['start_distance'])
                self.distance = float(msg['distance'])
                self.slope_ref = float(msg['slope_ref'])


                print("APPROACH_MOVING_TARGET msg received \n") #TEST
                #print(self.rel_des_pos_x)
                #print(self.rel_des_pos_y)
                #print(self.rel_des_pos_z)
                self.new_msg = True

        IVY_interface.subscribe(approach_moving_target_cb, PprzMessage("telemetry", "APPROACH_MOVING_TARGET"))

        # bind to TARGET_POS msgs
        def target_pos_info_cb(ac_id, msg):
            if msg.name == "TARGET_POS_INFO":
                global ship_lon
                global ship_lat
                global ship_alt
                global ship_course
                global ship_heading
                global off_heading
                global off_distance
                global off_heigth

                #i2p = 1. / 2**8     # integer to position
                #i2v = 1. / 2**19    # integer to velocity
                ship_lon = float(msg['lat']) / 1e7
                ship_lat = float(msg['lon']) / 1e7
                ship_alt = float(msg['alt']) / 100
                ship_course = float(msg['course'])
                ship_heading = float(msg['heading'])

                print("TARGET_POS_INFO msg received, heading: ", ship_heading) #TEST
                self.new_msg = True

        IVY_interface.subscribe(target_pos_info_cb, PprzMessage("telemetry", "TARGET_POS_INFO"))
        ### END PAPARAZZI setup

        plt.interactive(True)
        self.fig = plt.figure(tight_layout=True)
        self.ax = self.fig.add_subplot(111, projection='3d')

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def draw_3d_plot(self):

        zoom_max2drone_and_ship = True
        zoom_getFromPrevFrame = False

        #self.ax.cla()
        self.ax.clear()

        # keeps the screen at the last zoomed position 
        # instead of resetting the zoom every loop
        if self.x_lim != 0 and zoom_getFromPrevFrame and not zoom_max2drone_and_ship:
            self.ax.set_xlim(self.x_lim)
            self.ax.set_ylim(self.y_lim)
            self.ax.set_zlim(self.z_lim)

        #bc = boat-centered
        bc_drone_pos_x = -self.rel_target_pos_x
        bc_drone_pos_y = -self.rel_target_pos_y
        bc_drone_pos_z = -self.rel_target_pos_z # from North,East,Down to North,East,UP

        self.drone_track_x.append(bc_drone_pos_x)
        self.drone_track_y.append(bc_drone_pos_y)
        self.drone_track_z.append(-bc_drone_pos_z)

        bc_ref_pos_x = -self.rel_target_pos_x + self.rel_des_pos_x
        bc_ref_pos_y = -self.rel_target_pos_y + self.rel_des_pos_y
        bc_ref_pos_z = -self.rel_target_pos_z + self.rel_des_pos_z # from North,East,Down to North,East,UP

        bc_descent_line_x_vec = m.cos(m.radians(self.slope_ref)) * m.cos(m.radians(ship_heading+180))
        bc_descent_line_y_vec = m.cos(m.radians(self.slope_ref)) * m.sin(m.radians(ship_heading+180))
        bc_descent_line_z_vec = m.sin(m.radians(self.slope_ref))

        if not zoom_max2drone_and_ship:
            bc_descent_line_x = bc_descent_line_x_vec *self.start_distance
            bc_descent_line_y = bc_descent_line_y_vec *self.start_distance
            bc_descent_line_z = bc_descent_line_z_vec *self.start_distance
        else:
            bc_descent_line_x = bc_descent_line_x_vec *self.distance
            bc_descent_line_y = bc_descent_line_y_vec *self.distance
            bc_descent_line_z = bc_descent_line_z_vec *self.distance

        # if correct same as bc_ref_pos
        #bc_descent_point_x = bc_descent_line_x_vec * self.distance
        #bc_descent_point_y = bc_descent_line_y_vec * self.distance
        #bc_descent_point_z = bc_descent_line_z_vec * self.distance

        #self.ax.set_xlim3d(min(bc_ref_pos_x, bc_drone_pos_x), max(bc_ref_pos_x, bc_drone_pos_x))
        #self.ax.set_zlim3d(min(bc_ref_pos_y, bc_drone_pos_y), max(bc_ref_pos_y, bc_drone_pos_y))
        #self.ax.set_ylim3d(min(bc_ref_pos_z, bc_drone_pos_z), max(bc_ref_pos_z, bc_drone_pos_z))

        sys_boat = trplot( transl(0,0, 0)@trotz(ship_heading/360*2*m.pi), wtl=0.05, d2=1.5, frame='SHIP', rviz=True, width=1, originsize=3, color=["blue", "grey", "grey"]) # , dims=[0, 10, 0, 10, 0, 10]
        sys_ref = trplot( transl(bc_ref_pos_x, bc_ref_pos_y, -bc_ref_pos_z)@trotz(ship_heading/360*2*m.pi), frame='des', d2=1.5, axislabel= False, width=1, originsize=3) # , dims=[0, 10, 0, 10, 0, 10]
        sys_drone = trplot( transl(bc_drone_pos_x, bc_drone_pos_y, -bc_drone_pos_z), axislabel= False, color='red', d2=1.5, width=1, originsize=3) #, frame='DRONE'
        line_drone2ship = self.ax.plot([0, bc_descent_line_x], [0,bc_descent_line_y],zs=[0,bc_descent_line_z])
        if not zoom_max2drone_and_ship:
            drone_track = self.ax.scatter(self.drone_track_x, self.drone_track_y, self.drone_track_z, c="red", s=0.1)


        #sys_boat = trplot( transl(self.rel_target_pos_x,self.rel_target_pos_y,-self.rel_target_pos_z)@trotz(ship_heading/360*2*m.pi), frame='SHIP', rviz=True, width=1) # , dims=[0, 10, 0, 10, 0, 10]
        #sys_ref = trplot( transl(self.rel_des_pos_x,self.rel_des_pos_y,-self.rel_des_pos_z)@trotz(ship_heading/360*2*m.pi), frame='des', rviz=True, width=1) # , dims=[0, 10, 0, 10, 0, 10]
        #sys_drone = trplot( transl(0,0, 0), color='red', width=3) #, frame='DRONE'
        #line_drone2ship = self.ax.plot([0, self.rel_target_pos_x], [0,self.rel_target_pos_y],zs=[0,-self.rel_target_pos_z])
        self.new_msg = False
        print("plot updated")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.x_lim = self.ax.get_xlim()
        self.y_lim = self.ax.get_ylim()
        self.z_lim = self.ax.get_zlim()
        
        print(min(-bc_ref_pos_z, -bc_drone_pos_z))

        #if zoom_max2drone_and_ship:
        #    self.x_lim = (int(min(bc_ref_pos_x, bc_drone_pos_x)), int(max(bc_ref_pos_x, bc_drone_pos_x)))
        #    self.y_lim = (int(min(bc_ref_pos_y, bc_drone_pos_y)), int(max(bc_ref_pos_y, bc_drone_pos_y)))
        #    self.z_lim = (int(min(-bc_ref_pos_z, -bc_drone_pos_z)), int(max(-bc_ref_pos_z, -bc_drone_pos_z)))

        self.initialised = True

    def run(self):
        try:
            # The main loop
            print("\n 3D plot drone & ship - loop started!\n")
            while True:
                if self.new_msg:
                    self.draw_3d_plot()

        except KeyboardInterrupt:
            self.stop()

if __name__ == '__main__':
    import argparse

    amt_msg = AMT_msg()
    amt_msg.run()


'''

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
#PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '~/paparazzi'))) #TEST
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from scipy import linalg as la

class UAV:
    def __init__(self, ac_id=203):
        self.initialized = False
        self.id = ac_id
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.timeout = 0

ref_x = 0
ref_y = 0
ref_z = 0

#IVY_interface = IvyMessagesInterface("Moving Base")
IVY_interface = IvyMessagesInterface("Pygame visual")

# bind to APPROACH_MOVING_TARGET msgs
def approach_moving_target_cb(ac_id, msg):
    if msg.name == "APPROACH_MOVING_TARGET":
        global ref_x
        global ref_y
        global ref_z
        i2p = 1. / 2**8     # integer to position
        i2v = 1. / 2**19    # integer to velocity
        ref_x = float(msg['ref_pos_x'])# / 1e7
        ref_y = float(msg['ref_pos_y'])# / 1e7
        ref_z = float(msg['ref_pos_z'])# / 100
        print("APPROACH_MOVING_TARGET msg received \n") #TEST
        print(ref_x)
        print(ref_y)

IVY_interface.subscribe(approach_moving_target_cb, PprzMessage("telemetry", "APPROACH_MOVING_TARGET"))

# bind to TARGET_POS msgs
def target_pos_info_cb(ac_id, msg):
    if msg.name == "TARGET_POS_INFO":
        global ship_lon
        global ship_lat
        global ship_alt
        global ship_course
        global ship_heading
        global off_heading
        global off_distance
        global off_heigth

        #i2p = 1. / 2**8     # integer to position
        #i2v = 1. / 2**19    # integer to velocity
        ship_lon = float(msg['lat']) / 1e7
        ship_lat = float(msg['lon']) / 1e7
        ship_alt = float(msg['alt']) / 100
        ship_course = float(msg['course'])
        ship_heading = float(msg['heading'])

        print("TARGET_POS_INFO msg received, lon: %f\n", ship_lon) #TEST

IVY_interface.subscribe(target_pos_info_cb, PprzMessage("telemetry", "TARGET_POS_INFO"))
 
 
 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0)

a = Arrow3D([0, 1], [0, 0], [0, 0], **arrow_prop_dict, color='r')
ax.add_artist(a)
a = Arrow3D([0, 0], [0, 1], [0, 0], **arrow_prop_dict, color='b')
ax.add_artist(a)
a = Arrow3D([0, 0], [0, 0], [0, 1], **arrow_prop_dict, color='g')
ax.add_artist(a)

ax.text(0.0, 0.0, -0.1, r'$o$')
ax.text(1.1, 0, 0, r'$x$')
ax.text(0, 1.1, 0, r'$y$')
ax.text(0, 0, 1.1, r'$z$')

'''
