#!/usr/bin/env python
#
# Copyright (C) 2018 Freek van Tienen <freek.v.tienen@gmail.com>
#               2018 Dennis Wijngaarden
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#
"""
Mission communication through datalinks
"""

from os import path, getenv
from sets import Set
import sys
import logging
import threading
import time

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

class MissionComm(threading.Thread):
    THREAD_SLEEP = 1 # Time to sleep in between update checks in seconds

    def __init__(self, ac_id = None):
        """
        Initialize the Mission Communicator
        """
        threading.Thread.__init__(self)

        # Initialize the variables
        self.name = "MissionComm" 
        self.deamon = True
        self.ivy_interface = IvyMessagesInterface("Mission Communicator")
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.remote_lock = threading.Lock()
        self.ac_id = ac_id
        self.local_mission = []
        self.remote_idx = -1
        self.remote_mission = Set([])

        # Subscribe to ivy messages
        self.ivy_interface.subscribe(self.mission_path_status_cb, PprzMessage("telemetry", "MISSION_PATH_STATUS"))
        self.ivy_interface.subscribe(self.mission_path_add_ack_cb, PprzMessage("telemetry", "MISSION_PATH_ADD_ACK"))
        self.ivy_interface.subscribe(self.mission_path_delete_ack_cb, PprzMessage("telemetry", "MISSION_PATH_DELETE_ACK"))

    def mission_path_status_cb(self, ac_id, msg):
        """
        When a MISSION_PATH_STATUS message is received from an AC
        """
        # Check if it is for our aircraft
        if self.ac_id != None and self.ac_id != ac_id:
            return
        
        try:
            self.remote_lock.acquire()

            # Update the remote list
            self.remote_idx = msg['cur_idx']
            self.remote_mission = Set(msg['ids'])
        finally:
            self.remote_lock.release()

    def mission_path_add_ack_cb(self, ac_id, msg):
        """
        When a MISSION_PATH_ADD_ACK message is received from an ac
        """
        # Check if it is for our aircraft
        if self.ac_id != None and self.ac_id != ac_id:
            return
        
        try:
            self.remote_lock.acquire()

            # Update the remote list
            self.remote_idx = msg['cur_idx']
            self.remote_mission.add(msg['id'])
        finally:
            self.remote_lock.release()

    def mission_path_delete_ack_cb(self, ac_id, msg):
        """
        When a MISSION_PATH_DELETE_ACK message is received from an ac
        """
        # Check if it is for our aircraft
        if self.ac_id != None and self.ac_id != ac_id:
            return
        
        try:
            self.remote_lock.acquire()

            # Update the remote list
            self.remote_idx = msg['cur_idx']
            self.remote_mission.discard(msg['id'])
        finally:
            self.remote_lock.release()

    def run(self):
        """
        Communicate the mission to the aircraft
        """
        logging.info("Starting " + self.name)

        # Main loop which tries to send mission messages until mission is succesfully received
        while not self.stop_event.is_set():
            self.update_remote_mission()
            time.sleep(MissionComm.THREAD_SLEEP)

        logging.info("Stopping " + self.name)

    def update_remote_mission(self):
        """
        Try to update the remote mission
        """ 
        try:
            self.lock.acquire()
            self.remote_lock.acquire()

            # Send an add message for all new elements and register ids
            local_ids = Set([])
            for elem in self.local_mission:
                local_ids.add(elem.id)

                # Send a new remote mission
                if elem.id not in self.remote_mission:
                    self.send_mission_path_add(elem)

            # Delete all remote existing ids which are not local
            del_ids = self.remote_mission - local_ids
            for id in del_ids:
                self.send_mission_path_delete(id)

        finally:
            self.remote_lock.release()
            self.lock.release()

    def send_mission_path_add(self, elem):
        """
        Send a new mission path element
        """
        assert self.ac_id != None
        logging.debug("Send path add "+elem.id)

        # Convert the waypoint to integer
        wp_i = elem.wp.to_int()

        # Create the message
        msg = PprzMessage("datalink", "MISSION_PATH_ADD")
        msg['ac_id'] = self.ac_id
        msg['id'] = elem.id
        msg['wp_east'] = wp_i.x
        msg['wp_north'] = wp_i.y
        msg['wp_up'] = wp_i.z
        self.ivy_interface.send(msg)

    def send_mission_path_delete(self, id):
        """
        Send a delete mission path by id
        """
        assert self.ac_id != None
        logging.debug("Send path delete "+id)

        # Create the message
        msg = PprzMessage("datalink", "MISSION_PATH_DELETE")
        msg['ac_id'] = self.ac_id
        msg['id'] = id
        self.ivy_interface.send(msg)

    def set_mission(self, new_mission):
        """
        Update the full mission
        """
        try:
            self.lock.acquire()
            self.local_mission = new_mission
        finally:
            self.lock.release()


if __name__ == '__main__':
    mission_comm = MissionComm(29)
    mission_comm.start()




    