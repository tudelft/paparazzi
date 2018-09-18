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

import threading
from os import path, getenv
from sets import Set
import logging

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
path.append(PPRZ_SRC + "/sw/lib/python")
path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

class MissionComm(threading.Thread):

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
        self.ac_id = ac_id
        self.local_mission = []
        self.remote_idx = -1
        self.remote_mission = Set([])

        # Subscribe to ivy messages
        self.ivy_interface.subscribe(self.mission_path_status_cb, PprzMessage("telemetry", "MISSION_PATH_STATUS"))

    def mission_path_status_cb(self, ac_id, msg):
        """
        When a MISSION_PATH_STATUS message is received from an AC
        """
        # Check if it is for our aircraft
        if self.ac_id != None and self.ac_id != ac_id:
            return
        
        # Update the remote list
        self.remote_idx = msg['cur_idx']
        self.remote_mission = Set(msg['ids'])

    def run(self):
        """
        Communicate the mission to the aircraft
        """
        logging.info("Starting " + self.name)

        # Main loop which tries to send mission messages until mission is succesfully received
        while not self.stop_event.is_set():
            pass

        logging.info("Stopping " + self.name)

    def update_remote_mission(self):
        """
        Try to update the remote mission
        """
        for elem in self.local_mission:
            if elem.id not in 

    def set_mission(self, new_mission):
        """
        Update the full mission
        """
        try:
            self.lock.acquire()
            self.local_mission = new_mission
        finally:
            self.lock.release()




    