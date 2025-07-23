#!/usr/bin/env python3
#
# Copyright (C) 2017 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

'''
Moving base simulator
'''

# too many things here
from __future__ import print_function
import json
import time
import math as m
import sys
from os import path, getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class MovingBase(object):
    def __init__(self, msg_in):
        self.msg = PprzMessage("datalink", "TARGET_POS")
        self.msg['tow'] = int(msg_in['tow'])
        self.msg['lat'] = int(msg_in['lat'])
        self.msg['lon'] = int(msg_in['lon'])
        self.msg['alt'] = int(msg_in['alt'])
        self.msg['vnorth'] = float(msg_in['vnorth'])
        self.msg['veast'] = float(msg_in['veast'])
        self.msg['vdown'] = float(msg_in['vdown'])
        self.msg['body_qi'] = float(msg_in['body_qi'])
        self.msg['body_qx'] = float(msg_in['body_qx'])
        self.msg['body_qy'] = float(msg_in['body_qy'])
        self.msg['body_qz'] = float(msg_in['body_qz'])
        self.msg['p'] = float(msg_in['p'])
        self.msg['q'] = float(msg_in['q'])
        self.msg['r'] = float(msg_in['r'])

    def insert_ids(self, target_id, ac_id):
        self.msg['target_id'] = target_id
        self.msg['ac_id'] = ac_id


class Base:
    def __init__(self, moving_base_id: int, ac_ids: list):
        # Start IVY interface
        self.moving_base_id = moving_base_id
        self.uavs = ac_ids
        self._interface = IvyMessagesInterface("Moving Base")
        self._interface.subscribe(self.message_recv)
        self.msg = PprzMessage("datalink", "TARGET_POS")

    # Receive a TARGET_POS_INFO message from the moving base
    def message_recv(self, ac_id, msg):
        if msg.name == "TARGET_POS_INFO" and int(ac_id) == self.moving_base_id:
            for ac in self.uavs:
                self.message_send(msg, ac)

    # Send a TARGET_POS message to the UAVs
    def message_send(self, msg_in, ac_id):
        self.msg['ac_id'] = ac_id
        self.msg['target_id'] = self.moving_base_id
        self.msg['tow'] = int(msg_in['tow'])
        self.msg['lat'] = int(msg_in['lat'])
        self.msg['lon'] = int(msg_in['lon'])
        self.msg['alt'] = int(msg_in['alt'])
        self.msg['vnorth'] = float(msg_in['vnorth'])
        self.msg['veast'] = float(msg_in['veast'])
        self.msg['vdown'] = float(msg_in['vdown'])
        self.msg['body_qi'] = float(msg_in['body_qi'])
        self.msg['body_qx'] = float(msg_in['body_qx'])
        self.msg['body_qy'] = float(msg_in['body_qy'])
        self.msg['body_qz'] = float(msg_in['body_qz'])
        self.msg['p'] = float(msg_in['p'])
        self.msg['q'] = float(msg_in['q'])
        self.msg['r'] = float(msg_in['r'])
        self._interface.send(self.msg)

    def OnClose(self, event):
        self._interface.shutdown()
        self.Destroy()

    def run(self):
        try:
            # The main loop
            while True:
                time.sleep(10)
                pass

        except KeyboardInterrupt:
            self.Close()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Moving base HITL")
    parser.add_argument('-base', '--base_id', dest='moving_base_id', type=int, help="moving base id to receive messages from", required=True)
    parser.add_argument('-ac', '--ac_ids', dest='ac_ids', nargs="+", type=int, help="list of aircraft ids to forward message to", required=True)
    args = parser.parse_args()

    base = Base(moving_base_id=args.moving_base_id, ac_ids=args.ac_ids)
    base.run()
