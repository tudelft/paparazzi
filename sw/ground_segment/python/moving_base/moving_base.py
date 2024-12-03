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
import sys
from os import path, getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class MovingBase(object):
    def __init__(self, msg):
        self.tow = int(msg['tow'])
        self.lat = int(msg['lat'])
        self.lon = int(msg['lon'])
        self.alt = int(msg['alt'])
        self.vnorth = float(msg['vnorth'])
        self.veast = float(msg['veast'])
        self.vdown = float(msg['vdown'])
        self.body_qi = float(msg['body_qi'])
        self.body_qx = float(msg['body_qx'])
        self.body_qy = float(msg['body_qy'])
        self.body_qz = float(msg['body_qz'])
        self.body_p = float(msg['body_p'])
        self.body_q = float(msg['body_q'])
        self.body_r = float(msg['body_r'])


class Base:
    def __init__(self, AC_ID_moving_base: int, AC_ID: list):
        # Start IVY interface
        self.moving_base_ID = AC_ID_moving_base
        self.uavs = AC_ID
        self._interface = IvyMessagesInterface("Moving Base")
        self._interface.subscribe(self.message_recv)

    # Receive a TARGET_POS_INFO message from the moving base
    def message_recv(self, ac_id, msg):
        if msg.name == "TARGET_POS_INFO" and ac_id == self.moving_base_ID:
            self.moving_base = MovingBase(msg)

            for ac in self.uavs:
                self.message_send(self.moving_base, ac)

    # Send a TARGET_POS message to the UAVs
    def message_send(self, moving_base, AC_ID):
        msg = PprzMessage("datalink", "TARGET_POS")
        msg['ac_id'] = AC_ID
        msg['target_id'] = self.moving_base_ID
        msg['tow'] = moving_base.tow
        msg['lat'] = moving_base.lat
        msg['lon'] = moving_base.lon
        msg['alt'] = moving_base.alt
        msg['vnorth'] = moving_base.vnorth
        msg['veast'] = moving_base.veast
        msg['vdown'] = moving_base.vdown
        msg['body_qi'] = moving_base.body_qi
        msg['body_qx'] = moving_base.body_qx
        msg['body_qy'] = moving_base.body_qy
        msg['body_qz'] = moving_base.body_qz
        msg['body_p'] = moving_base.body_p
        msg['body_q'] = moving_base.body_q
        msg['body_r'] = moving_base.body_r
        
        self._interface.send(msg)

    def OnClose(self, event):
        self._interface.shutdown()
        self.Destroy()

if __name__ == '__main__':
    main()
