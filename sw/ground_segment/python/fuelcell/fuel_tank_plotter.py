#!/usr/bin/env python3
#
# Copyright (C) 2012 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#


import numpy as np
import matplotlib.pyplot as plt

import sys
import os
from datetime import datetime
import time

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

class PayloadMessage(object):
    def __init__(self, msg):
        self.values = ''.join(chr(int(x)) for x in msg['values'])

class TankPlotter(object):
    def __init__(self):
        self.interface = IvyMessagesInterface("fueltankplotter")
        self.interface.subscribe(self.message_recv)
        self.start_time = time.mktime(datetime.now().timetuple())
        self.timestamps = []
        self.pressures = []

    def message_recv(self, ac_id, msg):
        if msg.name == "PAYLOAD":
            self.payload = PayloadMessage(msg)
            self.update(self.payload.values)

    def update(self,msg):
        self.msg = msg
        self.msgtime = datetime.now()
        elements = self.msg.strip('<').strip('>').split(',')
        if (len(elements) == 4):
            self.tank = float(elements[0])
            self.bar = round(5 + self.tank / 100 * 295,1)
            self.battery = float(elements[1])
            self.status = elements[2]
            self.error = elements[3]

            self.errors = []
            hex = '0000'
            if (len(self.error) >= 4):
                hex = self.error[2:6]
            #array of 16 error codes
            self.error_bin = bin(int(hex, 16))[2:].zfill(16)

            self.update_plot()

    def update_plot(self):
        if len(self.pressures) == 0:
            self.start_time = time.mktime(datetime.now().timetuple())
            self.timestamps.append(time.mktime(datetime.now().timetuple()) - self.start_time)
            self.pressures.append(self.bar)
            #self.generate_plot(
            return

        if self.bar != self.pressures[-1]:
            self.timestamps.append(time.mktime(datetime.now().timetuple()) - self.start_time)
            self.pressures.append(self.bar)
            #self.generate_plot()

    def generate_plot(self):
        self.line.set_xdata(np.array(self.timestamps))
        self.line.set_ydata(np.array(self.pressures))
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

if __name__ == '__main__':
    plotter = TankPlotter()
    plt.ion()
    figure = plt.figure()
    ax = figure.add_subplot(111)
    line, = ax.plot([0], [0])
    ax.set_xlim((0, 60))
    ax.set_ylim((0, 300))
    ax.grid()
    figure.canvas.draw()
    figure.canvas.flush_events()

    while True:
        if len(plotter.pressures) > 0:
            line.set_xdata(plotter.timestamps)
            line.set_ydata(plotter.pressures)
            ax.set_xlim((0, plotter.timestamps[-1] + 5))
            ax.set_ylim((0, 300))
            print(plotter.pressures)
            print(plotter.timestamps)
        plt.draw()
        
        figure.canvas.draw()
        figure.canvas.flush_events()
        sys.stdout.flush()
        time.sleep(1)
