#!/usr/bin/env python

"""/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

 Modified from mesonh.py
"""

from __future__ import absolute_import, print_function, division
from wind_importer_cfd import CFDImporter
import sys
import signal
import time
import os
import numpy as np
import json

# Add pprz_home path; this is necessary for pprzlink
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))
if "PAPARAZZI_HOME" not in os.environ:
    os.environ["PAPARAZZI_HOME"] = PPRZ_HOME
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


# scale = np.array([1., 1/M_IN_KM, 1/M_IN_KM, 1/M_IN_KM])

"""
DEFAULT is ENU
ANSYS: NWU (sacrificing y axis RIP)
LTP position(aircraft): NED
"""


class WindDataImporter:
    def __init__(self, args):
        self.ac_id = args.ac_id
        # self.origin = np.array([args.origin_x, args.origin_y, args.origin_z])
        self.origin_east = args.origin_x
        self.origin_north = args.origin_y
        self.origin_up = args.origin_z

        self.config_path = args.config_file
        self.csv_path = args.csv_dir
        self.cfd_importers = {}

        # self.cfd_importer = CFDImporter()
        # self.cfd_importer.init_importer(args.file, args.inlet_wind_speed_east,
        #                                 args.inlet_wind_speed_north,
        #                                 args.inlet_wind_speed_up)

        self.wind_default_y = args.default_wind_speed_north
        self.wind_default_x = args.inlet_wind_speed_east
        self.wind_default_z = args.inlet_wind_speed_up

        self.wind_prev_x, self.wind_prev_y, self.wind_prev_z = self.wind_default_x, self.wind_default_y, self.wind_default_z
        self.target_altitude = args.target_altitude
        self.set_waypoints = args.set_waypoints

        # self.active_ws = self.wind_default_y

    def init_importer(self):
        cfd_config = None
        # f = io.open(self.config_path, mode='r', encoding="utf-8")
        with open(self.config_path) as f:
            cfd_config = json.load(f)

        for ws in cfd_config["files"]:
            f = cfd_config["files"][ws]
            self.cfd_importers[ws] = CFDImporter()
            self.cfd_importers[ws].init_importer(os.path.join(self.csv_path, f), self.wind_default_x,
                                                -float(ws), self.wind_default_z)

        if self.set_waypoints:
            x_min, x_max = self.cfd_importers[str(-self.wind_default_y)].get_soaring_region(self.target_altitude)
            self.send_wp_msg(x_min, x_max, self.target_altitude, self.ac_id)
            print("CFD Importer init completed. eq_points: ", x_min, x_max)

    # a simple low pass filter
    def lpf_simple(self, wind_x, wind_y, wind_z, alpha=0.5):
        lpf_x = (1-alpha)*wind_x + alpha*self.wind_prev_x
        lpf_y = (1-alpha)*wind_y + alpha*self.wind_prev_y
        lpf_z = (1-alpha)*wind_z + alpha*self.wind_prev_z
        return lpf_x, lpf_y, lpf_z
        # return 0, 0, 0

    def update_estimation(self, wind_x, wind_y, wind_z):
        self.wind_prev_x = wind_x
        self.wind_prev_y = wind_y
        self.wind_prev_z = wind_z

    def initialize_filter(self):
        self.wind_prev_x = self.wind_default_x
        self.wind_prev_y = self.wind_default_y
        self.wind_prev_z = self.wind_default_z

    def get_wind(self, east, north, up, ws):
        loc = np.array([north-self.origin_north, -east+self.origin_east, up-self.origin_up])       # ANSYS North West Up
        weast, wnorth, wup = self.cfd_importers[str(ws)].get_wind(loc)
        # weast, wnorth, wup = np.random.rand()*3, np.random.rand()*3, np.random.rand()*2
        return weast, wnorth, wup

    def cfd_wind_cb(self, ac_id, msg):
        """
            Callback for paparazzi CFD_WIND requests
            the response should be *ENU*
        """
        north, east, down, ws = float(msg.get_field(1)), \
                            float(msg.get_field(2)), \
                            float(msg.get_field(3)), \
                            int(msg.get_field(4))
        # print(north, east, down, ws)

        _weast, _wnorth, _wup = self.get_wind(east, north, -down, ws)

        weast, wnorth, wup = self.wind_default_x, self.wind_default_y, self.wind_default_z

        if _weast == _wnorth == _wup == 0:
            self.initialize_filter()
        else:
            weast, wnorth, wup = self.lpf_simple(_weast, _wnorth, _wup)
            self.update_estimation(_weast, _wnorth, _wup)

        msg_back = PprzMessage("datalink", "CFD_WIND_DATA")
        msg_back.set_value_by_name("ac_id", msg.get_field(0))
        msg_back.set_value_by_name("wind_east", weast)
        msg_back.set_value_by_name("wind_north", wnorth)
        msg_back.set_value_by_name("wind_up", wup)
        # ivy.send_raw_datalink(msg_back)
        ivy.send(msg_back)

    def send_wp_msg(self, x_min, x_max, alt, ac_id, wp_east=0):
        # WP3: "soaring-1", closer to the dune
        msg_1 = PprzMessage("datalink", "MOVE_WAYPOINT_LTP")
        msg_1.set_value_by_name("ac_id", ac_id)
        msg_1.set_value_by_name("wp_id", 3)
        msg_1.set_value_by_name("ltp_x", x_min+self.origin_north)
        msg_1.set_value_by_name("ltp_y", wp_east+self.origin_east)
        msg_1.set_value_by_name("ltp_z", -(alt+self.origin_up))
        ivy.send(msg_1)

        # WP4: "soaring-middle", reference point between WP3 and WP5
        msg_2 = PprzMessage("datalink", "MOVE_WAYPOINT_LTP")
        msg_2.set_value_by_name("ac_id", ac_id)
        msg_2.set_value_by_name("wp_id", 4)
        msg_2.set_value_by_name("ltp_x", (x_min+x_max)/2.+self.origin_north)
        msg_2.set_value_by_name("ltp_y", wp_east+self.origin_east)
        msg_2.set_value_by_name("ltp_z", -(alt+self.origin_up))
        ivy.send(msg_2)

        # WP5: "soaring-2", further from the dune
        msg_3 = PprzMessage("datalink", "MOVE_WAYPOINT_LTP")
        msg_3.set_value_by_name("ac_id", ac_id)
        msg_3.set_value_by_name("wp_id", 5)
        msg_3.set_value_by_name("ltp_x", x_max+self.origin_north)
        msg_3.set_value_by_name("ltp_y", wp_east+self.origin_east)
        msg_3.set_value_by_name("ltp_z", -(alt+self.origin_up))
        ivy.send(msg_3)

    def export_wind_field(self):
        import csv

        step_size = 0.5
        _x_range, _z_range = 260, 100
        # _x_range, _z_range = 160, 100
        x_n_steps, z_n_steps = (int)(_x_range/step_size), (int)(_z_range/step_size)

        x_range = np.arange(0, _x_range, step_size)
        z_range = np.arange(0, _z_range, step_size)

        xz_x, xz_z, xz_mag = np.zeros((z_n_steps, x_n_steps)), np.zeros((z_n_steps, x_n_steps)), np.zeros((z_n_steps, x_n_steps))

        csv_f = open('/home/sunyou/cfd_wind_field.csv', 'w')
        field_names = ['x', 'z', 'v_x', 'v_z', 'v_mag']
        csv_writer = csv.DictWriter(csv_f, fieldnames=field_names)
        csv_writer.writeheader()
        # csv_writer.writerow({'x', 'z', 'v_x', 'v_z', 'v_mag'})

        u_f = open('/home/sunyou/cfd_x.csv', 'w')
        u_writer = csv.writer(u_f)
        v_f = open('/home/sunyou/cfd_z.csv', 'w')
        v_writer = csv.writer(v_f)
        # y_f = open('/home/sunyou/cfd_y.csv', 'w')
        # y_writer = csv.writer(y_f)
        mag_f = open('/home/sunyou/cfd_mag.csv', 'w')
        mag_writer = csv.writer(mag_f)

        for i in range(0, len(z_range)):
            for j in range(0, len(x_range)):
                _weast, _wnorth, _wup = self.get_wind(0, x_range[j]+self.origin_north, z_range[i], 12)

                # if _weast == _wnorth == _wup == 0:
                #     self.initialize_filter()
                #     csv_writer.writerow({'x':i, 'z':j, 'v_x':self.wind_default_x, 'v_z':'0', 'v_mag':'0'})
                #     continue

                if _weast == _wnorth == _wup == 0:
                    weast, wnorth, wup = 0, 0, 0
                else:
                    weast, wnorth, wup = self.lpf_simple(_weast, _wnorth, _wup)
                    self.update_estimation(_weast, _wnorth, _wup)

                xz_x[i][j] = wnorth
                xz_z[i][j] = wup
                xz_mag[i][j] = np.sqrt((wnorth*wnorth+wup*wup))
                # print(i, j, _wnorth, _wup)
                csv_writer.writerow({'x':x_range[j], 'z':z_range[i], 'v_x':-wnorth, 'v_z':wup, 'v_mag':np.sqrt((wnorth*wnorth+wup*wup))})
            # print(i)
            u_writer.writerow(xz_x[i])
            v_writer.writerow(xz_z[i])
            mag_writer.writerow(xz_mag[i])

        u_f.close()
        v_f.close()
        mag_f.close()
        csv_f.close()


def signal_handler(signal, frame):
    print('\nShutting down IVY...')
    ivy.shutdown()
    print("Done.")


def main():
    # parse arguments
    import argparse as ap

    argp = ap.ArgumentParser(description="Wind data provider "
                                         "for Paparazzi from CFD or potential flow simulation")

    argp.add_argument("-config", "--config-file", required=False,
                      default=PPRZ_HOME+"/../nld_cfd_results/config.json",
                      help="CFD config file, relative from pprz home")

    argp.add_argument("-w", "--default-wind-speed-north", required=False,
                      default=-12,
                      help="Default wind speed (north)")

    # argp.add_argument("-f", "--file", required=False,
    #                   default=PPRZ_HOME+"/../nld_cfd_results/cfd_csv/export_hill_r_40_v_12m.csv",
    #                   help="CFD result file path, relative from pprz home")

    argp.add_argument("-dir", "--csv-dir", required=False,
                      default=PPRZ_HOME+'/../nld_cfd_results/cfd_csv',
                      help="Path to a *DIR* containing CFD results, relative path from the pprz home.")

    # argp.add_argument("-t", "--time-step", required=False, type=int,
    #                   help="Time step for importing dynamic/time-variant CFD simulation. "
    #                        "Not required for steady-states.")

    argp.add_argument("-x", "--origin-x", required=False, type=float,
                      default=0.,
                      help="Origin position x (EAST).")
    argp.add_argument("-y", "--origin-y", required=False, type=float,
                      default=230,
                      help="Origin position y (NORTH).")
    argp.add_argument("-z", "--origin-z", required=False, type=float,
                      default=0.,
                      help="Origin position z (UP).")

    # default wind speed in ENU
    argp.add_argument("-i", "--inlet-wind-speed-east", required=False, type=float,
                      default=0,
                      help="Inlet wind speed (EAST), default value is 0")
    # argp.add_argument("-j", "--inlet-wind-speed-north", required=False, type=float,
    #                   default=-12,
    #                   help="Inlet wind speed (NORTH), default value is (0, 10, 0) in ENU")
    argp.add_argument("-k", "--inlet-wind-speed-up", required=False, type=float,
                      default=0.,
                      help="Inlet wind speed (UP), default value is 0")

    argp.add_argument("-e", "--export", required=False, type=bool,
                      default=False, help="Flag for exporting wind field")

    argp.add_argument("-wp", "--set-waypoints", required=False, type=bool,
                      default=True, help="Set soaring waypoints")

    argp.add_argument("-alt", "--target-altitude", required=False, type=float,
                      default=60, help="Soaring WP altitude")

    argp.add_argument("-ac", "--ac_id", required=False, type=int,
                      default=8, help="AC_ID")

    args = argp.parse_args()

    # register signal handler for ctrl+c to stop the program
    signal.signal(signal.SIGINT, signal_handler)

    global ivy
    ivy = IvyMessagesInterface("WindSimulationData")

    importer = WindDataImporter(args)
    importer.init_importer()

    if args.export:
        importer.export_wind_field()
        return

    ivy.subscribe(importer.cfd_wind_cb, '(.* LTP_POSITION .*)')

    # wait for ivy to stop
    from ivy.std_api import IvyMainLoop  # noqa

    signal.pause()


if __name__ == '__main__':
    main()
