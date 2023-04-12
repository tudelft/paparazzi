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
"""

import numpy as np
from scipy.spatial import KDTree
import time
# from scipy import interpolate


class CFDImporter:
    def __init__(self):
        self.translate_origin = (0, 0, 0)
        self.wind_data = None
        self.count = 0
        self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max = 0, 0, 0, 0, 0, 0
        self.inlet_wind_speed_enu = (0, 0, 0)
        self.inlet_wind_speed_sum = 0
        self.kd_tree = None
        self.count = 0

    def init_importer(self, f_name, w_e, w_n, w_u):
        print("init importer")
        self.read_csv(f_name)
        self.x_min = np.min(self.wind_data[:, 0])
        self.x_max = np.max(self.wind_data[:, 0])
        self.y_min = np.min(self.wind_data[:, 1])
        self.y_max = np.max(self.wind_data[:, 1])
        self.z_min = np.min(self.wind_data[:, 2])
        self.z_max = np.max(self.wind_data[:, 2])
        self.inlet_wind_speed_enu = (w_e, w_n, w_u)
        self.inlet_wind_speed_sum = abs(w_e)+abs(w_n)+abs(w_u)
        print("x: ", self.x_min, self.x_max, ", y: ", self.y_min, self.y_max, ", z: ", self.z_min, self.z_max)

    def translate_field(self, x=0, y=0, z=0):
        self.translate_origin += (x, y, z)

    def read_csv(self, f_name):
        csv_data = np.genfromtxt(fname=f_name, delimiter=",", skip_header=6)
        _idx = np.where(csv_data[:, 3] > 0)
        _wind_data = csv_data[_idx]
        self.wind_data = np.concatenate((_wind_data[:, :3], _wind_data[:, 3:7]), axis=1)
        # self.wind_data = _wind_data
        self.kd_tree = KDTree(_wind_data[:, :3])
        # print(f_name, self.wind_data)

    def get_wind(self, loc):
        # ANSYS North West Up
        # print(self.inlet_wind_speed_enu, loc)
        dist, idx = self.kd_tree.query(loc, distance_upper_bound=3)

        wind_east, wind_north, wind_up = 0, 0, 0

        if dist is not np.inf:
            wind_east = -self.wind_data[idx, 5]
            wind_north = self.wind_data[idx, 4]
            wind_up = self.wind_data[idx, 6]

        """
        OpenFoam x:North y:West z:Up
        ANSYS: check frame setting
        return *ENU*
        """

        return wind_east, wind_north, wind_up

    def get_soaring_region(self, target_alt, p_threshold=10):
        m = 0.907  # mass, in kg

        S = 0.4234  # Wing surface area, in m^2
        rho = 1.225  # Air density, in kg/m^3
        g = 9.80665  # gtavitational acceleration, in m/s^2
        CL_alpha = 5  # Lift coefficient slope, in 1/rad
        alpha_0L = 0.05  # Zero lift angle of attack, in degrees
        AR = 8.8847  # Aspect ratio
        e = 0.8  # Oswald efficiency factor
        CD_0 = 0.028  # Zero-lift drag coefficient

        # Specific turbine disc area: S_turb_spec = S_turb_disc / S
        S_turb_spec = 0.36  # 0.006 for smallest point at gridsize of 0.1
        W = m * g
        Cdi_coef = 1. / (np.pi * AR * e)  # 1/pi*A*e

        # TODO: get rid of hardcoded params

        alt_low, alt_high = target_alt-3, target_alt+3

        x_points = self.wind_data[:, 0]
        z_points = self.wind_data[:, 2]

        cfd_wind_north = self.wind_data[:, 4]
        cfd_wind_up = self.wind_data[:, 6]
        cfd_wind_mag = self.wind_data[:, 3]

        def get_local_min_h_dot(V_loc):
            min_h_dot = (rho * S * CD_0) / (2. * m * g) * V_loc ** 3 + (2. * m * g * Cdi_coef) / (rho * S * V_loc)
            return min_h_dot

        def calc_eq(wind_mag, wind_u, wind_v):
            # This function calculates the
            C_L_req = W / (0.5 * rho * np.power(wind_mag, 2) * S) * (np.abs(wind_u) / wind_mag)
            C_D_req = W / (0.5 * rho * np.power(wind_mag, 2) * S) * (np.abs(wind_v) / wind_mag)

            # drag_turb_func = get_turbine_params()

            C_D_min_ach = CD_0 + np.power(C_L_req, 2) * Cdi_coef
            C_D_max_ach = C_D_min_ach + 1. * (2. / 9) * S_turb_spec
            # drag_turb_func(wind_mag)/(0.5* rho * np.power(wind_mag, 2) * S)#1*(2/9)*S_turb_spec

            C_D = CD_0 + (C_L_req*C_L_req)*Cdi_coef
            C_D_turb = C_D_req - C_D_min_ach

            # D_turb = 0.5 * rho * np.power(wind_mag, 2) * S * C_D_turb

            alpha = C_L_req / CL_alpha + alpha_0L

            stall = (alpha > np.deg2rad(15)) & (alpha < np.deg2rad(-10))
            C_L_req[stall] = np.nan
            C_D_req[stall] = np.nan
            # C_D_min_ach[stall] = np.nan
            # C_D_max_ach[stall] = np.nan
            # alpha[stall] = np.nan

            # eq_points = np.where((C_D_req <= C_D_min_ach) | (C_D_req >= C_D_max_ach))
            eq_points = (C_D>C_D_req-0.001) & (C_D<C_D_req+0.001)
            P_turb = 0.5 * rho * S * np.power(wind_mag, 3) * C_D_turb
            P_turb[eq_points == False] = np.nan
            alpha[eq_points == False] = np.nan
            # D_turb[eq_points] = np.nan

            # v_mask = np.where(wind_v < 0)
            # P_turb[v_mask] = np.nan
            # P_turb[v_mask] = -1

            # P_turbs = np.ma.masked_where(wind_v <= get_local_min_h_dot(wind_mag), P_turb)  # changed to P_maxs
            p_idx = np.where((wind_v < 0) | (wind_v <= get_local_min_h_dot(wind_mag)))
            P_turb[p_idx] = -1
            # P_turb_max = np.nanmax(P_turbs)
            # P_turbs = np.nan_to_num(P_turbs, nan=-1)
            P_turbs = [-1 if np.isnan(x) else x for x in P_turb]

            # return P_turb, np.rad2deg(alpha), D_turb
            return np.array(P_turbs)

        # P_turbs, alphas_eq, D_turb = calc_eq()
        # P_turbs = np.ma.masked_where(wind_v <= get_local_min_h_dot(wind_mag), P_turbs)  # changed to P_maxs
        # P_turb_max = np.nanmax(P_turbs)
        # min_p = np.nanmin(P_turbs)
        # # 5 * round(P_turb_max/5)+0.1
        # D_turbs = np.ma.masked_where(wind_v <= get_local_min_h_dot(wind_mag), D_turb)

        cfd_p_turbs = calc_eq(cfd_wind_mag, cfd_wind_north, cfd_wind_up)
        # pf_p_turbs = calc_eq(pf_wind_mag, pf_wind_u, pf_wind_v)

        alt_index = np.where((z_points > alt_low) & (z_points < alt_high))
        # over_thres_idx = np.where((cfd_p_turbs is not None) & (cfd_p_turbs > p_threshold))
        over_thres_idx = np.where((cfd_p_turbs is not None) & (cfd_p_turbs > 0))
        overlap = np.intersect1d(over_thres_idx, alt_index)
        x_overlap = x_points[overlap]
        # print(x_overlap, len(x_overlap))
        if len(x_overlap) == 0:
            return 0, 0

        x_min = np.nanmin(x_overlap)
        x_max = np.nanmax(x_overlap)

        return x_min, x_max


"""
could do some interpolation... but not the trilinear with all the values
"""


def main():
    print("This main fn is only for a test")


if __name__ == '__main__':
    main()
