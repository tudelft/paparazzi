# -*- coding: utf-8 -*-
"""
Created on Fri Jun 29 13:57:33 2018

@author: dennis
"""

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings
from pprz_math import geodetic

def enu_to_utm_fw(point_enu, ref_utm_i):
    point_utm = geodetic.UtmCoor_i(int(ref_utm_i.north + point_enu.y*100.), int(ref_utm_i.east + point_enu.x*100.), int(ref_utm_i.alt + point_enu.z*1000.), ref_utm_i.zone)
    return point_utm
    
def enu_to_lla_fw(point_enu, ref_utm_i):
    point_utm = enu_to_utm_fw(point_enu, ref_utm_i)
    point_lla = geodetic.LlaCoor_i()
    geodetic.lla_of_utm_i(point_lla, point_utm)
    return point_lla
    
def lla_to_enu_fw(point_lla, ref_utm_i):
    point_utm = geodetic.UtmCoor_i()
    geodetic.utm_of_lla_i(point_utm, point_lla)
    e = (point_utm.east - ref_utm_i.east)/100. #m
    n = (point_utm.north - ref_utm_i.north)/100. #m
    u = (point_utm.alt - ref_utm_i.alt)/1000. #m
    point_enu = geodetic.EnuCoor_d(e, n, u)
    return point_enu   
    
def lla_to_enu_rc(point_lla, ref_lla):
    ref_lla_f = geodetic.LlaCoor_f(np.deg2rad(ref_lla.lat*10.**-7), np.deg2rad(ref_lla.lon*10.**-7), ref_lla.alt*10.**-3)
    ref_ltp_f = ref_lla_f.to_ltp_def()
    point_lla_f = geodetic.LlaCoor_f(np.deg2rad(point_lla.lat*10.**-7), np.deg2rad(point_lla.lon*10.**-7), point_lla.alt*10.**-3)
    point_enu = point_lla_f.to_enu(ref_ltp_f)
    return point_enu
    
def enu_to_lla_rc(point_enu, ref_lla):
    ref_lla_f = geodetic.LlaCoor_f(np.deg2rad(ref_lla.lat*10.**-7), np.deg2rad(ref_lla.lon*10.**-7), ref_lla.alt*10.**-3)
    ref_ltp_f = ref_lla_f.to_ltp_def()
    point_ecef_f = point_enu.to_ecef(ref_ltp_f)
    point_lla_f = point_ecef_f.to_lla()    
    point_lla_i = geodetic.LlaCoor_i(int(np.rad2deg(point_lla_f.lat*10.**7)), int(np.rad2deg(point_lla_f.lon*10.**7)), int(point_lla_f.alt*10.**3))
    return point_lla_i