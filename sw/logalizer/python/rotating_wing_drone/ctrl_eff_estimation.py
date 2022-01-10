import sys
from os import path, getenv

import matplotlib.pyplot as plt

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/sw/logalizer/python/log_parser")
sys.path.append(PPRZ_HOME + "/sw/logalizer/python/ctrl_effectiveness_estimator")

from log_parser import LogParser
from ctrl_eff_est import CtrlEffEst

parsed_log = LogParser(580, 600)

eff_estimator = CtrlEffEst( parsed_log,\
                            [0.047,0.047,0.047,0.047,0.1,0.1,0.1,0.1,0.047],\
                            1.5,\
                            [[1100., 2000.],[1100.,2000.],[1100.,2000.],[1100.,2000.],[1100.,1900.],[1100.,1900.],[1100.,1900.],[1100.,1900.],[1100., 2000.]],\
                            [0,0,0,0,1,1,1,1,0],\
                            [0,1,2,3,4,5,6,7,8])
'''
eff_estimator = CtrlEffEst( parsed_log,\
                            [0.047,0.047,0.047,0.047],\
                            1.5,\
                            [[1100., 2000.],[1100.,2000.],[1100.,2000.],[1100.,2000.]],\
                            [0,0,0,0],\
                            [0,1,2,3])
'''
eff_estimator.get_effectiveness_values()