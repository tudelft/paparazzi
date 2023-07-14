import sys
from os import path, getenv

import matplotlib.pyplot as plt

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/sw/logalizer/python/log_parser")
sys.path.append(PPRZ_HOME + "/sw/logalizer/python/ctrl_effectiveness_estimator_doublet")

from log_parser import LogParser
from ctrl_eff_est import CtrlEffEst

parsed_log = LogParser(0, 3600)

eff_estimator = CtrlEffEst( parsed_log,\
                            [0.025,0.025,0.025,0.025],\
                            2.0,\
                            [[1200,2000], [1200,2000], [1200,2000], [1200,2000]],\
                            [0,0,0,0],\
                            [0,1,2,3])

# eff_estimator = CtrlEffEst( parsed_log,\
#                             [0.03,0.025,0.03,0.025,0.1,0.1,0.047],\
#                             2.0,\
#                             [[1000,8191],[1000,8191],[1000,8191],[1000,8191],[-8191,8191],[8191,-1395], [0,8191]],\
#                             [0,0,0,0,0,1,1,0],\
#                             [3,4,5,6,9,8,7])

# eff_estimator = CtrlEffEst( parsed_log,\
#                             [0.035,0.035,0.035,0.035,0.1,0.1,0.047],\
#                             2.0,\
#                             [[1000,8191],[1000,8191],[1000,8191],[1000,8191],[-8191,8191],[8191,-4300], [0,8191]],\
#                             [0,0,0,0,0,1,1,0],\
#                             [1,2,3,4,5,6,7])

# eff_estimator = CtrlEffEst( parsed_log,\
#                             [0.035,0.035,0.035,0.035],\
#                             2.0,\
#                             [[1000,8191],[1000,8191],[1000,8191],[1000,8191]],\
#                             [0,0,0,0,0],\
#                             [1,2,3,4])

eff_estimator.get_effectiveness_values()