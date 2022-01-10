import sys
from os import path, getenv

import matplotlib.pyplot as plt

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/sw/logalizer/python/log_parser") # pprzlink

from log_parser import LogParser

parsed_log = LogParser()
#parsed_log.plot_variable('STAB_ATTITUDE_FULL_INDI', ['angular_rate_p'], [[]])
#parsed_log.plot_variable('STAB_ATTITUDE_FULL_INDI', ['u'], [[0,1,2,3]])
#parsed_log.plot_variable('STAB_ATTITUDE_FULL_INDI', ['airspeed'])
#parsed_log.plot_variable('IMU_ACCEL_SCALED', ['ax', 'ay', 'az'])
#parsed_log.plot_variable('IMU_ACCEL_SCALED', ['ax', 'ay', 'az'])
parsed_log.plot_variable('INDI_G', ['G1_pitch'], [[0,1,2,3,4]])
parsed_log.plot_variable('ROT_WING_CONTROLLER', ['wing_angle_deg'])
plt.show()