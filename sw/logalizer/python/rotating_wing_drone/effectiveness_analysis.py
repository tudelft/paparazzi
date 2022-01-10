import numpy as np
import scipy as sp
from scipy import signal
import matplotlib.pyplot as plt

import sys
from os import path, getenv

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/sw/logalizer/python/log_parser") # pprzlink

from log_parser import LogParser

parsed_log = LogParser(610, 620)

# Get data
stab_attitude_msg = parsed_log.get_message_dict('STAB_ATTITUDE_FULL_INDI')

t = stab_attitude_msg['t']
airspeed = stab_attitude_msg['airspeed']['data']
ang_rate = stab_attitude_msg['angular_rate_r']['data']
# Apply butterworth filter
# 2nd order bUtterworth noise filter
b, a = sp.signal.butter(2, 1.5/(500./2), 'low', analog=False)

ang_rate_filt = sp.signal.lfilter(b, a, ang_rate, axis=0)
ang_acc = np.hstack((np.array([0.]), np.diff(ang_rate_filt) * 500))
u = np.array(stab_attitude_msg['u']['data'])[:,7]
u_f = sp.signal.lfilter(b, a, u, axis=0)

d_ang_acc = np.hstack((np.array([0.]), np.diff(ang_acc) * 500))
d_u_f = np.hstack((np.array([0.]), np.diff(u_f) * 500))

plt.subplots(3)
plt.subplot(311)
plt.title('Airspeed')
plt.plot(t, airspeed)
plt.subplot(312)
plt.title('Angular acceleration')
plt.plot(t, ang_acc)
plt.subplot(313)
plt.title('u')
plt.plot(t, u)
plt.plot(t, u_f)

plt.subplots(3)
plt.subplot(311)
plt.title('Airspeed')
plt.plot(t, airspeed)
plt.subplot(312)
plt.title('Delta Angular acceleration')
plt.plot(t, d_ang_acc)
plt.subplot(313)
plt.title('d_u')
plt.plot(t, d_u_f)
plt.show()