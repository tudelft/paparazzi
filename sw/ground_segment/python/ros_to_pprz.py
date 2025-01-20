#!/usr/bin/env python

from __future__ import print_function
import sys
from os import path, getenv
from time import sleep

import rospy
from ros_ati_mini45.msg import ForceMeasurement 


PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


class ForceSensorbject:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self._interface = IvyMessagesInterface("ForceSensor")

       
        rospy.init_node("ros_to_pprz", anonymous=True)

        
        rospy.Subscriber("/force/force_data_processed", ForceMeasurement, self.force_callback)

       
        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.Mx = 0.0
        self.My = 0.0
        self.Mz = 0.0 


    def shutdown(self):
        print("Shutting down ivy interface...")
        self._interface.shutdown()

    def __del__(self):
        self.shutdown()

    def force_sensor(Fx, Fy,Fz,Mx,My,Mz ):
        msg = PprzMessage("Force_SENSOR")
        msg['Fx'] = self.fx
        msg['Fy'] = self.fy
        msg['Fz'] = self.fz
        msg['Mx'] = self.Mx
        msg['My'] = self.My
        msg['Mz'] = self.Mz
        print("Sending message: %s" % msg)
        self._interface.send(msg)

    def force_callback(self, data):
    
        self.fx = data.Fx
        self.fy = data.Fy
        self.fy = data.Fz
        self.fy = data.Mx
        self.fy = data.My
        self.fy = data.Mz

        self.force_sensor(ac_id=202, wp_id=3, lat=lat, lon=lon, alt=alt)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        wm = ForceSensor()
        print("ros_to_pprz node is running...")
        wm.run()
    except KeyboardInterrupt:
        print("Stopping on request")
    wm.shutdown()
