import sys
from os import path, getenv
from time import sleep
import time


PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC =  getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class UAV:
    def __init__(self, ac_id):
        self.initialized = False
        self.id = ac_id
        self.x = 0
        self.y = 0
        self.z = 0
        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.timeout = 0

class PNMessage:
    def __init__(self):
        self.ids = 4
        self.uav = UAV(4)
        self._interface = IvyMessagesInterface("PN Interception")

        def gps_int_cb(ac_id, msg):
            #print(msg.name)
            if ac_id == self.ids and msg.name == "GPS_INT":
                uav = self.uav
                uav.x = msg['ecef_x']
                uav.y = msg['ecef_y']
                uav.z = msg['ecef_z']
                uav.dx = msg['ecef_xd']
                uav.dy = msg['ecef_yd']
                uav.dz = msg['ecef_zd']
                uav.timeout = 0
                uav.initialized = True
                
        self._interface.subscribe(gps_int_cb, PprzMessage("telemetry", "GPS_INT"))

    def send_pn_info(self):

        msgw = PprzMessage("datalink", "HITL_IMU")
        msgw["gp"] =  float(self.uav.x)*0.01
        msgw["gq"] =  float(self.uav.y)*0.01
        msgw["gr"] =  float(self.uav.z)*0.01
        msgw["ax"] =  float(self.uav.dx)*0.01
        msgw["ay"] =  float(self.uav.dy)*0.01
        msgw["az"] =  float(self.uav.dz)*0.01
        msgw["ac_id"] = 24
        self._interface.send(msgw)

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def run(self):
        try:
            while True:
                self.send_pn_info()
                sleep(1/50)
                
        except KeyboardInterrupt:
            self.stop()
    
    


if __name__ == '__main__':
    import argparse

    Message_pn = PNMessage()
    Message_pn.run()