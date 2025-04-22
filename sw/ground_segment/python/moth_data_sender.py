import sys
import logging
from time import sleep



# ── SETUP LOGGING ──────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)

# ── HARD‑CODED PAPARAZZI PATHS ─────────────────────────────────────────────────
sys.path.insert(0, "/home/merlijn/paparazzi/sw/ext/pprzlink/lib/v1.0/python")
sys.path.insert(1, "/home/merlijn/paparazzi/var/lib/python")

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface


# ── CALLBACKS ──────────────────────────────────────────────────────────────────

class Moth_data:

    def __init__(self):
        self.x = -3
        self.y = 2
        self.z = -5
        self.vx = 0
        self.vy = 0
        self.vz = 0

class Moth_data_sender:

    def __init__(self):
        self.ivy = IvyMessagesInterface(
            agent_name="PprzlinkIvyTutorial",
            start_ivy=True,
            verbose=True,
            ivy_bus="127.255.255.255:2010"
        )

    def send_moth_info(self, moth: Moth_data):
        msgw = PprzMessage("datalink", "REMOTE_GPS_LOCAL")
        msgw["enu_x"] =  float(moth.x)
        msgw["enu_y"] =  float(moth.y)
        msgw["enu_z"] =  float(moth.z)
        msgw["enu_xd"] =  float(moth.vx)
        msgw["enu_yd"] =  float(moth.vy)
        msgw["enu_zd"] =  float(moth.vz)
        msgw["ac_id"] = 42
        self.ivy.send(msgw)

    def run(self, moth):
        try:
            while True:
                self.send_moth_info(moth)
                sleep(1/50)
                
        except KeyboardInterrupt:
            self.stop()

moth_data = Moth_data()
sender = Moth_data_sender()
sender.run(moth_data)