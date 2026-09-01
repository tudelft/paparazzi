#!/usr/bin/env python3

import os
import sys
import threading
import time
from pathlib import Path

PAPARAZZI_HOME = Path(__file__).resolve().parents[2]
os.environ.setdefault("PAPARAZZI_HOME", str(PAPARAZZI_HOME))
sys.path.insert(0, str(PAPARAZZI_HOME / "var/lib/python"))

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

AIRCRAFT_ID = 135
TAKEOFF_BLOCK = 1
SURVEY_ALTITUDE_MSL = 245
IVY_BUS = "127.255.255.255:2010"


def send_setting(interface, index, value):
    message = PprzMessage("ground", "DL_SETTING")
    message["ac_id"] = AIRCRAFT_ID
    message["index"] = index
    message["value"] = value
    interface.send(message)


def main():
    ready = threading.Event()
    sample_count = 0

    def on_position(sender, _message):
        nonlocal sample_count
        if int(sender) == AIRCRAFT_ID:
            sample_count += 1
            if sample_count >= 10:
                ready.set()

    interface = IvyMessagesInterface("easystar_mora_camera_starter", ivy_bus=IVY_BUS)
    interface.subscribe(on_position, PprzMessage("telemetry", "NPS_SPEED_POS"))

    print("EasyStar MORA starter: waiting for live NPS telemetry")
    if not ready.wait(120):
        interface.shutdown()
        raise RuntimeError("EasyStar MORA starter: no NPS telemetry received")

    time.sleep(1)

    print("EasyStar MORA starter: selecting AUTO2 and camera survey block")
    send_setting(interface, 10, SURVEY_ALTITUDE_MSL)  # flight_altitude = ground + 60 m
    send_setting(interface, 7, 2)  # autopilot.mode = AP_MODE_AUTO2
    send_setting(interface, 9, 0)  # autopilot.kill_throttle = FALSE

    jump = PprzMessage("ground", "JUMP_TO_BLOCK")
    jump["ac_id"] = AIRCRAFT_ID
    jump["block_id"] = TAKEOFF_BLOCK
    interface.send(jump)
    time.sleep(0.5)

    send_setting(interface, 8, 1)  # autopilot.launch = TRUE
    print("EasyStar MORA starter: survey launched at 60 m AGL")
    time.sleep(1)
    interface.shutdown()


if __name__ == "__main__":
    main()
