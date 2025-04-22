import sys
import logging
import time

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

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# ── CALLBACK FUNCTION ──────────────────────────────────────────────────────────
def recv_remote_gps_local(ac_id, pprzMsg):
    # Print out all fields in the message
    fields = pprzMsg.fieldnames
    values = pprzMsg.fieldvalues
    data = dict(zip(fields, values))
    logger.info("Received REMOTE_GPS_LOCAL from ac_id=%s: %s", ac_id, data)

# ── MAIN ───────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # Create Ivy interface and start it
    ivy = IvyMessagesInterface(
        agent_name="MothDataListener",  # unique name
        start_ivy=True,
        ivy_bus="127.255.255.255:2010"
    )

    # Subscribe to REMOTE_GPS_LOCAL messages
    ivy.subscribe(recv_remote_gps_local, PprzMessage("datalink", "REMOTE_GPS_LOCAL"))
    logger.info("Subscribed to REMOTE_GPS_LOCAL, listening on Ivy bus...")

    try:
        # Keep the program alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Interrupted by user, shutting down...")
        ivy.shutdown()
