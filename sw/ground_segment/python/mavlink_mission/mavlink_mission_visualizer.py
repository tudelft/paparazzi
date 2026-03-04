#
# Copyright (C) 2024 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#

# This is not the main script. Run dist.py to have a distance counter.

import sys
import os
import time
import logging


PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))

sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MissionVisualizer(object):

    def __init__(self):
        """
        Initialize the Mission visualizer with state tracking
        """
        # Mission state tracking
        self.mission_count = 0  # Expected number of mission items
        self.received_items = {}  # Dict of seq -> (lat, lon) for received items
        self.last_update_time = time.time()
        self.mission_complete = False
        
        logger.info("Mission Visualizer initialized")

        # Create a new ivy interface
        self.ivy_interface = IvyMessagesInterface("mission visualizer")
        self.ivy_interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg):
        """Handle incoming messages"""
        try:
            if msg.name == "MISSION_ITEM":
                self.mission_item_receiver(ac_id, msg)
            elif msg.name == "MISSION_ITEM_INT":
                self.mission_item_receiver(ac_id, msg)
            elif msg.name == "MISSION_COUNT":
                logger.debug(f"Received MISSION_COUNT message")
                self.mission_count_receiver(ac_id, msg)
        except Exception as e:
            logger.error(f"Error processing message {msg.name}: {e}")
    
    def mission_count_receiver(self, ac_id, msg):
        """Handle MISSION_COUNT message to know how many items to expect"""
        try:
            count = int(msg['count'])
            self.mission_count = count
            self.mission_complete = False
            self.received_items = {}  # Reset received items for new mission
            self.clear_all_shapes()
            logger.info(f"Mission count received: expecting {count} waypoints")
            self.last_update_time = time.time()
        except Exception as e:
            logger.error(f"Error in mission_count_receiver: {e}")
    
    def mission_item_receiver(self, ac_id, msg):
        """Handle MISSION_ITEM or MISSION_ITEM_INT message"""
        try:
            seq = int(msg['seq'])
            
            # Try to get lat/lon - some mission items don't have meaningful coordinates
            try:
                lat = int(msg['lat'])  # e7 deg
                lon = int(msg['lon'])  # e7 deg
            except (KeyError, ValueError, TypeError):
                # Field doesn't exist or can't be parsed - skip this item
                logger.debug(f"Skipped mission item {seq} - no valid lat/lon fields")
                return
            
            # Validate coordinates are within valid geographic range
            # Latitude: -90 to 90 degrees, stored as e7
            # Longitude: -180 to 180 degrees, stored as e7
            max_lat = 90 * 1e7
            max_lon = 180 * 1e7
            
            if abs(lat) > max_lat or abs(lon) > max_lon:
                logger.debug(f"Skipped mission item {seq} with out-of-range coordinates: ({lat/1e7}, {lon/1e7})")
                return
            
            # Store the received item
            self.received_items[seq] = (lat, lon)
            logger.info(f"Received mission item {seq}: ({lat/1e7:.6f}, {lon/1e7:.6f})")
            
            # Draw the waypoint
            self.delete_point(seq)
            self.draw_point(seq, lat, lon)
            
            # Check if mission is complete
            if self.mission_count > 0 and len(self.received_items) == self.mission_count:
                if not self.mission_complete:
                    self.mission_complete = True
                    logger.info(f"Mission complete! Received all {self.mission_count} waypoints")
            
            # Log missing items if we know the expected count
            if self.mission_count > 0:
                missing = self.get_missing_items()
                if missing:
                    logger.warning(f"Missing waypoints: {missing}")
            
            self.last_update_time = time.time()
        except Exception as e:
            logger.error(f"Error in mission_item_receiver: {e}")
    
    def get_missing_items(self):
        """Return list of missing item sequence numbers"""
        if self.mission_count == 0:
            return []
        expected = set(range(self.mission_count))
        received = set(self.received_items.keys())
        return sorted(expected - received)
    
    def clear_all_shapes(self):
        """Clear all visualized shapes"""
        # Delete all waypoint markers
        for seq in self.received_items.keys():
            self.delete_point(seq)
        logger.info("Cleared all shapes")

    def draw_point(self, seq, lat, lon):
        """
        Draw a waypoint marker
        """
        try:
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = seq
            msg['linecolor'] = "yellow"
            msg['fillcolor'] = "yellow"
            msg['opacity'] = 3
            msg['shape'] = 0  # Circle
            msg['status'] = 0  # Create
            msg['latarr'] = [lat]  # e-7 deg
            msg['lonarr'] = [lon]  # e-7 deg
            msg['radius'] = 5
            self.ivy_interface.send(msg)
        except Exception as e:
            logger.error(f"Error drawing point {seq}: {e}")

    def delete_point(self, seq):
        """
        Delete a waypoint marker
        """
        try:
            msg = PprzMessage("ground", "SHAPE")
            msg['id'] = seq
            msg['linecolor'] = "yellow"
            msg['fillcolor'] = "yellow"
            msg['opacity'] = 3
            msg['shape'] = 0  # Circle
            msg['status'] = 1  # Delete
            msg['latarr'] = [0]  # e-7 deg
            msg['lonarr'] = [0]  # e-7 deg
            msg['radius'] = 5
            self.ivy_interface.send(msg)
        except Exception as e:
            logger.error(f"Error deleting point {seq}: {e}")
    
    def print_status(self):
        """Print current mission status"""
        if len(self.received_items) == 0 and self.mission_count == 0:
            logger.info("No mission loaded")
        else:
            received = len(self.received_items)
            if self.mission_count > 0:
                logger.info(f"Mission status: {received}/{self.mission_count} waypoints received")
                if not self.mission_complete:
                    missing = self.get_missing_items()
                    if missing:
                        logger.info(f"Missing waypoints: {missing}")
            else:
                # We're receiving items but didn't get MISSION_COUNT message
                logger.info(f"Mission status: {received} waypoints received (count unknown)")

if __name__ == '__main__':
    visualizer = MissionVisualizer()
    
    # Periodic status reporting
    import threading
    
    def periodic_status():
        while True:
            time.sleep(10)  # Print status every 10 seconds
            visualizer.print_status()
    
    # Start status reporting thread
    status_thread = threading.Thread(target=periodic_status, daemon=True)
    status_thread.start()
    
    logger.info("Mission Visualizer running. Press Ctrl+C to exit.")
    
    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Mission Visualizer stopped by user")