#!/usr/bin/env python3
"""Regression tests for dense self-organising slot allocation."""

from __future__ import annotations

import contextlib
import io
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mesh_slot_sim as simulator


class DenseFleetRegressionTest(unittest.TestCase):

    def test_late_join_starts_with_fresh_slot_history(self) -> None:
        node = simulator.Node(42, start_frame=100)

        self.assertEqual(node.last_frame, 100)
        self.assertTrue(all(last_frame == 100
                            for _, last_frame in node.slots.values()))
        self.assertFalse(any(node._free(slot, 100)
                             and simulator.frame_elapsed(100, node.slots[slot][1])
                             > 2 * simulator.AGE_FRAMES
                             for slot in range(simulator.NB_SLOTS)))

    def test_id_pool_covers_full_physical_slot_count(self) -> None:
        self.assertEqual(len(simulator.SIM_AC_IDS), simulator.NB_SLOTS)
        self.assertEqual(len(set(simulator.SIM_AC_IDS)), simulator.NB_SLOTS)
        self.assertEqual(simulator.SIM_AC_IDS[0], simulator.GCS_ID)
        self.assertNotIn(simulator.SLOT_FREE, simulator.SIM_AC_IDS)

    def test_static_seventeen_peer_fleet_converges(self) -> None:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = simulator.main((
                "--frames", "4000",
                "--settle", "300",
                "--churn", "0",
                "--state-fault", "0",
                "--priority", "0",
                "--cache-busy", "0",
                "--seed", "1",
                "--initial-nodes", "17",
                "--max-nodes", "17",
            ))
        self.assertEqual(result, 0)
        self.assertIn("      17", output.getvalue())
        self.assertIn("PASS", output.getvalue())


if __name__ == "__main__":
    unittest.main()
