#!/usr/bin/env python3
"""Regression tests for mesh telemetry phase and airtime accounting."""

from __future__ import annotations

import contextlib
import io
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mesh_phase_optimizer as optimizer


class NamedValueTest(unittest.TestCase):

    def test_parse_named_values(self) -> None:
        self.assertEqual(
            optimizer.parse_named_values(
                ["GPS_LLA=16", "WP_MOVED=32.5"], "--period"),
            {"GPS_LLA": 16.0, "WP_MOVED": 32.5})

    def test_invalid_named_values_are_rejected(self) -> None:
        for value in ("GPS_LLA", "=16", "GPS_LLA=fast", "GPS_LLA=0"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                optimizer.parse_named_values([value], "--period")


class AirtimeAccountingTest(unittest.TestCase):

    def test_event_reserve_scales_linearly(self) -> None:
        periodic = optimizer.MessageDef("PERIODIC", "telemetry", 1, 10, False)
        event = optimizer.MessageDef("EVENT", "datalink", 2, 20, False)
        entry = optimizer.ScheduleEntry(periodic, 1.0, tick=0, phase=0.0)
        network = optimizer.NetworkModel(
            optimizer.phy_for_rate(0), n_nodes=3, n_relay_nodes=3,
            csma_range_ms=20, target_utilisation=0.4)

        base = optimizer.simulate_cache(
            {0: ["PERIODIC"]}, [entry], network, 1, 1.0, 1)[2]
        reserved = optimizer.simulate_cache(
            {0: ["PERIODIC"]}, [entry], network, 1, 1.0, 1,
            event_frames=[(event, 0.25)])[2]

        self.assertAlmostEqual(
            reserved - base,
            0.25 * network.channel_cost_s(event.wire_bytes))

    def test_fair_slots_leave_churn_headroom(self) -> None:
        self.assertEqual(optimizer.steady_slot_count(32, 28, 17, 8), 28)
        self.assertEqual(optimizer.steady_slot_count(32, 28, 2, 8), 16)


class CommandLineTest(unittest.TestCase):

    def assert_cli_error(self, *arguments: str) -> str:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                optimizer.main(arguments)
        self.assertEqual(raised.exception.code, 2)
        return stderr.getvalue()

    def test_unknown_period_name_is_rejected(self) -> None:
        error = self.assert_cli_error("--period", "NOT_A_MESSAGE=1")
        self.assertIn("not in the mesh telemetry set", error)

    def test_legacy_control_rate_alias_is_retained(self) -> None:
        error = self.assert_cli_error("--move-wp-rate", "-1")
        self.assertIn("--control-rate must not be negative", error)


if __name__ == "__main__":
    unittest.main()