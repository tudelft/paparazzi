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
        self.assertEqual(optimizer.steady_slot_count(32, 30, 17, 8), 30)
        self.assertEqual(optimizer.steady_slot_count(32, 30, 2, 8), 16)

    def test_absolute_slot_bound_is_default(self) -> None:
        absolute = optimizer.SlotPlan(
            slot_s=0.5, n_slots=32, superframe_s=16.0, guard_s=0.022,
            span_statistical_s=0.369, span_worst_s=0.468)
        statistical = optimizer.SlotPlan(
            slot_s=0.5, n_slots=32, superframe_s=16.0, guard_s=0.062,
            span_statistical_s=0.369, span_worst_s=0.468,
            use_statistical_bound=True)
        self.assertTrue(absolute.ok)
        self.assertTrue(statistical.ok)
        self.assertEqual(absolute.required_span_s, absolute.span_worst_s)
        self.assertEqual(statistical.required_span_s,
                         statistical.span_statistical_s)


class PhaseSafetyTest(unittest.TestCase):

    def test_periodic_gap_matches_explicit_replay(self) -> None:
        for a_period, b_period in ((8, 12), (16, 64), (25, 40)):
            horizon = optimizer.math.lcm(a_period, b_period)
            for a_tick in range(a_period):
                for b_tick in range(b_period):
                    explicit = min(
                        abs(((left - right + horizon // 2) % horizon)
                            - horizon // 2)
                        for left in range(a_tick, horizon, a_period)
                        for right in range(b_tick, horizon, b_period)
                    )
                    self.assertEqual(
                        optimizer._periodic_gap(
                            a_tick, a_period, b_tick, b_period),
                        explicit)

    def test_pathological_horizon_is_rejected_before_phase_search(self) -> None:
        message = optimizer.MessageDef("TEST", "telemetry", 1, 1, False)
        entries = [
            optimizer.ScheduleEntry(message, 2.02),
            optimizer.ScheduleEntry(message, 2.06),
        ]
        with self.assertRaisesRegex(ValueError, "safety limit"):
            optimizer.solve_phases(entries, 50, 1, max_replay_ticks=1000)
        self.assertTrue(all(entry.tick == -1 for entry in entries))


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

    def test_cli_rejects_large_replay_horizon(self) -> None:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), \
                contextlib.redirect_stdout(io.StringIO()):
            result = optimizer.main((
                "--mesh-period", "2.02",
                "--superframe", "16.16",
                "--max-replay-ticks", "1000",
            ))
        self.assertEqual(result, 2)
        self.assertIn("schedule replay horizon", stderr.getvalue())

    def test_inadequate_phase_spacing_fails(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()), \
                contextlib.redirect_stdout(io.StringIO()):
            result = optimizer.main((
                "--ac-ids", "0,3,19,42,58,77,101,125,140,168,203,222,251",
                "--relay-nodes", "13",
                "--nb-slots", "32",
                "--fair-slots", "30",
                "--mesh-period", "1.8",
                "--superframe", "14.4",
                "--max-reuse", "8",
                "--period-scale", "2",
                "--utilisation", "0.60",
            ))
        self.assertEqual(result, 1)


if __name__ == "__main__":
    unittest.main()