#!/usr/bin/env python3
"""Regression tests for the asynchronous coded-mesh feasibility model."""

from __future__ import annotations

import random
import unittest

import mesh_coded_sim
from mesh_coding import GenerationDecoder, Membership, systematic_symbols


class MembershipTest(unittest.TestCase):

    def test_sparse_ids_are_opaque(self) -> None:
        ids = (0, 203, 3, 101, 42, 254, 19)
        membership = Membership(ids)
        self.assertEqual(membership.ac_ids, ids)
        self.assertEqual(
            [membership.ac_ids[membership.index(ac_id)] for ac_id in ids],
            list(ids))

    def test_broadcast_id_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "255 is broadcast"):
            Membership((0, 7, 255))

    def test_ids_must_be_distinct(self) -> None:
        with self.assertRaisesRegex(ValueError, "distinct"):
            Membership((0, 42, 42))


class CodingTest(unittest.TestCase):

    def test_sparse_recoding_round_trip(self) -> None:
        payloads = [bytes(range(22)), bytes(reversed(range(22))),
                    bytes([0xA5] * 22), bytes([0x5A] * 22)]
        encoder = GenerationDecoder(203, 65535, 4, 22)
        for symbol in systematic_symbols(203, 65535, payloads,
                                         [10, 20, 30, 40]):
            self.assertTrue(encoder.add(symbol))

        decoder = GenerationDecoder(203, 65535, 4, 22)
        rng = random.Random(7)
        for _attempt in range(32):
            decoder.add(encoder.recode(rng, density=0.5))
            if decoder.complete:
                break
        self.assertTrue(decoder.complete)
        self.assertEqual(decoder.decoded(), payloads)


class ScenarioTest(unittest.TestCase):

    def args(self, *extra: str):
        return mesh_coded_sim.parse_args([
            "--ac-ids", "0,3,19,42,101,203,254",
            "--duration", "6", "--seed", "7",
            "--shadow-sigma", "0", *extra])

    def test_gcs_is_mobile_receiver_and_router(self) -> None:
        args = self.args("--source-rate", "0.5")
        scenario = mesh_coded_sim.Scenario(args, args.seed)
        start = scenario.position(0)
        scenario.move(1.0)
        self.assertNotEqual(scenario.position(0), start)
        self.assertIn(0, scenario.nodes)

        metrics = mesh_coded_sim.run_coded(args)
        self.assertTrue(metrics.gcs_ages_ms)
        self.assertTrue(any(receiver_id == 0
                            for _source_id, receiver_id in metrics.latest_ms))
        possible = metrics.originated * (len(scenario.membership.ac_ids) - 1)
        self.assertLessEqual(len(metrics.delivered), possible)

    def test_sixty_four_random_aircraft_construct(self) -> None:
        args = mesh_coded_sim.parse_args(["--aircraft", "64", "--duration", "1"])
        scenario = mesh_coded_sim.Scenario(args, args.seed)
        self.assertEqual(len(scenario.aircraft_ids), 64)
        self.assertEqual(len(set(scenario.membership.ac_ids)), 65)
        self.assertIn(0, scenario.membership.ac_ids)

    def test_population_adapts_source_and_forwarding_rates(self) -> None:
        self.assertEqual(mesh_coded_sim.coded_source_rate(9, None), 0.75)
        self.assertEqual(mesh_coded_sim.coded_source_rate(16, None), 0.40)
        self.assertEqual(mesh_coded_sim.coded_source_rate(64, None), 0.10)
        self.assertEqual(mesh_coded_sim.coded_forward_probability(9, None), 0.15)
        self.assertEqual(mesh_coded_sim.coded_forward_probability(16, None), 0.10)
        self.assertEqual(mesh_coded_sim.coded_forward_probability(64, None), 0.01)


if __name__ == "__main__":
    unittest.main()