#!/usr/bin/env python3
"""Regression tests for bounded parallel GPS-denied mesh simulation."""

from __future__ import annotations

import contextlib
import io
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mesh_gps_denied_sim as simulator


class ParallelExecutionTest(unittest.TestCase):

    def test_two_worker_cli_run_completes(self) -> None:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = simulator.main((
                "--seeds", "2",
                "--workers", "2",
                "--duration", "900",
                "--denied-nodes", "6",
            ))
        self.assertEqual(result, 0)
        self.assertIn("seeds / duration : 2 / 900 s", output.getvalue())
        self.assertIn("PASS", output.getvalue())


if __name__ == "__main__":
    unittest.main()