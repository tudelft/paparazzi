#!/usr/bin/env python3
"""Synthetic geometry and real checker detection tests; no camera required."""

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import lwir_calibrate as calibration


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.matrix = np.array([[358., 0, 127.5], [0, 361., 95.5], [0, 0, 1]])
        self.distortion = np.array([-.12, .04, .001, -.002, -.008])
        self.nominal = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
        self.objects = calibration.board_points(7, 6, 50)
        self.gray = np.zeros((192, 256), np.uint8)
        self.previews = self.root / "previews"
        self.previews.mkdir()

    def args(self, name):
        return argparse.Namespace(columns=7, rows=6, square_mm=50.,
                                  output=self.root / name)

    def test_lens_recovers_known_geometry_and_checks(self):
        args = self.args("lens.yml")
        args.images, args.check = self.root / "fit", self.root / "check"
        fit_paths = [args.images / f"{index}.png" for index in range(25)]
        check_paths = [args.check / f"{index}.png" for index in range(6)]
        observations = {}
        random = np.random.default_rng(1701)
        for path in fit_paths + check_paths:
            rotation = random.uniform(-.45, .45, 3)
            translation = np.array([random.uniform(-.22, -.04), random.uniform(-.18, -.04),
                                    random.uniform(.8, 1.4)])
            corners, _ = cv2.projectPoints(self.objects, rotation, translation,
                                           self.matrix, self.distortion)
            observations[path] = (self.gray, corners)
        with patch.object(calibration, "image_paths", side_effect=[fit_paths, check_paths]), \
                patch.object(calibration, "detect", side_effect=lambda path, *unused: observations[path]):
            report = []
            calibration.fit_lens(args, self.previews, report)
        size, matrix, distortion = calibration.read_lens(args.output)
        self.assertEqual(size, (256, 192))
        np.testing.assert_allclose(matrix, self.matrix, atol=.01)
        np.testing.assert_allclose(distortion, self.distortion, atol=.002)
        self.assertEqual(sum(line.startswith("CHECK ") for line in report), 6)
        self.assert_profile_unverified(args.output)

    def assert_profile_unverified(self, path):
        file = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        self.assertEqual(file.getNode("verified").real(), 0)
        rotation = file.getNode("camera_to_body")
        self.assertTrue(rotation.isSeq())
        self.assertEqual(rotation.size(), 9)
        matrix = np.array([rotation.at(index).real() for index in range(9)]).reshape(3, 3)
        file.release()
        return matrix

    def test_mount_rotation_and_reversed_corners(self):
        lens = self.root / "lens.yml"
        calibration.write_profile(lens, (256, 192), self.matrix, self.distortion, self.nominal)
        perturbation, _ = cv2.Rodrigues(np.array([.04, -.07, .03]))
        expected = perturbation @ self.nominal
        rotation, _ = cv2.Rodrigues(expected.T)
        body = np.column_stack((-self.objects[:, 1], self.objects[:, 0], self.objects[:, 2]))
        corners, _ = cv2.projectPoints(body, rotation, np.array([-.14, -.12, 1.]),
                                       self.matrix, self.distortion)
        for reverse in (False, True):
            args = self.args(f"mount-{reverse}.yml")
            args.lens, args.image, args.reverse_corners = lens, self.root / "mount.png", reverse
            supplied = corners[::-1].copy() if reverse else corners
            with patch.object(calibration, "detect", return_value=(self.gray, supplied)):
                calibration.fit_mount(args, self.previews, [])
            np.testing.assert_allclose(self.assert_profile_unverified(args.output), expected, atol=1e-5)

    def test_real_detection_and_cli_overwrite_guard(self):
        image = np.full((480, 600), 127, np.uint8)
        for row in range(7):
            for column in range(8):
                image[50 + row * 50:100 + row * 50, 80 + column * 50:130 + column * 50] = \
                    235 if (row + column) % 2 else 20
        path = self.root / "board.png"
        self.assertTrue(cv2.imwrite(str(path), image))
        _, corners = calibration.detect(path, 7, 6)
        self.assertIsNotNone(corners)
        self.assertEqual(corners.shape, (42, 1, 2))
        output = self.root / "existing.yml"
        output.write_text("keep this evidence", encoding="ascii")
        result = subprocess.run([sys.executable, str(Path(calibration.__file__)), "lens",
                                 "--images", str(self.root), "--columns", "7", "--rows", "6",
                                 "--square-mm", "50", "--output", str(output)],
                                capture_output=True, text=True, check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("exists", result.stderr)
        self.assertEqual(output.read_text(), "keep this evidence")

    def test_insufficient_views_and_mixed_sizes(self):
        args = self.args("lens.yml")
        args.images, args.check = self.root, None
        corners = np.zeros((42, 1, 2), np.float32)
        with patch.object(calibration, "image_paths", return_value=[self.root / "one.png"]), \
                patch.object(calibration, "detect", return_value=(self.gray, corners)):
            with self.assertRaisesRegex(ValueError, "at least 15"):
                calibration.fit_lens(args, self.previews, [])
        with patch.object(calibration, "image_paths", return_value=[Path("one.png"), Path("two.png")]), \
                patch.object(calibration, "detect", side_effect=[(self.gray, corners),
                                      (np.zeros((191, 256), np.uint8), corners)]):
            with self.assertRaisesRegex(ValueError, "Mixed image"):
                calibration.fit_lens(args, self.previews, [])
        self.assertFalse(args.output.exists())


if __name__ == "__main__":
    unittest.main()