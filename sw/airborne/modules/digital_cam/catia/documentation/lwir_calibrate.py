#!/usr/bin/env python3
"""Desktop-only checkerboard calibration; never changes camera firmware."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def board_points(columns, rows, square_mm):
    points = np.zeros((columns * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * (square_mm / 1000)
    return points


def detect(path, columns, rows):
    gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise ValueError(f"Cannot read image: {path}")
    found, corners = cv2.findChessboardCornersSB(
        gray, (columns, rows), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
    )
    return gray, corners.reshape(-1, 1, 2) if found else None


def preview(gray, corners, columns, rows, destination, numbered=False):
    canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(canvas, (columns, rows), corners, True)
    canvas = cv2.resize(canvas, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    if numbered:
        for index in (0, 1, columns):
            point = tuple(np.rint(corners[index, 0] * 3).astype(int))
            cv2.putText(canvas, str(index), point, cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(canvas, str(index), point, cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(destination), canvas):
        raise ValueError(f"Cannot write preview: {destination}")


def image_paths(directory):
    paths = sorted(path for path in directory.iterdir()
                   if path.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not paths:
        raise ValueError(f"No JPEG or PNG images in {directory}")
    return paths


def rms_pixels(object_points, image_points, rotation, translation, matrix, distortion):
    projected, _ = cv2.projectPoints(object_points, rotation, translation, matrix, distortion)
    return float(np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=2))))


def write_profile(path, size, matrix, distortion, mounting):
    if not all(np.isfinite(array).all() for array in (matrix, distortion, mounting)):
        raise ValueError("Fit produced non-finite coefficients; no profile written")
    file = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not file.isOpened():
        raise ValueError(f"Cannot write {path}")
    try:
        file.write("image_width", int(size[0]))
        file.write("image_height", int(size[1]))
        for key, value in zip(("fx", "fy", "cx", "cy"),
                              (matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2])):
            file.write(key, float(value))
        for key, value in zip(("k1", "k2", "p1", "p2", "k3"), distortion.ravel()):
            file.write(key, float(value))
        file.write("verified", 0)
        file.startWriteStruct("camera_to_body", cv2.FileNode_SEQ | cv2.FileNode_FLOW)
        for value in mounting.ravel():
            file.write("", float(value))
        file.endWriteStruct()
    finally:
        file.release()


def read_lens(path):
    file = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not file.isOpened():
        raise ValueError(f"Cannot open lens profile: {path}")
    try:
        values = {}
        for key in ("image_width", "image_height", "fx", "fy", "cx", "cy",
                    "k1", "k2", "p1", "p2", "k3"):
            node = file.getNode(key)
            if node.empty() or not (node.isInt() or node.isReal()):
                raise ValueError(f"Missing or nonnumeric lens key: {key}")
            values[key] = node.real()
        if not all(np.isfinite(value) for value in values.values()):
            raise ValueError("Non-finite lens coefficient")
        if values["fx"] <= 0 or values["fy"] <= 0:
            raise ValueError("Focal lengths must be positive")
        matrix = np.array([[values["fx"], 0, values["cx"]],
                           [0, values["fy"], values["cy"]], [0, 0, 1]], np.float64)
        distortion = np.array([values[key] for key in ("k1", "k2", "p1", "p2", "k3")])
        return (int(values["image_width"]), int(values["image_height"])), matrix, distortion
    finally:
        file.release()


def fit_lens(args, previews, report):
    objects = board_points(args.columns, args.rows, args.square_mm)
    paths = image_paths(args.images)
    observations = []
    size = None
    for index, path in enumerate(paths):
        gray, corners = detect(path, args.columns, args.rows)
        current_size = gray.shape[::-1]
        if size is not None and size != current_size:
            raise ValueError(f"Mixed image dimensions at {path}; do not resize to force a match")
        size = current_size
        if corners is None:
            report.append(f"SKIPPED no complete pattern: {path}")
            print(report[-1])
            continue
        observations.append((path, corners))
        preview(gray, corners, args.columns, args.rows, previews / f"fit-{index:03d}.png")
    if len(observations) < 15:
        raise ValueError(f"Only {len(observations)} complete patterns; need at least 15 varied views")
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        [objects] * len(observations), [entry[1] for entry in observations], size, None, None
    )
    report.append(f"Fitting RMS: {rms:.6f} pixels; retained views: {len(observations)}")
    for (path, corners), rotation, translation in zip(observations, rotations, translations):
        error = rms_pixels(objects, corners, rotation, translation, matrix, distortion)
        report.append(f"FIT {error:.6f} px: {path}")
    check_count = 0
    if args.check:
        fit_files = {path.resolve() for path in paths}
        for index, path in enumerate(image_paths(args.check)):
            if path.resolve() in fit_files:
                raise ValueError("Check images must be separate from fitting images")
            gray, corners = detect(path, args.columns, args.rows)
            if gray.shape[::-1] != size:
                raise ValueError(f"Check-image dimensions differ: {path}")
            if corners is None:
                report.append(f"SKIPPED check pattern: {path}")
                continue
            success, rotation, translation = cv2.solvePnP(objects, corners, matrix, distortion)
            if not success:
                raise ValueError(f"Cannot estimate check-board pose: {path}")
            error = rms_pixels(objects, corners, rotation, translation, matrix, distortion)
            report.append(f"CHECK {error:.6f} px: {path}")
            check_count += 1
            preview(gray, corners, args.columns, args.rows, previews / f"check-{index:03d}.png")
        if check_count < 5:
            raise ValueError(f"Only {check_count} complete check patterns; supply at least 5")
    else:
        report.append("WARNING: no independent check set supplied")
    report.append("Mounting is NOMINAL ONLY; physical axis measurement is still required.")
    mounting = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], np.float64)
    write_profile(args.output, size, matrix, distortion, mounting)


def fit_mount(args, previews, report):
    size, matrix, distortion = read_lens(args.lens)
    gray, corners = detect(args.image, args.columns, args.rows)
    if gray.shape[::-1] != size:
        raise ValueError("Mount image does not match lens-profile dimensions")
    if corners is None:
        raise ValueError("No complete checker pattern; improve contrast, focus or framing")
    if args.reverse_corners:
        corners = corners[::-1].copy()
    preview(gray, corners, args.columns, args.rows, previews / "mount-order.png", numbered=True)
    board = board_points(args.columns, args.rows, args.square_mm)
    body = np.column_stack((-board[:, 1], board[:, 0], board[:, 2])).astype(np.float32)
    success, rotation, translation = cv2.solvePnP(body, corners, matrix, distortion)
    if not success:
        raise ValueError("Mounting pose did not converge")
    body_to_camera, _ = cv2.Rodrigues(rotation)
    camera_points = (body_to_camera @ body.T + translation).T
    if np.any(camera_points[:, 2] <= 0):
        raise ValueError("Board pose behind camera; check corner order")
    mounting = body_to_camera.T
    report.append(f"Mount reprojection RMS: {rms_pixels(body, corners, rotation, translation, matrix, distortion):.6f} pixels")
    report.append("ASSUMPTION: board columns point body-right; rows point toward tail; board is level with body datum.")
    report.append(f"CONFIRM preview labels 0, 1 and {args.columns} against physical board; no automatic verification.")
    report.append(f"Optical axis in body forward/right/down: {mounting[:, 2].tolist()}")
    write_profile(args.output, size, matrix, distortion, mounting)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("lens", "mount"):
        command = commands.add_parser(name)
        command.add_argument("--columns", type=int, required=True, help="inner corners per row")
        command.add_argument("--rows", type=int, required=True, help="inner-corner rows")
        command.add_argument("--square-mm", type=float, required=True, help="measured square pitch")
        command.add_argument("--output", type=Path, required=True)
        if name == "lens":
            command.add_argument("--images", type=Path, required=True)
            command.add_argument("--check", type=Path)
        else:
            command.add_argument("--image", type=Path, required=True)
            command.add_argument("--lens", type=Path, required=True)
            command.add_argument("--reverse-corners", action="store_true")
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3 or not np.isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error("Use at least 3x3 inner corners and a finite positive square pitch")
    previews = args.output.with_name(args.output.stem + "-previews")
    report_path = args.output.with_suffix(".report.txt")
    if any(path.exists() for path in (args.output, previews, report_path)):
        parser.error("Output, report or preview directory exists; choose a new output filename")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    previews.mkdir()
    report = [f"LWIR workshop calibration; OpenCV {cv2.__version__}",
              f"Pattern {args.columns}x{args.rows} inner corners, pitch {args.square_mm} mm",
              "verified=0; inspect previews and perform independent physical checks."]
    try:
        if args.command == "lens":
            fit_lens(args, previews, report)
        else:
            fit_mount(args, previews, report)
    except (ValueError, OSError, cv2.error) as error:
        report.append(f"STOP: {error}")
        report_path.write_text("\n".join(report) + "\n", encoding="utf-8")
        parser.exit(1, f"{error}\nDetails: {report_path}\nChoose a new output filename on retry.\n")
    report_path.write_text("\n".join(report) + "\n", encoding="utf-8")
    print(f"Profile: {args.output}\nReport: {report_path}\nPreviews: {previews}")
    print("Not automatically verified. Inspect the evidence before using coordinates.")


if __name__ == "__main__":
    main()