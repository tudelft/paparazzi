#!/usr/bin/env python3
"""Persistent on-sensor vehicle-detection capture server for CATIA's aicam-detect backend.

Started once by CATIA (vehicle_detect_pipe.c) and kept running for the whole flight/desk
test, exactly like the existing LWIRcam capture-server pattern -- the IMX500 network
firmware upload (17-60s) only has to happen once, not per shot.

Line protocol on stdin/stdout, mirroring lwir_cam_pipe.c's LWIR_SERVER_* protocol:
  startup:  prints "AICAM_SERVER_READY" once the model is loaded and the camera streaming
  request:  CATIA writes "<absolute output path>\n" to stdin
  response: "AICAM_SERVER_OK count=<n> conf=<c> box_x=<x> box_y=<y> box_w=<w> box_h=<h>\n"
            (count=0/conf=0.0000/box=0,0,0,0 when no vehicle was seen this shot)
            or "AICAM_SERVER_ERROR <message>\n" (single line, no embedded newlines)

All informational/debug logging goes to stderr; ONLY the lines above ever go to stdout,
and every stdout write is flushed immediately -- CATIA's C side reads stdout through a
pipe, not a tty, so unflushed output would simply never arrive.

This is the live-capture sibling of vehicle_detect_live.py (the standalone desk-test
script): same model-loading/detection logic, but request-response instead of a free-running
loop, and it always saves a frame (with a box drawn only when a vehicle was found) because
CATIA needs exactly one output file per shot regardless of the detection outcome.

The stdout reply only ever carries the single best-confidence box (CATIA's protocol has
room for one). Every qualifying box for a shot -- not just the best one -- is additionally
appended as its own row to a detections CSV (see append_detection_rows()), one row per
vehicle, so a shot with 3 trucks produces 3 rows sharing that image's filename.
"""
import argparse
import csv
import os
import sys
import time

from PIL import ImageDraw, ImageFont
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics


def log(message):
    print(message, file=sys.stderr, flush=True)


def reply(line):
    print(line, flush=True)


def get_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Path to the packaged .rpk network")
    parser.add_argument("--labels", required=True, help="Path to the labels file")
    parser.add_argument("--threshold", type=float, default=0.6, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="IoU threshold (kept for parity; NMS is baked on-chip)")
    parser.add_argument("--max-detections", type=int, default=5, help="Cap on detections per frame")
    parser.add_argument("--main-size", default="1536x1152", help="WxH of the saved main stream")
    parser.add_argument("--frame-timeout", type=float, default=2.0,
                         help="Max seconds to wait for a fresh output tensor per request")
    parser.add_argument("--csv", default=None,
                         help="Path to the detections CSV (default: detections.csv next to each saved image)")
    return parser.parse_args()


class Detection:
    def __init__(self, coords, category, conf, metadata, imx500, picam2):
        """Map raw network coords to real image pixels via the ISP/ScalerCrop-aware helper.

        This is a genuine capture (unlike the tensor-injection debug harness), so
        imx500.convert_inference_coords() is required and correct here.
        """
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)


def extract_vehicle_hits(np_outputs, metadata, imx500, picam2, intrinsics, vehicle_idx, threshold, max_detections):
    """Turn one already-fetched output tensor into Detection objects for the vehicle class.

    np_outputs must come from a single imx500.get_outputs() call by the caller --
    calling get_outputs() a second time for the same metadata silently returns None.
    """
    input_w, input_h = imx500.get_input_size()
    boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]

    if intrinsics.bbox_normalization:
        boxes = boxes / input_h
    if intrinsics.bbox_order == "xy":
        boxes = boxes[:, [1, 0, 3, 2]]

    hits = []
    for box, score, category in zip(boxes, scores, classes):
        if score > threshold and int(category) == vehicle_idx:
            hits.append(Detection(box, category, score, metadata, imx500, picam2))
        if len(hits) >= max_detections:
            break
    return hits


def load_label_font():
    try:
        return ImageFont.load_default(size=32)
    except TypeError:
        return ImageFont.load_default()  # older Pillow without the size= param


def draw_hits(image, hits, font):
    draw = ImageDraw.Draw(image)
    for d in hits:
        x, y, w, h = d.box
        draw.rectangle([x, y, x + w, y + h], outline=(0, 255, 0), width=4)
        label = f"vehicle {d.conf:.2f}"
        label_origin = (x, max(0, y - 34))
        label_box = draw.textbbox(label_origin, label, font=font)
        draw.rectangle(label_box, fill=(0, 255, 0))
        draw.text(label_origin, label, fill=(0, 0, 0), font=font)


DETECTION_CSV_HEADER = ["image", "vehicle_index", "confidence", "box_x", "box_y", "box_w", "box_h"]


def append_detection_rows(csv_path, image_filename, hits):
    """Append one row per detected vehicle in this shot to the shared detections CSV.

    Rows accumulate across the whole run (append mode); the header is written once, the
    first time csv_path doesn't exist yet. box_x/box_y/box_w/box_h are the same pixel
    coordinates drawn on the saved image, so a row can be cross-checked against the JPEG
    by eye.
    """
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(DETECTION_CSV_HEADER)
        for index, hit in enumerate(hits, start=1):
            x, y, w, h = hit.box
            writer.writerow([os.path.basename(image_filename), index, f"{hit.conf:.4f}", x, y, w, h])


def capture_once(picam2, imx500, intrinsics, vehicle_idx, args, deadline):
    """Grab frames until one yields a fresh output tensor, or the deadline passes.

    Returns (image, hits). image is always the most recently captured frame (even if
    no fresh tensor was ever seen before the deadline, so CATIA still gets a file);
    hits is [] unless a fresh tensor was read and it contained qualifying detections.
    """
    image = None
    hits = []
    while time.monotonic() < deadline:
        request = picam2.capture_request()
        try:
            if imx500.get_input_size() == (0, 0):
                continue  # transient post-event race noted in the injection harness; retry
            metadata = request.get_metadata()
            np_outputs = imx500.get_outputs(metadata, add_batch=True)
            image = request.make_image("main")
            if np_outputs is not None:
                hits = extract_vehicle_hits(np_outputs, metadata, imx500, picam2, intrinsics,
                                             vehicle_idx, args.threshold, args.max_detections)
                break
        finally:
            request.release()
    return image, hits


def main():
    args = get_args()

    imx500 = IMX500(args.model)  # NOT tensor_injection=True -- live capture, not debug
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    elif intrinsics.task != "object detection":
        log(f"AICAM_SERVER: packaged network task is {intrinsics.task!r}, not 'object detection'")
        sys.exit(1)

    with open(args.labels) as f:
        intrinsics.labels = [line.strip() for line in f if line.strip()]

    # Both facts confirmed against this exact model export via the tensor-injection
    # harness (Notes/Mission 1/IMAV input tensor injection.md): NMSWrapper emits
    # (x_min, y_min, x_max, y_max) -- "xy" -- and raw boxes are absolute pixel coords
    # within the 320x320 input tensor, not pre-normalized. bbox_normalization=True is
    # what makes the code below divide by input_h before the pixel-space conversion.
    intrinsics.bbox_order = "xy"
    intrinsics.bbox_normalization = True
    intrinsics.iou_threshold = args.iou
    intrinsics.max_detections = args.max_detections
    intrinsics.update_with_defaults()

    if intrinsics.postprocess:
        log(f"AICAM_SERVER: unexpected postprocess mode {intrinsics.postprocess!r}")
        sys.exit(1)

    if "vehicle" not in intrinsics.labels:
        log(f"AICAM_SERVER: 'vehicle' not found in labels: {intrinsics.labels}")
        sys.exit(1)
    vehicle_idx = intrinsics.labels.index("vehicle")

    picam2 = Picamera2(imx500.camera_num)
    width, height = (int(v) for v in args.main_size.split("x"))
    config = picam2.create_preview_configuration(
        main={"size": (width, height)},
        controls={"FrameRate": intrinsics.inference_rate or 10},
        buffer_count=4,
    )

    log("AICAM_SERVER: uploading network firmware to the IMX500 sensor -- observed 17s "
        "clean, up to 48-60s if the rp2040-gpio-bridge chatter is active. Once for the "
        "whole run, not per shot.")
    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=False)

    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()

    label_font = load_label_font()
    log(f"AICAM_SERVER: ready (labels={intrinsics.labels}, vehicle_idx={vehicle_idx}, "
        f"threshold={args.threshold})")
    reply("AICAM_SERVER_READY")

    try:
        for raw_line in sys.stdin:
            filename = raw_line.rstrip("\n")
            if not filename:
                continue
            try:
                deadline = time.monotonic() + args.frame_timeout
                image, hits = capture_once(picam2, imx500, intrinsics, vehicle_idx, args, deadline)
                if image is None:
                    reply("AICAM_SERVER_ERROR no frame available")
                    log(f"AICAM_SERVER: no frame available for {filename}")
                    continue

                if hits:
                    draw_hits(image, hits, label_font)
                image.save(filename)

                if hits:
                    csv_path = args.csv or os.path.join(os.path.dirname(filename) or ".", "detections.csv")
                    try:
                        append_detection_rows(csv_path, filename, hits)
                    except OSError as exc:
                        log(f"AICAM_SERVER: failed to write {csv_path}: {exc!r}")

                    best = max(hits, key=lambda d: d.conf)
                    x, y, w, h = best.box
                    reply(f"AICAM_SERVER_OK count={len(hits)} conf={best.conf:.4f} "
                          f"box_x={x} box_y={y} box_w={w} box_h={h}")
                    log(f"AICAM_SERVER: saved {filename} vehicle conf={best.conf:.2f} box={best.box}")
                else:
                    reply("AICAM_SERVER_OK count=0 conf=0.0000 box_x=0 box_y=0 box_w=0 box_h=0")
                    log(f"AICAM_SERVER: saved {filename} (no detection)")
            except Exception as exc:  # keep serving future shots after one bad request
                log(f"AICAM_SERVER: error handling request for {filename!r}: {exc!r}")
                reply(f"AICAM_SERVER_ERROR {exc}")
    finally:
        log("AICAM_SERVER: shutting down")
        picam2.stop()
        picam2.close()


if __name__ == "__main__":
    main()
