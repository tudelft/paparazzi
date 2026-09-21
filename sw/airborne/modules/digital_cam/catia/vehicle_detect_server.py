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
loop, and it always saves a frame because CATIA needs exactly one output file per shot
regardless of the detection outcome.

The saved frame is always the plain, undecorated capture -- no box is ever drawn here, so
every photo under the configured photo directory is a clean flight-record image regardless
of whether a vehicle was seen. The stdout reply only ever carries the single best-confidence
box (CATIA's protocol has room for one). Everything past detection -- drawing the box,
saving an annotated copy and a per-vehicle crop into their own review folders, recording a
bbox CSV -- is handled downstream by SODA, not here.

Defaults to the IMX500's 2028x1520 binned sensor mode rather than its full 4056x3040
resolution, because MORA's CMA pool cannot fit the full-resolution buffers: a 4056x3040
XBGR8888 main stream costs ~49MB per buffer, so the previous 4056x3040/buffer_count=4
default needed ~197MB against a 256MB CmaTotal that only ever has ~90-120MB free, and
picamera2 died at startup with "OSError: [Errno 12] Cannot allocate memory". Measured on
the board: 4056x3040 fits only at buffer_count<=2 (leaving ~20MB CMA headroom, too tight
to run alongside catia and SODA), while 2028x1520 at buffer_count=3 leaves ~90MB free.
Detection quality is unaffected -- the on-sensor network always sees its own 320x320 input
tensor, and the binned mode keeps the full field of view -- and crops still carry 4x the
pixels of the old 1536x1152 default. Pass --main-size 4056x3040 --buffer-count 2 to trade
that headroom back for maximum crop detail.

--fps explicitly caps the FrameRate control below the sensor's ceiling for the selected
mode (~10fps at full resolution): requesting a higher rate makes libcamera pick a smaller,
binned sensor mode, silently defeating --main-size.
"""
import argparse
import os
import sys
import time

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
    parser.add_argument("--main-size", default="2028x1520",
                         help="WxH of the saved main stream (default: the IMX500's binned sensor "
                              "mode, the largest that fits MORA's CMA pool -- see the module "
                              "docstring for the measured numbers)")
    parser.add_argument("--buffer-count", type=int, default=3,
                         help="Camera buffers to allocate. Each costs width*height*4 bytes of "
                              "CMA, so this trades capture smoothness against the CMA ceiling "
                              "that --main-size is bounded by")
    parser.add_argument("--fps", type=int, default=8,
                         help="FrameRate control for the main stream. Must stay explicitly below "
                              "the sensor's full-resolution ceiling (~10fps per the rpicam docs) "
                              "-- otherwise libcamera silently falls back to a smaller, binned "
                              "sensor mode that can sustain a higher rate, defeating --main-size")
    parser.add_argument("--frame-timeout", type=float, default=2.0,
                         help="Max seconds to wait for a fresh output tensor per request")
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
    # args.fps (not intrinsics.inference_rate) drives the main stream's FrameRate control:
    # the packaged network's own inference_rate is tuned for its on-chip detection tap, not
    # for what the host-side "main" JPEG stream can sustain at full sensor resolution, and
    # requesting a rate the full-resolution mode can't hit makes libcamera silently pick a
    # smaller, binned sensor mode instead (see Notes/Mission 1/Vehicle Detection.md).
    config = picam2.create_preview_configuration(
        main={"size": (width, height)},
        controls={"FrameRate": args.fps},
        buffer_count=args.buffer_count,
    )

    log("AICAM_SERVER: uploading network firmware to the IMX500 sensor -- observed 17s "
        "clean, up to 48-60s if the rp2040-gpio-bridge chatter is active. Once for the "
        "whole run, not per shot.")
    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=False)

    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()

    log(f"AICAM_SERVER: ready (labels={intrinsics.labels}, vehicle_idx={vehicle_idx}, "
        f"threshold={args.threshold}, main_size={width}x{height}, fps={args.fps}, "
        f"buffer_count={args.buffer_count}, "
        f"main_stream_cma={width * height * 4 * args.buffer_count / 1e6:.0f}MB)")
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

                image.save(filename)

                if hits:
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
