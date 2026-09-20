#!/usr/bin/env python3
"""Persistent on-sensor visual-homing capture server for CATIA's aicam-home backend.

Started once by CATIA (home_vector_pipe.c) and kept running for the whole flight/desk
test, exactly like the existing vehicle-detection capture-server pattern
(vehicle_detect_server.py) -- the IMX500 network firmware upload (17-60s) only has to
happen once, not per shot.

Line protocol on stdin/stdout, mirroring vehicle_detect_server.py's AICAM_SERVER_* protocol:
  startup:  prints "HOMEVEC_SERVER_READY" once the model is loaded and the camera streaming
  request:  CATIA writes "<absolute output path>\n" to stdin
  response: "HOMEVEC_SERVER_OK dx=<f> dy=<f> dist=<f>\n"
            (dx, dy: body-frame direction-to-home unit vector, L2-normalized here in Python
             since the on-chip network can't do that op on a 2-element vector -- see
             escnn_project/rpi_ai_cam_deploy/infer_and_log.py:_decode_output. dist: predicted
             distance to home, in whatever units the deployed checkpoint's label normalization
             used -- home_vector_ctrl.c on the flight-controller side treats it as-is.)
            or "HOMEVEC_SERVER_ERROR <message>\n" (single line, no embedded newlines)

All informational/debug logging goes to stderr; ONLY the lines above ever go to stdout,
and every stdout write is flushed immediately -- CATIA's C side reads stdout through a
pipe, not a tty, so unflushed output would simply never arrive.

Unlike vehicle detection (which only writes its result into the JPEG's EXIF), every shot's
prediction here is forwarded by CATIA over the UART link to the flight controller
(CATIA_HOME_VECTOR_RESULT) on top of being retrievable from the saved JPEG later -- this
server stays deliberately dumb (decode + normalize only, no world-frame rotation, no
telemetry dependency): the flight controller already knows its own current heading more
precisely and more freshly than anything MORA could supply, so body->world rotation happens
there (home_vector_ctrl.c), not here.
"""
import argparse
import math
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
    parser.add_argument("--main-size", default="1536x1152", help="WxH of the saved main stream")
    parser.add_argument("--frame-timeout", type=float, default=2.0,
                         help="Max seconds to wait for a fresh output tensor per request")
    return parser.parse_args()


def decode_output(raw_output):
    """raw_output: the model's raw (dx, dy, dist) triple (models.py raw_output=True
    convention -- see escnn_project/my_escnn_geotiff_pipeline/models.py:486 SimpleCNN).
    Normalizes the direction in Python, matching infer_and_log.py:_decode_output exactly."""
    dx, dy, dist = float(raw_output[0]), float(raw_output[1]), float(raw_output[2])
    norm = math.hypot(dx, dy)
    if norm < 1e-8:
        return 0.0, 0.0, dist
    return dx / norm, dy / norm, dist


def capture_once(picam2, imx500, args, deadline):
    """Grab frames until one yields a fresh output tensor, or the deadline passes.

    Returns (image, prediction). image is always the most recently captured frame (even
    if no fresh tensor was ever seen before the deadline, so CATIA still gets a file);
    prediction is None unless a fresh tensor was actually read.
    """
    image = None
    prediction = None
    while time.monotonic() < deadline:
        request = picam2.capture_request()
        try:
            if imx500.get_input_size() == (0, 0):
                continue  # transient post-event race, same as vehicle_detect_server.py; retry
            metadata = request.get_metadata()
            outputs = imx500.get_outputs(metadata, add_batch=True)
            image = request.make_image("main")
            if outputs is not None:
                prediction = decode_output(outputs[0].flatten())
                break
        finally:
            request.release()
    return image, prediction


def main():
    args = get_args()

    imx500 = IMX500(args.model)  # NOT tensor_injection=True -- live capture, not debug
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
    intrinsics.update_with_defaults()

    picam2 = Picamera2(imx500.camera_num)
    width, height = (int(v) for v in args.main_size.split("x"))
    config = picam2.create_preview_configuration(
        main={"size": (width, height)},
        controls={"FrameRate": intrinsics.inference_rate or 10},
        buffer_count=4,
    )

    log("HOMEVEC_SERVER: uploading network firmware to the IMX500 sensor -- observed 17s "
        "clean, up to 48-60s if the rp2040-gpio-bridge chatter is active. Once for the "
        "whole run, not per shot.")
    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=False)

    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()

    log("HOMEVEC_SERVER: ready")
    reply("HOMEVEC_SERVER_READY")

    try:
        for raw_line in sys.stdin:
            filename = raw_line.rstrip("\n")
            if not filename:
                continue
            try:
                deadline = time.monotonic() + args.frame_timeout
                image, prediction = capture_once(picam2, imx500, args, deadline)
                if image is None:
                    reply("HOMEVEC_SERVER_ERROR no frame available")
                    log(f"HOMEVEC_SERVER: no frame available for {filename}")
                    continue

                image.save(filename)

                if prediction is not None:
                    dx, dy, dist = prediction
                    reply(f"HOMEVEC_SERVER_OK dx={dx:.6f} dy={dy:.6f} dist={dist:.6f}")
                    log(f"HOMEVEC_SERVER: saved {filename} dx={dx:.3f} dy={dy:.3f} dist={dist:.3f}")
                else:
                    reply("HOMEVEC_SERVER_ERROR no output tensor before timeout")
                    log(f"HOMEVEC_SERVER: saved {filename} but no output tensor arrived in time")
            except Exception as exc:  # keep serving future shots after one bad request
                log(f"HOMEVEC_SERVER: error handling request for {filename!r}: {exc!r}")
                reply(f"HOMEVEC_SERVER_ERROR {exc}")
    finally:
        log("HOMEVEC_SERVER: shutting down")
        picam2.stop()
        picam2.close()


if __name__ == "__main__":
    main()
