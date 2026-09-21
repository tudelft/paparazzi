#!/usr/bin/env python3
"""Drive CATIA over the local PTY bridge: one targeted AICam shot.

Sends a single CATIA_SHOOT_TARGETED frame selecting the AI camera (camera id 2), then
prints any bytes CATIA sends back for a short window. Unlike EARcam's CATIA_EAR_RESULT,
there is no dedicated reply message for an AICam shot -- success is a photo file plus the
console line CATIA prints (and, for --aicam-detect, the vehicle-detection EXIF result);
this script only exercises the trigger path, matching earcam_desk_test.py's role for
desk tests without a flight controller.

Requires CATIA already running with --local (so /tmp/catia-sim exists) and either
--aicam or --aicam-detect selected.
"""
import argparse
import struct
import sys
import time

import serial  # pyserial

STX = 0x99
CATIA_SHOOT_TARGETED = 5
CATIA_CAMERA_AICAM = 2


def frame(msg_id: int, payload: bytes) -> bytes:
    length = len(payload) + 5
    ck_a = length & 0xFF
    ck_b = length & 0xFF
    body = bytes([msg_id]) + payload
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([STX, length]) + body + bytes([ck_a, ck_b])


def shot_payload(nr, lat, lon, alt_m, agl_m, camera_id):
    return struct.pack(
        "<11i",
        nr,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt_m * 1000),
        0, 0, 0,
        int(10.0 * 524288),
        0,
        int(agl_m * 256),
        camera_id,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/tmp/catia-sim")
    parser.add_argument("--shots", type=int, default=1)
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--lat", type=float, default=43.4630000)
    parser.add_argument("--lon", type=float, default=1.2730000)
    parser.add_argument("--alt", type=float, default=272.0)
    parser.add_argument("--agl", type=float, default=12.0)
    args = parser.parse_args()

    port = serial.Serial(args.port, 115200, timeout=0.2)
    for nr in range(1, args.shots + 1):
        port.write(frame(CATIA_SHOOT_TARGETED,
                         shot_payload(nr, args.lat, args.lon, args.alt, args.agl, CATIA_CAMERA_AICAM)))
        print(f"sent CATIA_SHOOT_TARGETED nr={nr} camera_id={CATIA_CAMERA_AICAM} (aicam)")
        time.sleep(args.period)

    deadline = time.monotonic() + 2.0
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            buf.extend(chunk)
    if buf:
        print(f"received {len(buf)} bytes back: {buf.hex()}")
    else:
        print("no bytes received back (status telemetry may be off by default)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
