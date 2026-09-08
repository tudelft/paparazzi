#!/usr/bin/env python3
"""Drive CATIA over the local PTY bridge: targeted EARcam shoots, then a stop.

Prints the decoded MORA_EAR_RESULT. Used for desk tests without a flight controller.
"""
import argparse
import struct
import sys
import time

import serial  # pyserial

STX = 0x99
MORA_SHOOT_TARGETED = 5
MORA_STOP_TARGETED = 6
MORA_EAR_RESULT = 7
MORA_CAMERA_EARCAM = 4


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


def read_result(port, timeout_s):
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(64)
        if chunk:
            buf.extend(chunk)
        while len(buf) >= 5:
            if buf[0] != STX:
                del buf[0]
                continue
            length = buf[1]
            if len(buf) < length:
                break
            msg = bytes(buf[:length])
            del buf[:length]
            if msg[2] == MORA_EAR_RESULT and length == 32 + 5:
                fields = struct.unpack("<8i", msg[3:3 + 32])
                return fields
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/tmp/catia-sim")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--period", type=float, default=0.05)
    args = parser.parse_args()

    port = serial.Serial(args.port, 115200, timeout=0.05)
    lat0, lon0 = 43.4630000, 1.2730000
    per_strip = args.samples // 4
    nr = 1
    for strip in range(4):
        north = strip * 8.0
        for step in range(per_strip):
            east = step * 0.5 if strip % 2 == 0 else 25.0 - step * 0.5
            lat = lat0 + north / 6378137.0 * 57.29577951308232
            lon = lon0 + east / (6378137.0 * 0.7261) * 57.29577951308232
            port.write(frame(MORA_SHOOT_TARGETED,
                             shot_payload(nr, lat, lon, 272.0, 12.0, MORA_CAMERA_EARCAM)))
            nr += 1
            time.sleep(args.period)
    port.write(frame(MORA_STOP_TARGETED, struct.pack("<i", MORA_CAMERA_EARCAM)))
    result = read_result(port, 5.0)
    if result is None:
        print("no MORA_EAR_RESULT received", file=sys.stderr)
        return 1
    status, lat, lon, agl_mm, alt_mm, level_cdb, conf, n = result
    print(f"EAR_RESULT status={status} lat={lat/1e7:.7f} lon={lon/1e7:.7f} "
          f"agl={agl_mm/1000:.1f} alt={alt_mm/1000:.1f} level={level_cdb/100:.1f} dB "
          f"conf={conf/1000:.2f} samples={n}")
    return 0 if status == 1 else 2


if __name__ == "__main__":
    sys.exit(main())
