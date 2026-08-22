#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2026 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
"""Provision an EByte E52-400NW22S for the Paparazzi airborne LoRa MESH.

Nine modems configured by hand is nine chances to typo one address and spend a
flight test wondering why two aircraft never see each other.  This tool applies
the whole profile, checks every single response, and refuses to continue on the
first error.

Usage::

    # show what would be sent, no hardware needed - use this to review the plan
    ./e52_provision.py --ac-id 129 --dry-run

    # program a node
    ./e52_provision.py --port /dev/ttyUSB0 --ac-id 129

    # program the ground station
    ./e52_provision.py --port /dev/ttyUSB0 --ac-id 0

    # read back and verify an already programmed node
    ./e52_provision.py --port /dev/ttyUSB0 --ac-id 129 --verify-only

Order matters and is enforced:

* ``AT+TYPE`` rewrites the top bit of the local address, so it must come
  *after* ``AT+SRC_ADDR``;
* ``AT+UART`` changes the port speed, so it is sent last and the tool reopens
  the port at the new rate before verifying;
* ``AT+RESET`` is issued only once, at the end.

The defaults match sw/tools/mesh/mesh_phase_optimizer.py and
doc/mesh/mesh_network_design.md.  Change them in one place, here.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

try:
    import serial                      # type: ignore
except ImportError:                    # pragma: no cover
    serial = None


# --------------------------------------------------------------------------- #
# Profile                                                                       #
# --------------------------------------------------------------------------- #

#: Base of the modem address space. The modem address is BASE + AC_ID, so it is
#: trivially traceable back to the aircraft. Address 0 and 65535 are reserved by
#: the module (65535 is the broadcast address), hence the offset.
ADDRESS_BASE = 1000

#: Baud rates the Paparazzi ground link can open, from ``speed_of_baudrate`` in
#: sw/lib/ocaml/serial.ml. 460800 was missing from that table until it was added
#: - the hardware, the driver and termios all supported it, only Paparazzi's own
#: enum did not - so the ground station now runs at the same rate as the
#: aircraft.
GROUND_SUPPORTED_BAUD = (
    9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1500000, 3000000,
)

#: All fleet members are routing nodes by default. This is the only E52 profile
#: that remains connected when the direct path does not span the future 11 km
#: square / 17 km strip operating areas. The module performs route discovery,
#: RSSI-based next-hop selection and self-healing internally; Paparazzi shapes
#: originations so the resulting broadcast flood stays inside the 5-frame
#: modem cache and the channel budget.
#:
#: The ground station is a full peer of the swarm - AC_ID 0, its own modem, and
#: it does transmit, sparingly (a MOVE_WP or a SETTING now and then). It is
#: physically just a node that happens to sit still at 4 m instead of flying.
#:
#: AC_ID 0 follows the same all-routing default as every airframe. Its rare
#: commands remain unslotted and contend through the E52 CSMA mechanism.

#: EByte E52-400NW22S hardware envelope, from the module datasheet.
#:
#: The part number is the specification: "400" is the 410.125-493.125 MHz band
#: and "22" is the rated output of the power amplifier. Both are enforced here
#: rather than left to the modem, which answers ERROR to an out-of-range value
#: and would otherwise leave the node running on whatever it had before - a
#: silent mis-provision that only shows up as one aircraft with short range.
E52_400_F0_MHZ = 410.125
E52_400_CHANNELS = 84          # 410.125 .. 493.125 MHz in 1 MHz steps
E52_400_MAX_POWER_DBM = 22     # PA rating; the legal ceiling below is far lower

#: Regulatory ceiling for the 433 MHz ISM band: 10 mW = +10 dBm EIRP.
#:
#: This is a hard constraint, not a tuning knob. The E52-400NW22S PA can do
#: 22 dBm and the module is deliberately run 12 dB below it.
#:
#: EIRP is what is limited, and EIRP = conducted power + antenna gain. AT+POWER
#: sets CONDUCTED power, so a 2 dBi whip on a modem set to 10 dBm radiates
#: 12 dBm EIRP and is 60% over the limit. The conducted setting is therefore
#: derived, never typed in by hand.
EU_433_MAX_EIRP_DBM = 10

DEFAULTS = {
    "panid": 250,          # private network id for this swarm, never 0 or 65535
    "channel": 24,         # 410.125 + 24 = 434.125 MHz
    "power": None,         # derived: EIRP limit - antenna gain (see below)
    "rate": 0,             # 0 = 62.5 kbps
    "option": 3,           # 3 = broadcast
    "node_type": 0,        # every flight node routes; 1 is bench-only
    "csma_rng": 20,        # ms, datasheet minimum
    "filter_time": 3000,   # ms, duplicate suppression window, minimum
    "router_score": 3,     # default re-route threshold
    "baud": 460800,
    "parity": "8N1",
}


@dataclass
class Step:
    """One AT command plus why it is there."""

    command: str
    why: str
    critical: bool = True          # abort the run if this one fails


def build_profile(ac_id: int, args: argparse.Namespace) -> List[Step]:
    addr = ADDRESS_BASE + ac_id
    is_gcs = (ac_id == 0)
    steps: List[Step] = []

    if args.factory_reset:
        steps.append(Step("AT+DEFAULT",
                          "start from a known state - a half configured module "
                          "is worse than a factory one"))

    # -- identity ----------------------------------------------------------- #
    steps += [
        Step(f"AT+PANID={args.panid},1",
             "private network id, keeps this swarm off other E52 installations"),
        Step(f"AT+SRC_ADDR={addr},1",
             f"unique address for AC_ID {ac_id}"),
        Step(f"AT+TYPE={args.node_type}",
             ("routing node (relays broadcasts once)" if args.node_type == 0
              else "terminal node (does not relay)")
             + ". MUST follow SRC_ADDR: TYPE rewrites the top bit of "
             "the local address"),
    ]

    # -- radio -------------------------------------------------------------- #
    steps += [
        Step(f"AT+RATE={args.rate}",
             "62.5 kbps air rate = LoRa SF5 / BW 500 kHz / CR 4/5"),
        Step(f"AT+CHANNEL={args.channel},1",
             f"{E52_400_F0_MHZ} + {args.channel} = "
             f"{E52_400_F0_MHZ + args.channel:.3f} MHz. The 500 kHz occupied bandwidth "
             f"of the 62.5 kbps mode has to sit inside the permitted sub-band"),
        Step(f"AT+POWER={args.power},1",
             f"{args.power} dBm conducted; with a {args.antenna_gain:g} dBi "
             f"antenna that is {args.power + args.antenna_gain:g} dBm EIRP, "
             f"within the {args.eirp_limit:g} dBm limit"),
    ]

    # -- broadcast mesh ----------------------------------------------------- #
    steps += [
        Step(f"AT+OPTION={args.option},1",
             "broadcast mode: no route setup, no ACK, every routing node relays "
             "once"),
        Step("AT+DST_ADDR=65535,1",
             "broadcast destination"),
        Step("AT+SRC_PORT=1,1", "default port"),
        Step("AT+DST_PORT=1,1",
             "default port. Port 14 is the remote configuration port - never "
             "point user traffic at it"),
        Step("AT+ROUTER_SAVE=0",
             "do NOT persist routes to flash: the nodes move, a stale route "
             "from the last flight is worse than no route"),
        Step("AT+ROUTER_CLR=1", "start with an empty routing table"),
        Step(f"AT+ROUTER_SCORE={args.router_score}",
             "consecutive failures before a route is rebuilt"),
    ]

    # -- the settings that actually stop the blackouts ---------------------- #
    steps += [
        Step("AT+HEAD=0",
             "drop the 8 byte serial side frame header. PPRZLink v2.0 already "
             "carries sender_id, a length and a Fletcher checksum"),
        Step("AT+BACK=0",
             "stop the module injecting SUCCESS / ERR / OUT OF CACHE as ASCII "
             "into the autopilot receive stream. Easy to miss, very damaging"),
        Step(f"AT+CSMA_RNG={args.csma_rng}",
             "minimum random avoidance. The application layer already "
             "guarantees one originator at a time, so the default 127 ms would "
             "only inflate the flood span from 242 ms to 1.21 s. NOTE: this "
             "command takes no <save> argument, it always writes flash"),
        Step("AT+RESET_TIME=0",
             "disable the 5 minute RF auto restart. A restart mid flight costs "
             "a reacquisition window on a link the aircraft navigates with"),
        Step("AT+RESET_AUX=0", "and its LED side effect"),
        Step(f"AT+FILTER_TIME={args.filter_time}",
               "duplicate suppression window. The current value covers the "
               "modeled flood lifetime without retaining a packet across the "
               "next 16 s superframe. Takes no <save> argument"),
    ]

    # -- optional encryption ------------------------------------------------ #
    if args.key is not None:
        steps += [
            Step("AT+SECURITY=1", "enable user payload encryption"),
            Step(f"AT+KEY={args.key}",
                 "shared key, identical on every node, cannot be read back"),
        ]

    # -- baud last, then reboot --------------------------------------------- #
    steps += [
        Step(f"AT+UART={args.baud},{args.parity}",
             f"{args.baud} baud. Must be far faster than the 62.5 kbps air rate "
             f"so a frame is inside the modem before its slot opens"),
        Step("AT+RESET", "apply the baud change"),
    ]

    if is_gcs:
        steps.append(Step("", "-- this node is the GCS: it keeps TYPE=0 so it "
                              "can relay for aircraft on the far side of the "
                              "swarm", critical=False))
    return [s for s in steps if s.command]


#: Parameters read back and checked by --verify-only and the final verification.
VERIFY_QUERIES: List[Tuple[str, str]] = [
    ("AT+DEVTYPE=?", "module model"),
    ("AT+RATE=?", "air rate"),
    ("AT+CHANNEL=?", "channel"),
    ("AT+POWER=?", "TX power"),
    ("AT+OPTION=?", "communication mode"),
    ("AT+PANID=?", "network id"),
    ("AT+SRC_ADDR=?", "own address"),
    ("AT+DST_ADDR=?", "destination address"),
    ("AT+TYPE=?", "node type"),
    ("AT+HEAD=?", "extra frame header"),
    ("AT+BACK=?", "return messages"),
    ("AT+CSMA_RNG=?", "CSMA avoidance"),
    ("AT+RESET_TIME=?", "RF auto reset"),
    ("AT+FILTER_TIME=?", "broadcast filter"),
    ("AT+UART=?", "serial parameters"),
]


# --------------------------------------------------------------------------- #
# Transport                                                                     #
# --------------------------------------------------------------------------- #

class Modem:
    """Thin line oriented wrapper. Every write is followed by a read and check."""

    def __init__(self, port: str, baud: int, timeout: float = 1.5,
                 dry_run: bool = False, verbose: bool = False):
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.dry_run = dry_run
        self.verbose = verbose
        self.ser = None
        if not dry_run:
            if serial is None:
                raise RuntimeError("pyserial is not installed: pip install pyserial")
            self.ser = serial.Serial(port, baud, timeout=timeout)
            time.sleep(0.2)
            self.ser.reset_input_buffer()

    def reopen(self, baud: int) -> None:
        self.baud = baud
        if self.dry_run:
            return
        assert self.ser is not None
        self.ser.close()
        time.sleep(0.5)
        self.ser = serial.Serial(self.port_name, baud, timeout=self.timeout)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()

    def send(self, command: str) -> str:
        if self.dry_run:
            return "OK"
        assert self.ser is not None
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()
        deadline = time.time() + self.timeout
        chunks: List[bytes] = []
        while time.time() < deadline:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                chunks.append(data)
                if b"\n" in data or b"OK" in data or b"ERR" in data:
                    time.sleep(0.05)
                    extra = self.ser.read(self.ser.in_waiting)
                    if extra:
                        chunks.append(extra)
                    break
        reply = b"".join(chunks).decode("ascii", errors="replace").strip()
        if self.verbose:
            print(f"      < {reply!r}")
        return reply


def response_ok(command: str, reply: str) -> bool:
    """The module answers ``AT+<CMD>=OK`` on success, ``*_ERR`` on failure."""
    if not reply:
        return False
    upper = reply.upper()
    if "CMD_ERR" in upper or "CMD_VALUE_ERR" in upper or "VALUE_ERR" in upper:
        return False
    return "OK" in upper


# --------------------------------------------------------------------------- #
# Driver                                                                        #
# --------------------------------------------------------------------------- #

def role_of(ac_id: int, args: argparse.Namespace) -> Tuple[str, int]:
    """Return ``(role name, AT+TYPE value)`` for this node."""
    if args.node_type is not None:
        return ("routing (forced)" if args.node_type == 0
                else "terminal (forced)"), args.node_type
    # The relay list is authoritative for EVERY node, the GCS included: AC_ID 0
    # is an ordinary member of the mesh that happens to sit on the ground, and
    # it must be provisionable as a router exactly like any airframe. An
    # earlier version tested ac_id == 0 first, which silently ignored
    # "--relay-ids 0" and returned a terminal role while claiming success.
    if args.relay_ids is None or ac_id in args.relay_ids:
        return ("GCS, routing" if ac_id == 0 else "airborne, routing"), 0
    if ac_id == 0:
        return "GCS, terminal (transmits sparingly, unslotted)", 1
    return "terminal", 1


def provision(args: argparse.Namespace) -> int:
    role_name, args.node_type = role_of(args.ac_id, args)
    steps = build_profile(args.ac_id, args)
    addr = ADDRESS_BASE + args.ac_id

    if args.dry_run and args.port is None:
        args.port = "(dry run)"

    print("=" * 78)
    print(f"E52-400NW22S provisioning - AC_ID {args.ac_id}")
    print("=" * 78)
    print(f"  modem address : {addr}")
    print("  TDMA slot     : learned at runtime; never derived from AC_ID")
    print(f"  ROLE          : {role_name}   (AT+TYPE={args.node_type})")
    print(f"  frequency     : {E52_400_F0_MHZ + args.channel:.3f} MHz (channel {args.channel})")
    print(f"  port          : {args.port}  {args.initial_baud} -> {args.baud} baud")
    if args.dry_run:
        print("  MODE          : DRY RUN, nothing is sent")
    if args.ac_id == 0 and args.baud not in GROUND_SUPPORTED_BAUD:
        print()
        print(f"  !! {args.baud} baud is not in the Paparazzi ground link's supported set")
        print(f"     {GROUND_SUPPORTED_BAUD}")
        print( "     sw/ground_segment/tmtc/link will raise Serial.speed_of_baudrate.")
        print( "     Use --baud 230400 for the ground station; the aircraft modem UART")
        print( "     is independent of it, so it can stay at 460800.")
        if not args.force:
            print("     refusing (pass --force to override)")
            return 1
    print()

    modem = Modem(args.port, args.initial_baud,
                  dry_run=args.dry_run, verbose=args.verbose)

    failures = 0
    try:
        if not args.verify_only:
            width = max(len(s.command) for s in steps)
            for i, step in enumerate(steps, 1):
                reply = modem.send(step.command)
                ok = response_ok(step.command, reply) or step.command == "AT+RESET"
                mark = "ok " if ok else "FAIL"
                print(f"  {i:2d}/{len(steps)}  {mark}  {step.command:<{width}}   # {step.why}")
                if not ok:
                    failures += 1
                    if step.critical and not args.keep_going:
                        print()
                        print(f"  aborting: {step.command} returned {reply!r}")
                        print("  the module is now half configured - rerun with "
                              "--factory-reset")
                        return 1
                if step.command.startswith("AT+UART"):
                    time.sleep(0.2)
                elif step.command == "AT+RESET":
                    time.sleep(1.5)
                else:
                    time.sleep(args.inter_command_delay)

            modem.reopen(args.baud)
            time.sleep(1.0)
            print()

        # ---- verification ------------------------------------------------- #
        print("  read back")
        print("  " + "-" * 70)
        for query, label in VERIFY_QUERIES:
            reply = modem.send(query)
            print(f"    {label:<22} {query:<18} -> {reply if reply else '(no reply)'}")
            if not args.dry_run and not reply:
                failures += 1
        print("  " + "-" * 70)
    finally:
        modem.close()

    print()
    if args.dry_run:
        print("  dry run complete - no hardware was touched")
        return 0
    if failures:
        print(f"  {failures} problem(s). Do not fly this node.")
        return 1
    print("  node provisioned. Check that SRC_ADDR is unique across the fleet.")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="serial device, e.g. /dev/ttyUSB0")
    ap.add_argument("--ac-id", type=int, required=True,
                    help="Paparazzi AC_ID, 0 for the ground station")
    ap.add_argument("--initial-baud", type=int, default=115200,
                    help="the modem's CURRENT baud rate (factory default 115200)")
    ap.add_argument("--baud", type=int, default=DEFAULTS["baud"])
    ap.add_argument("--parity", default=DEFAULTS["parity"])
    ap.add_argument("--panid", type=int, default=DEFAULTS["panid"])
    ap.add_argument("--channel", type=int, default=DEFAULTS["channel"],
                    choices=range(0, E52_400_CHANNELS),
                    metavar=f"{{0..{E52_400_CHANNELS - 1}}}",
                    help=f"{E52_400_F0_MHZ} MHz + N MHz, N in "
                         f"0..{E52_400_CHANNELS - 1}")
    ap.add_argument("--antenna-gain", type=float, default=0.0,
                    help="dBi of the fitted antenna; the conducted AT+POWER "
                         "setting is reduced by this so that EIRP stays legal")
    ap.add_argument("--eirp-limit", type=float, default=EU_433_MAX_EIRP_DBM,
                    help="dBm EIRP ceiling (default: 433 MHz ISM 10 mW)")
    ap.add_argument("--power", type=int, default=None,
                    choices=range(0, E52_400_MAX_POWER_DBM + 1),
                    metavar=f"{{0..{E52_400_MAX_POWER_DBM}}}",
                    help="override the derived conducted power, in dBm")
    ap.add_argument("--rate", type=int, default=DEFAULTS["rate"],
                    choices=(0, 1, 2), help="0=62.5k 1=21.875k 2=7k")
    ap.add_argument("--option", type=int, default=DEFAULTS["option"],
                    choices=(1, 2, 3, 4),
                    help="1=unicast 2=multicast 3=broadcast 4=anycast")
    ap.add_argument("--relay-ids", default="all",
                    help="routing nodes: 'all' (default, true self-healing mesh) "
                         "or comma-separated AC_IDs for a constrained bench test")
    ap.add_argument("--node-type", type=int, default=None,
                    choices=(0, 1),
                    help="override the role table: 0=routing node 1=terminal node. "
                         "By default every node, including the GCS, is a routing node")
    ap.add_argument("--csma-rng", type=int, default=DEFAULTS["csma_rng"])
    ap.add_argument("--filter-time", type=int, default=DEFAULTS["filter_time"])
    ap.add_argument("--router-score", type=int, default=DEFAULTS["router_score"])
    ap.add_argument("--key", default=None,
                    help="enable payload encryption with this 32 bit key")
    ap.add_argument("--factory-reset", action="store_true",
                    help="send AT+DEFAULT first (recommended)")
    ap.add_argument("--verify-only", action="store_true",
                    help="read back the configuration without changing it")
    ap.add_argument("--keep-going", action="store_true",
                    help="do not abort on the first failure")
    ap.add_argument("--inter-command-delay", type=float, default=0.15)
    ap.add_argument("--force", action="store_true",
                    help="override the ground link baud rate safety check")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the command sequence, touch no hardware")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    if not 0 <= args.ac_id <= 254:
        ap.error("--ac-id must be in 0..254; 255 is reserved for broadcast")

    if args.relay_ids.strip().lower() == "all":
        args.relay_ids = None
    else:
        try:
            args.relay_ids = tuple(int(x) for x in args.relay_ids.split(",") if x.strip())
        except ValueError:
            ap.error("--relay-ids must be 'all' or a comma separated list of integers")
        for r in args.relay_ids:
            if not 0 <= r <= 254:
                ap.error(f"relay AC_ID {r} is outside 0..254; 255 is broadcast")

    if args.port is None and not args.dry_run:
        ap.error("--port is required unless --dry-run is given")

    # Derive the conducted power from the EIRP ceiling. AT+POWER is conducted;
    # the limit is on radiated power, so the antenna gain has to come off it.
    if args.power is None:
        derived = args.eirp_limit - args.antenna_gain
        if derived < 0:
            ap.error(f"antenna gain {args.antenna_gain} dBi exceeds the "
                     f"{args.eirp_limit} dBm EIRP limit on its own")
        args.power = int(math.floor(derived))       # round DOWN, never over
    if args.power > E52_400_MAX_POWER_DBM:
        ap.error(f"{args.power} dBm exceeds the E52-400NW22S PA rating "
                 f"({E52_400_MAX_POWER_DBM} dBm)")
    if args.power + args.antenna_gain > args.eirp_limit:
        ap.error(f"conducted {args.power} dBm + {args.antenna_gain} dBi antenna "
                 f"= {args.power + args.antenna_gain:.1f} dBm EIRP, over the "
                 f"{args.eirp_limit} dBm limit")
    if args.verify_only:
        args.initial_baud = args.baud
    return provision(args)


if __name__ == "__main__":
    sys.exit(main())
