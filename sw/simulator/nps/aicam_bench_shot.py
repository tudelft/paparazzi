#!/usr/bin/env python3
"""Headless trigger for Mission 1's Test_AIcam_Shot bench block.

Jumps the given aircraft straight to Test_AIcam_Shot in the Adam flight plan
(conf/flight_plans/TUDELFT/tudelft_imav2026_strasbourg.xml -> included
tudelft_include_imav2026_mission1.xml), which sends one DC_SHOOT (AICam mask
only) per jump and then holds -- "one AI camera shot per click" per that
block's own comment. Repeats --count times, --period apart, so a toy vehicle
can be presented to the camera between shots.

This connects to an ALREADY-RUNNING Simulator/Server/Data Link session (e.g.
one launched from Paparazzi Center) over the Ivy bus -- it starts no new
processes, and does not need GCS Classic/Messages/Plotter, which is the point:
it exists for benches where those Qt tools fail to launch (library/env
mismatches) but the core Simulator/Server/Data Link stack is fine.

Example:
  python3 sw/simulator/nps/aicam_bench_shot.py --count 15 --period 3
"""
import argparse
import os
import sys
import time
from pathlib import Path

PAPARAZZI_HOME = Path(os.environ.get("PAPARAZZI_HOME", Path(__file__).resolve().parents[2]))
PPRZLINK_PYTHON = PAPARAZZI_HOME / "sw/ext/pprzlink/lib/v2.0/python/src"
sys.path.insert(0, str(PPRZLINK_PYTHON))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from pprzlink.ivy import IvyMessagesInterface  # noqa: E402
from pprzlink.message import PprzMessage  # noqa: E402
from nps_fixedwing_tuning import flight_plan_data  # noqa: E402

DEFAULT_FLIGHT_PLAN = PAPARAZZI_HOME / "conf/flight_plans/TUDELFT/tudelft_imav2026_strasbourg.xml"


def jump_to_block(interface, aircraft_id, block_id):
    # Forwarded by link directly, same as nps_earcam_mission.py's jump_to_block;
    # does not require Server to interpret it (unlike ground/JUMP_TO_BLOCK).
    message = PprzMessage("datalink", "BLOCK")
    message["ac_id"] = aircraft_id
    message["block_id"] = block_id
    interface.send(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ac-id", type=int, default=129, help="Adam's aircraft id")
    parser.add_argument("--block", default="Test_AIcam_Shot")
    parser.add_argument("--block-id", type=int, default=None,
                         help="Skip the flight-plan file lookup and use this numeric id directly "
                              "(the generated var/aircrafts/<AC>/flight_plan.xml can be rewritten "
                              "by Paparazzi Center in the background, making live lookups flaky)")
    parser.add_argument("--flight-plan", default=str(DEFAULT_FLIGHT_PLAN))
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--period", type=float, default=3.0)
    parser.add_argument("--bus", default="127.255.255.255:2010")
    args = parser.parse_args()

    if args.block_id is not None:
        block_id = args.block_id
        print(f"using explicit block id {block_id} (skipped file lookup)")
    else:
        data = flight_plan_data(Path(args.flight_plan))
        if args.block not in data["blocks"]:
            sys.exit(f"block {args.block!r} not found; available: {sorted(data['blocks'])}")
        block_id = data["blocks"][args.block]
        print(f"block {args.block!r} = id {block_id}")

    interface = IvyMessagesInterface("aicam_bench_shot", ivy_bus=args.bus)
    try:
        for shot in range(1, args.count + 1):
            jump_to_block(interface, args.ac_id, block_id)
            print(f"[{shot}/{args.count}] jumped to {args.block} (ac_id={args.ac_id})")
            if shot < args.count:
                time.sleep(args.period)
    finally:
        interface.shutdown()


if __name__ == "__main__":
    main()
