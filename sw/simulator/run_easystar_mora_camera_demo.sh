#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PAPARAZZI_HOME=$(cd -- "$SCRIPT_DIR/../.." && pwd)
AIRCRAFT="Easystar_3_Camera_Demo"
CAMERA_DEVICE="/tmp/catia-sim"

cd "$PAPARAZZI_HOME"
export PAPARAZZI_HOME

echo "EasyStar MORA demo: waiting for CATIA endpoint $CAMERA_DEVICE"
for attempt in $(seq 1 200); do
  if [[ -e "$CAMERA_DEVICE" ]]; then
    break
  fi
  read -r -t 0.1 _ || true
done

if [[ ! -e "$CAMERA_DEVICE" ]]; then
  echo "EasyStar MORA demo: CATIA did not create $CAMERA_DEVICE" >&2
  exit 1
fi

echo "EasyStar MORA demo: regenerating $AIRCRAFT for local UART camera"
make AIRCRAFT="$AIRCRAFT" clean_ac
make AIRCRAFT="$AIRCRAFT" SITL_SERIAL="$CAMERA_DEVICE" nps.compile

echo "EasyStar MORA demo: starting NPS camera survey"
exec "var/aircrafts/$AIRCRAFT/nps/simsitl" --norc --rc_script 0 --time_factor 2