#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PAPARAZZI_HOME=$(cd -- "$SCRIPT_DIR/../.." && pwd)
AIRCRAFT="Easystar_3"
CAMERA_DEVICE="/tmp/catia-sim"

cd "$PAPARAZZI_HOME"
export PAPARAZZI_HOME

if [[ -e "$CAMERA_DEVICE" ]]; then
  echo "EasyStar MORA demo: CATIA endpoint is ready at $CAMERA_DEVICE"
else
  echo "EasyStar MORA demo: CATIA is not running yet; UART will connect automatically"
fi

echo "EasyStar MORA demo: regenerating $AIRCRAFT for local UART camera"
make AIRCRAFT="$AIRCRAFT" clean_ac
make AIRCRAFT="$AIRCRAFT" SITL_SERIAL="$CAMERA_DEVICE" nps.compile

echo "EasyStar MORA demo: starting NPS camera survey"
exec "var/aircrafts/$AIRCRAFT/nps/simsitl" --norc --rc_script 0 --time_factor 1