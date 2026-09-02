#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PAPARAZZI_HOME=$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)

cd "$PAPARAZZI_HOME"

echo "MORA CATIA: building local camera application"
make -C sw/airborne/modules/digital_cam/catia

echo "MORA CATIA: starting verbose local mode"
echo "MORA CATIA: simulator endpoint is /tmp/catia-sim"
exec stdbuf -oL -eL sw/airborne/modules/digital_cam/catia/catia --local --debug --mocktransform --test
