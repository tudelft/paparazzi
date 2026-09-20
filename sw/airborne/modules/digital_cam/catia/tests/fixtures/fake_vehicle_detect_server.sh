#!/usr/bin/env bash
# Stand-in for the persistent vehicle-detection capture server: answers every
# request with a fixed detection outcome controlled by FAKE_DETECT_COUNT/CONF,
# mirroring vehicle_detect_server.py's line protocol closely enough to drive
# vehicle_detect_pipe.c without a real IMX500 sensor.
set -euo pipefail
count="${FAKE_DETECT_COUNT:-0}"
conf="${FAKE_DETECT_CONF:-0.0000}"

echo "AICAM_SERVER_READY"
while IFS= read -r request; do
  [[ -z "$request" ]] && continue
  printf 'fake jpeg data' > "$request"
  if [[ "$count" -gt 0 ]]; then
    echo "AICAM_SERVER_OK count=$count conf=$conf box_x=10 box_y=20 box_w=30 box_h=40"
  else
    echo "AICAM_SERVER_OK count=0 conf=0.0000 box_x=0 box_y=0 box_w=0 box_h=0"
  fi
done
