#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(cd "$root/../../../../.." && pwd)
output=$(mktemp -d /tmp/catia-pose-sender.XXXXXX)
trap 'rm -rf "$output"' EXIT
for stream in 0 1; do
  for gps in 0 1; do
    for fixedwing in 0 1; do
      flags=()
      if [[ "$stream" == 1 ]]; then flags+=(-DDIGITAL_CAM_UART_POSE_STREAM=1); fi
      gcc -std=c11 -Wall -Wextra -Werror -DPERIODIC_TELEMETRY=0 \
        -DUSE_GPS="$gps" -DFIXEDWING_FIRMWARE="$fixedwing" "${flags[@]}" \
        -I"$root/tests/pose_sender_stubs" -I"$workspace/sw/airborne" \
        "$root/tests/pose_sender_test.c" "$root/protocol.c" -o "$output/sender"
      "$output/sender"
    done
  done
done
for configuration in legacy-lwir explicit-mask none; do
  case "$configuration" in
    legacy-lwir) flags=(-DDIGITAL_CAM_UART_CAMERA_ID=3 -DEXPECTED_CAMERA_MASK=4) ;;
    explicit-mask) flags=(-DDIGITAL_CAM_UART_CAMERA_ID=3 -DDIGITAL_CAM_UART_CAMERA_MASK=10 -DEXPECTED_CAMERA_MASK=10) ;;
    none) flags=(-DDIGITAL_CAM_UART_CAMERA_MASK=0 -DEXPECTED_CAMERA_MASK=0) ;;
  esac
  gcc -std=c11 -Wall -Wextra -Werror -DPERIODIC_TELEMETRY=0 -DUSE_GPS=0 \
    -DFIXEDWING_FIRMWARE=1 "${flags[@]}" \
    -I"$root/tests/pose_sender_stubs" -I"$workspace/sw/airborne" \
    "$root/tests/pose_sender_test.c" "$root/protocol.c" -o "$output/sender"
  "$output/sender"
done
gcc -std=c11 -Wall -Wextra -Werror "$root/tests/pose_protocol_test.c" "$root/protocol.c" -o "$output/protocol"
"$output/protocol"