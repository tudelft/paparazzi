#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-vehicle-detect-save-copy.XXXXXX)
trap 'rm -rf "$output"' EXIT
chmod +x "$root/tests/fixtures/fake_vehicle_detect_server.sh"
gcc -std=gnu11 -Wall -Wextra -Werror \
  -DCATIA_VEHICLE_DETECT_PHOTO_DIR='"photos"' \
  -DCATIA_VEHICLE_DETECT_COMMAND="\"$root/tests/fixtures/fake_vehicle_detect_server.sh\"" \
  -DCATIA_VEHICLE_DETECT_MODEL='"unused.rpk"' \
  -DCATIA_VEHICLE_DETECT_LABELS='"unused.txt"' \
  "$root/tests/vehicle_detect_save_copy_test.c" -o "$output/test"

FAKE_DETECT_COUNT=1 FAKE_DETECT_CONF=0.87 "$output/test"
FAKE_DETECT_COUNT=0 "$output/test"
