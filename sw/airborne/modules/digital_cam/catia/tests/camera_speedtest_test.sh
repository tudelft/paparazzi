#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-speedtest.XXXXXXXX)
trap 'rm -rf "$output"' EXIT

gcc -std=gnu11 -O2 -Wall -Wextra -Werror \
  -I"$root" \
  "$root/tests/camera_speedtest_test.c" "$root/camera_speedtest.c" \
  -lpthread -o "$output/test"

cd "$output"
./test >speedtest.out 2>speedtest.err
grep -q 'capture 10/10 completed' speedtest.out
grep -q 'practical sustained speed:.*photos/s (1 photo every .* s, .* ms)' speedtest.out
grep -q 'average camera-call speed:.*photos/s (1 photo every .* s, .* ms)' speedtest.out
grep -q 'average / fastest / slowest capture time:' speedtest.out
grep -q 'fastest single capture:.*ms (equivalent to .* photos/s)' speedtest.out
grep -q 'measured capture failed; aborting benchmark' speedtest.err
grep -q 'warm-up capture failed; aborting benchmark' speedtest.err

help_output=$($root/catia --help)
grep -q -- '--speedtest' <<<"$help_output"

set +e
missing_backend=$($root/catia --speedtest 2>&1)
missing_status=$?
incompatible=$($root/catia --aicam --speedtest --serial /dev/null 2>&1)
incompatible_status=$?
set -e
[[ $missing_status -eq 2 ]]
[[ $incompatible_status -eq 2 ]]
grep -q -- '--speedtest requires --chdk, --aicam, or --lwircam' <<<"$missing_backend"
grep -q -- '--speedtest cannot be combined' <<<"$incompatible"

echo 'camera speedtest CLI tests passed'