#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-chdk-failure.XXXXXXXX)
trap 'rm -rf "$output"' EXIT
for command in /nonexistent/catia-chdk /usr/bin/false; do
  gcc -std=gnu11 -Wall -Wextra -Werror -DCATIA_CHDK_COMMAND="\"$command\"" \
    "$root/tests/chdk_failure_test.c" "$root/chdk_pipe.c" -o "$output/test"
  "$output/test"
done
printf '%s\n' 'Missing and failed CHDK commands are nonfatal and retryable'