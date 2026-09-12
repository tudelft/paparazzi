#!/usr/bin/env bash
# Stand-in for the persistent LWIR capture server: answers exactly one request
# then exits, simulating a server killed by a mid-flight USB re-enumeration.
set -euo pipefail
counter_file="${FAKE_SERVER_COUNTER:?FAKE_SERVER_COUNTER not set}"
count=0
if [[ -f "$counter_file" ]]; then
  count=$(cat "$counter_file")
fi
count=$((count + 1))
echo "$count" > "$counter_file"

echo "LWIR_SERVER_READY"
if ! IFS= read -r request; then
  exit 0
fi
case "$request" in
  GEO:*)
    echo "LWIR_SERVER_OK"
    ;;
  *)
    printf 'fake capture %d\n' "$count" > "$request"
    echo "LWIR_SERVER_OK"
    ;;
esac
exit 0
