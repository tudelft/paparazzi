#!/usr/bin/env bash
trap '' TERM
printf 'LWIR_SERVER_READY\n'
while IFS= read -r request; do
  printf '%010000d' 0
  exec sleep 30
done