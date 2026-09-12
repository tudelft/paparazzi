#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
make -s -C "$root" -j"${BUILD_JOBS:-32}"
native_machine=$(LC_ALL=C readelf -h /proc/$$/exe | awk -F: '/Machine:/ {gsub(/^ +/, "", $2); print $2}')
for name in catia soda lwircam-native earcam-native; do
  machine=$(LC_ALL=C readelf -h "$root/$name" | awk -F: '/Machine:/ {gsub(/^ +/, "", $2); print $2}')
  [[ "$machine" == "$native_machine" ]]
  "$root/$name" --version
done
"$root/earcam-native" --self-test
before=$(sha256sum "$root/catia" "$root/soda" "$root/lwircam-native" "$root/earcam-native")
make -s -C "$root" -j"${BUILD_JOBS:-32}" arm64
[[ "$before" == "$(sha256sum "$root/catia" "$root/soda" "$root/lwircam-native" "$root/earcam-native")" ]]
for name in catia soda lwircam earcam; do
  LC_ALL=C readelf -h "$root/$name-arm64" | grep -q 'Machine:.*AArch64'
  sections=$(LC_ALL=C readelf -SW "$root/$name-arm64")
  if grep -Eq '\.(debug[[:alnum:]_]*|zdebug[[:alnum:]_]*|symtab|strtab)([[:space:]]|$)' <<< "$sections"; then
    printf 'ARM64 release contains debug or static symbol sections: %s\n' "$name" >&2
    exit 1
  fi
done
grep -Fxq 'OPTFLAGS=-O2 -g0' "$root/.build/arm64/config.stamp"
symbols=$(nm "$root/catia")
grep -q ' T ai_cam_pipe_shoot$' <<< "$symbols"
grep -q ' T chdk_pipe_shoot$' <<< "$symbols"
symbols=$(nm "$root/.build/arm64/project/ai_cam_pipe.o" "$root/.build/arm64/project/chdk_pipe.o")
grep -q ' T ai_cam_pipe_shoot$' <<< "$symbols"
grep -q ' T chdk_pipe_shoot$' <<< "$symbols"
strings "$root/catia-arm64" | grep -Fx '/home/air/digital_cam/lwircam'
strings "$root/catia-arm64" | grep -Fx '/home/air/digital_cam/earcam'
strings "$root/catia-arm64" | grep -Fx '/home/air/Pictures'
strings "$root/catia-arm64" | grep -Fx '/home/air/usher_debug_data'
strings "$root/catia" | grep -Fx "$root/lwircam-native"
strings "$root/catia" | grep -Fx "$root/earcam-native"
printf 'Dual-architecture build layout tests passed\n'