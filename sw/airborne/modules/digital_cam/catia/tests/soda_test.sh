#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
temporary=$(mktemp -d /tmp/catia-soda-test.XXXXXXXX)
trap 'rm -rf "$temporary"' EXIT
make -s -C "$root" soda
cp "$root/soda" "$temporary/soda"
image="$root/lwircam/mock_lwir_01.jpg"
for camera in aicam chdkcam lwircam earcam; do
  [[ $("$temporary/soda" "--$camera" "$image") == "Now I can do nifty stuff for $camera" ]]
  [[ $("$temporary/soda" "$image" "--$camera" 39 -1 2 50000 0 -3 4 5 6 12800) == "Now I can do nifty stuff for $camera" ]]
done
[[ $("$temporary/soda" "$image") == 'SODA: I looked at the image. Cool, right?' ]]
"$temporary/soda" --help > /dev/null
# The Makefile build (unlike the old ad hoc "g++ soda.cpp" compile this test used to do)
# embeds the real checked-out commit via CATIA_GIT_SHA, so accept either that or the
# no-git-available fallback rather than hardcoding "unknown".
"$temporary/soda" --local --help | grep -qE '^SODA v1\.0 \(Git ([0-9a-f]{12}|unknown)\)$'
"$temporary/soda" --version | grep -qE '^SODA v1\.0 \(Git ([0-9a-f]{12}|unknown)\)$'
[[ $("$temporary/soda" --local --lwircam "$image") == 'Now I can do nifty stuff for lwircam' ]]
expect_failure()
{
  if "$temporary/soda" "$@" > "$temporary/log" 2>&1; then
    printf 'Unexpected success: %s\n' "$*" >&2
    exit 1
  fi
  ! grep -q 'Now I can do nifty stuff' "$temporary/log"
}
expect_failure
expect_failure --earcam
expect_failure --aicam --earcam "$image"
expect_failure --lwircam --lwircam "$image"
expect_failure -lwircam "$image"
expect_failure --unknown "$image"
expect_failure --local --local "$image"
expect_failure --aicam "$temporary/missing.jpg"
touch "$temporary/empty.jpg"
expect_failure --aicam "$temporary/empty.jpg"
expect_failure "$image" 1 2
expect_failure "$image" 1 2 3 4 5 6 7 8 9 2147483648
expect_failure "$image" 1 2 3 4 5 6 7 8 9 invalid
cp "$image" "$temporary/-image with spaces.jpg"
pushd "$temporary" > /dev/null
[[ $(./soda --earcam -- '-image with spaces.jpg') == 'Now I can do nifty stuff for earcam' ]]
popd > /dev/null
printf 'SODA CLI tests passed\n'