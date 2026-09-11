#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
temporary=$(mktemp -d /tmp/catia-deploy-test.XXXXXXXX)
trap 'rm -rf "$temporary"' EXIT
mkdir "$temporary/bin"
for name in sudo systemctl systemd-analyze udevadm pgrep install; do
  /usr/bin/install -m 0755 "$root/tests/deploy_mock_command.sh" "$temporary/bin/$name"
done
export PATH="$temporary/bin:$PATH"
for SCENARIO in success self-test mock-test stop install restart first-install unmanaged enabled inactive; do
  export SCENARIO
  TEST_STATE="$temporary/$SCENARIO"
  export TEST_STATE
  mkdir -p "$TEST_STATE/installed" "$TEST_STATE/stage" "$TEST_STATE/etc/systemd/system" "$TEST_STATE/etc/udev/rules.d"
  printf 'active\n' > "$TEST_STATE/active"
  printf 'disabled\n' > "$TEST_STATE/enabled"
  if [[ "$SCENARIO" == enabled ]]; then printf 'enabled\n' > "$TEST_STATE/enabled"; fi
  if [[ "$SCENARIO" == inactive ]]; then printf 'inactive\n' > "$TEST_STATE/active"; fi
  files=(catia soda lwircam earcam mock_image_01.jpg mock_lwir_01.jpg catia.service 99-tiny1c.rules)
  for name in "${files[@]}"; do
    printf 'old %s\n' "$name" > "$TEST_STATE/installed/$name"
    printf 'new %s\n' "$name" > "$TEST_STATE/stage/$name"
  done
  for name in earcam lwircam; do
    /usr/bin/install -m 0755 "$root/tests/deploy_mock_command.sh" "$TEST_STATE/stage/$name"
  done
  cp "$TEST_STATE/installed/catia.service" "$TEST_STATE/etc/systemd/system/catia.service"
  cp "$TEST_STATE/installed/99-tiny1c.rules" "$TEST_STATE/etc/udev/rules.d/99-tiny1c.rules"
  if [[ "$SCENARIO" == first-install ]]; then
    rm "$TEST_STATE/installed/"* "$TEST_STATE/etc/systemd/system/catia.service" "$TEST_STATE/etc/udev/rules.d/99-tiny1c.rules"
    printf 'inactive\n' > "$TEST_STATE/active"
  fi
  if bash "$root/deploy_mora_remote.sh" "$TEST_STATE/installed" "$TEST_STATE/stage" "$TEST_STATE/etc" > "$TEST_STATE/log" 2>&1; then
    [[ "$SCENARIO" == success ]]
    for name in "${files[@]}"; do cmp "$TEST_STATE/stage/$name" "$TEST_STATE/installed/$name"; done
    [[ $(cat "$TEST_STATE/enabled") == enabled && $(cat "$TEST_STATE/active") == active ]]
  else
    [[ "$SCENARIO" != success ]] || { cat "$TEST_STATE/log"; exit 1; }
    if [[ "$SCENARIO" == first-install ]]; then
      for name in "${files[@]}"; do test ! -e "$TEST_STATE/installed/$name"; done
      test ! -e "$TEST_STATE/etc/systemd/system/catia.service"
      test ! -e "$TEST_STATE/etc/udev/rules.d/99-tiny1c.rules"
      [[ $(cat "$TEST_STATE/active") == inactive ]]
    else
      for name in "${files[@]}"; do [[ $(cat "$TEST_STATE/installed/$name") == "old $name" ]]; done
      [[ $(cat "$TEST_STATE/etc/systemd/system/catia.service") == 'old catia.service' ]]
      [[ $(cat "$TEST_STATE/etc/udev/rules.d/99-tiny1c.rules") == 'old 99-tiny1c.rules' ]]
      if [[ "$SCENARIO" == inactive ]]; then
        [[ $(cat "$TEST_STATE/active") == inactive ]]
      elif [[ "$SCENARIO" != unmanaged ]]; then
        [[ $(cat "$TEST_STATE/active") == active ]]
      fi
    fi
    if [[ "$SCENARIO" == enabled ]]; then
      [[ $(cat "$TEST_STATE/enabled") == enabled ]]
    else
      [[ $(cat "$TEST_STATE/enabled") == disabled ]]
    fi
    if [[ "$SCENARIO" == self-test || "$SCENARIO" == mock-test ]]; then
      ! grep -q '^systemctl stop' "$TEST_STATE/commands"
    fi
  fi
  printf 'Deployment scenario passed: %s\n' "$SCENARIO"
done
bash -n "$root/deploy_mora.sh"
grep -Fq '"$SCRIPT_DIR/lwircam/mock_lwir_01.jpg"' "$root/deploy_mora.sh"
! grep -q 'mock_lwir_photos' "$root/deploy_mora.sh"