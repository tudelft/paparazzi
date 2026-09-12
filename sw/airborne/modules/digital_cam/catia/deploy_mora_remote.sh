#!/usr/bin/env bash
set -euo pipefail

install_dir=${1:?installation directory required}
stage=${2:?staging directory required}
system_dir=${3:-/etc}
backup="$stage/backup"
changed=false
stopped=false
was_active=false
was_enabled=false

if [[ -f /home/air/pass.txt ]]; then
  sudo -S -v < /home/air/pass.txt 2>/dev/null || true
elif [[ -f "${HOME:-}/pass.txt" ]]; then
  sudo -S -v < "${HOME:-}/pass.txt" 2>/dev/null || true
fi

exec 9>"$install_dir/.deploy.lock"
flock -n 9 || { echo 'Another CATIA deployment is in progress' >&2; exit 1; }

files=(catia soda lwircam earcam mock_image_01.jpg mock_lwir_01.jpg catia.service 99-tiny1c.rules)
destinations=()
for name in "${files[@]}"; do destinations+=("$install_dir/$name"); done
destinations+=("$system_dir/systemd/system/catia.service" "$system_dir/udev/rules.d/99-tiny1c.rules")

rollback()
{
  local status=$?
  trap - EXIT HUP INT TERM
  if [[ $status -ne 0 && "$stopped" == true ]]; then
    echo "MORA CATIA: activation failed; restoring previous installation from $backup" >&2
    if [[ "$changed" == true ]]; then
      local rollback_load
      rollback_load=$(sudo -n systemctl show catia.service -p LoadState --value) || exit "$status"
      if [[ "$rollback_load" != not-found ]] && ! sudo -n systemctl stop catia.service; then
        echo 'Rollback aborted: cannot stop CATIA safely; backup retained' >&2
        exit "$status"
      fi
      if [[ "$was_enabled" == false && -e "$system_dir/systemd/system/catia.service" ]]; then
        sudo -n systemctl disable catia.service || exit "$status"
      fi
      for index in "${!destinations[@]}"; do
        if ! sudo -n rm -f -- "${destinations[index]}"; then exit "$status"; fi
        if [[ -e "$backup/$index" || -L "$backup/$index" ]]; then
          if ! sudo -n cp -a -- "$backup/$index" "${destinations[index]}"; then exit "$status"; fi
        fi
      done
      sudo -n systemctl daemon-reload || exit "$status"
      if [[ "$was_enabled" == true ]]; then
        sudo -n systemctl enable catia.service || exit "$status"
      fi
      sudo -n udevadm control --reload-rules || exit "$status"
      sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0bda --attr-match=idProduct=5840 || exit "$status"
    fi
    if [[ "$was_active" == true ]]; then
      sudo -n systemctl start catia.service || exit "$status"
    fi
  fi
  exit "$status"
}
trap rollback EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

for name in "${files[@]}"; do test -s "$stage/$name"; done
"$stage/earcam" --self-test
"$stage/lwircam" --mock-image "$stage/mock_lwir_01.jpg" --output "$stage/mock-validation.jpg"
test -s "$stage/mock-validation.jpg"
load_state=$(sudo -n systemctl show catia.service -p LoadState --value)
if [[ "$load_state" != not-found ]]; then
  enabled_state=$(sudo -n systemctl show catia.service -p UnitFileState --value)
  case "$enabled_state" in
    enabled) was_enabled=true ;;
    disabled) ;;
    *) echo "Unsupported prior service enablement: $enabled_state" >&2; exit 1 ;;
  esac
  active_state=$(sudo -n systemctl show catia.service -p ActiveState --value)
  case "$active_state" in
    active|activating|reloading) was_active=true ;;
    inactive|failed) ;;
    *) echo "Service is changing state: $active_state" >&2; exit 1 ;;
  esac
fi
mkdir "$backup"
for index in "${!destinations[@]}"; do
  if [[ -e "${destinations[index]}" || -L "${destinations[index]}" ]]; then
    sudo -n cp -a -- "${destinations[index]}" "$backup/$index"
  fi
done
if [[ "$load_state" != not-found ]]; then
  sudo -n systemctl stop catia.service
  stopped=true
  test "$(sudo -n systemctl show catia.service -p ActiveState --value)" = inactive
fi
stopped=true
if pgrep -af "^$install_dir/(catia|lwircam|earcam)( |$)"; then
  stopped=false
  echo 'Stop the unmanaged camera processes before deploying' >&2
  exit 1
else
  result=$?
  [[ $result -eq 1 ]] || exit "$result"
fi
changed=true
mkdir -p "$install_dir/photos" "$install_dir/earlogs" "${HOME:-/home/air}/Pictures" "${HOME:-/home/air}/usher_debug_data"
for name in catia soda lwircam earcam; do install -m 0755 "$stage/$name" "$install_dir/$name"; done
for name in mock_image_01.jpg mock_lwir_01.jpg catia.service 99-tiny1c.rules; do
  install -m 0644 "$stage/$name" "$install_dir/$name"
done
sudo -n install -m 0644 "$stage/catia.service" "$system_dir/systemd/system/catia.service"
sudo -n install -m 0644 "$stage/99-tiny1c.rules" "$system_dir/udev/rules.d/99-tiny1c.rules"
sudo -n systemd-analyze verify "$system_dir/systemd/system/catia.service"
sudo -n udevadm control --reload-rules
sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0bda --attr-match=idProduct=5840
sudo -n systemctl daemon-reload
card=$(awk -F'[][]' '/USB-Audio/ {print $2; exit}' /proc/asound/cards 2>/dev/null | tr -d ' ') || card=
if [[ -n "$card" ]]; then
  amixer -q -c "$card" set 'Auto Gain Control' off 2>/dev/null || echo 'Warning: cannot disable microphone AGC' >&2
  if ! amixer -q -c "$card" set Mic 14 cap 2>/dev/null && ! amixer -q -c "$card" set Mic 88% cap 2>/dev/null; then
    echo 'Warning: cannot configure microphone gain' >&2
  fi
else
  echo 'Warning: no USB audio capture card enumerated' >&2
fi
sudo -n systemctl enable catia.service
sudo -n systemctl restart catia.service
sudo -n systemctl is-enabled --quiet catia.service
sudo -n systemctl is-active --quiet catia.service
test "$(sudo -n systemctl show catia.service -p SubState --value)" = running
changed=false
stopped=false
echo "MORA CATIA: deployment active; previous installation retained in $backup"