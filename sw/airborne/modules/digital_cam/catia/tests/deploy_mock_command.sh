#!/usr/bin/env bash
set -euo pipefail
command_name=${0##*/}
printf '%s %s\n' "$command_name" "$*" >> "$TEST_STATE/commands"
case "$command_name" in
  sudo)
    if [[ "${1:-}" == -S && "${2:-}" == -v ]]; then
      exit 0
    elif [[ "${1:-}" == -v ]]; then
      exit 0
    fi
    [[ "$1" == -n ]]
    shift
    exec "$@"
    ;;
  earcam)
    [[ "$SCENARIO" != self-test ]]
    ;;
  lwircam)
    [[ "$SCENARIO" != mock-test ]]
    printf 'JPEG' > "$4"
    ;;
  pgrep)
    [[ "$SCENARIO" == unmanaged ]]
    ;;
  systemd-analyze|udevadm) exit 0 ;;
  install)
    if [[ "$SCENARIO" == install && "$*" == *stage/soda* && ! -e "$TEST_STATE/injected" ]]; then
      touch "$TEST_STATE/injected"
      exit 1
    fi
    exec /usr/bin/install "$@"
    ;;
  systemctl)
    case "$1" in
      show)
        case "$4" in
          LoadState)
            if [[ -e "$TEST_STATE/etc/systemd/system/catia.service" ]]; then printf 'loaded\n'; else printf 'not-found\n'; fi ;;
          UnitFileState) cat "$TEST_STATE/enabled" ;;
          ActiveState) cat "$TEST_STATE/active" ;;
          SubState) printf 'running\n' ;;
          *) exit 2 ;;
        esac ;;
      stop)
        [[ "$SCENARIO" != stop ]]
        printf 'inactive\n' > "$TEST_STATE/active" ;;
      start) printf 'active\n' > "$TEST_STATE/active" ;;
      restart)
        [[ "$SCENARIO" != restart && "$SCENARIO" != first-install && "$SCENARIO" != enabled && "$SCENARIO" != inactive ]]
        printf 'active\n' > "$TEST_STATE/active" ;;
      enable) printf 'enabled\n' > "$TEST_STATE/enabled" ;;
      disable) printf 'disabled\n' > "$TEST_STATE/enabled" ;;
      is-enabled) [[ $(cat "$TEST_STATE/enabled") == enabled ]] ;;
      is-active) [[ $(cat "$TEST_STATE/active") == active ]] ;;
      daemon-reload) ;;
      *) exit 2 ;;
    esac
    ;;
  *) exit 2 ;;
esac