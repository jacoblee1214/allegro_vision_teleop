#!/usr/bin/env bash
# check_hand.sh — 1-second Allegro Hand V6 Communication Diagnostic Script
set -e

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$(cd -P "$(dirname "$SOURCE")" >/dev/null 2>&1 && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" >/dev/null 2>&1 && pwd)"

exec python3 /home/jake/humble_ws/allegro_vision_teleop/check_hand.py "$@"
