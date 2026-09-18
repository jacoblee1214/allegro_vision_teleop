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

TARGET_IP="${1:-192.168.1.100}"
python3 "$SCRIPT_DIR/check_hand.py" "$TARGET_IP"
