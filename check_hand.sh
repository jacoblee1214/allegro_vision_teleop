#!/usr/bin/env bash
# check_hand.sh — hand communication / hand type / encoder check.
# No ROS and no venv needed: tools/check_hand.py uses only the Python standard library.
#
#   ./check_hand.sh              # address from config/teleop.env, with fallback probing
#   ./check_hand.sh 101          # 192.168.1.101
#   ./check_hand.sh 192.168.1.5
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
# shellcheck source=/dev/null
[ -f "$DIR/config/teleop.env" ] && source "$DIR/config/teleop.env"
if [ $# -gt 0 ]; then
    exec python3 "$DIR/tools/check_hand.py" "$@"
fi
exec python3 "$DIR/tools/check_hand.py" "${HAND_IP:-192.168.1.100}"
