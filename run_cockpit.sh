#!/usr/bin/env bash
# run_cockpit.sh — single-window cockpit UI (embedded 3D hand, no separate RViz2).
#   ./run_cockpit.sh real              # physical hand, hand type auto-detected
#   ./run_cockpit.sh sim --hand left   # no hardware ('sim' has no auto-detect)
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$DIR/run_teleop.sh" "${@:-real}" --cockpit
