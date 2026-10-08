#!/usr/bin/env bash
# run_cockpit_v2.sh — cockpit UI on the v2 (DexPilot) retargeting pipeline.
#   ./run_cockpit_v2.sh real              # physical hand, hand type auto-detected
#   ./run_cockpit_v2.sh sim --hand left   # no hardware ('sim' has no auto-detect)
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$DIR/run_teleop_v2.sh" "${@:-real}" --cockpit
