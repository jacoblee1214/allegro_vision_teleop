#!/usr/bin/env bash
# run_dashboard.sh — classic 2D dashboard plus a separate RViz2 window.
#   ./run_dashboard.sh real
#   ./run_dashboard.sh sim --hand right
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$DIR/run_teleop.sh" "${@:-real}" --classic --rviz
