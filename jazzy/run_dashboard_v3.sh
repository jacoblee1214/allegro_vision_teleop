#!/usr/bin/env bash
# run_dashboard_v3.sh — the one command for daily use: dashboard + RViz2 + tactile pressures,
# DexPilot retargeting with smoothed input, on the real hand.
#
#   ./run_dashboard_v3.sh                    # real hand, found automatically
#   ./run_dashboard_v3.sh --hand right       # both hands connected: take the right one
#   ./run_dashboard_v3.sh --smooth 9         # trembling: smooth harder
#   ./run_dashboard_v3.sh sim --hand left    # no robot
#
# Every other option of run_teleop_v3.sh works here too.
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
case "${1:-}" in
    real|sim|nodes) MODE="$1"; shift ;;
    *)              MODE="real" ;;
esac
exec "$DIR/run_teleop_v3.sh" "$MODE" --classic --rviz "$@"
