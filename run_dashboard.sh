#!/usr/bin/env bash
# run_dashboard.sh — classic 2D dashboard plus a separate RViz2 window, on the latest pipeline.
#
# This is the entry point that always follows the newest version. It now runs **v3**
# (run_dashboard_v3.sh): DexPilot retargeting with smoothed landmarks, the tactile
# pressure panel and the Wonik look. Details: README.md, "빠른 실행".
#
#   ./run_dashboard.sh                 # real hand, found automatically (right preferred)
#   ./run_dashboard.sh sim --hand left
#   ./run_dashboard.sh --v2            # previous pipeline: DexPilot, no tactile panel or smoothing
#   ./run_dashboard.sh --v1            # oldest: geometric retargeting
#
# v1 mapped bone angles straight to joint angles and never checked where the robot's
# fingertips ended up, so a pinch closed on screen and left the robot's thumb ~13 cm
# short; v2 solves for the fingertip positions every frame (docs/dexpilot_retargeting_v2.md).
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

VERSION="v3"
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --v1) VERSION="v1" ;;
        --v2) VERSION="v2" ;;
        *)    ARGS+=("$arg") ;;
    esac
done

case "$VERSION" in
    v3) exec "$DIR/run_dashboard_v3.sh" "${ARGS[@]}" ;;
    v2) exec "$DIR/run_teleop_v2.sh" "${ARGS[@]:-real}" --classic --rviz ;;
    v1) exec "$DIR/run_teleop.sh" "${ARGS[@]:-real}" --classic --rviz ;;
esac
