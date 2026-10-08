#!/usr/bin/env bash
# run_dashboard.sh — classic 2D dashboard plus a separate RViz2 window.
#
# This one runs the **v2 (DexPilot) pipeline**: run_teleop_v2.sh, with
# teleop_dashboard_v2.py and retargeting_node_v2.py. v1 mapped bone angles
# straight to joint angles and never checked where the robot's fingertips ended
# up, so a pinch closed on screen and left the robot's thumb ~13 cm short; v2
# solves for the fingertip positions every frame. Verified on the physical hand
# on 2026-09-30. Details: docs/dexpilot_retargeting_v2.md
#
#   ./run_dashboard.sh real
#   ./run_dashboard.sh sim --hand right
#   ./run_dashboard.sh real --v1      # fall back to the old geometric retargeting
#
# run_cockpit.sh and run_teleop.sh are deliberately left on v1. run_dashboard_v2.sh
# is this same pipeline without the --v1 switch.
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

LAUNCHER="run_teleop_v2.sh"
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --v1) LAUNCHER="run_teleop.sh" ;;
        *)    ARGS+=("$arg") ;;
    esac
done

exec "$DIR/$LAUNCHER" "${ARGS[@]:-real}" --classic --rviz
