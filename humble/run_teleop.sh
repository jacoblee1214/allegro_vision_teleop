#!/usr/bin/env bash
# ==============================================================================
# run_teleop.sh — Dual-Model Launcher for Allegro Hand (V4: 4-Finger / V6: 5-Finger)
#
# Usage:
#   ./run_teleop.sh [--v6|--v4] [mode] [options]
#   (Default hand version is V6)
#
# Examples:
#   ./run_teleop.sh sim           # Launch V6 simulation (default)
#   ./run_teleop.sh real          # Launch V6 physical hardware
#   ./run_teleop.sh --v4 sim      # Launch V4 simulation
#   ./run_teleop.sh --v4 real     # Launch V4 physical hardware
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

HAND_VERSION="v6"
PASS_ARGS=()

for arg in "$@"; do
    case "$arg" in
        --v4|--v4-hand|-4)
            HAND_VERSION="v4"
            ;;
        --v6|--v6-hand|-6)
            HAND_VERSION="v6"
            ;;
        *)
            PASS_ARGS+=("$arg")
            ;;
    esac
done

if [ "$HAND_VERSION" = "v4" ]; then
    exec "$SCRIPT_DIR/run_teleop_v4.sh" "${PASS_ARGS[@]}"
else
    exec "$SCRIPT_DIR/run_teleop_v6.sh" "${PASS_ARGS[@]}"
fi
