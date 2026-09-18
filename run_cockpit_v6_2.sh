#!/usr/bin/env bash
# ==============================================================================
# run_cockpit_v6_2.sh — Direct Launcher for Unified 3D Cockpit UI (Single Window)
#
# Usage:
#   ./run_cockpit_v6_2.sh [nodes|sim|real] [options]
#
# Examples:
#   ./run_cockpit_v6_2.sh real              # Physical hardware + Unified 3D Cockpit
#   ./run_cockpit_v6_2.sh sim               # Mock simulation + Unified 3D Cockpit
#   ./run_cockpit_v6_2.sh real --hand left  # Left hand model
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$SCRIPT_DIR/run_teleop_v6_2.sh" "$@" --cockpit
