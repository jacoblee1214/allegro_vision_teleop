#!/usr/bin/env bash
# ==============================================================================
# run_cockpit.sh — Direct Launcher for Unified 3D Cockpit UI (Single Window)
#
# Usage:
#   ./run_cockpit.sh [nodes|sim|real] [options]
#
# Examples:
#   ./run_cockpit.sh real              # Physical hardware + Unified 3D Cockpit
#   ./run_cockpit.sh sim               # Mock simulation + Unified 3D Cockpit
#   ./run_cockpit.sh real --hand left  # Left hand model
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$SCRIPT_DIR/run_teleop_v6.sh" "$@" --cockpit
