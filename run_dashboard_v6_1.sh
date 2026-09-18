#!/usr/bin/env bash
# ==============================================================================
# run_dashboard_v6_1.sh — Direct Launcher for Classic 2D Teleop Dashboard + RViz2
#
# Usage:
#   ./run_dashboard_v6_1.sh [nodes|sim|real] [options]
#   ./run_dashboard_v6_1.sh --gui-only [options]   # Launch standalone Dashboard window only
#
# Examples:
#   ./run_dashboard_v6_1.sh real              # Physical hardware + Classic Dashboard + RViz2
#   ./run_dashboard_v6_1.sh sim               # Mock simulation + Classic Dashboard + RViz2
#   ./run_dashboard_v6_1.sh real --hand left  # Left hand model
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

# Check if standalone GUI only is requested
if [ "$1" = "--gui-only" ]; then
    shift
    # Auto-delegation to Docker container if executed on host machine
    if [ ! -f "/.dockerenv" ] && [ "${RUN_ON_HOST:-0}" != "1" ]; then
        xhost +local:docker >/dev/null 2>&1 || true
        if ! docker ps --format '{{.Names}}' | grep -q "^ros_humble_dev$"; then
            echo "[*] Starting ros_humble_dev container..."
            docker start ros_humble_dev >/dev/null
        fi
        DOCKER_FLAGS="-i"
        if [ -t 0 ] && [ -t 1 ]; then
            DOCKER_FLAGS="-it"
        fi
        exec docker exec $DOCKER_FLAGS \
            -e DISPLAY="${DISPLAY:-:1}" \
            ros_humble_dev /home/humble_ws/allegro_vision_teleop/run_dashboard_v6_1.sh --gui-only "$@"
    fi

    # Inside Docker environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    if [ -f "/home/humble_ws/install/setup.bash" ]; then
        source /home/humble_ws/install/setup.bash
    fi
    export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
    exec python3 "$SCRIPT_DIR/teleop_dashboard_v6_1.py" "$@"
fi

# Default: Full pipeline with Classic Dashboard + RViz2
exec "$SCRIPT_DIR/run_teleop_v6_1.sh" "$@" --classic
