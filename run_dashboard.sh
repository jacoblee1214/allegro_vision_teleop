#!/usr/bin/env bash
# ==============================================================================
# run_dashboard.sh — Direct Launcher for Unified Teleoperation Cockpit Dashboard
# ==============================================================================
set -e

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
        ros_humble_dev /home/humble_ws/allegro_vision_teleop/run_dashboard.sh "$@"
fi

# Inside Docker environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
fi
if [ -f "/home/humble_ws/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source /home/humble_ws/install/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"

python3 "$SCRIPT_DIR/teleop_dashboard_v6.py" "$@"
