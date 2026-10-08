#!/usr/bin/env bash
# ==============================================================================
# run_teleop.sh — All-in-One Launcher for Allegro Hand V4 Vision Teleoperation
#
# Usage:
#   ./run_teleop.sh [mode] [options]
#
# Modes:
#   nodes (default) : Launches the 3 teleop nodes (vision -> retargeting -> bridge).
#   sim             : Launches mock hardware (RViz2) + all 3 teleop nodes.
#   real            : Launches physical hardware (CAN) + all 3 teleop nodes.
#
# Options:
#   --alpha <val>   : EMA filter alpha (default: 0.15)
#   --device <id>   : Webcam device ID (default: 0)
#   --no-gui        : Run vision tracker in headless mode without OpenCV debug window
#
# Examples:
#   # Run nodes inside container (when hardware/sim is already running):
#   ./run_teleop.sh
#
#   # Run full simulation pipeline inside container:
#   ./run_teleop.sh sim
#
#   # Run full physical hardware pipeline in headless mode:
#   ./run_teleop.sh real --alpha 0.20 --no-gui
# ==============================================================================

set -e

MODE="nodes"
ALPHA="0.15"
DEVICE="0"
GUI_FLAG=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        nodes|sim|real)
            MODE="$1"
            shift
            ;;
        --alpha)
            ALPHA="$2"
            shift 2
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --no-gui|--headless)
            GUI_FLAG="--no-gui"
            shift
            ;;
        -h|--help)
            echo "Usage: ./run_teleop.sh [nodes|sim|real] [--alpha 0.15] [--device 0] [--no-gui]"
            echo ""
            echo "Modes:"
            echo "  nodes (default) : Launch vision_tracker, retargeting_node, sim_bridge_node"
            echo "  sim             : Launch mock hardware + RViz2 + 3 teleop nodes"
            echo "  real            : Launch physical hardware controller (CAN) + 3 teleop nodes"
            echo ""
            echo "Options:"
            echo "  --alpha <float> : EMA smoothing factor (default: 0.15)"
            echo "  --device <int>  : Webcam device index (default: 0)"
            echo "  --no-gui        : Disable OpenCV debug window"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Run './run_teleop.sh --help' for usage."
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

# Ensure ROS 2 environment is sourced
if [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
fi

if [ -f "/home/humble_ws/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source /home/humble_ws/install/setup.bash
fi

# Use CycloneDDS to prevent buffer overrun warnings on large URDF descriptions
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# OpenGL software rendering for RViz2 stability on hybrid GPUs
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY="${DISPLAY:-:1}"

echo "========================================================"
echo " Allegro Hand V4 Vision Teleoperation Launcher"
echo " Mode   : $MODE"
echo " Alpha  : $ALPHA"
echo " Device : /dev/video$DEVICE"
echo " GUI    : $([ -n "$GUI_FLAG" ] && echo "Disabled (Headless)" || echo "Enabled")"
echo " Dir    : $SCRIPT_DIR"
echo "========================================================"

# Array to keep track of background PIDs
PIDS=()

cleanup() {
    echo ""
    echo "[!] Stopping all teleoperation nodes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    echo "[✓] All nodes stopped cleanly."
}

trap cleanup SIGINT SIGTERM EXIT

case "$MODE" in
    sim)
        echo "[1/4] Launching Mock Hardware & RViz2..."
        ros2 launch allegro_hand_bringup allegro_hand.launch.py ros2_control_hardware_type:=mock_components use_sim_time:=false &
        PIDS+=($!)
        sleep 3
        ;;
    real)
        echo "[1/4] Checking CAN interface (can0)..."
        if command -v ip >/dev/null 2>&1; then
            ip link set can0 up 2>/dev/null || true
        fi
        echo "[2/4] Launching Physical Allegro Hand Hardware Controller..."
        ros2 launch allegro_hand_bringup allegro_hand.launch.py ros2_control_hardware_type:=physical_device use_sim_time:=false &
        PIDS+=($!)
        sleep 3
        ;;
    nodes)
        echo "[*] Launching Vision Teleoperation nodes only..."
        ;;
esac

echo "[+] Starting Kinematic Retargeting Node (V4)..."
python3 "$SCRIPT_DIR/retargeting_node_v4.py" --quiet &
PIDS+=($!)
sleep 0.5

echo "[+] Starting Controller Bridge Node (V4, EMA alpha=$ALPHA)..."
python3 "$SCRIPT_DIR/sim_bridge_node_v4.py" --alpha "$ALPHA" &
PIDS+=($!)
sleep 0.5

echo "[+] Starting Vision Tracker Node (V4, /dev/video$DEVICE)..."
python3 "$SCRIPT_DIR/vision_tracker_v4.py" --device "$DEVICE" $GUI_FLAG &
PIDS+=($!)

echo "========================================================"
echo " All nodes running! Press Ctrl+C in this terminal to exit."
echo "========================================================"

# Wait for all background processes
wait
