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

# ------------------------------------------------------------------------------
# Auto-delegation: If executed directly on the host machine, run inside Docker
# ------------------------------------------------------------------------------
if [ ! -f "/.dockerenv" ] && [ "${RUN_ON_HOST:-0}" != "1" ]; then
    echo "========================================================"
    echo "[*] Host machine detected: $(hostname)"
    echo "[*] Connecting to Docker container 'ros_humble_dev'..."

    # 1. Allow container access to local X11 display
    xhost +local:docker >/dev/null 2>&1 || true

    # 2. Check if container is running; start if stopped
    if ! docker ps --format '{{.Names}}' | grep -q "^ros_humble_dev$"; then
        echo "[*] Starting ros_humble_dev container..."
        docker start ros_humble_dev >/dev/null
    fi

    # 3. Interactive TTY flags
    DOCKER_FLAGS="-i"
    if [ -t 0 ] && [ -t 1 ]; then
        DOCKER_FLAGS="-it"
    fi

    echo "[*] Executing inside ros_humble_dev (with GUI forwarding)..."
    echo "========================================================"

    exec docker exec $DOCKER_FLAGS \
        -e DISPLAY="${DISPLAY:-:1}" \
        ros_humble_dev /home/humble_ws/allegro_vision_teleop/run_teleop_v6.sh "$@"
fi

MODE="nodes"
ALPHA="0.25"
DEVICE=""
GUI_FLAG=""
DESCRIPTOR="modbus_tcp:192.168.1.100:502"
SCALE="1.75"
RATE="100"
FPS="60"
RECORD_FLAG=false
USE_DASHBOARD=true
HAND_SIDE="left"  # Default to 'left' since connected hardware is Left Hand (P6LA0020)
USER_SPECIFIED_HAND=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        nodes|sim|real)
            MODE="$1"
            shift
            ;;
        --hand)
            HAND_SIDE="$2"
            USER_SPECIFIED_HAND=true
            shift 2
            ;;
        --alpha)
            ALPHA="$2"
            shift 2
            ;;
        --rate|--hz)
            RATE="$2"
            shift 2
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --scale)
            SCALE="$2"
            shift 2
            ;;
        --descriptor|-d)
            DESCRIPTOR="$2"
            shift 2
            ;;
        --ip)
            USER_IP="$2"
            if [[ "$USER_IP" =~ ^[0-9]+$ ]]; then
                DESCRIPTOR="modbus_tcp:192.168.1.${USER_IP}:502"
            elif [[ "$USER_IP" =~ ^[0-9.]+$ ]]; then
                DESCRIPTOR="modbus_tcp:${USER_IP}:502"
            else
                DESCRIPTOR="$USER_IP"
            fi
            shift 2
            ;;
        --no-gui|--headless)
            GUI_FLAG="--no-gui"
            shift
            ;;
        --record)
            RECORD_FLAG=true
            shift
            ;;
        --simple-gui)
            USE_DASHBOARD=false
            shift
            ;;
        --dashboard|--ui)
            USE_DASHBOARD=true
            shift
            ;;
        -h|--help)
            echo "Usage: ./run_teleop.sh [nodes|sim|real] [--hand left|right] [--ip <ip|end_num>] [--rate 60|100] [--alpha 0.20] [--device 8] [--scale 1.75] [--fps 60] [--descriptor <desc>] [--no-gui]"
            echo ""
            echo "Modes:"
            echo "  nodes (default) : Launch vision_tracker, retargeting_node, sim_bridge_node"
            echo "  sim             : Launch Allegro Hand V6 mock hardware + RViz2 + 3 teleop nodes"
            echo "  real            : Launch Allegro Hand V6 physical hardware (Modbus TCP/RTU) + 3 teleop nodes"
            echo ""
            echo "Options:"
            echo "  --hand <left|right>  : Hand side model (default: auto-detected from hardware register 0x0071, or 'left')"
            echo "  --ip <str>           : Target robot IP (e.g. 192.168.1.101, 101, or 192.168.1.100)"
            echo "  --rate, --hz <int>   : Command publishing frequency to robot (default: 100 Hz, e.g. 60 or 100 for tremor suppression)"
            echo "  --fps <int>          : Camera capture frame rate (default: 60 FPS)"
            echo "  --alpha <float>      : EMA smoothing factor (default: 0.25, range: 0.05~0.5)"
            echo "  --device <int>       : Camera device index (default: auto-detect Intel RealSense RGB -> /dev/video8, or fallback to 0)"
            echo "  --scale <float>      : Camera GUI display scale (default: 1.75 -> 1120x840, resizable)"
            echo "  --descriptor <str>   : Hardware descriptor (default: modbus_tcp:192.168.1.100:502)"
            echo "  --record             : Launch VLA dataset recorder node (auto-records episodes on 'R' and tags on 'S')"
            echo "  --no-gui             : Disable OpenCV debug window"
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

# 1. Ensure all host video device nodes exist inside container
for sys_dev in /sys/class/video4linux/video*; do
    if [ -d "$sys_dev" ]; then
        v_idx=$(basename "$sys_dev" | sed 's/video//')
        if [ ! -e "/dev/video$v_idx" ] && [ -f "$sys_dev/dev" ]; then
            dev_maj_min=$(cat "$sys_dev/dev")
            mknod "/dev/video$v_idx" c "${dev_maj_min%:*}" "${dev_maj_min#*:}" 2>/dev/null && chmod 666 "/dev/video$v_idx" 2>/dev/null || true
        fi
    fi
done

# 2. Camera selection & RealSense auto-detection
if [ -z "$DEVICE" ]; then
    DETECTED_DEV=$(python3 -c "
import glob, subprocess
for dev in sorted(glob.glob('/dev/video*'), key=lambda x: int(x.replace('/dev/video', '')) if x.replace('/dev/video', '').isdigit() else 999):
    try:
        out = subprocess.check_output(['v4l2-ctl', '-d', dev, '--all'], stderr=subprocess.DEVNULL, timeout=1.0).decode('utf-8', errors='ignore')
        if 'RealSense' in out and ('YUYV' in out or 'white_balance' in out):
            print(dev.replace('/dev/video', ''))
            break
    except Exception:
        pass
" 2>/dev/null || true)

    if [ -n "$DETECTED_DEV" ]; then
        DEVICE="$DETECTED_DEV"
        CAM_DESC="Intel RealSense RGB Camera (/dev/video$DEVICE [Auto-detected])"
    else
        DEVICE="0"
        CAM_DESC="Default Webcam (/dev/video0)"
    fi
else
    CAM_DESC="/dev/video$DEVICE (User specified)"
fi

echo "========================================================"
echo " Allegro Hand V6 (5-Finger) Vision Teleoperation Launcher"
echo " Mode       : $MODE"
echo " Rate (Hz)  : ${RATE} Hz (Command Frequency to Robot)"
echo " Alpha      : $ALPHA"
echo " Camera     : $CAM_DESC (Capture: ${FPS} FPS)"
echo " Descriptor : $DESCRIPTOR"
echo " Scale      : ${SCALE}x"
echo " GUI        : $([ -n "$GUI_FLAG" ] && echo "Disabled (Headless)" || echo "Enabled")"
echo " Dir        : $SCRIPT_DIR"
echo "========================================================"

# Array to keep track of background PIDs
PIDS=()

cleanup() {
    echo ""
    echo "[!] Stopping all teleoperation nodes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    sleep 0.5
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    pkill -P $$ 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[✓] All nodes stopped cleanly."
}

trap cleanup SIGINT SIGTERM EXIT

case "$MODE" in
    sim)
        echo "[1/4] Launching Allegro Hand V6 ($HAND_SIDE hand) Mock Hardware & RViz2..."
        ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py ros2_control_hardware_type:=mock_components hand:="$HAND_SIDE" &
        PIDS+=($!)
        sleep 3
        ;;
    real)
        echo "[1/4] Checking Allegro Hand V6 hardware connection ($DESCRIPTOR)..."
        # Auto-configure host Ethernet interface if needed
        for iface in enp129s0 eth0; do
            if [ -d "/sys/class/net/$iface" ]; then
                ip addr add 192.168.1.10/24 dev "$iface" 2>/dev/null || true
                ip addr add 192.168.40.10/24 dev "$iface" 2>/dev/null || true
                ip link set "$iface" up 2>/dev/null || true
            fi
        done

        if [[ "$DESCRIPTOR" =~ modbus_tcp:([0-9.]+):([0-9]+) ]]; then
            TARGET_IP="${BASH_REMATCH[1]}"
            if ! ping -c 1 -W 1 "$TARGET_IP" >/dev/null 2>&1; then
                echo "--------------------------------------------------------"
                echo "[!] WARNING: Cannot ping Allegro Hand at $TARGET_IP!"
                echo "[!] If the hand is powered off or disconnected, ros2_control will fail to activate and RViz cannot receive joint states."
                echo "[!] Please verify:"
                echo "    1. Robot 24V power supply is ON."
                echo "    2. Ethernet cable is securely connected."
                echo "    3. PC Ethernet IP is set (e.g. 192.168.1.10/24)."
                echo "[!] (Tip: If testing without physical hand, run 'run_v6 sim' instead)"
                echo "--------------------------------------------------------"
            else
                echo "[✓] Allegro Hand V6 at $TARGET_IP is reachable!"
                # Auto-detect Hand Type (Left vs Right) from Modbus Register 0x0071 if not manually specified
                if [ "$USER_SPECIFIED_HAND" = false ]; then
                    AUTO_HAND=$(python3 -c "
import socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.6)
    s.connect(('$TARGET_IP', int('${BASH_REMATCH[2]}')))
    # Modbus TCP Read Holding Register 0x0071 (count: 1)
    s.sendall(b'\x00\x01\x00\x00\x00\x06\x01\x03\x00\x71\x00\x01')
    res = s.recv(32)
    s.close()
    if len(res) >= 11 and res[7] == 3:
        val = int.from_bytes(res[9:11], 'big')
        print('right' if val == 1 else 'left')
except Exception:
    pass
" 2>/dev/null || true)
                    if [ -n "$AUTO_HAND" ]; then
                        HAND_SIDE="$AUTO_HAND"
                        echo "[✓] Hardware auto-detected hand type: $HAND_SIDE hand (Modbus register 0x0071)"
                    else
                        echo "[*] Using default hand type: $HAND_SIDE hand"
                    fi
                else
                    echo "[*] Hand type specified by user: $HAND_SIDE hand"
                fi
            fi
        fi
        echo "[1/4] Launching Physical Allegro Hand V6 Hardware Interface ($DESCRIPTOR, hand:=$HAND_SIDE)..."
        ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py ros2_control_hardware_type:=hardware io_interface_descriptor:="$DESCRIPTOR" hand:="$HAND_SIDE" &
        PIDS+=($!)
        sleep 3
        ;;
    nodes)
        echo "[*] Launching Vision Teleoperation nodes only..."
        ;;
esac

echo "[+] Starting Kinematic Retargeting Node (V6, hand: $HAND_SIDE)..."
python3 "$SCRIPT_DIR/retargeting_node_v6.py" --quiet --hand "$HAND_SIDE" &
PIDS+=($!)
sleep 0.5

echo "[+] Starting Controller Bridge Node (V6, ${RATE} Hz, EMA alpha=$ALPHA)..."
python3 "$SCRIPT_DIR/sim_bridge_node_v6.py" --rate "$RATE" --alpha "$ALPHA" &
PIDS+=($!)
sleep 0.5

if [ "$USE_DASHBOARD" = true ] && [ -z "$GUI_FLAG" ]; then
    echo "[+] Starting Unified Teleoperation Cockpit Dashboard (V6, PyQt5 @ ${FPS} FPS, hand: $HAND_SIDE)..."
    python3 "$SCRIPT_DIR/teleop_dashboard_v6.py" --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" &
    PIDS+=($!)
else
    echo "[+] Starting MediaPipe Vision Tracker Node (V6, $CAM_DESC, ${SCALE}x scale, ${FPS} FPS)..."
    python3 "$SCRIPT_DIR/vision_tracker_v6.py" --device "$DEVICE" --scale "$SCALE" --fps "$FPS" $GUI_FLAG &
    PIDS+=($!)
fi

if [ "$RECORD_FLAG" = true ]; then
    sleep 0.5
    echo "[+] Starting VLA Dataset Recorder Node (V6, 20-DOF @ ${RATE}Hz)..."
    python3 "$SCRIPT_DIR/dataset_recorder.py" --sample-hz "$RATE" --dof 20 &
    PIDS+=($!)
fi

echo "========================================================"
echo " All nodes running! Press Ctrl+C in this terminal to exit."
echo "========================================================"

# Wait for all background processes
wait
