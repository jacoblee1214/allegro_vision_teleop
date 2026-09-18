#!/usr/bin/env bash
# ==============================================================================
# run_teleop_v6_1.sh — All-in-One Launcher for Allegro Hand V6 Vision Teleoperation (v6_1)
#
# v6_1 vs v6: left-hand MCP motor offset (-90 deg) is converted in ONE place (sim_bridge_node_v6_1):
#   commands  URDF -> motor : joint11/21/31/41 -= pi/2   (real + left only)
#   feedback  motor -> URDF : /joint_states -> /allegro/joint_states_urdf
# RViz (robot_state_publisher), cockpit/dashboard and dataset recorder all read /allegro/joint_states_urdf.
# Requires allegro_hand_v6_bringup/launch/allegro_hand_v6_1.launch.py (Wonik package patch, see README).
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
        ros_humble_dev /home/humble_ws/allegro_vision_teleop/run_teleop_v6_1.sh "$@"
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
UI_MODE="classic"      # UI Mode: 'cockpit' (embedded 3D), 'classic' (dashboard + RViz), 'simple' (OpenCV only), 'none'
HAND_SIDE="right"      # Default to 'right' for Right Hand model
USER_SPECIFIED_HAND=false
USE_RVIZ=""

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
            UI_MODE="none"
            shift
            ;;
        --record)
            RECORD_FLAG=true
            shift
            ;;
        --cockpit|--3d)
            UI_MODE="cockpit"
            shift
            ;;
        --classic|--dashboard|--ui)
            UI_MODE="classic"
            shift
            ;;
        --simple-gui)
            UI_MODE="simple"
            shift
            ;;
        --rviz)
            USE_RVIZ=true
            shift
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        -h|--help)
            echo "Usage: ./run_v6_1.sh [nodes|sim|real] [UI options] [HW options]"
            echo ""
            echo "Modes:"
            echo "  nodes (default) : Launch vision tracker, retargeting, and bridge nodes"
            echo "  sim             : Launch Allegro Hand V6 mock hardware + teleop nodes"
            echo "  real            : Launch Allegro Hand V6 physical hardware (Modbus TCP/RTU) + teleop nodes"
            echo ""
            echo "UI Options:"
            echo "  --cockpit, --3d      : Launch Unified 3D Cockpit UI (Single Window, embedded 3D robot hand)"
            echo "  --classic            : Launch Classic 2D Teleop Dashboard + RViz2 3D window (Default)"
            echo "  --dashboard          : Alias for --classic"
            echo "  --simple-gui         : Launch MediaPipe OpenCV window only"
            echo "  --no-gui, --headless : Run headless (no GUI windows)"
            echo "  --rviz / --no-rviz   : Force enable or disable separate RViz2 window"
            echo ""
            echo "Options:"
            echo "  --hand <right|left>  : Hand side model (default: 'right', or auto-detected from hardware register 0x0071)"
            echo "  --ip <str>           : Target robot IP (e.g. 192.168.1.101, 101, or 192.168.1.100)"
            echo "  --rate, --hz <int>   : Command publishing frequency to robot (default: 100 Hz)"
            echo "  --fps <int>          : Camera capture frame rate (default: 60 FPS)"
            echo "  --alpha <float>      : EMA smoothing factor (default: 0.25, range: 0.05~0.5)"
            echo "  --device <int>       : Camera device index (default: auto-detect Intel RealSense RGB -> /dev/video8, or 0)"
            echo "  --scale <float>      : Camera GUI display scale (default: 1.75)"
            echo "  --descriptor <str>   : Hardware descriptor (default: modbus_tcp:192.168.1.100:502)"
            echo "  --record             : Launch VLA dataset recorder node"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Run './run_v6_1.sh --help' for usage."
            exit 1
            ;;
    esac
done

# Configure RViz default based on UI Mode if not explicitly overridden
if [ -z "$USE_RVIZ" ]; then
    if [ "$UI_MODE" = "cockpit" ]; then
        USE_RVIZ="false"   # Cockpit has embedded 3D robot hand widget
    elif [ "$UI_MODE" = "classic" ] || [ "$UI_MODE" = "simple" ]; then
        USE_RVIZ="true"    # Classic dashboard / simple GUI uses separate RViz2 window
    else
        USE_RVIZ="false"
    fi
fi

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
if [ ! -f "$SCRIPT_DIR/sim_bridge_node_v6_1.py" ]; then
    if [ -f "/home/humble_ws/allegro_vision_teleop/retargeting_node_v6.py" ]; then
        SCRIPT_DIR="/home/humble_ws/allegro_vision_teleop"
    elif [ -f "/home/jake/humble_ws/allegro_vision_teleop/retargeting_node_v6.py" ]; then
        SCRIPT_DIR="/home/jake/humble_ws/allegro_vision_teleop"
    fi
fi

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

# 1.1 Clean up any stale orphaned teleop processes that might hold camera video device locks
pkill -f 'teleop_cockpit_v6(_1)?\.py|teleop_dashboard_v6(_1)?\.py|vision_tracker_v6\.py' 2>/dev/null || true
sleep 0.2

# 2. Camera selection & RealSense/Webcam auto-detection
if [ -z "$DEVICE" ]; then
    DETECTED_DEV=$(python3 -c "
import glob, cv2, subprocess

# 1. First prioritize Intel RealSense if available
for dev in sorted(glob.glob('/dev/video*'), key=lambda x: int(x.replace('/dev/video', '')) if x.replace('/dev/video', '').isdigit() else 999):
    try:
        out = subprocess.check_output(['v4l2-ctl', '-d', dev, '--all'], stderr=subprocess.DEVNULL, timeout=0.5).decode('utf-8', errors='ignore')
        if 'RealSense' in out and ('YUYV' in out or 'white_balance' in out):
            print(f'realsense:{dev.replace(\"/dev/video\", \"\")}')
            raise SystemExit(0)
    except Exception:
        pass

# 2. Probe working capture device using OpenCV
dev_paths = sorted(glob.glob('/dev/video*'), key=lambda x: int(x.replace('/dev/video', '')) if x.replace('/dev/video', '').isdigit() else 999)
for dev in dev_paths:
    idx = int(dev.replace('/dev/video', ''))
    try:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None and frame.size > 0:
                name = 'Camera'
                try:
                    with open(f'/sys/class/video4linux/video{idx}/name') as f:
                        name = f.read().strip()
                except Exception:
                    pass
                print(f'{name}:{idx}')
                raise SystemExit(0)
    except Exception:
        pass
" 2>/dev/null || true)

    if [[ "$DETECTED_DEV" =~ realsense:([0-9]+) ]]; then
        DEVICE="${BASH_REMATCH[1]}"
        CAM_DESC="Intel RealSense RGB Camera (/dev/video$DEVICE [Auto-detected])"
    elif [[ "$DETECTED_DEV" =~ (.*):([0-9]+) ]]; then
        CAM_NAME="${BASH_REMATCH[1]}"
        DEVICE="${BASH_REMATCH[2]}"
        CAM_DESC="$CAM_NAME (/dev/video$DEVICE [Auto-detected])"
    else
        DEVICE="0"
        CAM_DESC="Default Camera (/dev/video0)"
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
echo " Hand Side  : ${HAND_SIDE^^} Hand (default/detected)"
echo " UI Mode    : $UI_MODE ($([ "$UI_MODE" = "cockpit" ] && echo "Unified 3D Cockpit" || ([ "$UI_MODE" = "classic" ] && echo "Classic Dashboard" || echo "$UI_MODE")))"
echo " RViz2      : $([ "$USE_RVIZ" = true ] && echo "Enabled (Separate Window)" || echo "Disabled")"
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
        echo "[1/4] Launching Allegro Hand V6 ($HAND_SIDE hand) Mock Hardware ($([ "$USE_RVIZ" = true ] && echo "with RViz2" || echo "Headless + Cockpit 3D"))..."
        ros2 launch allegro_hand_v6_bringup allegro_hand_v6_1.launch.py joint_states_topic:=/allegro/joint_states_urdf ros2_control_hardware_type:=mock_components hand:="$HAND_SIDE" use_rviz:="$USE_RVIZ" &
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

        # Check primary default IP (192.168.1.100) first; fallback to other candidates only if 100 is unreachable
        if [ -z "$USER_IP" ] && [[ "$DESCRIPTOR" =~ modbus_tcp:192.168.1.100:502 ]]; then
            FOUND_IP=$(python3 -c "
import socket
for ip in ['192.168.1.100', '192.168.1.101', '192.168.1.201']:
    try:
        s = socket.socket()
        s.settimeout(0.3)
        s.connect((ip, 502))
        # Verify Allegro Hand identity: Holding Register 0x0070 (FW version >= 0x0100)
        s.sendall(b'\x00\x01\x00\x00\x00\x06\x01\x03\x00\x70\x00\x01')
        res = s.recv(32)
        s.close()
        if len(res) >= 9 and res[7] == 3:
            print(ip)
            break
    except Exception:
        pass
" 2>/dev/null || true)
            if [ -n "$FOUND_IP" ]; then
                DESCRIPTOR="modbus_tcp:${FOUND_IP}:502"
                if [ "$FOUND_IP" != "192.168.1.100" ]; then
                    echo "[✓] Allegro Hand fallback discovered at ${FOUND_IP}:502"
                fi
            fi
        fi

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
                echo "[!] (Tip: If testing without physical hand, run 'run_v6_1 sim' instead)"
                echo "--------------------------------------------------------"
            else
                echo "[✓] Allegro Hand V6 at $TARGET_IP is reachable!"
                # Auto-detect Hand Type (Left vs Right) from Modbus Register 0x0071 if not manually specified
                if [ "$USER_SPECIFIED_HAND" = false ]; then
                    AUTO_HAND=$(python3 -c "
import socket, struct
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.6)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
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
                # Allow MCU TCP stack 0.4s to be cleanly ready for ros2_control connection
                sleep 0.4
            fi
        fi
        echo "[1/4] Launching Physical Allegro Hand V6 Hardware Interface ($DESCRIPTOR, hand:=$HAND_SIDE, rviz:=$USE_RVIZ)..."
        ros2 launch allegro_hand_v6_bringup allegro_hand_v6_1.launch.py joint_states_topic:=/allegro/joint_states_urdf ros2_control_hardware_type:=hardware io_interface_descriptor:="$DESCRIPTOR" hand:="$HAND_SIDE" use_rviz:="$USE_RVIZ" &
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

echo "[+] Starting Controller Bridge Node (V6, ${RATE} Hz, EMA alpha=$ALPHA, mode: $MODE, hand: $HAND_SIDE)..."
python3 "$SCRIPT_DIR/sim_bridge_node_v6_1.py" --rate "$RATE" --alpha "$ALPHA" --mode "$MODE" --hand "$HAND_SIDE" &
PIDS+=($!)
sleep 0.5

case "$UI_MODE" in
    cockpit)
        echo "[+] Starting Unified 3D Cockpit Dashboard (V6, PyQt5 with Embedded 3D Hand @ ${FPS} FPS, hand: $HAND_SIDE, mode: $MODE)..."
        python3 "$SCRIPT_DIR/teleop_cockpit_v6_1.py" --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" --mode "$MODE" &
        PIDS+=($!)
        ;;
    classic)
        echo "[+] Starting Classic 2D Teleop Dashboard (V6, PyQt5 Gauge UI @ ${FPS} FPS, hand: $HAND_SIDE, mode: $MODE)..."
        python3 "$SCRIPT_DIR/teleop_dashboard_v6_1.py" --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" --mode "$MODE" &
        PIDS+=($!)
        ;;
    simple)
        echo "[+] Starting MediaPipe Vision Tracker Node (V6, $CAM_DESC, ${SCALE}x scale, ${FPS} FPS)..."
        python3 "$SCRIPT_DIR/vision_tracker_v6.py" --device "$DEVICE" --scale "$SCALE" --fps "$FPS" &
        PIDS+=($!)
        ;;
    none|*)
        echo "[+] Starting Headless MediaPipe Vision Tracker Node (V6, $CAM_DESC, ${FPS} FPS)..."
        python3 "$SCRIPT_DIR/vision_tracker_v6.py" --device "$DEVICE" --fps "$FPS" --no-gui &
        PIDS+=($!)
        ;;
esac

if [ "$RECORD_FLAG" = true ]; then
    sleep 0.5
    echo "[+] Starting VLA Dataset Recorder Node (V6, 20-DOF @ ${RATE}Hz)..."
    python3 "$SCRIPT_DIR/dataset_recorder_v6_1.py" --sample-hz "$RATE" --dof 20 &
    PIDS+=($!)
fi

echo "========================================================"
echo " All nodes running! Press Ctrl+C in this terminal to exit."
echo "========================================================"

# Wait for all background processes
wait
