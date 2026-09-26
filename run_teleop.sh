#!/usr/bin/env bash
# ==============================================================================
# run_teleop.sh — all-in-one launcher for Allegro Hand V6 vision teleoperation.
#
# Native ROS 2 Jazzy on Ubuntu 24.04: no container, no host->docker delegation.
# Upstream (jacoblee1214/allegro_vision_teleop) re-executes itself inside
# `ros_humble_dev` at a fixed /home/humble_ws path; this port runs in place and
# takes every machine-specific value from config/teleop.env.
#
# Python nodes run from ./.venv (created by tools/setup.sh with
# --system-site-packages), because mediapipe needs numpy 1.x while rclpy,
# cv_bridge and PyQt5 come from the apt ROS 2 Jazzy install.
#
# Modes:
#   nodes (default) : teleop nodes only (hardware/sim already running elsewhere)
#   sim             : mock_components bring-up + teleop nodes (no hand needed)
#   real            : Modbus TCP bring-up on the physical hand + teleop nodes
#
# Run './run_teleop.sh --help' for the full option list.
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
cd "$REPO_DIR"

# ------------------------------------------------------------------------------
# Machine defaults. Environment wins over the file; command line flags win over both.
# ------------------------------------------------------------------------------
# shellcheck source=/dev/null
[ -f "$REPO_DIR/config/teleop.env" ] && source "$REPO_DIR/config/teleop.env"

MODE="nodes"
DESCRIPTOR="modbus_tcp:${HAND_IP:-192.168.1.100}:${HAND_PORT:-502}"
USER_IP=""
USER_SPECIFIED_HAND=false
GUI_FLAG=""
RECORD_FLAG=false
USE_RVIZ=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        nodes|sim|real)         MODE="$1"; shift ;;
        --hand)                 HAND_SIDE="$2"; USER_SPECIFIED_HAND=true; shift 2 ;;
        --alpha)                ALPHA="$2"; shift 2 ;;
        --rate|--hz)            RATE="$2"; shift 2 ;;
        --fps)                  FPS="$2"; shift 2 ;;
        --device)               DEVICE="$2"; shift 2 ;;
        --scale)                SCALE="$2"; shift 2 ;;
        --descriptor|-d)        DESCRIPTOR="$2"; shift 2 ;;
        --ip)
            USER_IP="$2"
            if [[ "$USER_IP" =~ ^[0-9]+$ ]]; then
                DESCRIPTOR="modbus_tcp:192.168.1.${USER_IP}:${HAND_PORT:-502}"
            elif [[ "$USER_IP" =~ ^[0-9.]+$ ]]; then
                DESCRIPTOR="modbus_tcp:${USER_IP}:${HAND_PORT:-502}"
            else
                DESCRIPTOR="$USER_IP"
            fi
            shift 2 ;;
        --no-gui|--headless)    GUI_FLAG="--no-gui"; UI_MODE="none"; shift ;;
        --record)               RECORD_FLAG=true; shift ;;
        --cockpit|--3d)         UI_MODE="cockpit"; shift ;;
        --classic|--dashboard|--ui) UI_MODE="classic"; shift ;;
        --simple-gui)           UI_MODE="simple"; shift ;;
        --rviz)                 USE_RVIZ=true; shift ;;
        --no-rviz)              USE_RVIZ=false; shift ;;
        -h|--help)
            cat <<'HELP'
Usage: ./run_teleop.sh [nodes|sim|real] [UI options] [HW options]

Modes:
  nodes (default) : vision tracker + retargeting + bridge nodes only
  sim             : mock_components bring-up (RViz2) + teleop nodes, no hand needed
  real            : physical hand over Modbus TCP + teleop nodes

UI options:
  --cockpit, --3d      single window with embedded 3D hand (recommended)
  --classic            2D dashboard + separate RViz2 window
  --dashboard          alias for --classic
  --simple-gui         MediaPipe OpenCV window only
  --no-gui, --headless no GUI windows
  --rviz / --no-rviz   force the separate RViz2 window on or off

Options:
  --hand <right|left>  hand model (default: right; 'real' auto-detects from register 0x0071)
  --ip <str>           hand IP: 192.168.1.101, or just 101, or a full descriptor
  --descriptor <str>   io interface descriptor (default: from config/teleop.env)
  --rate, --hz <int>   command rate to the hand [Hz]
  --fps <int>          camera capture rate
  --alpha <float>      EMA smoothing factor (0.05~0.5)
  --device <int>       camera index (default: auto-detect RealSense RGB, else first working)
  --scale <float>      camera window scale for --simple-gui
  --record             also launch the VLA dataset recorder

Machine defaults live in config/teleop.env (ALLEGRO_WS, HAND_IP, ETH_IFACE, ...).
HELP
            exit 0 ;;
        *)
            echo "Unknown argument: $1"
            echo "Run './run_teleop.sh --help' for usage."
            exit 1 ;;
    esac
done

UI_MODE="${UI_MODE:-cockpit}"
HAND_SIDE="${HAND_SIDE:-right}"
RATE="${RATE:-100}"
ALPHA="${ALPHA:-0.25}"
FPS="${FPS:-60}"
SCALE="${SCALE:-1.75}"

# RViz default follows the UI: the cockpit draws its own 3D hand.
if [ -z "$USE_RVIZ" ]; then
    case "$UI_MODE" in
        classic|simple) USE_RVIZ="true" ;;
        *)              USE_RVIZ="false" ;;
    esac
fi

# ------------------------------------------------------------------------------
# ROS 2 environment
# ------------------------------------------------------------------------------
# shellcheck source=/dev/null
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

WS_SETUP="${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation}/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
elif [ "$MODE" != "nodes" ]; then
    echo "[!] Driver workspace not found: $WS_SETUP"
    echo "[!] '$MODE' mode needs allegro_hand_v6_bringup. Build it first:"
    echo "      cd ${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation} && colcon build --symlink-install"
    exit 1
fi

# CycloneDDS handles the large URDF on /robot_description without the buffer-overrun
# warnings the default rmw logs. Only select it when it is actually installed: ros2 exits
# with "RMW implementation not installed" if the shared library is missing, and the apt
# package (ros-jazzy-rmw-cyclonedds-cpp, installed by tools/setup.sh) needs sudo.
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
    if ldconfig -p 2>/dev/null | grep -q librmw_cyclonedds_cpp.so \
       || [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/lib/librmw_cyclonedds_cpp.so" ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    else
        echo "[*] rmw_cyclonedds_cpp not installed; using the default rmw"
        echo "    (sudo apt install ros-${ROS_DISTRO:-jazzy}-rmw-cyclonedds-cpp, or ./tools/setup.sh)"
    fi
else
    export RMW_IMPLEMENTATION
fi

# This machine renders the desktop on the Intel iGPU but has an NVIDIA dGPU, and the
# 476k-triangle STL hand is ~50 ms/frame on llvmpipe vs ~1.4 ms on the dGPU. Unlike the
# upstream container, Mesa is available here, so RViz2 is left on the iGPU (no global
# LIBGL_ALWAYS_SOFTWARE) and only the cockpit is offloaded via PRIME.
COCKPIT_GL_ENV=""
if nvidia-smi >/dev/null 2>&1; then
    COCKPIT_GL_ENV="env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia"
fi
export DISPLAY="${DISPLAY:-:0}"

# Python: the repo venv if tools/setup.sh has run, otherwise system python3.
PY="$REPO_DIR/.venv/bin/python"
if [ ! -x "$PY" ]; then
    echo "[!] $REPO_DIR/.venv not found — falling back to system python3."
    echo "[!] mediapipe is probably missing; run ./tools/setup.sh once."
    PY="python3"
fi

# Release camera locks held by an earlier run that did not exit cleanly.
pkill -f "$REPO_DIR/(teleop_cockpit|teleop_dashboard|vision_tracker)\.py" 2>/dev/null || true
sleep 0.2

# ------------------------------------------------------------------------------
# Camera selection (RealSense RGB first, then the first device that really captures)
# ------------------------------------------------------------------------------
if [ -z "${DEVICE:-}" ]; then
    DETECTED_DEV=$("$PY" -c "
import glob, subprocess
import cv2

def idx_of(path):
    tail = path.replace('/dev/video', '')
    return int(tail) if tail.isdigit() else 999

devs = sorted(glob.glob('/dev/video*'), key=idx_of)

for dev in devs:
    try:
        out = subprocess.check_output(['v4l2-ctl', '-d', dev, '--all'],
                                      stderr=subprocess.DEVNULL, timeout=0.5).decode('utf-8', 'ignore')
        if 'RealSense' in out and ('YUYV' in out or 'white_balance' in out):
            print('realsense:%d' % idx_of(dev))
            raise SystemExit(0)
    except SystemExit:
        raise
    except Exception:
        pass

for dev in devs:
    idx = idx_of(dev)
    try:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None and frame.size > 0:
                name = 'Camera'
                try:
                    with open('/sys/class/video4linux/video%d/name' % idx) as fh:
                        name = fh.read().strip()
                except Exception:
                    pass
                print('%s:%d' % (name, idx))
                raise SystemExit(0)
    except SystemExit:
        raise
    except Exception:
        pass
" 2>/dev/null || true)

    if [[ "$DETECTED_DEV" =~ realsense:([0-9]+) ]]; then
        DEVICE="${BASH_REMATCH[1]}"
        CAM_DESC="Intel RealSense RGB (/dev/video$DEVICE [auto])"
    elif [[ "$DETECTED_DEV" =~ (.*):([0-9]+)$ ]]; then
        DEVICE="${BASH_REMATCH[2]}"
        CAM_DESC="${BASH_REMATCH[1]} (/dev/video$DEVICE [auto])"
    else
        DEVICE="0"
        CAM_DESC="Default camera (/dev/video0) — auto-detect found nothing"
    fi
else
    CAM_DESC="/dev/video$DEVICE (user specified)"
fi

echo "========================================================"
echo " Allegro Hand V6 (5-finger, 20-DOF) Vision Teleoperation"
echo " Mode       : $MODE"
echo " Rate       : ${RATE} Hz    EMA alpha: $ALPHA"
echo " Camera     : $CAM_DESC (capture ${FPS} FPS)"
echo " Descriptor : $DESCRIPTOR"
echo " Hand side  : ${HAND_SIDE} (default/detected)"
echo " UI mode    : $UI_MODE"
echo " RViz2      : $([ "$USE_RVIZ" = true ] && echo "enabled (separate window)" || echo "disabled")"
echo " Python     : $PY"
echo " ROS        : ${ROS_DISTRO:-?}   rmw: ${RMW_IMPLEMENTATION:-default}"
echo "========================================================"

PIDS=()

cleanup() {
    echo ""
    echo "[!] Stopping all teleoperation nodes..."
    for pid in "${PIDS[@]}"; do
        kill -0 "$pid" 2>/dev/null && kill -INT "$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "${PIDS[@]}"; do
        kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
    done
    pkill -P $$ 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[OK] All nodes stopped."
}
trap cleanup SIGINT SIGTERM EXIT

BRINGUP="$REPO_DIR/launch/allegro_hand_bringup.launch.py"

case "$MODE" in
    sim)
        echo "[1/4] Launching mock_components bring-up ($HAND_SIDE hand, rviz:=$USE_RVIZ)..."
        ros2 launch "$BRINGUP" \
            hardware:=mock_components \
            hand:="$HAND_SIDE" \
            rviz:="$USE_RVIZ" \
            joint_states_topic:=/allegro/joint_states_urdf &
        PIDS+=($!)
        sleep 3
        ;;
    real)
        echo "[1/4] Checking hand connection ($DESCRIPTOR)..."
        # The PC needs an address on the hand's subnet. Never 192.168.1.100 itself:
        # the PC would answer its own pings and the hand looks reachable while it is not.
        for iface in "${ETH_IFACE:-enp129s0}" eth0; do
            if [ -d "/sys/class/net/$iface" ]; then
                ip addr add "${PC_IP:-192.168.1.10/24}" dev "$iface" 2>/dev/null || true
                ip link set "$iface" up 2>/dev/null || true
            fi
        done

        # Only probe alternatives when the user did not name an address.
        if [ -z "$USER_IP" ] && [[ "$DESCRIPTOR" =~ modbus_tcp:${HAND_IP:-192.168.1.100}:${HAND_PORT:-502} ]]; then
            FOUND_IP=$("$PY" -c "
import socket
for ip in ['${HAND_IP:-192.168.1.100}', '192.168.1.101', '192.168.1.201']:
    try:
        s = socket.socket()
        s.settimeout(0.3)
        s.connect((ip, ${HAND_PORT:-502}))
        # Allegro Hand identity: holding register 0x0070 = firmware version
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
                DESCRIPTOR="modbus_tcp:${FOUND_IP}:${HAND_PORT:-502}"
                [ "$FOUND_IP" != "${HAND_IP:-192.168.1.100}" ] && \
                    echo "[OK] Allegro Hand found at ${FOUND_IP}:${HAND_PORT:-502} (not the configured address)"
            fi
        fi

        if [[ "$DESCRIPTOR" =~ modbus_tcp:([0-9.]+):([0-9]+) ]]; then
            TARGET_IP="${BASH_REMATCH[1]}"
            TARGET_PORT="${BASH_REMATCH[2]}"
            if ! ping -c 1 -W 1 "$TARGET_IP" >/dev/null 2>&1; then
                echo "--------------------------------------------------------"
                echo "[!] Cannot ping the hand at $TARGET_IP."
                echo "[!] ros2_control will fail to activate and RViz gets no joint states. Check:"
                echo "      1. 24 V robot power is ON"
                echo "      2. Ethernet cable seated (iface ${ETH_IFACE:-enp129s0})"
                echo "      3. PC address on that subnet: ip addr show ${ETH_IFACE:-enp129s0}"
                echo "[!] To test without the hand: ./run_teleop.sh sim --hand $HAND_SIDE"
                echo "--------------------------------------------------------"
            else
                echo "[OK] Hand at $TARGET_IP is reachable."
                if [ "$USER_SPECIFIED_HAND" = false ]; then
                    AUTO_HAND=$("$PY" -c "
import socket, struct
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.6)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
    s.connect(('$TARGET_IP', $TARGET_PORT))
    # Read holding register 0x0071: 1 = right hand, otherwise left
    s.sendall(b'\x00\x01\x00\x00\x00\x06\x01\x03\x00\x71\x00\x01')
    res = s.recv(32)
    s.close()
    if len(res) >= 11 and res[7] == 3:
        print('right' if int.from_bytes(res[9:11], 'big') == 1 else 'left')
except Exception:
    pass
" 2>/dev/null || true)
                    if [ -n "$AUTO_HAND" ]; then
                        HAND_SIDE="$AUTO_HAND"
                        echo "[OK] Hand type auto-detected from register 0x0071: $HAND_SIDE"
                    else
                        echo "[*] Auto-detect failed; using default hand type: $HAND_SIDE"
                    fi
                else
                    echo "[*] Hand type given by user: $HAND_SIDE"
                fi
                # Let the MCU TCP stack settle before ros2_control connects.
                sleep 0.4
            fi
        fi

        echo "[1/4] Launching hardware bring-up ($DESCRIPTOR, hand:=$HAND_SIDE, rviz:=$USE_RVIZ)..."
        ros2 launch "$BRINGUP" \
            hardware:=hardware \
            descriptor:="$DESCRIPTOR" \
            hand_id:="${HAND_ID:-1}" \
            hand:="$HAND_SIDE" \
            rviz:="$USE_RVIZ" \
            joint_states_topic:=/allegro/joint_states_urdf &
        PIDS+=($!)
        sleep 3
        ;;
    nodes)
        echo "[*] Teleop nodes only (bring-up assumed to be running elsewhere)."
        ;;
esac

echo "[2/4] Kinematic retargeting node (hand: $HAND_SIDE)..."
"$PY" "$REPO_DIR/retargeting_node.py" --quiet --hand "$HAND_SIDE" &
PIDS+=($!)
sleep 0.5

echo "[3/4] Controller bridge node (${RATE} Hz, alpha=$ALPHA, mode: $MODE, hand: $HAND_SIDE)..."
"$PY" "$REPO_DIR/sim_bridge_node.py" --rate "$RATE" --alpha "$ALPHA" --mode "$MODE" --hand "$HAND_SIDE" &
PIDS+=($!)
sleep 0.5

case "$UI_MODE" in
    cockpit)
        echo "[4/4] Cockpit UI (embedded 3D hand @ ${FPS} FPS, hand: $HAND_SIDE)..."
        $COCKPIT_GL_ENV "$PY" "$REPO_DIR/teleop_cockpit.py" \
            --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" --mode "$MODE" &
        PIDS+=($!)
        ;;
    classic)
        echo "[4/4] Classic dashboard (@ ${FPS} FPS, hand: $HAND_SIDE) + RViz2..."
        "$PY" "$REPO_DIR/teleop_dashboard.py" \
            --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" --mode "$MODE" &
        PIDS+=($!)
        ;;
    simple)
        echo "[4/4] MediaPipe OpenCV window ($CAM_DESC, ${SCALE}x, ${FPS} FPS)..."
        "$PY" "$REPO_DIR/vision_tracker.py" --device "$DEVICE" --scale "$SCALE" --fps "$FPS" &
        PIDS+=($!)
        ;;
    none|*)
        echo "[4/4] Headless vision tracker ($CAM_DESC, ${FPS} FPS)..."
        "$PY" "$REPO_DIR/vision_tracker.py" --device "$DEVICE" --fps "$FPS" --no-gui &
        PIDS+=($!)
        ;;
esac

if [ "$RECORD_FLAG" = true ]; then
    sleep 0.5
    echo "[+] VLA dataset recorder (20-DOF @ ${RATE} Hz)..."
    "$PY" "$REPO_DIR/dataset_recorder.py" --sample-hz "$RATE" --dof 20 &
    PIDS+=($!)
fi

echo "========================================================"
echo " Running. Press Ctrl+C in this terminal to stop everything."
echo "========================================================"
wait
