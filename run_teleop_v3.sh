#!/usr/bin/env bash
# ==============================================================================
# run_teleop_v3.sh — all-in-one launcher for Allegro Hand V6 vision teleoperation (v3).
#
# v3 on top of v2 (everything below still applies):
#   * retargeting_node_v3.py: median + EMA on the landmarks before DexPilot, which
#     roughly halves the trembling (13.8 -> 5.9 mrad/frame on a held-still hand
#     with 3 mm of noise), and a warm-start reset when tracking drops (--smooth, --ema)
#   * teleop_dashboard_v3.py: Wonik Robotics look + tactile pressure panel (18 ch)
#   * hand discovery checks each candidate is an Allegro Hand and, with --hand,
#     picks the one reporting that side
#
# v2 = DexPilot retargeting. v1 mapped bone angles to joints with hand-tuned
# gains and never checked where the robot's fingertips ended up, so a pinch
# closed on screen and not on the hand (the thumb stopped ~0.6 rad short of
# where it needed to be). v2 solves for the fingertip positions every frame,
# using the retargeting core, V6 Force URDF and measured pad offsets from the
# MANUS teleop project. Measured: 129 mm between the pads before, 6 mm now
# (./tools/check_pinch_v2.py).
#
# Three things change against run_teleop.sh, nothing else:
#   * the producers are the *_v2.py ones, which also publish MediaPipe's metric
#     world landmarks on /allegro/vision/world_landmarks
#   * the retargeting node is retargeting_node_v2.py (v3: retargeting_node_v3.py)
#   * that node runs on the MANUS project's venv, because DexPilot needs
#     dex_retargeting + pinocchio (numpy 2.x) while mediapipe here needs
#     numpy 1.x. They are separate processes, so the split costs nothing.
#
# run_teleop.sh is untouched and still runs the v1 pipeline.
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
# V6F_TELEOP_ROOT points at the MANUS checkout that owns the retargeting core
# (default ~/v6f_manuse_teleoperation/V6_Teleoperation); set it in
# config/teleop.env or config/teleop_v2.env if it lives elsewhere.
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
# v2-only overrides, so the shared file stays the one both launchers read.
# shellcheck source=/dev/null
[ -f "$REPO_DIR/config/teleop_v2.env" ] && source "$REPO_DIR/config/teleop_v2.env"

MODE="nodes"
DESCRIPTOR="modbus_tcp:${HAND_IP:-192.168.1.100}:${HAND_PORT:-502}"
USER_IP=""
USER_SPECIFIED_HAND=false
GUI_FLAG=""
RECORD_FLAG=false
USE_RVIZ=""
SMOOTH=""          # v3: landmark median window for the retargeting node (default in the node: 5)
INPUT_EMA=""       # v3: landmark EMA for the retargeting node (default in the node: 0.3)
TACTILE_TOPIC=""   # v3: tactile pressure topic for the dashboard (default: discovered)

while [[ $# -gt 0 ]]; do
    case "$1" in
        nodes|sim|real)         MODE="$1"; shift ;;
        --hand)                 HAND_SIDE="$2"; USER_SPECIFIED_HAND=true; shift 2 ;;
        --alpha)                ALPHA="$2"; shift 2 ;;
        --scaling)              SCALING="$2"; shift 2 ;;
        --calibrate)            CALIBRATE="$2"; shift 2 ;;
        --no-undo-mirror)       UNDO_MIRROR=false; shift ;;
        --retarget-alpha)       RETARGET_ALPHA="$2"; shift 2 ;;
        --smooth)               SMOOTH="$2"; shift 2 ;;
        --ema|--input-ema)      INPUT_EMA="$2"; shift 2 ;;
        --tactile-topic)        TACTILE_TOPIC="$2"; shift 2 ;;
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
Usage: ./run_teleop_v3.sh [nodes|sim|real] [UI options] [HW options]
       (v2 = DexPilot retargeting; run_teleop.sh is the v1 pipeline)

Modes:
  nodes (default) : vision tracker + DexPilot retargeting + bridge nodes only
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

DexPilot options (v2 only):
  --scaling <float>    fixed operator/robot finger-length ratio (skips measuring)
  --calibrate <int>    frames to measure that ratio over (default 150, 0 = off)
  --no-undo-mirror     keep the camera's horizontal flip in the keypoints.
                       Use this only if the robot thumb opposes the wrong way.
  --retarget-alpha <f> DexPilot output filter (default 0.25 from the config).
  --smooth <int>       v3: median over this many landmark frames (default 5). Raise if it trembles.
  --ema <float>        v3: EMA on the landmarks after the median (default 0.3; lower = smoother, slower).
  --tactile-topic <t>  v3: tactile pressure topic (default: any */tactile_pressures).
                       Stepped once per CAMERA frame, so its lag is
                       (1-a)/a frames: at 30 fps that is 100 ms at 0.25 and
                       22 ms at 0.6. The single biggest delay in the chain —
                       raise it first when the hand feels sluggish, lower it
                       if the fingers start to jitter.

Machine defaults live in config/teleop.env (ALLEGRO_WS, HAND_IP, ETH_IFACE, ...);
v2-only ones in config/teleop_v2.env (V6F_TELEOP_ROOT, RETARGET_PY, SCALING, ...).
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
CALIBRATE="${CALIBRATE:-150}"
RETARGET_ALPHA="${RETARGET_ALPHA:-}"
UNDO_MIRROR="${UNDO_MIRROR:-true}"
V6F_TELEOP_ROOT="${V6F_TELEOP_ROOT:-$HOME/v6f_manuse_teleoperation/V6_Teleoperation}"
export V6F_TELEOP_ROOT

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

# The retargeting node runs on the MANUS project's venv: dex_retargeting and
# pinocchio are built against numpy 2.x, and this repo's venv is pinned to
# numpy 1.x for mediapipe. One interpreter cannot hold both, but they are two
# processes talking over ROS topics, so each gets the one it needs.
RETARGET_PY="${RETARGET_PY:-$V6F_TELEOP_ROOT/.venv/bin/python}"
if [ ! -x "$RETARGET_PY" ]; then
    echo "[!] No DexPilot interpreter at $RETARGET_PY."
    echo "[!] Point V6F_TELEOP_ROOT at the MANUS teleop checkout (the directory"
    echo "[!] holding src/v6f_teleop and urdf/v6_force), or set RETARGET_PY to a"
    echo "[!] python that can import dex_retargeting."
    exit 1
fi
if ! "$RETARGET_PY" -c "import dex_retargeting, pinocchio" >/dev/null 2>&1; then
    echo "[!] $RETARGET_PY cannot import dex_retargeting / pinocchio."
    echo "[!] In $V6F_TELEOP_ROOT:  .venv/bin/pip install -e ."
    exit 1
fi

# Release camera locks held by an earlier run that did not exit cleanly. A UI killed
# without its launcher's trap (closing the terminal, SIGKILL) survives as an orphan and
# keeps the camera open, which used to push the auto-detect onto the IR sensor. Say so
# out loud instead of silently reaping, because an orphan also means the previous
# session's ros2_control may still be driving the hand.
STALE_UI_PATTERN="$REPO_DIR/(teleop_cockpit|teleop_dashboard|vision_tracker)(_v2)?\.py"
STALE_PIDS=$(pgrep -f "$STALE_UI_PATTERN" || true)
if [ -n "$STALE_PIDS" ]; then
    echo "[!] Teleop UI already running (pid: $(echo "$STALE_PIDS" | tr '\n' ' ')); stopping it first."
    kill -INT $STALE_PIDS 2>/dev/null || true
    for _ in 1 2 3 4 5 6; do
        sleep 0.5
        pgrep -f "$STALE_UI_PATTERN" >/dev/null || break
    done
    STALE_PIDS=$(pgrep -f "$STALE_UI_PATTERN" || true)
    [ -n "$STALE_PIDS" ] && kill -KILL $STALE_PIDS 2>/dev/null || true
    if pgrep -f "$REPO_DIR/run_teleop(_v2)?\.sh" | grep -qv "^$$\$"; then
        echo "[!] Another run_teleop.sh is still up. Two sessions share one"
        echo "[!] controller_manager and one camera; stop the other terminal first."
    fi
    sleep 0.5
fi

# ------------------------------------------------------------------------------
# Camera selection. tools/detect_camera.py prefers an Intel RealSense RGB stream and
# rejects greyscale-only nodes: this laptop's Windows Hello IR sensor (/dev/video2)
# captures perfectly well, so a naive "first device that returns a frame" probe picks
# it whenever the RGB node is busy and the operator gets a flickering mono image.
# ------------------------------------------------------------------------------
if [ -z "${DEVICE:-}" ]; then
    echo "[*] Detecting camera..."
    DETECTED_DEV=$("$PY" "$REPO_DIR/tools/detect_camera.py" || true)

    if [[ "$DETECTED_DEV" =~ realsense:([0-9]+) ]]; then
        DEVICE="${BASH_REMATCH[1]}"
        CAM_DESC="Intel RealSense RGB (/dev/video$DEVICE [auto])"
    elif [[ "$DETECTED_DEV" =~ (.*):([0-9]+)$ ]]; then
        DEVICE="${BASH_REMATCH[2]}"
        CAM_DESC="${BASH_REMATCH[1]} (/dev/video$DEVICE [auto])"
    else
        echo "[!] No usable colour camera found."
        echo "[!] Run './tools/detect_camera.py --list' to see why each node was skipped,"
        echo "[!] or pass --device <n> to force one. A teleop UI left over from an earlier"
        echo "[!] run holds the camera open; check with: pgrep -af teleop_"
        exit 1
    fi
else
    CAM_DESC="/dev/video$DEVICE (user specified)"
fi

echo "========================================================"
echo " Allegro Hand V6 (5-finger, 20-DOF) Vision Teleoperation  [v2: DexPilot]"
echo " Mode       : $MODE"
echo " Rate       : ${RATE} Hz    EMA alpha: $ALPHA"
echo " Camera     : $CAM_DESC (capture ${FPS} FPS)"
echo " Descriptor : $DESCRIPTOR"
echo " Hand side  : ${HAND_SIDE} (default/detected)"
echo " UI mode    : $UI_MODE"
echo " RViz2      : $([ "$USE_RVIZ" = true ] && echo "enabled (separate window)" || echo "disabled")"
echo " Python     : $PY"
echo " Retarget   : $RETARGET_PY"
echo " DexPilot   : $V6F_TELEOP_ROOT  (scaling: ${SCALING:-measured over $CALIBRATE frames}, undo-mirror: $UNDO_MIRROR)"
echo " Filters    : retarget alpha ${RETARGET_ALPHA:-0.25 (config)} @ camera rate, bridge EMA $ALPHA @ ${RATE} Hz"
echo " ROS        : ${ROS_DISTRO:-?}   rmw: ${RMW_IMPLEMENTATION:-default}"
echo "========================================================"

PIDS=()

# Shutdown is SIGINT -> grace period -> SIGTERM -> SIGKILL, in that order and with a real
# wait in between: `ros2 launch` needs several seconds to bring its own children down, and
# TERMing it early leaves ros2_control_node running, which in `real` mode means the hand
# stays under control after the launcher is gone. The final KILL is not optional either,
# because teleop_cockpit.py can deadlock on exit (its Qt timer publishes once more after
# rclpy's context is gone: "publisher's context is invalid").
BRINGUP_PGID=""

# Signal every recorded pid, plus the bring-up's process group.
signal_all() {
    local sig="$1" pid
    for pid in "${PIDS[@]}"; do
        kill -0 "$pid" 2>/dev/null && kill "-$sig" "$pid" 2>/dev/null || true
    done
    [ -n "$BRINGUP_PGID" ] && kill "-$sig" "-$BRINGUP_PGID" 2>/dev/null || true
    return 0
}

# Number of recorded pids still alive.
count_alive() {
    local pid n=0
    for pid in "${PIDS[@]}"; do
        kill -0 "$pid" 2>/dev/null && n=$((n + 1))
    done
    echo "$n"
}

# Wait up to $1 seconds for every recorded pid to exit.
wait_for_exit() {
    local limit="$1" waited=0
    while [ "$waited" -lt "$limit" ]; do
        [ "$(count_alive)" -eq 0 ] && return 0
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "[!] Stopping all teleoperation nodes..."

    signal_all INT
    if ! wait_for_exit 8; then
        echo "[*] Still running after SIGINT; sending SIGTERM."
        signal_all TERM
        wait_for_exit 3 || true
    fi

    local pid
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[!] pid $pid did not exit; sending SIGKILL."
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    [ -n "$BRINGUP_PGID" ] && kill -KILL "-$BRINGUP_PGID" 2>/dev/null || true
    pkill -KILL -P $$ 2>/dev/null || true
    echo "[OK] All nodes stopped."
}
trap cleanup SIGINT SIGTERM EXIT

BRINGUP="$REPO_DIR/launch/allegro_hand_bringup.launch.py"

case "$MODE" in
    sim)
        echo "[1/4] Launching mock_components bring-up ($HAND_SIDE hand, rviz:=$USE_RVIZ)..."
        setsid ros2 launch "$BRINGUP" \
            hardware:=mock_components \
            hand:="$HAND_SIDE" \
            rviz:="$USE_RVIZ" \
            joint_states_topic:=/allegro/joint_states_urdf &
        PIDS+=($!)
        BRINGUP_PGID=$!   # setsid makes the child its own group leader, so PGID == PID
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

        # Only probe when the user did not name an address. Every candidate is checked to be an
        # Allegro Hand (firmware register non-zero, hand-type register 0 or 1), so other Modbus
        # devices on the subnet are never taken for it. The hand reporting HAND_SIDE wins
        # (right unless --hand says otherwise), so a left and a right hand can both stay connected.
        # A hand already in use by another session refuses the connection and is not seen.
        if [ -z "$USER_IP" ] && [[ "$DESCRIPTOR" =~ modbus_tcp:${HAND_IP:-192.168.1.100}:${HAND_PORT:-502} ]]; then
            # Prefer HAND_SIDE (right by default, config/teleop.env) even without --hand, so the
            # right hand wins when both are connected; any hand is still taken if that one is absent.
            SCAN=$(WANT_HAND="$HAND_SIDE" "$PY" -c "
import os, socket, struct
want = os.environ.get('WANT_HAND', '')
found = []
for ip in ['${HAND_IP:-192.168.1.100}', '192.168.1.110', '192.168.1.101', '192.168.1.201']:
    try:
        s = socket.socket(); s.settimeout(0.3); s.connect((ip, ${HAND_PORT:-502}))
        s.sendall(struct.pack('>HHHBBHH', 1, 0, 6, 1, 3, 0x0070, 2))
        res = s.recv(64); s.close()
    except Exception:
        continue
    if len(res) < 13 or res[7] != 3:
        continue
    fw = int.from_bytes(res[9:11], 'big'); hand = int.from_bytes(res[11:13], 'big')
    if fw == 0 or hand not in (0, 1):
        continue
    found.append((ip, 'right' if hand == 1 else 'left', fw))
pick = next((f for f in found if not want or f[1] == want), found[0] if found else None)
if pick:
    print(f'{pick[0]} {pick[1]} {pick[2]:#06x}')
" 2>/dev/null || true)
            if [ -n "$SCAN" ]; then
                read -r FOUND_IP FOUND_HAND FOUND_FW <<< "$SCAN"
                DESCRIPTOR="modbus_tcp:${FOUND_IP}:${HAND_PORT:-502}"
                echo "[OK] Allegro Hand found: ${FOUND_IP}:${HAND_PORT:-502} (${FOUND_HAND} hand, FW ${FOUND_FW})"
                if [ "$USER_SPECIFIED_HAND" = true ] && [ "$FOUND_HAND" != "$HAND_SIDE" ]; then
                    echo "[!] You asked for the $HAND_SIDE hand, but only a $FOUND_HAND hand answered."
                fi
            else
                echo "[!] No Allegro Hand answered on the known addresses. Use --ip if it is elsewhere,"
                echo "[!] and close any other session that is connected to it (one connection at a time)."
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
        setsid ros2 launch "$BRINGUP" \
            hardware:=hardware \
            descriptor:="$DESCRIPTOR" \
            hand_id:="${HAND_ID:-1}" \
            hand:="$HAND_SIDE" \
            rviz:="$USE_RVIZ" \
            joint_states_topic:=/allegro/joint_states_urdf &
        PIDS+=($!)
        BRINGUP_PGID=$!   # setsid makes the child its own group leader, so PGID == PID
        sleep 3
        ;;
    nodes)
        echo "[*] Teleop nodes only (bring-up assumed to be running elsewhere)."
        ;;
esac

echo "[2/4] DexPilot retargeting node (hand: $HAND_SIDE)..."
RETARGET_ARGS=(--quiet --hand "$HAND_SIDE" --calibrate "$CALIBRATE")
[ -n "${SCALING:-}" ] && RETARGET_ARGS+=(--scaling "$SCALING")
[ "$UNDO_MIRROR" = false ] && RETARGET_ARGS+=(--no-undo-mirror)
[ -n "${RETARGET_ALPHA:-}" ] && RETARGET_ARGS+=(--alpha "$RETARGET_ALPHA")
[ -n "$SMOOTH" ] && RETARGET_ARGS+=(--smooth "$SMOOTH")
[ -n "$INPUT_EMA" ] && RETARGET_ARGS+=(--input-ema "$INPUT_EMA")
"$RETARGET_PY" "$REPO_DIR/retargeting_node_v3.py" "${RETARGET_ARGS[@]}" &
PIDS+=($!)
sleep 1.5

echo "[3/4] Controller bridge node (${RATE} Hz, alpha=$ALPHA, mode: $MODE, hand: $HAND_SIDE)..."
"$PY" "$REPO_DIR/sim_bridge_node.py" --rate "$RATE" --alpha "$ALPHA" --mode "$MODE" --hand "$HAND_SIDE" &
PIDS+=($!)
sleep 0.5

case "$UI_MODE" in
    cockpit)
        echo "[4/4] Cockpit UI (embedded 3D hand @ ${FPS} FPS, hand: $HAND_SIDE)..."
        $COCKPIT_GL_ENV "$PY" "$REPO_DIR/teleop_cockpit_v2.py" \
            --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" --mode "$MODE" &
        PIDS+=($!)
        ;;
    classic)
        echo "[4/4] Classic dashboard (@ ${FPS} FPS, hand: $HAND_SIDE) + RViz2..."
        echo "      (tactile pressures: ${TACTILE_TOPIC:-discovered, any */tactile_pressures})"
        "$PY" "$REPO_DIR/teleop_dashboard_v3.py" \
            --device "$DEVICE" --fps "$FPS" --hand "$HAND_SIDE" --mode "$MODE" \
            ${TACTILE_TOPIC:+--tactile-topic "$TACTILE_TOPIC"} &
        PIDS+=($!)
        ;;
    simple)
        echo "[4/4] MediaPipe OpenCV window ($CAM_DESC, ${SCALE}x, ${FPS} FPS)..."
        "$PY" "$REPO_DIR/vision_tracker_v2.py" --device "$DEVICE" --scale "$SCALE" --fps "$FPS" &
        PIDS+=($!)
        ;;
    none|*)
        echo "[4/4] Headless vision tracker ($CAM_DESC, ${FPS} FPS)..."
        "$PY" "$REPO_DIR/vision_tracker_v2.py" --device "$DEVICE" --fps "$FPS" --no-gui &
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
