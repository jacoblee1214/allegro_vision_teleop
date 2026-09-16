#!/usr/bin/env bash
# ==============================================================================
# run_gui.sh — Wonik Robotics Allegro Hand V6 Official Control & Diagnostic GUI
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
GUI_DIR="/home/jake/humble_ws/src/allegro_hand_v6/allegro_hand_v6_hardware/test_comm"

echo "========================================================"
echo " Allegro Hand V6 Official Calibration & Control GUI     "
echo " Target: 192.168.1.100:502 (Modbus TCP)                 "
echo "========================================================"

cd "$GUI_DIR"
exec /home/jake/.local/bin/uv run \
    --with pyqt6 --with pymodbus --with pyserial --with numpy \
    python3 gui.py "$@"
