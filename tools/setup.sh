#!/usr/bin/env bash
# ==============================================================================
# tools/setup.sh — one-shot install for this machine (Ubuntu 24.04 + ROS 2 Jazzy, no container).
#
# What it does:
#   1. apt: ROS 2 Jazzy packages the teleop nodes and the bring-up launch file need.
#   2. .venv (--system-site-packages): mediapipe / PyOpenGL / opencv-contrib from pip,
#      while rclpy, cv_bridge and PyQt5 keep coming from apt.
#   3. meshes/ symlink to the driver package STL meshes, so the cockpit 3D view also
#      works when $ALLEGRO_WS is not sourced.
#   4. Verifies that the Wonik driver packages are built and findable.
#
# Usage:
#   ./tools/setup.sh              # everything
#   ./tools/setup.sh --no-apt     # skip apt (no sudo prompt)
#   ./tools/setup.sh --venv-only
# ==============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
cd "$REPO_DIR"

# shellcheck source=/dev/null
set +u
[ -f config/teleop.env ] && source config/teleop.env
set -u

DO_APT=true
DO_VENV=true
DO_LINK=true
for arg in "$@"; do
    case "$arg" in
        --no-apt)    DO_APT=false ;;
        --venv-only) DO_APT=false; DO_LINK=false ;;
        --apt-only)  DO_VENV=false; DO_LINK=false ;;
        -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "Unknown argument: $arg"; exit 1 ;;
    esac
done

echo "========================================================"
echo " allegro_vision_teleop — native ROS 2 Jazzy setup"
echo " Repo       : $REPO_DIR"
echo " ROS setup  : ${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
echo " Driver ws  : ${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation}"
echo "========================================================"

# ------------------------------------------------------------------ 1. apt
if [ "$DO_APT" = true ]; then
    echo "[1/4] Installing apt packages (sudo)..."
    sudo apt-get update
    sudo apt-get install -y \
        python3-venv python3-pip \
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
        ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-tf2-ros \
        ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-cv-bridge ros-jazzy-rmw-cyclonedds-cpp \
        python3-pyqt5 \
        v4l-utils iputils-ping
else
    echo "[1/4] apt step skipped."
fi

# ------------------------------------------------------------------ 2. venv
if [ "$DO_VENV" = true ]; then
    echo "[2/4] Creating .venv (--system-site-packages) and installing pip deps..."
    if [ ! -x .venv/bin/python ]; then
        /usr/bin/python3 -m venv --system-site-packages .venv
    fi
    ./.venv/bin/pip install --upgrade pip
    ./.venv/bin/pip install -r requirements.txt
else
    echo "[2/4] venv step skipped."
fi

# ------------------------------------------------------------------ 3. meshes
if [ "$DO_LINK" = true ]; then
    echo "[3/4] Linking driver package meshes..."
    MESH_SRC=""
    for cand in \
        "${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation}/install/allegro_hand_v6_description/share/allegro_hand_v6_description/meshes" \
        "${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation}/allegro_hand_v6/allegro_hand_v6_description/meshes"
    do
        [ -d "$cand" ] && MESH_SRC="$cand" && break
    done
    if [ -n "$MESH_SRC" ]; then
        ln -sfn "$MESH_SRC" meshes
        echo "      meshes -> $MESH_SRC"
    else
        echo "      [!] STL meshes not found under \$ALLEGRO_WS; the cockpit 3D view needs"
        echo "          allegro_hand_v6_description to be built (see step 4)."
    fi
else
    echo "[3/4] meshes link skipped."
fi

# ------------------------------------------------------------------ 4. verify
echo "[4/4] Verifying environment..."
# ROS 2's setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES and friends), which
# `set -u` turns into a fatal error, so nounset is lifted around every source.
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation}/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
    set -u
    echo "      driver workspace sourced: $WS_SETUP"
else
    set -u
    echo "      [!] $WS_SETUP not found. Build the Wonik driver packages first:"
    echo "          cd ${ALLEGRO_WS:-$HOME/v6f_manuse_teleoperation} && colcon build --symlink-install"
fi

for pkg in allegro_hand_v6_bringup allegro_hand_v6_description; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "      [OK] $pkg -> $(ros2 pkg prefix "$pkg")"
    else
        echo "      [!!] $pkg NOT FOUND — 'real'/'sim' modes will fail."
    fi
done

PYBIN="./.venv/bin/python"
[ -x "$PYBIN" ] || PYBIN="python3"
if "$PYBIN" - <<'PYEOF'
import importlib, sys
print(f"      python      {sys.version.split()[0]}  ({sys.executable})")
ok = True
for m in ("numpy", "cv2", "mediapipe", "scipy", "rclpy", "cv_bridge", "PyQt5.QtWidgets",
          "OpenGL.GL", "ament_index_python.packages"):
    try:
        mod = importlib.import_module(m)
        print(f"      [OK] {m:<28} {getattr(mod, '__version__', '')}")
    except Exception as exc:
        ok = False
        print(f"      [!!] {m:<28} {type(exc).__name__}: {exc}")

# The cockpit's 3D view: QOpenGLWidget ships inside QtWidgets on this Qt build,
# so python3-pyqt5.qtopengl is deliberately not an apt dependency.
try:
    from PyQt5.QtWidgets import QOpenGLWidget  # noqa: F401
    print(f"      [OK] {'PyQt5 QOpenGLWidget':<28}")
except Exception as exc:
    ok = False
    print(f"      [!!] {'PyQt5 QOpenGLWidget':<28} {type(exc).__name__}: {exc}")
sys.exit(0 if ok else 1)
PYEOF
then
    echo "      all python dependencies OK"
else
    echo "      [!!] some python dependencies are missing (see above)"
fi

echo "========================================================"
echo " Setup done. Next:"
echo "   ./check_hand.sh                 # hand communication / hand type / encoders"
echo "   ./run_cockpit.sh real           # cockpit UI on the physical hand"
echo "   ./run_cockpit.sh sim --hand right   # no hardware needed"
echo "========================================================"
