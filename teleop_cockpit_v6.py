#!/usr/bin/env python3
"""
teleop_dashboard_v6.py — Unified Teleoperation Cockpit Dashboard for Allegro Hand V6 (5-Finger, 20-DOF).

Layout:
  ┌───────────────────────────────────────────────┬───────────────────────────────────────────────┐
  │ [LEFT] Live Camera View & Hand Tracking      │ [RIGHT-TOP] 20-DOF Joint Telemetry (5 Fingers)│
  │ • RealSense / Webcam RGB Feed (60 FPS)        │ • Thumb, Index, Middle, Ring, Pinky           │
  │ • MediaPipe 21 3D Landmarks & Skeleton        │ • Normalized angle gauge bars & rad values    │
  │ • Live Pinch Metric (d_pinch in cm)           ├───────────────────────────────────────────────┤
  │                                               │ [RIGHT-BOTTOM] Operator Control Cockpit       │
  │                                               │ • CLUTCH: [RELEASED ▶] / [ENGAGED ⏸] (Key: C) │
  │                                               │ • RECORD: [IDLE ⚪] / [RECORDING 🔴]  (Key: R) │
  │                                               │ • TAG: [SUCCESS 🟢] (S) / [FAIL 🔴] (F)       │
  │                                               │ • Live Timer & Episode History Counter        │
  └───────────────────────────────────────────────┴───────────────────────────────────────────────┘

Usage:
    python3 teleop_dashboard_v6.py [--device auto] [--fps 60]
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Dict, List, Optional

# ------------------------------------------------------------------------------
# Auto-delegation: If executed directly on host machine, forward to Docker
# ------------------------------------------------------------------------------
if not os.path.exists("/.dockerenv") and os.environ.get("RUN_ON_HOST", "0") != "1":
    container_name = "ros_humble_dev"
    try:
        res = subprocess.run(
            ["docker", "ps", "-a", "--filter", f"name={container_name}", "--format", "{{.Names}}"],
            capture_output=True,
            text=True,
        )
        if container_name in res.stdout.split():
            print("========================================================")
            print(f"[*] Host environment detected: {os.uname().nodename}")
            print(f"[*] Forwarding execution to Docker container '{container_name}'...")
            subprocess.run(["xhost", "+local:docker"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["docker", "start", container_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            display = os.environ.get("DISPLAY", ":1")
            script_path = "/home/humble_ws/allegro_vision_teleop/teleop_cockpit_v6.py"
            cmd_args = " ".join(f'"{a}"' for a in sys.argv[1:])
            exec_cmd = [
                "docker", "exec",
                "-it" if sys.stdin.isatty() and sys.stdout.isatty() else "-i",
                "-e", f"DISPLAY={display}",
                container_name,
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash; source /home/humble_ws/install/setup.bash 2>/dev/null || true; python3 {script_path} {cmd_args}"
            ]
            sys.exit(subprocess.call(exec_cmd))
    except Exception as e:
        pass

# Fix Qt / OpenCV conflict:
# 1. Point Qt to system platform plugins
if "QT_QPA_PLATFORM_PLUGIN_PATH" not in os.environ and os.path.exists("/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"):
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"

# 2. Import PyQt5 BEFORE cv2 and lock libraryPaths to system Qt plugins
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QCoreApplication, QPointF, Qt, QTimer
if os.path.exists("/usr/lib/x86_64-linux-gnu/qt5/plugins"):
    QCoreApplication.setLibraryPaths(["/usr/lib/x86_64-linux-gnu/qt5/plugins"])
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QLinearGradient,
    QPainter,
    QPalette,
    QPen,
    QPixmap,
    QPolygonF,
    QRadialGradient,
)
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QOpenGLWidget,
)
import struct
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import OpenGL.GL as gl

import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

TOPIC_LANDMARKS = "/allegro/vision/landmarks"
TOPIC_TELEOP_STATE = "/allegro/teleop_state"
TOPIC_CAMERA_IMAGE = "/allegro/camera/image_raw"
TOPIC_TARGET_JOINTS = "/allegro/target_joints"
TOPIC_JOINT_STATES = "/joint_states"

NUM_POINTS = 21

QOS_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES_V6 = [
    "joint00", "joint01", "joint02", "joint03",  # Thumb
    "joint10", "joint11", "joint12", "joint13",  # Index
    "joint20", "joint21", "joint22", "joint23",  # Middle
    "joint30", "joint31", "joint32", "joint33",  # Ring
    "joint40", "joint41", "joint42", "joint43",  # Pinky
]

# Joint limit ranges for normalization [min, max]
LIMITS_V6 = {
    "joint00": (-0.035, 1.658),
    "joint01": (-1.658, 1.658),
    "joint02": (-0.175, 1.309),
    "joint03": (-0.175, 1.396),
    "joint10": (-1.309, 1.309),
    "joint11": (-0.070, 1.571),
    "joint12": (-0.175, 1.396),
    "joint13": (-0.175, 1.396),
    "joint20": (-1.135, 1.135),
    "joint21": (-0.070, 1.571),
    "joint22": (-0.175, 1.396),
    "joint23": (-0.175, 1.396),
    "joint30": (-1.309, 1.309),
    "joint31": (-0.070, 1.571),
    "joint32": (-0.175, 1.396),
    "joint33": (-0.175, 1.396),
    "joint40": (-1.309, 1.309),
    "joint41": (-0.070, 1.571),
    "joint42": (-0.175, 1.396),
    "joint43": (-0.175, 1.396),
}


def find_best_camera_device() -> tuple[int, str]:
    import glob
    import subprocess

    dev_paths = sorted(
        glob.glob("/dev/video*"),
        key=lambda p: int(p.replace("/dev/video", "")) if p.replace("/dev/video", "").isdigit() else 999,
    )
    for dev in dev_paths:
        idx_str = dev.replace("/dev/video", "")
        if not idx_str.isdigit():
            continue
        idx = int(idx_str)
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "-d", dev, "--all"],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            ).decode("utf-8", errors="ignore")
            if "RealSense" in out and ("YUYV" in out or "white_balance" in out):
                return idx, f"Intel RealSense RGB (/dev/video{idx})"
        except Exception:
            pass
    return 0, "Webcam (/dev/video0)"


def get_urdf_content(hand_side: str) -> str:
    """Finds and reads the Allegro Hand V6 URDF for the specified hand side."""
    # 1. Versioned teleop URDF has highest priority
    teleop_urdf = Path(__file__).resolve().parent / "urdf" / f"allegro_hand_v6_{hand_side}_v6.1_teleop.urdf"
    if teleop_urdf.exists():
        return teleop_urdf.read_text(encoding="utf-8")

    # 2. Local package URDFs
    local_urdf = Path(__file__).resolve().parent / "urdf" / f"allegro_hand_v6_{hand_side}.urdf"
    if local_urdf.exists():
        return local_urdf.read_text(encoding="utf-8")

    candidates = [
        Path(f"/home/humble_ws/src/allegro_hand_v6/allegro_hand_v6_description/urdf/allegro_hand_v6_{hand_side}.urdf"),
        Path(f"/home/jake/humble_ws/src/allegro_hand_v6/allegro_hand_v6_description/urdf/allegro_hand_v6_{hand_side}.urdf"),
        Path(f"/home/humble_ws/install/allegro_hand_v6_description/share/allegro_hand_v6_description/urdf/allegro_hand_v6_{hand_side}.urdf"),
        Path(f"/home/jake/humble_ws/install/allegro_hand_v6_description/share/allegro_hand_v6_description/urdf/allegro_hand_v6_{hand_side}.urdf"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("allegro_hand_v6_description")
        candidates.append(Path(share_dir) / "urdf" / f"allegro_hand_v6_{hand_side}.urdf")
    except Exception:
        pass

    for path in candidates:
        if path.exists():
            return path.read_text(encoding="utf-8")
    return ""


def fast_load_stl(filepath):
    """Parses binary STL file into numpy float32 vertex and normal arrays."""
    with open(filepath, "rb") as f:
        f.seek(80)
        n_tri = struct.unpack("<I", f.read(4))[0]
        data = np.frombuffer(f.read(n_tri * 50), dtype=np.uint8).reshape(n_tri, 50)
        verts = np.frombuffer(data[:, 12:48].copy(), dtype=np.float32).reshape(n_tri * 3, 3)
        normals = np.frombuffer(data[:, 0:12].copy(), dtype=np.float32).reshape(n_tri, 3)
        v_normals = np.repeat(normals, 3, axis=0)
        return verts, v_normals


class URDFModel:
    def __init__(self, urdf_path: Path, mesh_base_dir: Path):
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.mesh_base_dir = mesh_base_dir

        self.link_meshes = {}
        for lname, l in self.links.items():
            vis = l.find("visual")
            if vis is not None:
                geom = vis.find("geometry")
                if geom is not None:
                    mesh = geom.find("mesh")
                    if mesh is not None:
                        fn = mesh.get("filename").replace("package://allegro_hand_v6_description/meshes/", "")
                        mpath = self.mesh_base_dir / fn
                        if mpath.exists():
                            self.link_meshes[lname] = fast_load_stl(mpath)

        self.fingers = [
            (1, ["joint00", "joint01", "joint02", "joint03"]),
            (2, ["joint10", "joint11", "joint12", "joint13"]),
            (3, ["joint20", "joint21", "joint22", "joint23"]),
            (4, ["joint30", "joint31", "joint32", "joint33"]),
            (5, ["joint40", "joint41", "joint42", "joint43"]),
        ]

    def get_joint_T(self, jname: str, q: float = 0.0) -> np.ndarray:
        j = self.joints[jname]
        orig = j.find("origin")
        axis = j.find("axis")
        xyz = [float(x) for x in orig.get("xyz").split()]
        rpy = [float(x) for x in orig.get("rpy").split()]
        ax = [float(x) for x in axis.get("xyz").split()] if axis is not None else [0, 0, 1]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = (R.from_euler("xyz", rpy).as_matrix() @ R.from_rotvec(np.array(ax) * q).as_matrix()).astype(np.float32)
        T[:3, 3] = xyz
        return T

    def compute_link_transforms(self, q_dict: dict) -> dict:
        transforms = {"base_link": np.eye(4, dtype=np.float32)}
        for f_idx, j_names in self.fingers:
            prev_T = transforms["base_link"]
            for l_idx, jname in enumerate(j_names, 1):
                q = q_dict.get(jname, 0.0)
                T_j = self.get_joint_T(jname, q)
                curr_T = prev_T @ T_j
                link_name = f"Finger0{f_idx}_Link0{l_idx}"
                transforms[link_name] = curr_T
                prev_T = curr_T
        return transforms


class RobotHand3DWidget(QOpenGLWidget):
    """
    Photorealistic Real-time 3D URDF CAD Visualizer for Allegro Hand V6 (5-Finger, 20-DOF).
    Renders exact STL meshes, metallic shading, and interactive 360-degree orbit camera.
    """

    def __init__(self, hand_side: str = "right", parent=None):
        super().__init__(parent)
        self.hand_side = hand_side.lower()
        self.setMinimumSize(380, 420)
        self.setStyleSheet("background-color: #0c0d14; border: 1px solid #333348; border-radius: 8px;")

        # Joint angles (rad)
        self.joint_positions = [0.0] * 20
        self.pinch_active = False

        # Camera spherical view parameters
        self.azimuth_default = 28.0 if self.hand_side == "left" else -28.0
        self.elevation_default = 18.0
        self.distance_default = 0.46
        self.pan_default = [0.0, 0.0, 0.09]

        self.azimuth = self.azimuth_default
        self.elevation = self.elevation_default
        self.distance = self.distance_default
        self.pan = list(self.pan_default)

        self._last_mouse_pos = None
        self._urdf_model = None
        self._display_lists = {}
        self._gl_initialized = False

    def set_hand_side(self, hand_side: str):
        new_side = hand_side.lower()
        if new_side != self.hand_side:
            self.hand_side = new_side
            self.reset_camera()
            if self._gl_initialized:
                self._load_urdf_and_compile_lists()
            self.update()

    def reset_camera(self):
        self.azimuth = 28.0 if self.hand_side == "left" else -28.0
        self.elevation = self.elevation_default
        self.distance = self.distance_default
        self.pan = list(self.pan_default)
        self.update()

    def update_joints(self, joint_angles: list, pinch_active: bool = False):
        if len(joint_angles) >= 20:
            self.joint_positions = list(joint_angles[:20])
            self.pinch_active = pinch_active
            self.update()

    def _find_urdf_path(self) -> Path:
        base_dir = Path(__file__).resolve().parent
        candidates = [
            base_dir / "urdf" / f"allegro_hand_v6_{self.hand_side}.urdf",
            Path(f"/home/humble_ws/allegro_vision_teleop/urdf/allegro_hand_v6_{self.hand_side}.urdf"),
            Path(f"/home/jake/humble_ws/allegro_vision_teleop/urdf/allegro_hand_v6_{self.hand_side}.urdf"),
            Path(f"/home/humble_ws/src/allegro_hand_v6/allegro_hand_v6_description/urdf/allegro_hand_v6_{self.hand_side}.urdf"),
            Path(f"/home/jake/humble_ws/src/allegro_hand_v6/allegro_hand_v6_description/urdf/allegro_hand_v6_{self.hand_side}.urdf"),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("allegro_hand_v6_description")
            candidates.append(Path(share_dir) / "urdf" / f"allegro_hand_v6_{self.hand_side}.urdf")
        except Exception:
            pass
        for p in candidates:
            if p.exists():
                return p
        raise FileNotFoundError(f"Cannot find URDF for {self.hand_side} hand.")

    def _find_mesh_dir(self) -> Path:
        base_dir = Path(__file__).resolve().parent
        candidates = [
            base_dir / "meshes",
            Path("/home/humble_ws/allegro_vision_teleop/meshes"),
            Path("/home/jake/humble_ws/allegro_vision_teleop/meshes"),
            Path("/home/humble_ws/src/allegro_hand_v6/allegro_hand_v6_description/meshes"),
            Path("/home/jake/humble_ws/src/allegro_hand_v6/allegro_hand_v6_description/meshes"),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("allegro_hand_v6_description")
            candidates.append(Path(share_dir) / "meshes")
        except Exception:
            pass
        for p in candidates:
            if p.exists():
                return p
        raise FileNotFoundError("Cannot find meshes directory.")

    def _load_urdf_and_compile_lists(self):
        self.makeCurrent()
        # Clean up existing display lists
        for dl in self._display_lists.values():
            gl.glDeleteLists(dl, 1)
        self._display_lists.clear()

        try:
            urdf_path = self._find_urdf_path()
            mesh_dir = self._find_mesh_dir()
            self._urdf_model = URDFModel(urdf_path, mesh_dir)

            for lname, (verts, normals) in self._urdf_model.link_meshes.items():
                dl = gl.glGenLists(1)
                gl.glNewList(dl, gl.GL_COMPILE)
                gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
                gl.glEnableClientState(gl.GL_NORMAL_ARRAY)
                gl.glVertexPointer(3, gl.GL_FLOAT, 0, verts)
                gl.glNormalPointer(gl.GL_FLOAT, 0, normals)
                gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(verts))
                gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
                gl.glDisableClientState(gl.GL_NORMAL_ARRAY)
                gl.glEndList()
                self._display_lists[lname] = dl
        except Exception as e:
            print(f"[!] Warning: Failed to load URDF CAD meshes in 3D widget: {e}")

    def initializeGL(self):
        gl.glClearColor(0.06, 0.06, 0.09, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        gl.glEnable(gl.GL_LIGHT1)
        gl.glEnable(gl.GL_COLOR_MATERIAL)
        gl.glEnable(gl.GL_NORMALIZE)
        gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE)

        # Key light (Top-front-right)
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, [0.6, -0.6, 1.2, 0.0])
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE, [0.95, 0.95, 1.0, 1.0])
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_SPECULAR, [0.4, 0.4, 0.5, 1.0])

        # Fill light (Bottom-back-left)
        gl.glLightfv(gl.GL_LIGHT1, gl.GL_POSITION, [-0.6, 0.6, -0.2, 0.0])
        gl.glLightfv(gl.GL_LIGHT1, gl.GL_DIFFUSE, [0.35, 0.35, 0.45, 1.0])

        self._gl_initialized = True
        self._load_urdf_and_compile_lists()

    def resizeGL(self, w: int, h: int):
        gl.glViewport(0, 0, w, h)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = w / max(1, h)
        fov, near, far = 45.0, 0.02, 5.0
        f = 1.0 / np.tan(np.radians(fov) / 2.0)
        proj = np.array([
            [f / aspect, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, (far + near) / (near - far), (2 * far * near) / (near - far)],
            [0, 0, -1, 0]
        ], dtype=np.float32)
        gl.glMultMatrixf(proj.T)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def _draw_grid(self):
        gl.glDisable(gl.GL_LIGHTING)
        gl.glColor4f(0.18, 0.20, 0.28, 0.6)
        gl.glLineWidth(1.0)
        gl.glBegin(gl.GL_LINES)
        step, count = 0.02, 6
        for i in range(-count, count + 1):
            coord = i * step
            gl.glVertex3f(coord, -count * step, 0.0)
            gl.glVertex3f(coord, count * step, 0.0)
            gl.glVertex3f(-count * step, coord, 0.0)
            gl.glVertex3f(count * step, coord, 0.0)
        gl.glEnd()
        gl.glEnable(gl.GL_LIGHTING)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()

        # Camera transformation
        az = np.radians(self.azimuth)
        el = np.radians(self.elevation)
        d = self.distance
        cx = self.pan[0] + d * np.cos(el) * np.sin(az)
        cy = self.pan[1] - d * np.cos(el) * np.cos(az)
        cz = self.pan[2] + d * np.sin(el)

        fwd = np.array(self.pan) - np.array([cx, cy, cz])
        fwd /= np.linalg.norm(fwd)
        up = np.array([0.0, 0.0, 1.0])
        side = np.cross(fwd, up)
        side_norm = np.linalg.norm(side)
        if side_norm > 1e-6:
            side /= side_norm
            u = np.cross(side, fwd)
        else:
            side = np.array([1.0, 0.0, 0.0])
            u = np.array([0.0, 1.0, 0.0])

        R_cam = np.eye(4, dtype=np.float32)
        R_cam[0, :3] = side
        R_cam[1, :3] = u
        R_cam[2, :3] = -fwd
        T_cam = np.eye(4, dtype=np.float32)
        T_cam[:3, 3] = -np.array([cx, cy, cz])
        view_mat = R_cam @ T_cam
        gl.glMultMatrixf(view_mat.T)

        # Draw ground grid
        self._draw_grid()

        # Compute URDF Forward Kinematics
        if self._urdf_model and self._display_lists:
            jnames = [
                "joint00", "joint01", "joint02", "joint03",
                "joint10", "joint11", "joint12", "joint13",
                "joint20", "joint21", "joint22", "joint23",
                "joint30", "joint31", "joint32", "joint33",
                "joint40", "joint41", "joint42", "joint43"
            ]
            q_dict = {name: self.joint_positions[i] for i, name in enumerate(jnames)}
            transforms = self._urdf_model.compute_link_transforms(q_dict)

            # Render 21 links with CAD metallic materials
            for lname, dl in self._display_lists.items():
                if lname in transforms:
                    gl.glPushMatrix()
                    gl.glMultMatrixf(transforms[lname].T)
                    if lname == "base_link":
                        # Palm: Matte Gunmetal Gray
                        gl.glColor3f(0.28, 0.30, 0.35)
                    elif "Link04" in lname:
                        # Fingertips
                        if self.pinch_active and lname in ("Finger01_Link04", "Finger02_Link04"):
                            gl.glColor3f(1.0, 0.70, 0.15)  # Glowing pinch contact
                        else:
                            gl.glColor3f(0.82, 0.85, 0.90)  # Polished aluminum tip
                    elif "Link01" in lname:
                        gl.glColor3f(0.68, 0.70, 0.76)  # Darker base knurl
                    else:
                        gl.glColor3f(0.74, 0.77, 0.82)  # Sleek titanium link
                    gl.glCallList(dl)
                    gl.glPopMatrix()

        # Draw Cockpit 2D HUD Text Overlay
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Badge: Hand Side
        painter.setPen(QColor(180, 200, 235))
        painter.setFont(QFont("Monospace", 9, QFont.Bold))
        hand_txt = f"URDF 3D CAD: [ {'LEFT' if self.hand_side == 'left' else 'RIGHT'} HAND ]"
        painter.drawText(14, 24, hand_txt)

        # Camera stats
        painter.setPen(QColor(120, 135, 165))
        painter.setFont(QFont("SansSerif", 8))
        cam_info = f"Azimuth: {self.azimuth:.0f}° | Tilt: {self.elevation:.0f}° | Zoom: {1.0/self.distance:.1f}x"
        painter.drawText(14, 42, cam_info)

        # Controls Hint
        tip_info = "🖱️ Drag: Rotate | Right-Drag: Pan | Scroll: Zoom | Dbl-Click: Reset"
        painter.drawText(14, int(self.height() - 12), tip_info)
        painter.end()

    def mousePressEvent(self, event):
        self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._last_mouse_pos is not None:
            dx = event.x() - self._last_mouse_pos.x()
            dy = event.y() - self._last_mouse_pos.y()
            if event.buttons() & Qt.LeftButton:
                self.azimuth = (self.azimuth + dx * 0.7) % 360.0
                self.elevation = float(np.clip(self.elevation - dy * 0.6, -85.0, 85.0))
            elif event.buttons() & (Qt.RightButton | Qt.MiddleButton):
                az = np.radians(self.azimuth)
                pan_scale = 0.0006 * self.distance
                self.pan[0] += (-dx * np.cos(az) - dy * np.sin(az)) * pan_scale
                self.pan[1] += (-dx * np.sin(az) + dy * np.cos(az)) * pan_scale
                self.pan[2] += dy * pan_scale
            self._last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        self._last_mouse_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        zoom_factor = 0.92 if delta > 0 else 1.08
        self.distance = float(np.clip(self.distance * zoom_factor, 0.15, 1.8))
        self.update()

    def mouseDoubleClickEvent(self, event):
        self.reset_camera()


class RosWorkerNode(Node):
    """ROS 2 Node running inside PyQt5 event loop."""

    def __init__(self) -> None:
        super().__init__("teleop_dashboard_worker")
        self.pub_landmarks = self.create_publisher(Float32MultiArray, TOPIC_LANDMARKS, QOS_RELIABLE)
        self.pub_state = self.create_publisher(String, TOPIC_TELEOP_STATE, QOS_RELIABLE)
        self.pub_image = self.create_publisher(Image, TOPIC_CAMERA_IMAGE, QOS_RELIABLE)

        # Publisher for /robot_description (Transient Local for RViz2 dynamic reloading)
        qos_transient = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub_robot_description = self.create_publisher(String, "/robot_description", qos_transient)
        self.cli_set_param = self.create_client(SetParameters, "/robot_state_publisher/set_parameters")

        self.latest_target_joints: List[float] = [0.0] * 20
        self.latest_actual_joints: List[float] = [0.0] * 20
        self.hand_side: str = "right"

        self.sub_target = self.create_subscription(
            Float64MultiArray,
            TOPIC_TARGET_JOINTS,
            self._target_callback,
            QOS_RELIABLE,
        )
        self.sub_joint_states = self.create_subscription(
            JointState,
            TOPIC_JOINT_STATES,
            self._joint_states_callback,
            QOS_SENSOR,
        )

    def update_robot_description(self, urdf_xml: str) -> None:
        """Publishes new URDF to /robot_description (for RViz) and updates robot_state_publisher."""
        if not urdf_xml:
            return

        # 1. Publish directly to /robot_description topic (consumed by RViz2 RobotModel)
        msg = String()
        msg.data = urdf_xml
        self.pub_robot_description.publish(msg)
        self.get_logger().info(f"[LIVE MODEL UPDATE] Published new URDF ({len(urdf_xml)} bytes) to /robot_description")

        # 2. Update robot_state_publisher node parameter so TF frame tree updates
        if self.cli_set_param.wait_for_service(timeout_sec=0.2):
            req = SetParameters.Request()
            p = Parameter()
            p.name = "robot_description"
            p.value.type = ParameterType.PARAMETER_STRING
            p.value.string_value = urdf_xml
            req.parameters.append(p)
            future = self.cli_set_param.call_async(req)
            def done_cb(fut):
                try:
                    res = fut.result()
                    if res and res.results and res.results[0].successful:
                        self.get_logger().info("[LIVE MODEL UPDATE] /robot_state_publisher updated successfully.")
                    else:
                        self.get_logger().warn(f"[LIVE MODEL UPDATE] /robot_state_publisher update failed: {res}")
                except Exception as e:
                    self.get_logger().warn(f"[LIVE MODEL UPDATE] Error in set_parameters: {e}")
            future.add_done_callback(done_cb)

    def _target_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) == 20:
            self.latest_target_joints = [float(v) for v in msg.data]

    def _joint_states_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        reordered = [0.0] * 20
        for out_idx, jname in enumerate(JOINT_NAMES_V6):
            for cand in [jname, f"ah_{jname}", jname.removeprefix("ah_")]:
                if cand in name_to_idx:
                    val = float(msg.position[name_to_idx[cand]])
                    reordered[out_idx] = val
                    break
        self.latest_actual_joints = reordered


class TeleopDashboardWindow(QMainWindow):
    def __init__(self, device_id: int | str = "auto", target_fps: int = 60, hand_side: str = "right") -> None:
        super().__init__()
        self.target_fps = target_fps
        self.hand_side = hand_side.lower()

        # Camera detection
        if str(device_id).lower() in ("auto", "-1", "none", ""):
            self.device_id, self.dev_desc = find_best_camera_device()
        else:
            self.device_id = int(device_id)
            self.dev_desc = f"/dev/video{self.device_id}"

        # Video capture setup
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened() and self.device_id != 0:
            print(f"[!] Warning: Failed to open /dev/video{self.device_id}. Falling back to /dev/video0 (Webcam)...")
            self.device_id = 0
            self.dev_desc = "Webcam (/dev/video0) [Fallback]"
            self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, float(target_fps))

        if not self.cap.isOpened():
            print(f"[!] ERROR: Could not open camera {self.dev_desc}. Please verify camera connection.")
        else:
            print(f"[✓] Camera successfully opened: {self.dev_desc} ({target_fps} FPS target)")

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None

        # Teleop State Variables
        self.clutch_engaged = False
        self.is_recording = False
        self.last_tag = "NONE"
        self.episode_counter = 0
        self.record_start_mono: Optional[float] = None
        self.total_episodes_saved = 0
        self.success_count = 0
        self.fail_count = 0

        # Performance & Stats
        self._fps_counter = 0
        self._last_fps_time = time.monotonic()
        self.current_fps = 0.0
        self.current_pinch_cm = 0.0

        # ROS 2 Node setup
        self.ros_node = RosWorkerNode()
        self.ros_node.hand_side = self.hand_side

        # Preload Left and Right Hand URDF models for instant live switching
        self.urdf_left = get_urdf_content("left")
        self.urdf_right = get_urdf_content("right")

        # Build UI
        self.init_ui()

        # Update Timer (60Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_tick)
        self.timer.start(int(1000.0 / target_fps))

        # Initial teleop state broadcast
        self.broadcast_state()

        # Initial live RViz model sync
        init_urdf = self.urdf_left if self.hand_side == "left" else self.urdf_right
        if init_urdf:
            self.ros_node.update_robot_description(init_urdf)

    def init_ui(self) -> None:
        self.setWindowTitle("Allegro Hand V6 — Teleoperation Cockpit & VLA Dataset Collector")
        self.resize(1560, 860)
        self.setMinimumSize(1200, 700)

        # Apply Modern Dark Theme Stylesheet
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1a1a24;
                color: #e6e6f0;
                font-family: 'Segoe UI', 'DejaVu Sans', sans-serif;
            }
            QGroupBox {
                border: 1px solid #333348;
                border-radius: 8px;
                margin-top: 14px;
                padding-top: 12px;
                font-size: 13px;
                font-weight: bold;
                color: #a0a0c0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #2c2c3e;
                border: 1px solid #444460;
                border-radius: 6px;
                color: #ffffff;
                font-size: 13px;
                font-weight: bold;
                padding: 8px 14px;
            }
            QPushButton:hover {
                background-color: #383852;
                border-color: #606088;
            }
            QPushButton:pressed {
                background-color: #202030;
            }
            QProgressBar {
                border: 1px solid #333348;
                border-radius: 4px;
                background-color: #12121a;
                text-align: center;
                color: #ffffff;
                font-size: 10px;
                height: 12px;
            }
            QProgressBar::chunk {
                background-color: #3b82f6;
                border-radius: 3px;
            }
            QTableWidget {
                background-color: #12121a;
                border: 1px solid #333348;
                border-radius: 6px;
                gridline-color: #242436;
                font-size: 11px;
            }
            QHeaderView::section {
                background-color: #242436;
                color: #b0b0d0;
                padding: 4px;
                border: 1px solid #333348;
                font-weight: bold;
            }
        """)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(14)

        # ─── LEFT COLUMN: Live Camera Feed & Hand Tracker ────────────────────────
        left_layout = QVBoxLayout()
        left_layout.setSpacing(10)

        cam_header = QHBoxLayout()
        self.lbl_cam_title = QLabel("🎥 Live Camera & Hand Tracker (MediaPipe 21-pt)")
        self.lbl_cam_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #ffffff;")
        self.lbl_fps_badge = QLabel("FPS: 0.0")
        self.lbl_fps_badge.setStyleSheet("background-color: #242436; padding: 4px 10px; border-radius: 4px; font-weight: bold; color: #38bdf8;")
        self.lbl_tracking_badge = QLabel("SEARCHING...")
        self.lbl_tracking_badge.setStyleSheet("background-color: #3f2020; color: #f87171; padding: 4px 10px; border-radius: 4px; font-weight: bold;")
        cam_header.addWidget(self.lbl_cam_title)
        cam_header.addStretch()
        cam_header.addWidget(self.lbl_tracking_badge)
        cam_header.addWidget(self.lbl_fps_badge)
        left_layout.addLayout(cam_header)

        # Video Frame Label
        self.lbl_video = QLabel()
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setMinimumSize(480, 360)
        self.lbl_video.setStyleSheet("background-color: #0b0b10; border: 1px solid #333348; border-radius: 8px;")
        left_layout.addWidget(self.lbl_video, stretch=1)

        # Camera Meta & Metrics Bar
        meta_bar = QHBoxLayout()
        self.lbl_device_info = QLabel(f"Device: {self.dev_desc}")
        self.lbl_device_info.setStyleSheet("color: #9090b0; font-size: 12px;")
        self.lbl_pinch_info = QLabel("Pinch Distance: -- cm")
        self.lbl_pinch_info.setStyleSheet("color: #a78bfa; font-weight: bold; font-size: 13px;")
        meta_bar.addWidget(self.lbl_device_info)
        meta_bar.addStretch()
        meta_bar.addWidget(self.lbl_pinch_info)
        left_layout.addLayout(meta_bar)

        main_layout.addLayout(left_layout, stretch=3)

        # ─── CENTER COLUMN: 3D Robot Hand Kinematics Visualizer ───────────────────
        center_layout = QVBoxLayout()
        center_layout.setSpacing(10)

        hand_3d_header = QHBoxLayout()
        self.lbl_3d_title = QLabel("🤖 3D Robot Hand (URDF CAD Meshes)")
        self.lbl_3d_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #ffffff;")
        self.btn_reset_view = QPushButton("Reset View")
        self.btn_reset_view.setStyleSheet("background-color: #242436; color: #38bdf8; border: 1px solid #383850; padding: 4px 10px; border-radius: 4px; font-size: 11px; font-weight: bold;")
        hand_3d_header.addWidget(self.lbl_3d_title)
        hand_3d_header.addStretch()
        hand_3d_header.addWidget(self.btn_reset_view)
        center_layout.addLayout(hand_3d_header)

        self.widget_3d = RobotHand3DWidget(hand_side=self.hand_side)
        self.btn_reset_view.clicked.connect(self.widget_3d.reset_camera)
        center_layout.addWidget(self.widget_3d, stretch=1)

        # 3D Helper status bar
        hand_3d_footer = QHBoxLayout()
        self.lbl_3d_status = QLabel("Interactive 3D CAD: Drag: Orbit | Right-Drag: Pan | Scroll: Zoom | Dbl-Click: Reset")
        self.lbl_3d_status.setStyleSheet("color: #8888a8; font-size: 11px;")
        hand_3d_footer.addWidget(self.lbl_3d_status)
        center_layout.addLayout(hand_3d_footer)

        main_layout.addLayout(center_layout, stretch=3)

        # ─── RIGHT COLUMN: Joint Telemetry & Cockpit Controls ────────────────────
        right_layout = QVBoxLayout()
        right_layout.setSpacing(12)

        # 1. 20-DOF Joint Telemetry Box
        telemetry_box = QGroupBox("🤖 20-DOF Joint Telemetry (Allegro Hand V6)")
        telemetry_layout = QVBoxLayout(telemetry_box)
        telemetry_layout.setSpacing(6)

        self.joint_bars: Dict[str, QProgressBar] = {}
        self.joint_labels: Dict[str, QLabel] = {}

        grid_layout = QGridLayout()
        grid_layout.setHorizontalSpacing(8)
        grid_layout.setVerticalSpacing(4)

        for col, finger in enumerate(FINGER_NAMES):
            f_label = QLabel(f"{finger}")
            f_label.setAlignment(Qt.AlignCenter)
            f_label.setStyleSheet("font-weight: bold; color: #60a5fa; font-size: 12px; margin-bottom: 2px;")
            grid_layout.addWidget(f_label, 0, col)

            for row in range(4):
                j_idx = col * 4 + row
                j_name = JOINT_NAMES_V6[j_idx]

                j_frame = QFrame()
                j_frame.setStyleSheet("background-color: #161622; border-radius: 4px; padding: 2px;")
                j_vbox = QVBoxLayout(j_frame)
                j_vbox.setContentsMargins(3, 2, 3, 2)
                j_vbox.setSpacing(1)

                j_val_lbl = QLabel(f"{j_name[5:]}: 0.00")
                j_val_lbl.setStyleSheet("font-size: 10px; color: #b0b0d0;")
                pbar = QProgressBar()
                pbar.setRange(0, 100)
                pbar.setValue(50)

                j_vbox.addWidget(j_val_lbl)
                j_vbox.addWidget(pbar)
                grid_layout.addWidget(j_frame, row + 1, col)

                self.joint_bars[j_name] = pbar
                self.joint_labels[j_name] = j_val_lbl

        telemetry_layout.addLayout(grid_layout)
        right_layout.addWidget(telemetry_box, stretch=2)

        # 2. Main Cockpit Operator Control Panel
        cockpit_box = QGroupBox("🎮 Teleoperation Cockpit & VLA Data Control")
        cockpit_layout = QVBoxLayout(cockpit_box)
        cockpit_layout.setSpacing(10)

        # 0. HAND Selection Card (Left / Right Model Switch)
        hand_card = QFrame()
        hand_card.setStyleSheet("background-color: #20202e; border: 1px solid #383850; border-radius: 6px; padding: 6px;")
        hand_card_layout = QHBoxLayout(hand_card)
        self.lbl_hand_badge = QLabel("HAND: [ ✋ LEFT HAND (왼손) ]" if self.hand_side == "left" else "HAND: [ 🤚 RIGHT HAND (오른손) ]")
        self.lbl_hand_badge.setStyleSheet("font-size: 14px; font-weight: bold; color: #38bdf8;" if self.hand_side == "left" else "font-size: 14px; font-weight: bold; color: #f59e0b;")
        self.btn_toggle_hand = QPushButton("Switch Hand [H]")
        self.btn_toggle_hand.setStyleSheet("background-color: #1e293b; color: #38bdf8; border: 1px solid #0284c7;" if self.hand_side == "left" else "background-color: #3b2010; color: #f59e0b; border: 1px solid #d97706;")
        self.btn_toggle_hand.clicked.connect(self.toggle_hand)
        hand_card_layout.addWidget(self.lbl_hand_badge)
        hand_card_layout.addStretch()
        hand_card_layout.addWidget(self.btn_toggle_hand)
        cockpit_layout.addWidget(hand_card)

        # A. CLUTCH Control Card
        clutch_card = QFrame()
        clutch_card.setStyleSheet("background-color: #20202e; border: 1px solid #383850; border-radius: 6px; padding: 6px;")
        clutch_card_layout = QHBoxLayout(clutch_card)
        self.lbl_clutch_badge = QLabel("CLUTCH: [ RELEASED - ACTIVE ▶ ]")
        self.lbl_clutch_badge.setStyleSheet("font-size: 14px; font-weight: bold; color: #4ade80;")
        self.btn_clutch = QPushButton("Toggle Clutch [C]")
        self.btn_clutch.setStyleSheet("background-color: #1e3a5f; color: #93c5fd; border: 1px solid #3b82f6;")
        self.btn_clutch.clicked.connect(self.toggle_clutch)
        clutch_card_layout.addWidget(self.lbl_clutch_badge)
        clutch_card_layout.addStretch()
        clutch_card_layout.addWidget(self.btn_clutch)
        cockpit_layout.addWidget(clutch_card)

        # B. RECORD Control Card
        record_card = QFrame()
        record_card.setStyleSheet("background-color: #20202e; border: 1px solid #383850; border-radius: 6px; padding: 6px;")
        record_card_layout = QHBoxLayout(record_card)
        self.lbl_rec_status = QLabel("RECORD: [ IDLE ⚪ ]")
        self.lbl_rec_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #9ca3af;")
        self.lbl_rec_timer = QLabel("00:00.0 (0 f)")
        self.lbl_rec_timer.setStyleSheet("font-size: 13px; font-weight: bold; color: #f3f4f6; margin-left: 10px;")
        self.btn_record = QPushButton("Start Recording [R]")
        self.btn_record.setStyleSheet("background-color: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444;")
        self.btn_record.clicked.connect(self.toggle_record)
        record_card_layout.addWidget(self.lbl_rec_status)
        record_card_layout.addWidget(self.lbl_rec_timer)
        record_card_layout.addStretch()
        record_card_layout.addWidget(self.btn_record)
        cockpit_layout.addWidget(record_card)

        # C. TAGGING Card (Success / Fail)
        tag_layout = QHBoxLayout()
        self.btn_tag_success = QPushButton("🟢 Tag SUCCESS [S]")
        self.btn_tag_success.setStyleSheet("background-color: #064e3b; color: #6ee7b7; border: 1px solid #10b981; padding: 10px;")
        self.btn_tag_success.clicked.connect(self.tag_success)

        self.btn_tag_fail = QPushButton("🔴 Tag FAIL [F]")
        self.btn_tag_fail.setStyleSheet("background-color: #450a0a; color: #fca5a5; border: 1px solid #dc2626; padding: 10px;")
        self.btn_tag_fail.clicked.connect(self.tag_fail)

        tag_layout.addWidget(self.btn_tag_success)
        tag_layout.addWidget(self.btn_tag_fail)
        cockpit_layout.addLayout(tag_layout)

        # D. Episode History Log & Table
        history_box = QHBoxLayout()
        self.lbl_manifest_info = QLabel("Episodes Saved: 0 (0 Success, 0 Fail)")
        self.lbl_manifest_info.setStyleSheet("color: #a0a0c0; font-size: 12px; font-weight: bold;")
        history_box.addWidget(self.lbl_manifest_info)
        history_box.addStretch()
        cockpit_layout.addLayout(history_box)

        self.table_episodes = QTableWidget(0, 4)
        self.table_episodes.setHorizontalHeaderLabels(["Episode ID", "Tag", "Frames", "Duration"])
        self.table_episodes.horizontalHeader().setStretchLastSection(True)
        self.table_episodes.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table_episodes.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table_episodes.setFixedHeight(120)
        cockpit_layout.addWidget(self.table_episodes)

        right_layout.addWidget(cockpit_box, stretch=3)
        main_layout.addLayout(right_layout, stretch=2)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Global key listener for C, R, S, F, H, Q shortcuts."""
        key = event.key()
        if key == Qt.Key_C:
            self.toggle_clutch()
        elif key == Qt.Key_R:
            self.toggle_record()
        elif key == Qt.Key_S:
            self.tag_success()
        elif key == Qt.Key_F:
            self.tag_fail()
        elif key == Qt.Key_H:
            self.toggle_hand()
        elif key == Qt.Key_Q or key == Qt.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)

    def toggle_hand(self) -> None:
        """Toggles between Left Hand and Right Hand in UI, broadcasts to retargeting node, and reloads RViz model."""
        if self.hand_side == "left":
            self.hand_side = "right"
            self.lbl_hand_badge.setText("HAND: [ 🤚 RIGHT HAND (오른손) ]")
            self.lbl_hand_badge.setStyleSheet("font-size: 14px; font-weight: bold; color: #f59e0b;")
            self.btn_toggle_hand.setStyleSheet("background-color: #3b2010; color: #f59e0b; border: 1px solid #d97706;")
        else:
            self.hand_side = "left"
            self.lbl_hand_badge.setText("HAND: [ ✋ LEFT HAND (왼손) ]")
            self.lbl_hand_badge.setStyleSheet("font-size: 14px; font-weight: bold; color: #38bdf8;")
            self.btn_toggle_hand.setStyleSheet("background-color: #1e293b; color: #38bdf8; border: 1px solid #0284c7;")

        # 1. Update ROS worker node hand_side
        self.ros_node.hand_side = self.hand_side

        # 2. Broadcast state to retargeting node and sim_bridge_node
        self.broadcast_state()

        # 2. Update embedded 3D Robot Hand model
        self.widget_3d.set_hand_side(self.hand_side)

        # 3. Live update RViz and robot_state_publisher with the new hand model!
        new_urdf = self.urdf_right if self.hand_side == "right" else self.urdf_left
        if new_urdf:
            self.ros_node.update_robot_description(new_urdf)

    def toggle_clutch(self) -> None:
        self.clutch_engaged = not self.clutch_engaged
        if self.clutch_engaged:
            self.lbl_clutch_badge.setText("CLUTCH: [ ENGAGED - PAUSED ⏸ ]")
            self.lbl_clutch_badge.setStyleSheet("font-size: 14px; font-weight: bold; color: #fbbf24;")
            self.btn_clutch.setText("Resume Tracking [C]")
        else:
            self.lbl_clutch_badge.setText("CLUTCH: [ RELEASED - ACTIVE ▶ ]")
            self.lbl_clutch_badge.setStyleSheet("font-size: 14px; font-weight: bold; color: #4ade80;")
            self.btn_clutch.setText("Pause Hand [C]")
        self.broadcast_state()

    def toggle_record(self) -> None:
        self.is_recording = not self.is_recording
        if self.is_recording:
            self.episode_counter += 1
            self.record_start_mono = time.monotonic()
            self.last_tag = "NONE"
            self.lbl_rec_status.setText(f"RECORD: [ RECORDING 🔴 (Ep #{self.episode_counter}) ]")
            self.lbl_rec_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #ef4444;")
            self.btn_record.setText("Stop Recording [R]")
            self.btn_record.setStyleSheet("background-color: #991b1b; color: #ffffff; border: 1px solid #f87171;")
        else:
            self.lbl_rec_status.setText("RECORD: [ IDLE ⚪ ]")
            self.lbl_rec_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #9ca3af;")
            self.btn_record.setText("Start Recording [R]")
            self.btn_record.setStyleSheet("background-color: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444;")
        self.broadcast_state()

    def tag_success(self) -> None:
        self.last_tag = "SUCCESS"
        self.total_episodes_saved += 1
        self.success_count += 1
        self._add_table_row(f"ep_{datetime.now().strftime('%H%M%S')}_{self.episode_counter:03d}", "SUCCESS", self.lbl_rec_timer.text())
        self.lbl_manifest_info.setText(f"Episodes Saved: {self.total_episodes_saved} ({self.success_count} Success, {self.fail_count} Fail)")
        self.broadcast_state()

    def tag_fail(self) -> None:
        self.last_tag = "FAIL"
        self.total_episodes_saved += 1
        self.fail_count += 1
        self._add_table_row(f"ep_{datetime.now().strftime('%H%M%S')}_{self.episode_counter:03d}", "FAIL", self.lbl_rec_timer.text())
        self.lbl_manifest_info.setText(f"Episodes Saved: {self.total_episodes_saved} ({self.success_count} Success, {self.fail_count} Fail)")
        self.broadcast_state()

    def _add_table_row(self, ep_id: str, tag: str, dur_str: str) -> None:
        row = self.table_episodes.rowCount()
        self.table_episodes.insertRow(row)
        self.table_episodes.setItem(row, 0, QTableWidgetItem(ep_id))
        tag_item = QTableWidgetItem(tag)
        tag_item.setForeground(QColor("#4ade80") if tag == "SUCCESS" else QColor("#f87171"))
        self.table_episodes.setItem(row, 1, tag_item)
        self.table_episodes.setItem(row, 2, QTableWidgetItem("Buffered"))
        self.table_episodes.setItem(row, 3, QTableWidgetItem(dur_str))
        self.table_episodes.scrollToBottom()

    def broadcast_state(self) -> None:
        payload = {
            "clutch": self.clutch_engaged,
            "clutch_state": "ENGAGED" if self.clutch_engaged else "RELEASED",
            "record": self.is_recording,
            "record_state": "RECORDING" if self.is_recording else "IDLE",
            "tag": self.last_tag,
            "hand_side": self.hand_side,
            "episode_index": self.episode_counter,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.ros_node.pub_state.publish(msg)

    def update_tick(self) -> None:
        """Main update loop: processes video frame, runs MediaPipe, updates UI, spins ROS."""
        # Spin ROS 2 events
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.001)

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # Compute FPS
        self._fps_counter += 1
        now = time.monotonic()
        if now - self._last_fps_time >= 1.0:
            self.current_fps = self._fps_counter / (now - self._last_fps_time)
            self._fps_counter = 0
            self._last_fps_time = now
            self.lbl_fps_badge.setText(f"FPS: {self.current_fps:.1f}")

        # Update Recording Timer
        if self.is_recording and self.record_start_mono is not None:
            dur = time.monotonic() - self.record_start_mono
            mins = int(dur // 60)
            secs = dur % 60
            frames = int(dur * self.target_fps)
            self.lbl_rec_timer.setText(f"{mins:02d}:{secs:04.1f} ({frames} f)")

        # MediaPipe Processing
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True

        if results.multi_hand_landmarks:
            self.lbl_tracking_badge.setText("TRACKING 21 PTS 🟢")
            self.lbl_tracking_badge.setStyleSheet("background-color: #143820; color: #4ade80; padding: 4px 10px; border-radius: 4px; font-weight: bold;")
            lm_list = results.multi_hand_landmarks[0]

            # Publish 21 landmarks (63 floats)
            coords: List[float] = []
            for i in range(NUM_POINTS):
                pt = lm_list.landmark[i]
                coords.extend([float(pt.x), float(pt.y), float(pt.z)])

            msg_lm = Float32MultiArray()
            msg_lm.data = coords
            self.ros_node.pub_landmarks.publish(msg_lm)

            # Draw landmarks
            self.mp_drawing.draw_landmarks(
                frame,
                lm_list,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style(),
            )

            # Compute pinch distance (Thumb tip 4 to Index tip 8) in normalized coords
            p_thumb = np.array([lm_list.landmark[4].x, lm_list.landmark[4].y, lm_list.landmark[4].z])
            p_index = np.array([lm_list.landmark[8].x, lm_list.landmark[8].y, lm_list.landmark[8].z])
            dist_norm = float(np.linalg.norm(p_thumb - p_index))
            dist_cm = dist_norm * 25.0  # approximate scale to cm
            self.lbl_pinch_info.setText(f"Pinch Dist: {dist_cm:.1f} cm")
        else:
            self.lbl_tracking_badge.setText("SEARCHING... ⚪")
            self.lbl_tracking_badge.setStyleSheet("background-color: #3f2020; color: #f87171; padding: 4px 10px; border-radius: 4px; font-weight: bold;")
            self.lbl_pinch_info.setText("Pinch Dist: -- cm")

        # Publish camera image if bridge available
        if self.bridge is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.ros_node.pub_image.publish(img_msg)
            except Exception:
                pass

        # Display Frame in QLabel
        qimg = QImage(frame.data, w, h, 3 * w, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qimg)
        scaled_pix = pixmap.scaled(self.lbl_video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video.setPixmap(scaled_pix)

        # Update 20-DOF Joint Bars
        target_q = self.ros_node.latest_target_joints
        for i, jname in enumerate(JOINT_NAMES_V6):
            val = target_q[i] if i < len(target_q) else 0.0
            min_lim, max_lim = LIMITS_V6.get(jname, (-1.5, 1.5))
            pct = int(np.clip((val - min_lim) / (max_lim - min_lim + 1e-6) * 100.0, 0, 100))

            if jname in self.joint_bars:
                self.joint_bars[jname].setValue(pct)
            if jname in self.joint_labels:
                short_name = jname.replace("joint", "j")
                self.joint_labels[jname].setText(f"{short_name}: {val:+.2f}")

        # Update 3D Robot Hand Kinematic View
        is_pinch = (dist_cm < 2.5) if "dist_cm" in locals() else False
        self.widget_3d.update_joints(target_q, pinch_active=is_pinch)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.timer.stop()
        if self.cap.isOpened():
            self.cap.release()
        self.hands.close()
        self.ros_node.destroy_node()
        event.accept()


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand V6 Teleoperation Cockpit Dashboard")
    parser.add_argument("--device", default="auto", help="Webcam device ID (default: auto for Intel RealSense)")
    parser.add_argument("--fps", type=int, default=60, help="Target camera frame rate (default: 60)")
    parser.add_argument("--hand", default="right", choices=["left", "right"], help="Hand model side (default: right)")
    args = parser.parse_args()

    rclpy.init()
    app = QApplication(sys.argv)
    window = TeleopDashboardWindow(device_id=args.device, target_fps=args.fps, hand_side=args.hand)
    window.show()

    exit_code = app.exec_()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
