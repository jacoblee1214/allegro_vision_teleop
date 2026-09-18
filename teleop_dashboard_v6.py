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
            script_path = "/home/humble_ws/allegro_vision_teleop/teleop_dashboard_v6.py"
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
from PyQt5.QtCore import QCoreApplication, Qt, QTimer
if os.path.exists("/usr/lib/x86_64-linux-gnu/qt5/plugins"):
    QCoreApplication.setLibraryPaths(["/usr/lib/x86_64-linux-gnu/qt5/plugins"])
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPalette, QPixmap
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
)

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
    "joint00": (-1.658, 1.658),
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
                    # Real left hand motor reports with -90 deg offset on MCP; normalize to standard URDF 0.0 rad
                    if self.hand_side == "left" and out_idx in (5, 9, 13, 17) and val < -0.5:
                        val += np.deg2rad(90.0)
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
        self.resize(1380, 860)
        self.setMinimumSize(1100, 700)

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
        self.lbl_video.setMinimumSize(640, 480)
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
