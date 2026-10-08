#!/usr/bin/env python3
"""
vision_tracker_v6.py — ROS 2 Vision Publisher Node for 5-Finger Robot Hand (Allegro Hand V6) Retargeting.

Features:
- Captures RGB stream from Intel RealSense (auto-detected) or standard webcam (/dev/video0).
- Extracts full 21 3D hand landmarks via MediaPipe Hands (Thumb, Index, Middle, Ring, Pinky).
- Publishes 63-dimensional Float32MultiArray to `/allegro/vision/landmarks`.
- Publishes RGB camera frames to `/allegro/camera/image_raw` for VLA dataset collection.
- Real-time OpenCV HUD & Operator Teleop Controls:
    * 'C' Key: Toggle Clutch (ENGAGED / RELEASED). When ENGAGED, freezes robot hand movement (Hold).
    * 'R' Key: Toggle Recording (RECORDING 🔴 / IDLE ⚪). Buffers teleop episode for VLA training.
    * 'S' Key: Tag episode as SUCCESS 🟢.
    * 'F' Key: Tag episode as FAIL 🔴.
    * 'Q' / ESC: Graceful exit.
- Publishes operator state (Clutch, Record, Tag, Ep#) to `/allegro/teleop_state`.
- Scaled up window display (1.75x) for high visibility during live teleoperation.

Usage:
    python3 vision_tracker_v6.py [--device auto] [--scale 1.75] [--fps 60] [--no-gui]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import List, Optional

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

TOPIC_LANDMARKS = "/allegro/vision/landmarks"
TOPIC_TELEOP_STATE = "/allegro/teleop_state"
TOPIC_CAMERA_IMAGE = "/allegro/camera/image_raw"

# 21 landmarks: 0 (Wrist), 1-4 (Thumb), 5-8 (Index), 9-12 (Middle), 13-16 (Ring), 17-20 (Pinky)
NUM_POINTS = 21
NUM_COORDS = NUM_POINTS * 3  # 63 floats

VISION_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def find_best_camera_device() -> tuple[int, str]:
    """Auto-detect working camera device: prioritizes Intel RealSense, then probes active capture devices via OpenCV."""
    import glob
    import subprocess
    import cv2

    dev_paths = sorted(
        glob.glob("/dev/video*"),
        key=lambda p: int(p.replace("/dev/video", "")) if p.replace("/dev/video", "").isdigit() else 999,
    )

    # 1. First prioritize Intel RealSense RGB camera if connected
    for dev in dev_paths:
        dev_idx_str = dev.replace("/dev/video", "")
        if not dev_idx_str.isdigit():
            continue
        idx = int(dev_idx_str)
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "-d", dev, "--all"],
                stderr=subprocess.DEVNULL,
                timeout=0.5,
            ).decode("utf-8", errors="ignore")
            if "RealSense" in out and ("YUYV" in out or "white_balance" in out):
                return idx, f"Intel RealSense RGB Camera (/dev/video{idx})"
        except Exception:
            pass

    # 2. Probe working video capture devices using OpenCV
    for dev in dev_paths:
        dev_idx_str = dev.replace("/dev/video", "")
        if not dev_idx_str.isdigit():
            continue
        idx = int(dev_idx_str)
        try:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None and frame.size > 0:
                    name = "Camera"
                    try:
                        with open(f"/sys/class/video4linux/video{idx}/name") as f:
                            name = f.read().strip()
                    except Exception:
                        pass
                    return idx, f"{name} (/dev/video{idx})"
        except Exception:
            pass

    return 0, "Default Camera (/dev/video0)"


class VisionTrackerNode(Node):
    def __init__(self, device_id: int | str = "auto", gui: bool = True, scale: float = 1.75, fps: int = 60) -> None:
        super().__init__("vision_tracker")
        if str(device_id).lower() in ("auto", "-1", "none", ""):
            self.device_id, self.dev_desc = find_best_camera_device()
        else:
            self.device_id = int(device_id)
            self.dev_desc = f"/dev/video{self.device_id}"

        self.gui = gui
        self.scale = max(0.5, float(scale))
        self.fps = max(15, int(fps))
        self.window_name = "Allegro Hand V6 (5-Finger) Vision Tracker & Data Collector"
        self._window_initialized = False

        # ROS 2 Publishers
        self._pub_landmarks = self.create_publisher(Float32MultiArray, TOPIC_LANDMARKS, VISION_QOS)
        self._pub_state = self.create_publisher(String, TOPIC_TELEOP_STATE, STATE_QOS)
        self._pub_image = self.create_publisher(Image, TOPIC_CAMERA_IMAGE, VISION_QOS)

        if CV_BRIDGE_AVAILABLE:
            self._bridge = CvBridge()
        else:
            self._bridge = None

        # MediaPipe Hands Setup (21-point tracking)
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # Video Capture Setup
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device {self.dev_desc}")
            raise RuntimeError(f"Cannot open camera {self.dev_desc}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        # Teleop Operator States
        self.clutch_engaged: bool = False
        self.is_recording: bool = False
        self.last_tag: str = "NONE"
        self.episode_counter: int = 0
        self.record_start_time: Optional[float] = None

        # Statistics & FPS
        self._seq = 0
        self._fps_counter = 0
        self._last_fps_time = time.monotonic()
        self._current_fps = 0.0

        # UI blinker for recording indication
        self._blink_state = False
        self._last_blink_time = time.monotonic()

        disp_w = int(640 * self.scale)
        disp_h = int(480 * self.scale)
        gui_str = f"GUI Window Active ({disp_w}x{disp_h}, scale={self.scale:.2f}x)" if self.gui else "Headless Mode"
        self.get_logger().info(f"Vision tracker V6 initialized on {self.dev_desc} (640x480@{self.fps}fps) | {gui_str}")
        self.get_logger().info(f"Publishing 21 landmarks (63-dim) -> '{TOPIC_LANDMARKS}'")
        self.get_logger().info(f"Publishing teleop state -> '{TOPIC_TELEOP_STATE}'")

        # Initial state broadcast
        self.publish_teleop_state()

    def publish_teleop_state(self) -> None:
        """Broadcasts operator status (Clutch, Record, Tag) as JSON."""
        state_payload = {
            "clutch": self.clutch_engaged,
            "clutch_state": "ENGAGED" if self.clutch_engaged else "RELEASED",
            "record": self.is_recording,
            "record_state": "RECORDING" if self.is_recording else "IDLE",
            "tag": self.last_tag,
            "episode_index": self.episode_counter,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(state_payload)
        self._pub_state.publish(msg)

    def step(self) -> bool:
        """Reads one frame, extracts 21 landmarks, renders HUD, handles operator keys."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab video frame.")
            return False

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        results = self.hands.process(rgb_frame)
        rgb_frame.flags.writeable = True

        # FPS calculation
        self._fps_counter += 1
        now = time.monotonic()
        if now - self._last_fps_time >= 1.0:
            self._current_fps = self._fps_counter / (now - self._last_fps_time)
            self._fps_counter = 0
            self._last_fps_time = now

        # Toggle recording indicator blink every 500ms
        if now - self._last_blink_time >= 0.5:
            self._blink_state = not self._blink_state
            self._last_blink_time = now

        hand_detected = False
        hand_landmarks = None
        if results.multi_hand_landmarks:
            hand_detected = True
            hand_landmarks = results.multi_hand_landmarks[0]

            # Extract full 21 3D landmarks (Wrist 0, Thumb 1~4, Index 5~8, Middle 9~12, Ring 13~16, Pinky 17~20)
            coords: List[float] = []
            for i in range(NUM_POINTS):
                lm = hand_landmarks.landmark[i]
                coords.extend([float(lm.x), float(lm.y), float(lm.z)])

            # Publish landmarks topic
            msg_lm = Float32MultiArray()
            msg_lm.data = coords
            self._pub_landmarks.publish(msg_lm)
            self._seq += 1

        # Publish camera frame for VLA dataset recorder
        if self._bridge is not None:
            try:
                img_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_optical_frame"
                self._pub_image.publish(img_msg)
            except Exception:
                pass

        # Periodically publish teleop operator state
        self.publish_teleop_state()

        if self.gui:
            disp_w = int(w * self.scale)
            disp_h = int(h * self.scale)
            disp_frame = cv2.resize(frame, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)

            if not self._window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, disp_w, disp_h)
                self._window_initialized = True

            # Draw MediaPipe hand mesh and 21 landmark points
            if hand_landmarks is not None:
                self.mp_drawing.draw_landmarks(
                    disp_frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )
                point_radius = max(4, int(5 * (self.scale / 1.5)))
                for i in range(NUM_POINTS):
                    lm = hand_landmarks.landmark[i]
                    cx, cy = int(lm.x * disp_w), int(lm.y * disp_h)
                    cv2.circle(disp_frame, (cx, cy), point_radius, (0, 255, 0), -1)
                    cv2.circle(disp_frame, (cx, cy), point_radius + 1, (255, 255, 255), 1)

            # Draw HUD Overlay Box (Top-Left)
            hud_w, hud_h = int(430 * (self.scale / 1.5)), int(155 * (self.scale / 1.5))
            overlay = disp_frame.copy()
            cv2.rectangle(overlay, (12, 12), (12 + hud_w, 12 + hud_h), (25, 25, 25), -1)
            cv2.addWeighted(overlay, 0.70, disp_frame, 0.30, 0, disp_frame)
            cv2.rectangle(disp_frame, (12, 12), (12 + hud_w, 12 + hud_h), (80, 80, 80), 1)

            font_scale = 0.58 * (self.scale / 1.5)
            y_base = int(20 * (self.scale / 1.5))
            y_step = int(30 * (self.scale / 1.5))

            # 1. Hand Tracking & FPS
            hand_color = (0, 255, 0) if hand_detected else (0, 140, 255)
            hand_text = f"HAND: {'TRACKING (21 pts)' if hand_detected else 'SEARCHING...'}"
            cv2.putText(disp_frame, hand_text, (20, y_base + y_step), cv2.FONT_HERSHEY_SIMPLEX, font_scale, hand_color, 2)
            cv2.putText(disp_frame, f"FPS: {self._current_fps:.1f}", (int(290 * (self.scale / 1.5)), y_base + y_step), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.9, (220, 220, 220), 1)

            # 2. Clutch Status (Pause robot hand hold)
            if self.clutch_engaged:
                clutch_text = "CLUTCH: [ ENGAGED - PAUSED ]"
                clutch_color = (0, 215, 255)  # Bright amber
            else:
                clutch_text = "CLUTCH: [ RELEASED - ACTIVE ]"
                clutch_color = (0, 255, 0)  # Bright green
            cv2.putText(disp_frame, clutch_text, (20, y_base + y_step * 2), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 1.05, clutch_color, 2)

            # 3. Recording Status
            if self.is_recording:
                rec_dot_color = (0, 0, 255) if self._blink_state else (50, 50, 200)
                rec_dur = time.monotonic() - (self.record_start_time or time.monotonic())
                rec_text = f"RECORD: [ RECORDING ] (Ep #{self.episode_counter} | {rec_dur:.1f}s)"
                cv2.circle(disp_frame, (int(410 * (self.scale / 1.5)), y_base + y_step * 3 - int(5 * self.scale / 1.5)), int(7 * (self.scale / 1.5)), rec_dot_color, -1)
                rec_color = (0, 0, 255)
            else:
                rec_text = f"RECORD: [ IDLE ] (Total Ep: {self.episode_counter})"
                rec_color = (180, 180, 180)
            cv2.putText(disp_frame, rec_text, (20, y_base + y_step * 3), cv2.FONT_HERSHEY_SIMPLEX, font_scale, rec_color, 2)

            # 4. Episode Tagging Status
            if self.last_tag == "SUCCESS":
                tag_color = (0, 255, 0)
                tag_str = "SUCCESS"
            elif self.last_tag == "FAIL":
                tag_color = (0, 0, 255)
                tag_str = "FAIL"
            else:
                tag_color = (200, 200, 200)
                tag_str = "NONE (Pending tag)"
            cv2.putText(disp_frame, f"LAST TAG: {tag_str}", (20, y_base + y_step * 4), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.95, tag_color, 2)

            # 5. Hotkey shortcuts bar (Bottom)
            cv2.putText(
                disp_frame,
                "Shortcuts: [C] Clutch (Hold Hand) | [R] Record Episode | [S] Tag Success | [F] Tag Fail | [Q] Quit",
                (15, disp_h - int(30 * (self.scale / 1.5))),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.85,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                disp_frame,
                f"Camera: {self.dev_desc} | 5 Fingers Active: Thumb, Index, Middle, Ring, Pinky (21 keypoints)",
                (15, disp_h - int(12 * (self.scale / 1.5))),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.75,
                (180, 180, 180),
                1,
            )

            cv2.imshow(self.window_name, disp_frame)
            key = cv2.waitKey(1) & 0xFF

            # Key Actions
            if key == ord("q") or key == ord("Q") or key == 27:
                self.get_logger().info("Quit requested via GUI.")
                return False

            elif key == ord("c") or key == ord("C"):
                self.clutch_engaged = not self.clutch_engaged
                state_str = "ENGAGED (Robot hand holding current pose)" if self.clutch_engaged else "RELEASED (Robot tracking live hand)"
                self.get_logger().info(f"[OPERATOR] Clutch toggled -> {state_str}")
                self.publish_teleop_state()

            elif key == ord("r") or key == ord("R"):
                self.is_recording = not self.is_recording
                if self.is_recording:
                    self.episode_counter += 1
                    self.record_start_time = time.monotonic()
                    self.last_tag = "NONE"
                    self.get_logger().info(f"[OPERATOR] 🔴 Recording STARTED for Episode #{self.episode_counter}")
                else:
                    dur = time.monotonic() - (self.record_start_time or time.monotonic())
                    self.get_logger().info(
                        f"[OPERATOR] ⏹ Recording STOPPED (Episode #{self.episode_counter}, Duration: {dur:.2f}s). "
                        "Press 'S' (Success) or 'F' (Fail) to tag."
                    )
                self.publish_teleop_state()

            elif key == ord("s") or key == ord("S"):
                self.last_tag = "SUCCESS"
                self.get_logger().info(f"[OPERATOR] 🟢 Episode #{self.episode_counter} tagged as SUCCESS!")
                self.publish_teleop_state()

            elif key == ord("f") or key == ord("F"):
                self.last_tag = "FAIL"
                self.get_logger().warn(f"[OPERATOR] 🔴 Episode #{self.episode_counter} tagged as FAIL!")
                self.publish_teleop_state()

        return True

    def cleanup(self) -> None:
        self.get_logger().info("Cleaning up vision tracker resources...")
        if self.cap.isOpened():
            self.cap.release()
        if self.gui:
            cv2.destroyAllWindows()
        self.hands.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand V6 (5-Finger) Vision Tracker Node")
    parser.add_argument("--device", default="auto", help="Webcam device ID (default: auto for Intel RealSense)")
    parser.add_argument("--scale", type=float, default=1.75, help="Window display scale factor (default: 1.75)")
    parser.add_argument("--fps", type=int, default=60, help="Camera capture frame rate (default: 60)")
    parser.add_argument("--no-gui", action="store_true", help="Run without OpenCV GUI window")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = VisionTrackerNode(device_id=args.device, gui=not args.no_gui, scale=args.scale, fps=args.fps)
    except Exception as e:
        print(f"[ERROR] Failed to start VisionTrackerNode: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        while rclpy.ok():
            if not node.step():
                break
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.001)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down vision tracker.")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
