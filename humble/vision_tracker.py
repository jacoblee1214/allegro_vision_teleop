#!/usr/bin/env python3
"""
vision_tracker.py — ROS 2 Vision Publisher Node with Teleop HUD & Operator Controls.

Features:
- Captures RGB camera stream (/dev/video0 or user device).
- Real-time hand landmark extraction via MediaPipe Hands (0~16: Wrist, Thumb, Index, Middle, Ring; 17~20 Pinky excluded).
- Publishes 51-dim Float32MultiArray to `/allegro/vision/landmarks`.
- Publishes RGB camera frame to `/allegro/camera/image_raw` for VLA dataset recording.
- OpenCV HUD & Operator Control Keys:
    * 'C' Key: Toggle Clutch (ENGAGED / RELEASED). When ENGAGED, freezes teleop joint tracking.
    * 'R' Key: Toggle Recording (RECORDING 🔴 / IDLE ⚪).
    * 'S' Key: Tag current/last episode as SUCCESS 🟢.
    * 'F' Key: Tag current/last episode as FAIL 🔴.
    * 'Q' / ESC: Graceful exit.
- Publishes teleop operator status (Clutch, Record, Tag) to `/allegro/teleop_state` as JSON string.

Usage:
    python3 vision_tracker.py [--device 0] [--fps 60] [--no-gui]
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

# 17 landmarks: 0 (Wrist), 1-4 (Thumb), 5-8 (Index), 9-12 (Middle), 13-16 (Ring)
NUM_POINTS = 17
NUM_COORDS = NUM_POINTS * 3  # 51 floats

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


class VisionTrackerNode(Node):
    def __init__(self, device_id: int = 0, target_fps: int = 60, gui: bool = True) -> None:
        super().__init__("vision_tracker")
        self.device_id = device_id
        self.target_fps = target_fps
        self.gui = gui

        # ROS 2 Publishers
        self._pub_landmarks = self.create_publisher(Float32MultiArray, TOPIC_LANDMARKS, VISION_QOS)
        self._pub_state = self.create_publisher(String, TOPIC_TELEOP_STATE, STATE_QOS)
        self._pub_image = self.create_publisher(Image, TOPIC_CAMERA_IMAGE, VISION_QOS)

        if CV_BRIDGE_AVAILABLE:
            self._bridge = CvBridge()
        else:
            self._bridge = None
            self.get_logger().warn("cv_bridge not available; image topic publishing will be disabled.")

        # MediaPipe Hands Setup
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
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device /dev/video{self.device_id}")
            raise RuntimeError(f"Cannot open webcam /dev/video{self.device_id}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

        # Teleop & Recording State
        self.clutch_engaged: bool = False
        self.is_recording: bool = False
        self.last_tag: str = "NONE"  # "NONE", "SUCCESS", "FAIL"
        self.episode_counter: int = 0
        self.record_start_time: Optional[float] = None

        # Statistics & FPS
        self._seq = 0
        self._fps_counter = 0
        self._last_fps_time = time.monotonic()
        self._current_fps = 0.0

        # UI blinker timer
        self._blink_state = False
        self._last_blink_time = time.monotonic()

        self.get_logger().info(f"Vision tracker initialized on /dev/video{self.device_id} (640x480 @ {self.target_fps}Hz)")
        self.get_logger().info(f"Publishing landmarks -> '{TOPIC_LANDMARKS}'")
        self.get_logger().info(f"Publishing teleop state -> '{TOPIC_TELEOP_STATE}'")
        self.get_logger().info(f"Publishing camera image -> '{TOPIC_CAMERA_IMAGE}'")

        # Initial state broadcast
        self.publish_teleop_state()

    def publish_teleop_state(self) -> None:
        """Publishes the current teleop operator status as a structured JSON message."""
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
        """Processes one video frame, tracks hand landmarks, publishes state & GUI. Returns False on quit."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab video frame.")
            return False

        # Flip horizontally for natural mirror interaction
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # Convert BGR to RGB for MediaPipe inference
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        results = self.hands.process(rgb_frame)
        rgb_frame.flags.writeable = True

        # Compute FPS
        self._fps_counter += 1
        now = time.monotonic()
        if now - self._last_fps_time >= 1.0:
            self._current_fps = self._fps_counter / (now - self._last_fps_time)
            self._fps_counter = 0
            self._last_fps_time = now

        # Toggle blink state every 500ms for recording indicator
        if now - self._last_blink_time >= 0.5:
            self._blink_state = not self._blink_state
            self._last_blink_time = now

        hand_detected = False
        if results.multi_hand_landmarks:
            hand_detected = True
            hand_landmarks = results.multi_hand_landmarks[0]

            # Extract landmarks 0~16 (17 points total: Wrist + 4 fingers)
            coords: List[float] = []
            for i in range(NUM_POINTS):
                lm = hand_landmarks.landmark[i]
                coords.extend([float(lm.x), float(lm.y), float(lm.z)])

            # Publish landmarks topic
            msg_lm = Float32MultiArray()
            msg_lm.data = coords
            self._pub_landmarks.publish(msg_lm)
            self._seq += 1

            if self.gui:
                # Draw MediaPipe hand skeleton
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

                # Draw included landmarks (Green) vs excluded Pinky (Red)
                for i in range(NUM_POINTS):
                    lm = hand_landmarks.landmark[i]
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

                for i in range(17, 21):
                    lm = hand_landmarks.landmark[i]
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        # Publish Camera Image topic if bridge is available
        if self._bridge is not None:
            try:
                img_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_optical_frame"
                self._pub_image.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish camera image: {e}", throttle_duration_sec=5.0)

        # Always publish state periodically to keep subscribers updated
        self.publish_teleop_state()

        # Render OpenCV HUD and handle keyboard interaction
        if self.gui:
            # Draw HUD overlay background
            hud_w, hud_h = 420, 155
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (10 + hud_w, 10 + hud_h), (25, 25, 25), -1)
            cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
            cv2.rectangle(frame, (10, 10), (10 + hud_w, 10 + hud_h), (80, 80, 80), 1)

            # 1. Hand Tracking & FPS status
            hand_color = (0, 255, 0) if hand_detected else (0, 140, 255)
            hand_str = "TRACKING (17 pts)" if hand_detected else "SEARCHING..."
            cv2.putText(frame, f"HAND: {hand_str}", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, hand_color, 2)
            cv2.putText(frame, f"FPS: {self._current_fps:.1f}", (280, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1)

            # 2. Clutch Status
            if self.clutch_engaged:
                clutch_text = "CLUTCH: [ ENGAGED - PAUSED ]"
                clutch_color = (0, 215, 255)  # Bright amber/yellow
            else:
                clutch_text = "CLUTCH: [ RELEASED - ACTIVE ]"
                clutch_color = (0, 255, 0)  # Bright green
            cv2.putText(frame, clutch_text, (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.58, clutch_color, 2)

            # 3. Recording Status
            if self.is_recording:
                # Blinking REC dot
                rec_dot_color = (0, 0, 255) if self._blink_state else (50, 50, 200)
                rec_dur = time.monotonic() - (self.record_start_time or time.monotonic())
                rec_text = f"RECORD: [ RECORDING ] (Ep #{self.episode_counter} | {rec_dur:.1f}s)"
                rec_color = (0, 0, 255)
                cv2.circle(frame, (390, 87), 7, rec_dot_color, -1)
            else:
                rec_text = f"RECORD: [ IDLE ] (Total Ep: {self.episode_counter})"
                rec_color = (180, 180, 180)
            cv2.putText(frame, rec_text, (20, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, rec_color, 2)

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
            cv2.putText(frame, f"LAST TAG: {tag_str}", (20, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.52, tag_color, 2)

            # 5. Hotkey shortcuts bar (Bottom of window)
            hotkey_str = "[C] Clutch | [R] Record | [S] Tag Success | [F] Tag Fail | [Q] Quit"
            cv2.putText(frame, hotkey_str, (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(
                frame,
                "Green: Active 4 fingers (0~16) | Red: Excluded Pinky (17~20)",
                (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (170, 170, 170),
                1,
            )

            # Display Window
            cv2.imshow("Allegro Hand V4 Vision Teleop & Data Collector", frame)
            key = cv2.waitKey(1) & 0xFF

            # Key Event Handling
            if key == ord("q") or key == ord("Q") or key == 27:  # 'q' or ESC
                self.get_logger().info("Quit requested via OpenCV GUI.")
                return False

            elif key == ord("c") or key == ord("C"):
                # Toggle Clutch
                self.clutch_engaged = not self.clutch_engaged
                state_str = "ENGAGED (Teleop Paused)" if self.clutch_engaged else "RELEASED (Teleop Resumed)"
                self.get_logger().info(f"[OPERATOR] Clutch toggled -> {state_str}")
                self.publish_teleop_state()

            elif key == ord("r") or key == ord("R"):
                # Toggle Recording
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
                # Tag Success
                self.last_tag = "SUCCESS"
                self.get_logger().info(f"[OPERATOR] 🟢 Episode #{self.episode_counter} tagged as SUCCESS!")
                self.publish_teleop_state()

            elif key == ord("f") or key == ord("F"):
                # Tag Fail
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
    parser = argparse.ArgumentParser(description="Allegro Hand V4 Vision Tracker & Data Collector Node")
    parser.add_argument("--device", type=int, default=0, help="Webcam device ID (default: 0 for /dev/video0)")
    parser.add_argument("--fps", type=int, default=60, help="Target camera frame rate (default: 60)")
    parser.add_argument("--no-gui", action="store_true", help="Run without OpenCV imshow GUI window (headless mode)")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = VisionTrackerNode(device_id=args.device, target_fps=args.fps, gui=not args.no_gui)
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
