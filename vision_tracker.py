#!/usr/bin/env python3
"""
vision_tracker.py — ROS 2 Vision Publisher Node for Allegro Hand Retargeting.

Captures webcam stream from /dev/video0, extracts hand landmarks using MediaPipe Hands,
filters out the pinky finger (keeping landmarks 0~16: Wrist, Thumb, Index, Middle, Ring),
and publishes a 51-dimensional Float32MultiArray to `/allegro/vision/landmarks`.
Includes real-time OpenCV GUI visualization for debugging (disable with --no-gui).

Usage (inside container):
    source /opt/ros/humble/setup.bash
    python3 /home/humble_ws/allegro_vision_teleop/vision_tracker.py [--device 0] [--no-gui]
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray

TOPIC_LANDMARKS = "/allegro/vision/landmarks"

# 17 landmarks: 0 (Wrist), 1-4 (Thumb), 5-8 (Index), 9-12 (Middle), 13-16 (Ring)
NUM_POINTS = 17
NUM_COORDS = NUM_POINTS * 3  # 51 floats

VISION_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class VisionTrackerNode(Node):
    def __init__(self, device_id: int = 0, gui: bool = True) -> None:
        super().__init__("vision_tracker")
        self.device_id = device_id
        self.gui = gui

        # ROS 2 Publisher
        self._pub = self.create_publisher(Float32MultiArray, TOPIC_LANDMARKS, VISION_QOS)
        self.get_logger().info(f"Publishing 3D hand landmarks (51-dim) to '{TOPIC_LANDMARKS}'")

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

        self._seq = 0
        self._fps_counter = 0
        self._last_fps_time = time.monotonic()
        self._current_fps = 0.0

        gui_str = "GUI Window Active (Press 'q' or ESC to exit)" if self.gui else "Headless Mode (No GUI)"
        self.get_logger().info(
            f"Vision tracker initialized. Device: /dev/video{self.device_id} (640x480) | {gui_str}"
        )

    def step(self) -> bool:
        """Reads one frame, processes landmarks, publishes topic, displays GUI if enabled. Returns False to exit."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab video frame.")
            return False

        # Flip horizontally for natural mirror view
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # Convert BGR to RGB for MediaPipe
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

        hand_detected = False
        if results.multi_hand_landmarks:
            hand_detected = True
            hand_landmarks = results.multi_hand_landmarks[0]

            # Extract landmarks 0 to 16 (0: Wrist, 1-4: Thumb, 5-8: Index, 9-12: Middle, 13-16: Ring)
            # Exclude 17-20 (Pinky)
            coords: List[float] = []
            for i in range(NUM_POINTS):
                lm = hand_landmarks.landmark[i]
                coords.extend([float(lm.x), float(lm.y), float(lm.z)])

            # Publish ROS 2 message
            msg = Float32MultiArray()
            msg.data = coords
            self._pub.publish(msg)
            self._seq += 1

            if self.gui:
                # Visual overlay: draw MediaPipe landmarks
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

                # Highlight excluded pinky vs included fingers on screen
                for i in range(NUM_POINTS):
                    lm = hand_landmarks.landmark[i]
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

                for i in range(17, 21):
                    lm = hand_landmarks.landmark[i]
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)  # Red indicates excluded pinky

            if self._seq % 30 == 0:
                self.get_logger().info(
                    f"Published {self._seq} frames | Points: {len(coords)//3} (51 floats) | FPS: {self._current_fps:.1f}"
                )

        if self.gui:
            # Status HUD
            status_color = (0, 255, 0) if hand_detected else (0, 0, 255)
            status_text = f"Hand: {'TRACKING' if hand_detected else 'SEARCHING'} | Points: {NUM_POINTS} (51 floats)"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
            cv2.putText(
                frame,
                f"FPS: {self._current_fps:.1f} | Frames Pub: {self._seq}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                frame,
                "Green: Used (0-16: Thumb, Index, Mid, Ring) | Red: Excluded Pinky (17-20)",
                (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (200, 200, 200),
                1,
            )

            # Display window
            cv2.imshow("Allegro Hand Vision Tracker (MediaPipe)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:  # 'q' or ESC
                self.get_logger().info("Quit requested by user via GUI.")
                return False

        return True

    def cleanup(self) -> None:
        self.get_logger().info("Cleaning up vision tracker resources...")
        if self.cap.isOpened():
            self.cap.release()
        if self.gui:
            cv2.destroyAllWindows()
        self.hands.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand Vision Tracker Node")
    parser.add_argument("--device", type=int, default=0, help="Webcam device ID (default: 0 for /dev/video0)")
    parser.add_argument("--no-gui", action="store_true", help="Run without OpenCV imshow GUI window (headless mode)")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = VisionTrackerNode(device_id=args.device, gui=not args.no_gui)
    except Exception as e:
        print(f"[ERROR] Failed to start VisionTrackerNode: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        while rclpy.ok():
            if not node.step():
                break
            rclpy.spin_once(node, timeout_sec=0.001)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
