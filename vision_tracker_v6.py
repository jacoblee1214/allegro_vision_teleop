#!/usr/bin/env python3
"""
vision_tracker.py — ROS 2 Vision Publisher Node for 5-Finger Robot Hand (Allegro Hand V6) Retargeting.

Captures webcam stream from /dev/video0, extracts hand landmarks using MediaPipe Hands,
extracts all 21 3D landmarks (0~20: Wrist, Thumb, Index, Middle, Ring, Pinky),
and publishes a 63-dimensional Float32MultiArray to `/allegro/vision/landmarks`.
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

# 21 landmarks: 0 (Wrist), 1-4 (Thumb), 5-8 (Index), 9-12 (Middle), 13-16 (Ring), 17-20 (Pinky)
NUM_POINTS = 21
NUM_COORDS = NUM_POINTS * 3  # 63 floats

VISION_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def find_best_camera_device() -> tuple[int, str]:
    """
    Auto-detect Intel RealSense RGB camera device (/dev/video*) if connected;
    otherwise fallback to default webcam (device 0).
    Returns (device_id, description_string).
    """
    import glob
    import subprocess

    dev_paths = sorted(
        glob.glob("/dev/video*"),
        key=lambda p: int(p.replace("/dev/video", "")) if p.replace("/dev/video", "").isdigit() else 999,
    )

    realsense_candidates = []
    for dev in dev_paths:
        dev_idx_str = dev.replace("/dev/video", "")
        if not dev_idx_str.isdigit():
            continue
        idx = int(dev_idx_str)
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "-d", dev, "--all"],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            ).decode("utf-8", errors="ignore")
            # RealSense RGB camera has YUYV format and color/white-balance controls
            if "RealSense" in out:
                if "YUYV" in out or "white_balance" in out:
                    return idx, f"Intel RealSense RGB Camera (/dev/video{idx})"
                realsense_candidates.append(idx)
        except Exception:
            pass

    if realsense_candidates:
        return realsense_candidates[0], f"Intel RealSense (/dev/video{realsense_candidates[0]})"

    return 0, "Default Webcam (/dev/video0)"


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
        self.window_name = "Allegro Hand Vision Tracker (MediaPipe)"
        self._window_initialized = False

        # ROS 2 Publisher
        self._pub = self.create_publisher(Float32MultiArray, TOPIC_LANDMARKS, VISION_QOS)
        self.get_logger().info(f"Publishing 3D hand landmarks (63-dim) to '{TOPIC_LANDMARKS}'")

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
            self.get_logger().error(f"Failed to open video device {self.dev_desc}")
            raise RuntimeError(f"Cannot open webcam {self.dev_desc}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        self._seq = 0
        self._fps_counter = 0
        self._last_fps_time = time.monotonic()
        self._current_fps = 0.0

        disp_w = int(640 * self.scale)
        disp_h = int(480 * self.scale)
        gui_str = f"GUI Window Active ({disp_w}x{disp_h}, scale={self.scale:.2f}x, resizable)" if self.gui else "Headless Mode (No GUI)"
        self.get_logger().info(
            f"Vision tracker initialized. Camera: {self.dev_desc} (target: 640x480@{self.fps}fps) | {gui_str}"
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

        # Convert BGR to RGB for MediaPipe (at native 640x480 for real-time speed)
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
        hand_landmarks = None
        if results.multi_hand_landmarks:
            hand_detected = True
            hand_landmarks = results.multi_hand_landmarks[0]

            # Extract all 21 landmarks (0: Wrist, 1-4: Thumb, 5-8: Index, 9-12: Middle, 13-16: Ring, 17-20: Pinky)
            coords: List[float] = []
            for i in range(NUM_POINTS):
                lm = hand_landmarks.landmark[i]
                coords.extend([float(lm.x), float(lm.y), float(lm.z)])

            # Publish ROS 2 message
            msg = Float32MultiArray()
            msg.data = coords
            self._pub.publish(msg)
            self._seq += 1

            if self._seq % 30 == 0:
                self.get_logger().info(
                    f"Published {self._seq} frames | Points: {len(coords)//3} (63 floats) | FPS: {self._current_fps:.1f}"
                )

        if self.gui:
            disp_w = int(w * self.scale)
            disp_h = int(h * self.scale)

            # Upscale frame for crisp, large display suitable for video recording
            disp_frame = cv2.resize(frame, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)

            if not self._window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, disp_w, disp_h)
                self._window_initialized = True

            if hand_landmarks is not None:
                # Visual overlay: draw MediaPipe landmarks on upscaled display frame
                self.mp_drawing.draw_landmarks(
                    disp_frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

                # Highlight all 21 landmarks in green (all 5 fingers active) with white border
                point_radius = max(4, int(5 * (self.scale / 1.5)))
                for i in range(NUM_POINTS):
                    lm = hand_landmarks.landmark[i]
                    cx, cy = int(lm.x * disp_w), int(lm.y * disp_h)
                    cv2.circle(disp_frame, (cx, cy), point_radius, (0, 255, 0), -1)
                    cv2.circle(disp_frame, (cx, cy), point_radius + 1, (255, 255, 255), 1)

            # Status HUD
            status_color = (0, 255, 0) if hand_detected else (0, 0, 255)
            status_text = f"Hand: {'TRACKING' if hand_detected else 'SEARCHING'} | {self.dev_desc}"
            font_scale = 0.75 * (self.scale / 1.5)
            cv2.putText(disp_frame, status_text, (15, int(35 * self.scale / 1.5)), cv2.FONT_HERSHEY_SIMPLEX, font_scale, status_color, 2)
            cv2.putText(
                disp_frame,
                f"FPS: {self._current_fps:.1f} | Frames: {self._seq} | Res: {disp_w}x{disp_h}",
                (15, int(70 * self.scale / 1.5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.85,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                disp_frame,
                "Green: Active 5 Fingers (0-20: Thumb, Index, Mid, Ring, Pinky)",
                (15, disp_h - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.65,
                (200, 200, 200),
                1,
            )

            # Display window
            cv2.imshow(self.window_name, disp_frame)
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
    parser.add_argument("--device", type=str, default="auto", help="Webcam device ID (default: 'auto' for RealSense RGB or fallback, or int e.g. 0, 8)")
    parser.add_argument("--scale", type=float, default=1.75, help="GUI window display scale (default: 1.75, e.g. 1120x840)")
    parser.add_argument("--fps", type=int, default=60, help="Camera capture frame rate (default: 60)")
    parser.add_argument("--no-gui", action="store_true", help="Run without OpenCV imshow GUI window (headless mode)")
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
                try:
                    rclpy.spin_once(node, timeout_sec=0.001)
                except Exception:
                    break
    except Exception:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
