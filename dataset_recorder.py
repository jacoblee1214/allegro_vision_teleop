#!/usr/bin/env python3
"""
dataset_recorder.py — VLA Model Dataset Recorder Node for Allegro Hand Teleoperation (V6 5-Finger & V4 4-Finger).

Subscribes to:
  1. `/allegro/teleop_state` (std_msgs/msg/String): Clutch, Record, Tagging operator states
  2. `/joint_states` (sensor_msgs/msg/JointState): Actual hardware joint positions, velocities, efforts
  3. `/allegro/target_joints` (std_msgs/msg/Float64MultiArray): Commanded target joint angles (20-dim V6 or 16-dim V4)
  4. `/allegro/vision/landmarks` (std_msgs/msg/Float32MultiArray): 3D hand keypoints (63-dim for V6, 51-dim for V4)
  5. `/allegro/camera/image_raw` (sensor_msgs/msg/Image, optional): Camera frame stream

Workflow:
  - When Operator presses 'R' in vision_tracker GUI, Record transitions to 'True' -> In-memory buffering begins.
  - While recording, synchronizes actual joint states, target joint commands, MediaPipe 3D coordinates, and clutch states.
  - When Operator toggles 'R' off -> Recording stops and episode is placed into pending buffer.
  - When Operator presses 'S' (Success) -> Dumps the episode to `/home/humble_ws/pinn_hw/results/episodes/episode_<timestamp>_ep<idx>_success.json`.
  - When Operator presses 'F' (Fail) -> Dumps to `episode_<timestamp>_ep<idx>_fail.json` (or discards if --discard-failed).
  - Maintains `manifest.json` indexing all recorded episodes for easy VLA training pipeline loading.

Usage:
    python3 dataset_recorder.py [--sample-hz 60] [--output-dir /home/humble_ws/pinn_hw/results/episodes]
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

# Topic definitions
TOPIC_TELEOP_STATE = "/allegro/teleop_state"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_TARGET_JOINTS = "/allegro/target_joints"
TOPIC_LANDMARKS = "/allegro/vision/landmarks"
TOPIC_CAMERA_IMAGE = "/allegro/camera/image_raw"

DEFAULT_OUTPUT_DIR = "/home/humble_ws/pinn_hw/results/episodes"
FALLBACK_OUTPUT_DIR = "/home/jake/humble_ws/pinn_hw/results/episodes"

QOS_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=20,
)

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

JOINTS_V6_20DOF = [
    "joint00", "joint01", "joint02", "joint03",  # Thumb
    "joint10", "joint11", "joint12", "joint13",  # Index
    "joint20", "joint21", "joint22", "joint23",  # Middle
    "joint30", "joint31", "joint32", "joint33",  # Ring
    "joint40", "joint41", "joint42", "joint43",  # Pinky
]

JOINTS_V4_16DOF = [
    "ah_joint00", "ah_joint01", "ah_joint02", "ah_joint03",  # Thumb
    "ah_joint10", "ah_joint11", "ah_joint12", "ah_joint13",  # Index
    "ah_joint20", "ah_joint21", "ah_joint22", "ah_joint23",  # Middle
    "ah_joint30", "ah_joint31", "ah_joint32", "ah_joint33",  # Ring
]


class DatasetRecorderNode(Node):
    def __init__(
        self,
        output_dir: str = DEFAULT_OUTPUT_DIR,
        sample_hz: float = 60.0,
        keep_failed: bool = True,
        dof_hint: int = 20,
    ) -> None:
        super().__init__("dataset_recorder_node")

        target_path = Path(output_dir)
        if not target_path.parent.exists() and Path(FALLBACK_OUTPUT_DIR).parent.exists():
            target_path = Path(FALLBACK_OUTPUT_DIR)
        target_path.mkdir(parents=True, exist_ok=True)
        self.output_dir = target_path

        self.sample_hz = sample_hz
        self.sample_interval = 1.0 / sample_hz
        self.keep_failed = keep_failed

        # Dynamic model detection (defaults to V6 20-DOF)
        self.dof = 20 if dof_hint == 20 else 16
        self.robot_model = "allegro_hand_v6" if self.dof == 20 else "allegro_hand_v4"
        self.joint_names = list(JOINTS_V6_20DOF if self.dof == 20 else JOINTS_V4_16DOF)

        # Recording State Machine
        self.is_recording = False
        self.episode_counter = 0
        self.episode_buffer: List[Dict[str, Any]] = []
        self.pending_episode: Optional[Dict[str, Any]] = None
        self.record_start_wall: Optional[str] = None
        self.record_start_mono: Optional[float] = None
        self.last_tag_processed: str = "NONE"

        # Cached latest incoming values
        self.latest_clutch: bool = False
        self.latest_actual_positions: List[float] = [0.0] * self.dof
        self.latest_actual_velocities: List[float] = [0.0] * self.dof
        self.latest_actual_efforts: List[float] = [0.0] * self.dof
        self.latest_target_joints: List[float] = [0.0] * self.dof
        self.latest_landmarks: List[float] = [0.0] * (self.dof // 4 * 12 + 3 if self.dof == 16 else 63)
        self.latest_image_stamp: Optional[float] = None

        # Subscribers
        self._sub_state = self.create_subscription(
            String,
            TOPIC_TELEOP_STATE,
            self._teleop_state_callback,
            QOS_RELIABLE,
        )

        self._sub_joint_states = self.create_subscription(
            JointState,
            TOPIC_JOINT_STATES,
            self._joint_states_callback,
            QOS_SENSOR,
        )

        self._sub_target_joints = self.create_subscription(
            Float64MultiArray,
            TOPIC_TARGET_JOINTS,
            self._target_joints_callback,
            QOS_RELIABLE,
        )

        self._sub_landmarks = self.create_subscription(
            Float32MultiArray,
            TOPIC_LANDMARKS,
            self._landmarks_callback,
            QOS_RELIABLE,
        )

        self._sub_image = self.create_subscription(
            Image,
            TOPIC_CAMERA_IMAGE,
            self._image_callback,
            QOS_RELIABLE,
        )

        self._timer = self.create_timer(self.sample_interval, self._sampling_timer_callback)

        self.get_logger().info(f"Dataset Recorder Node initialized (Default: {self.robot_model} {self.dof}-DOF).")
        self.get_logger().info(f"Storage Directory: {self.output_dir.resolve()}")
        self.get_logger().info(f"Sampling Frequency: {self.sample_hz:.1f} Hz")
        self.get_logger().info("Ready for recording. Toggle 'R' in vision_tracker to start recording.")

    def _teleop_state_callback(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse teleop_state JSON: {e}")
            return

        self.latest_clutch = state.get("clutch", False)
        record_flag = state.get("record", False)
        current_tag = state.get("tag", "NONE")
        ep_index = state.get("episode_index", self.episode_counter)

        # Detect Record Start (False -> True)
        if record_flag and not self.is_recording:
            self.is_recording = True
            self.episode_counter = ep_index
            self.episode_buffer = []
            self.pending_episode = None
            self.record_start_wall = datetime.now().isoformat()
            self.record_start_mono = time.monotonic()
            self.last_tag_processed = "NONE"
            self.get_logger().info(f"[RECORDER] 🔴 Recording STARTED (Episode #{self.episode_counter} | {self.robot_model})")

        # Detect Record Stop (True -> False)
        elif not record_flag and self.is_recording:
            self.is_recording = False
            dur = time.monotonic() - (self.record_start_mono or time.monotonic())
            num_frames = len(self.episode_buffer)
            self.get_logger().info(
                f"[RECORDER] ⏹ Recording STOPPED (Episode #{self.episode_counter}). "
                f"Buffered {num_frames} frames ({dur:.2f}s). Awaiting operator tag ('S' / 'F')..."
            )

            self.pending_episode = {
                "episode_index": self.episode_counter,
                "start_wall": self.record_start_wall,
                "end_wall": datetime.now().isoformat(),
                "duration_sec": dur,
                "num_frames": num_frames,
                "fps": (num_frames / dur) if dur > 0 else 0.0,
                "frames": list(self.episode_buffer),
            }

        # Process Tagging ('S' or 'F')
        if self.pending_episode is not None and current_tag != self.last_tag_processed:
            if current_tag in ("SUCCESS", "FAIL"):
                self.last_tag_processed = current_tag
                self._dump_episode(current_tag)

    def _target_joints_callback(self, msg: Float64MultiArray) -> None:
        """Caches commanded target joint angles and auto-detects 20-DOF (V6) vs 16-DOF (V4)."""
        n = len(msg.data)
        if n in (16, 20) and n != self.dof:
            self.dof = n
            self.robot_model = "allegro_hand_v6" if self.dof == 20 else "allegro_hand_v4"
            self.joint_names = list(JOINTS_V6_20DOF if self.dof == 20 else JOINTS_V4_16DOF)
            self.latest_actual_positions = [0.0] * self.dof
            self.latest_actual_velocities = [0.0] * self.dof
            self.latest_actual_efforts = [0.0] * self.dof
            self.get_logger().info(f"[RECORDER] Auto-detected robot model: {self.robot_model} ({self.dof}-DOF)")

        if len(msg.data) == self.dof:
            self.latest_target_joints = [float(v) for v in msg.data]

    def _joint_states_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        reordered_pos = [0.0] * self.dof
        reordered_vel = [0.0] * self.dof
        reordered_eff = [0.0] * self.dof

        for out_idx, jname in enumerate(self.joint_names):
            # Check direct name, ah_ prefix, or strip ah_ prefix
            cand_names = [jname, f"ah_{jname}", jname.removeprefix("ah_")]
            found_idx = None
            for c in cand_names:
                if c in name_to_idx:
                    found_idx = name_to_idx[c]
                    break

            if found_idx is not None:
                reordered_pos[out_idx] = float(msg.position[found_idx])
                if len(msg.velocity) > found_idx:
                    reordered_vel[out_idx] = float(msg.velocity[found_idx])
                if len(msg.effort) > found_idx:
                    reordered_eff[out_idx] = float(msg.effort[found_idx])
            elif out_idx < len(msg.position):
                reordered_pos[out_idx] = float(msg.position[out_idx])
                if len(msg.velocity) > out_idx:
                    reordered_vel[out_idx] = float(msg.velocity[out_idx])
                if len(msg.effort) > out_idx:
                    reordered_eff[out_idx] = float(msg.effort[out_idx])

        self.latest_actual_positions = reordered_pos
        self.latest_actual_velocities = reordered_vel
        self.latest_actual_efforts = reordered_eff

    def _landmarks_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) in (51, 63):
            self.latest_landmarks = [float(v) for v in msg.data]

    def _image_callback(self, msg: Image) -> None:
        self.latest_image_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _sampling_timer_callback(self) -> None:
        if not self.is_recording or self.record_start_mono is None:
            return

        now_mono = time.monotonic()
        now_wall = time.time()
        t_rel = now_mono - self.record_start_mono

        frame_data = {
            "step": len(self.episode_buffer),
            "timestamp": now_wall,
            "t_rel": round(t_rel, 4),
            "clutch": self.latest_clutch,
            "actual_positions": [round(x, 5) for x in self.latest_actual_positions],
            "actual_velocities": [round(x, 5) for x in self.latest_actual_velocities],
            "actual_efforts": [round(x, 5) for x in self.latest_actual_efforts],
            "target_positions": [round(x, 5) for x in self.latest_target_joints],
            "landmarks_3d": [round(x, 5) for x in self.latest_landmarks],
            "image_timestamp": self.latest_image_stamp,
        }
        self.episode_buffer.append(frame_data)

    def _dump_episode(self, tag: str) -> None:
        if not self.pending_episode:
            return

        ep_info = self.pending_episode
        if tag == "FAIL" and not self.keep_failed:
            self.get_logger().info(f"[RECORDER] 🗑 Discarded FAIL episode #{ep_info['episode_index']}.")
            self.pending_episode = None
            return

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag_lower = tag.lower()
        filename = f"episode_{ts_str}_ep{ep_info['episode_index']:03d}_{tag_lower}.json"
        filepath = self.output_dir / filename

        clutch_engaged_count = sum(1 for f in ep_info["frames"] if f.get("clutch", False))

        episode_payload = {
            "metadata": {
                "episode_id": f"ep_{ts_str}_{ep_info['episode_index']:03d}",
                "tag": tag,
                "robot": self.robot_model,
                "dof": self.dof,
                "joint_order": list(self.joint_names),
                "num_landmarks": len(self.latest_landmarks) // 3,
                "sample_hz": self.sample_hz,
                "start_time": ep_info["start_wall"],
                "end_time": ep_info["end_wall"],
                "duration_sec": round(ep_info["duration_sec"], 3),
                "num_frames": ep_info["num_frames"],
                "average_fps": round(ep_info["fps"], 2),
                "clutch_frames": clutch_engaged_count,
            },
            "frames": ep_info["frames"],
        }

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(episode_payload, f, indent=2)

            self.get_logger().info(
                f"[RECORDER] 💾 Episode successfully saved to: {filepath}\n"
                f"            [Robot: {self.robot_model} ({self.dof}-DOF) | Tag: {tag} | Frames: {ep_info['num_frames']} | Duration: {ep_info['duration_sec']:.2f}s | FPS: {ep_info['fps']:.1f}]"
            )
            self._update_manifest(filepath, episode_payload["metadata"])
        except Exception as e:
            self.get_logger().error(f"Failed to dump episode JSON: {e}")

        self.pending_episode = None

    def _update_manifest(self, filepath: Path, meta: Dict[str, Any]) -> None:
        manifest_path = self.output_dir / "episodes_manifest.json"
        manifest_data = {"episodes": []}

        if manifest_path.exists():
            try:
                with open(manifest_path, "r", encoding="utf-8") as f:
                    manifest_data = json.load(f)
            except Exception:
                manifest_data = {"episodes": []}

        manifest_data["episodes"].append({
            "episode_id": meta["episode_id"],
            "filename": filepath.name,
            "robot": meta["robot"],
            "dof": meta["dof"],
            "tag": meta["tag"],
            "duration_sec": meta["duration_sec"],
            "num_frames": meta["num_frames"],
            "fps": meta["average_fps"],
            "timestamp": meta["start_time"],
        })
        manifest_data["total_episodes"] = len(manifest_data["episodes"])
        manifest_data["success_count"] = sum(1 for ep in manifest_data["episodes"] if ep["tag"] == "SUCCESS")
        manifest_data["fail_count"] = sum(1 for ep in manifest_data["episodes"] if ep["tag"] == "FAIL")

        try:
            with open(manifest_path, "w", encoding="utf-8") as f:
                json.dump(manifest_data, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f"Failed to update manifest: {e}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand V6/V4 Dataset Recorder Node")
    parser.add_argument(
        "--output-dir",
        type=str,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Path to store recorded JSON episodes (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument("--sample-hz", type=float, default=60.0, help="Dataset sampling rate in Hz (default: 60.0)")
    parser.add_argument("--dof", type=int, default=20, choices=[16, 20], help="Robot DOF: 20 for V6, 16 for V4 (default: 20)")
    parser.add_argument("--discard-failed", action="store_true", help="Discard episodes tagged as FAIL instead of saving")
    args = parser.parse_args()

    rclpy.init()
    node = DatasetRecorderNode(
        output_dir=args.output_dir,
        sample_hz=args.sample_hz,
        keep_failed=not args.discard_failed,
        dof_hint=args.dof,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down dataset recorder.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
