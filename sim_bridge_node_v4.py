#!/usr/bin/env python3
"""
sim_bridge_node.py — Controller Bridge Node with Low-Pass EMA Filter for Allegro Hand V4.

Subscribes to `/allegro/target_joints` (16-dim Float64MultiArray from retargeting_node),
applies an Exponential Moving Average (EMA) low-pass filter to smooth commands and protect
the physical hardware motors from noise / sudden jerks, and publishes to
`/allegro_hand_position_controller/commands`.

EMA Formula:
    Filtered_Angle = (alpha * New_Angle) + ((1 - alpha) * Previous_Filtered_Angle)

Usage (inside container):
    source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash
    python3 /home/humble_ws/allegro_vision_teleop/sim_bridge_node.py [--alpha 0.15]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64MultiArray

# Dynamically import safety_utils_v4 from this script's directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_utils_v4 import CONTROLLER_JOINT_ORDER, JOINT_LIMITS, N_JOINTS

INPUT_TOPIC = "/allegro/target_joints"
OUTPUT_TOPIC = "/allegro_hand_position_controller/commands"
DEFAULT_ALPHA = 0.15

BRIDGE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class SimBridgeNode(Node):
    def __init__(
        self,
        input_topic: str = INPUT_TOPIC,
        output_topic: str = OUTPUT_TOPIC,
        alpha: float = DEFAULT_ALPHA,
    ) -> None:
        super().__init__("sim_bridge_node")
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.alpha = float(np.clip(alpha, 0.01, 1.0))

        self._seq = 0
        self._last_log_time = time.monotonic()
        self._filtered_angles: Optional[np.ndarray] = None

        # Subscriber: Target Joints from Retargeting Node
        self._sub = self.create_subscription(
            Float64MultiArray,
            self.input_topic,
            self._target_joints_callback,
            BRIDGE_QOS,
        )

        # Publisher: Command to ros2_control Position Controller
        self._pub = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            BRIDGE_QOS,
        )

        self.get_logger().info(
            f"Bridge initialized: '{self.input_topic}' → '{self.output_topic}' (EMA alpha={self.alpha:.2f})"
        )
        self.get_logger().info(
            f"Expected joints count: {N_JOINTS} ({CONTROLLER_JOINT_ORDER[0]} ~ {CONTROLLER_JOINT_ORDER[-1]})"
        )

    def _target_joints_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != N_JOINTS:
            self.get_logger().warn(
                f"Received joint command with length {len(msg.data)} (expected {N_JOINTS}). Dropping frame."
            )
            return

        raw_data = np.array(msg.data, dtype=np.float64)

        # 1. Clamp raw target values within joint limits
        clamped_data = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            lo, hi = JOINT_LIMITS[joint_name]
            clamped_data[i] = np.clip(raw_data[i], lo, hi)

        # 2. Apply Exponential Moving Average (EMA) Low-Pass Filter
        # Formula: Filtered_Angle = (alpha * New_Angle) + ((1 - alpha) * Previous_Filtered_Angle)
        if self._filtered_angles is None:
            self._filtered_angles = clamped_data.copy()
        else:
            self._filtered_angles = (self.alpha * clamped_data) + ((1.0 - self.alpha) * self._filtered_angles)

        # Re-clamp filtered result to guarantee absolute limit compliance
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            lo, hi = JOINT_LIMITS[joint_name]
            self._filtered_angles[i] = np.clip(self._filtered_angles[i], lo, hi)

        # 3. Publish smoothed command to hardware position controller
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self._filtered_angles.tolist()
        self._pub.publish(cmd_msg)
        self._seq += 1

        now = time.monotonic()
        if now - self._last_log_time >= 0.5 or self._seq == 1:
            self._last_log_time = now
            th = self._filtered_angles[0:4]
            ix = self._filtered_angles[4:8]
            md = self._filtered_angles[8:12]
            rg = self._filtered_angles[12:16]
            self.get_logger().info(
                f"[EMA Filtered #{self._seq} (alpha={self.alpha:.2f})] → {self.output_topic}\n"
                f"  Thumb : [{th[0]:+.3f}, {th[1]:+.3f}, {th[2]:+.3f}, {th[3]:+.3f}]\n"
                f"  Index : [{ix[0]:+.3f}, {ix[1]:+.3f}, {ix[2]:+.3f}, {ix[3]:+.3f}]\n"
                f"  Middle: [{md[0]:+.3f}, {md[1]:+.3f}, {md[2]:+.3f}, {md[3]:+.3f}]\n"
                f"  Ring  : [{rg[0]:+.3f}, {rg[1]:+.3f}, {rg[2]:+.3f}, {rg[3]:+.3f}]"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand Controller Bridge Node with EMA Filter")
    parser.add_argument("--input", type=str, default=INPUT_TOPIC, help="Input target joints topic")
    parser.add_argument("--output", type=str, default=OUTPUT_TOPIC, help="Output controller command topic")
    parser.add_argument("--alpha", type=float, default=DEFAULT_ALPHA, help="EMA smoothing factor (0.0~1.0, default: 0.15)")
    args = parser.parse_args()

    rclpy.init()
    node = SimBridgeNode(input_topic=args.input, output_topic=args.output, alpha=args.alpha)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down bridge node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
