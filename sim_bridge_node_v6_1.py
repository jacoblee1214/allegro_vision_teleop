#!/usr/bin/env python3
"""
sim_bridge_node_v6_1.py — Controller Bridge Node with Low-Pass EMA Filter for 5-Finger Robot Hand (Allegro Hand V6).

Subscribes to `/allegro/target_joints` (20-dim Float64MultiArray from retargeting_node),
applies an Exponential Moving Average (EMA) low-pass filter to smooth commands and protect
the physical hardware motors from noise / sudden jerks, and publishes to
`/allegro_hand_position_controller/commands`.

v6_1: This node is the single place that converts between URDF and motor coordinates.
  - Command path  (URDF -> motor): real + left hand, MCP joint11/21/31/41 -= pi/2
  - Feedback path (motor -> URDF): `/joint_states` -> `/allegro/joint_states_urdf` with the inverse offset.
    robot_state_publisher (RViz), cockpit/dashboard 3D views and the dataset recorder all read this topic.

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
import json
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

# Dynamically import safety_utils_v6 from this script's directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_utils_v6 import CONTROLLER_JOINT_ORDER, JOINT_LIMITS, N_JOINTS

INPUT_TOPIC = "/allegro/target_joints"
OUTPUT_TOPIC = "/allegro_hand_position_controller/commands"
STATE_TOPIC = "/allegro/teleop_state"  # same topic the cockpit/dashboard/retargeting use
JOINT_STATES_TOPIC = "/joint_states"
JOINT_STATES_URDF_TOPIC = "/allegro/joint_states_urdf"
DEFAULT_ALPHA = 0.25
DEFAULT_RATE = 100.0  # Hz (matches controller_manager 100Hz update_rate)



# Left hand physical hardware MCP motor offset (-90 deg) intentionally configured by developer
LEFT_MCP_JOINT_INDICES: tuple[int, ...] = (5, 9, 13, 17)  # joint11, joint21, joint31, joint41
LEFT_MCP_JOINT_NAMES: tuple[str, ...] = ("joint11", "joint21", "joint31", "joint41")
MOTOR_OFFSET_90DEG: float = 1.5707963267948966  # 90 degrees in radians


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
        rate: float = DEFAULT_RATE,
        mode: str = "real",
        hand_side: str = "right",
    ) -> None:
        super().__init__("sim_bridge_node")
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.alpha = float(np.clip(alpha, 0.01, 1.0))
        self.rate = max(10.0, float(rate))
        self.mode = mode.lower()
        self.hand_side = hand_side.lower()

        self._seq = 0
        self._last_log_time = time.monotonic()
        # Immediately initialize to default zero position (손가락 쫙 펴진 상태) from tick 0
        self._latest_target: np.ndarray = np.zeros(N_JOINTS, dtype=np.float64)
        self._filtered_angles: np.ndarray = np.zeros(N_JOINTS, dtype=np.float64)

        self._tick_counter = 0
        self._last_rate_time = time.monotonic()
        self._actual_rate = 0.0

        # Subscriber: Target Joints from Retargeting Node (Standard URDF coordinates)
        self._sub = self.create_subscription(
            Float64MultiArray,
            self.input_topic,
            self._target_joints_callback,
            BRIDGE_QOS,
        )

        # Subscriber: Teleop Operator State (Dynamic Hand Model Switch from UI)
        self._sub_state = self.create_subscription(
            String,
            STATE_TOPIC,
            self._teleop_state_callback,
            BRIDGE_QOS,
        )

        # Publisher: Command to ros2_control Position Controller
        self._pub = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            BRIDGE_QOS,
        )

        # Feedback relay: hardware /joint_states (motor coordinates) -> URDF coordinates
        self._pub_js_urdf = self.create_publisher(JointState, JOINT_STATES_URDF_TOPIC, BRIDGE_QOS)
        self._sub_js = self.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_states_callback, BRIDGE_QOS)

        # High-frequency timer loop ensuring constant command frequency to hardware
        timer_period = 1.0 / self.rate
        self._timer = self.create_timer(timer_period, self._control_loop_tick)

        self.get_logger().info(
            f"Bridge initialized: '{self.input_topic}' → '{self.output_topic}'"
            f" | Mode: {self.mode.upper()} | Hand Side: {self.hand_side.upper()}"
            f" | Command Rate: {self.rate:.1f} Hz (Period: {timer_period*1000:.1f}ms) | EMA alpha={self.alpha:.2f}"
        )

        self.get_logger().info(
            f"Expected joints count: {N_JOINTS} ({CONTROLLER_JOINT_ORDER[0]} ~ {CONTROLLER_JOINT_ORDER[-1]})"
        )

    def _teleop_state_callback(self, msg: String) -> None:
        """Handles dynamic hand model switching [H] from Dashboard / Cockpit UI."""
        try:
            state = json.loads(msg.data)
            new_hand = state.get("hand_side")
            if new_hand and new_hand in ("left", "right") and new_hand != self.hand_side:
                if self.mode == "real":
                    # Physical hand is fixed; never drop/apply the left MCP motor offset at runtime
                    return
                self.hand_side = new_hand
                self._filtered_angles = None  # Reset filter history on hand switch
                self.get_logger().info(
                    f"[UI HAND SWITCH] 🔄 Active Hand Model switched to: {self.hand_side.upper()} HAND in bridge"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to parse teleop_state message in bridge: {e}")

    def _joint_states_callback(self, msg: JointState) -> None:
        """Republish /joint_states in URDF coordinates (inverse of the command-path motor offset)."""
        if self.mode == "real" and self.hand_side == "left":
            positions = list(msg.position)
            for i, name in enumerate(msg.name):
                if i < len(positions) and name.removeprefix("ah_") in LEFT_MCP_JOINT_NAMES:
                    positions[i] += MOTOR_OFFSET_90DEG
            msg.position = positions
        self._pub_js_urdf.publish(msg)

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

        self._latest_target = clamped_data

    def _control_loop_tick(self) -> None:
        """High-frequency control loop running at self.rate (e.g. 60Hz / 100Hz)."""
        if self._latest_target is None:
            return

        # Measure actual loop rate
        self._tick_counter += 1
        now = time.monotonic()
        if now - self._last_rate_time >= 1.0:
            self._actual_rate = self._tick_counter / (now - self._last_rate_time)
            self._tick_counter = 0
            self._last_rate_time = now

        # 2. Apply Exponential Moving Average (EMA) Low-Pass Filter
        # Formula: Filtered_Angle = (alpha * New_Angle) + ((1 - alpha) * Previous_Filtered_Angle)
        if self._filtered_angles is None:
            self._filtered_angles = self._latest_target.copy()
        else:
            self._filtered_angles = (self.alpha * self._latest_target) + ((1.0 - self.alpha) * self._filtered_angles)

        # Re-clamp filtered result to guarantee absolute limit compliance
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            lo, hi = JOINT_LIMITS[joint_name]
            self._filtered_angles[i] = np.clip(self._filtered_angles[i], lo, hi)

        # 3. Target Command Vector
        cmd_data = self._filtered_angles.copy()

        # Physical Hardware Offset:
        # In 'real' mode on Left Hand, physical MCP motors require -90 deg (-1.5708 rad)
        # to physically be straight flat open (as configured by hardware developer).
        # In 'sim' mode, standard 0.0 rad is sent directly so simulation URDF does NOT bend backwards!
        if self.mode == "real" and self.hand_side == "left":
            for idx in LEFT_MCP_JOINT_INDICES:
                cmd_data[idx] -= MOTOR_OFFSET_90DEG

        # Publish command to hardware position controller at constant high frequency
        cmd_msg = Float64MultiArray()
        cmd_msg.data = cmd_data.tolist()
        self._pub.publish(cmd_msg)
        self._seq += 1

        if now - self._last_log_time >= 1.0 or self._seq == 1:
            self._last_log_time = now
            th = self._filtered_angles[0:4]
            ix = self._filtered_angles[4:8]
            md = self._filtered_angles[8:12]
            rg = self._filtered_angles[12:16]
            pk = self._filtered_angles[16:20]
            self.get_logger().info(
                f"[Output Rate: {self._actual_rate:.1f} Hz (Target: {self.rate:.0f} Hz) | EMA alpha={self.alpha:.2f} | Frame #{self._seq} | Mode: {self.mode.upper()}]\n"
                f"  Thumb : [{th[0]:+.3f}, {th[1]:+.3f}, {th[2]:+.3f}, {th[3]:+.3f}]\n"
                f"  Index : [{ix[0]:+.3f}, {ix[1]:+.3f}, {ix[2]:+.3f}, {ix[3]:+.3f}]\n"
                f"  Middle: [{md[0]:+.3f}, {md[1]:+.3f}, {md[2]:+.3f}, {md[3]:+.3f}]\n"
                f"  Ring  : [{rg[0]:+.3f}, {rg[1]:+.3f}, {rg[2]:+.3f}, {rg[3]:+.3f}]\n"
                f"  Pinky : [{pk[0]:+.3f}, {pk[1]:+.3f}, {pk[2]:+.3f}, {pk[3]:+.3f}]"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand Controller Bridge Node with EMA Filter & High-Frequency Loop")
    parser.add_argument("--input", type=str, default=INPUT_TOPIC, help="Input target joints topic")
    parser.add_argument("--output", type=str, default=OUTPUT_TOPIC, help="Output controller command topic")
    parser.add_argument("--alpha", type=float, default=DEFAULT_ALPHA, help="EMA smoothing factor (0.0~1.0, default: 0.25)")
    parser.add_argument("--rate", "--hz", dest="rate", type=float, default=DEFAULT_RATE, help="Command publishing frequency in Hz (default: 100.0, e.g. 60 or 100 for tremor suppression)")
    parser.add_argument("--mode", type=str, default="real", choices=["real", "sim", "nodes"], help="Operation mode (default: real)")
    parser.add_argument("--hand", type=str, default="right", choices=["left", "right"], help="Hand model side (default: right)")
    args = parser.parse_args()

    rclpy.init()
    node = SimBridgeNode(
        input_topic=args.input,
        output_topic=args.output,
        alpha=args.alpha,
        rate=args.rate,
        mode=args.mode,
        hand_side=args.hand,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
