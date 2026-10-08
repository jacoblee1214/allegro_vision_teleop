#!/usr/bin/env python3
"""
retargeting_node.py — Advanced Kinematic Retargeting Node for Allegro Hand V4.

Features:
- Subscribes to `/allegro/vision/landmarks` (51-dim Float32MultiArray: 17 landmarks x 3D).
- Subscribes to `/allegro/teleop_state` (JSON string from vision_tracker).
- Clutch Logic:
    * When Clutch is 'ENGAGED', pauses calculation from vision landmarks.
    * Re-publishes the last commanded target joints to keep the robot firmly in place (Hold state).
- Enhanced Adaptive Retargeting (ported and adapted from v6_adaptive_ / v6_retargeting):
    1. Adaptive Pinch Assistance:
       Monitors 3D distance between thumb tip and index/middle tips.
       Dynamically blends pinch compensation weights (alpha in [0, 1]) when distance < d2 (6.5cm).
       Boosts MCP/PIP/DIP flexion and aligns thumb opposition to guarantee positive fingertip contact.
    2. DIP-PIP Anatomical Coupling:
       Enforces natural human finger kinematic coupling between PIP and DIP joints (w_couple = 0.25).
       Prevents DIP hyperextension or erratic tip twitching.
    3. Segment Scaling & Joint Calibration:
       Thumb base angle offset and individual finger gain tuning for Allegro Hand V4 kinematics.
    4. Exponential Moving Average (EMA) Low-Pass Filter:
       Applies 1st-order IIR smoothing (alpha = 0.40) to eliminate camera jitter while retaining responsiveness.
- Publishes 16-dim Float64MultiArray to `/allegro/target_joints` in CONTROLLER_JOINT_ORDER [Thumb, Index, Middle, Ring].

Usage:
    python3 retargeting_node.py [--alpha 0.40] [--pinch-assist] [--quiet]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

# Dynamically import safety_utils from this script's directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_utils import CONTROLLER_JOINT_ORDER, JOINT_LIMITS, N_JOINTS

INPUT_TOPIC = "/allegro/vision/landmarks"
STATE_TOPIC = "/allegro/teleop_state"
OUTPUT_TOPIC = "/allegro/target_joints"

# Landmark indices (MediaPipe 17-point subset)
# 0: Wrist
# 1~4: Thumb (1: CMC, 2: MCP, 3: IP, 4: TIP)
# 5~8: Index (5: MCP, 6: PIP, 7: DIP, 8: TIP)
# 9~12: Middle (9: MCP, 10: PIP, 11: DIP, 12: TIP)
# 13~16: Ring (13: MCP, 14: PIP, 15: DIP, 16: TIP)
WRIST = 0
THUMB_INDICES = (1, 2, 3, 4)
INDEX_INDICES = (5, 6, 7, 8)
MIDDLE_INDICES = (9, 10, 11, 12)
RING_INDICES = (13, 14, 15, 16)

QOS_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Calculates angle in radians between two 3D vectors."""
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    if norm1 < 1e-6 or norm2 < 1e-6:
        return 0.0
    cos_theta = np.dot(v1, v2) / (norm1 * norm2)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return float(np.arccos(cos_theta))


def signed_abduction_angle(v_bone: np.ndarray, v_ref: np.ndarray, normal: np.ndarray) -> float:
    """Calculates signed angle of v_bone relative to v_ref around the palm normal axis."""
    normal_unit = normal / (np.linalg.norm(normal) + 1e-8)
    v_bone_proj = v_bone - np.dot(v_bone, normal_unit) * normal_unit
    v_ref_proj = v_ref - np.dot(v_ref, normal_unit) * normal_unit

    norm_b = np.linalg.norm(v_bone_proj)
    norm_r = np.linalg.norm(v_ref_proj)
    if norm_b < 1e-6 or norm_r < 1e-6:
        return 0.0

    cos_ang = np.clip(np.dot(v_bone_proj, v_ref_proj) / (norm_b * norm_r), -1.0, 1.0)
    angle = np.arccos(cos_ang)

    cross_prod = np.cross(v_ref_proj, v_bone_proj)
    if np.dot(cross_prod, normal_unit) < 0:
        angle = -angle

    return float(angle)


class KinematicRetargetingNode(Node):
    def __init__(
        self,
        verbose: bool = True,
        lp_alpha: float = 0.40,
        enable_pinch_assist: bool = True,
    ) -> None:
        super().__init__("kinematic_retargeting_node")
        self.verbose = verbose
        self.lp_alpha = lp_alpha
        self.enable_pinch_assist = enable_pinch_assist

        # State tracking
        self._clutch_engaged: bool = False
        self._last_target_angles: Optional[np.ndarray] = None
        self._filtered_q: Optional[np.ndarray] = None
        self._seq = 0
        self._last_log_time = time.monotonic()

        # Adaptive Pinch parameters (inspired by v6_adaptive_retarget.yaml)
        self.pinch_d1 = 0.025  # 2.5cm: full pinch boost (alpha = 1.0)
        self.pinch_d2 = 0.065  # 6.5cm: start of pinch assist transition (alpha = 0.0)
        self.w_couple = 0.25   # Anatomical DIP-PIP coupling weight
        self.couple_ratio = 0.85

        # Subscriber: Vision Landmarks
        self._sub_landmarks = self.create_subscription(
            Float32MultiArray,
            INPUT_TOPIC,
            self._landmarks_callback,
            QOS_RELIABLE,
        )

        # Subscriber: Teleop Operator State (Clutch, Record, Tag)
        self._sub_state = self.create_subscription(
            String,
            STATE_TOPIC,
            self._teleop_state_callback,
            QOS_RELIABLE,
        )

        # Publisher: Target Joint Angles
        self._pub_joints = self.create_publisher(
            Float64MultiArray,
            OUTPUT_TOPIC,
            QOS_RELIABLE,
        )

        self.get_logger().info("Advanced Kinematic Retargeting Node initialized.")
        self.get_logger().info(f"Subscribed to landmarks -> '{INPUT_TOPIC}'")
        self.get_logger().info(f"Subscribed to teleop state -> '{STATE_TOPIC}'")
        self.get_logger().info(f"Publishing target joint angles ({N_JOINTS} joints) -> '{OUTPUT_TOPIC}'")
        self.get_logger().info(f"Features: Clutch Control, Adaptive Pinch Assist={enable_pinch_assist}, EMA Filter alpha={lp_alpha:.2f}")

    def _teleop_state_callback(self, msg: String) -> None:
        """Parses teleop operator status and toggles clutch hold behavior."""
        try:
            state = json.loads(msg.data)
            new_clutch = state.get("clutch", False)
            if new_clutch != self._clutch_engaged:
                self._clutch_engaged = new_clutch
                if self._clutch_engaged:
                    self.get_logger().warn("[CLUTCH] ⏸ ENGAGED: Freezing joint angles. Operator hand repositioning active.")
                else:
                    self.get_logger().info("[CLUTCH] ▶ RELEASED: Resuming real-time vision tracking.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse teleop_state message: {e}")

    def _compute_finger_joints(
        self,
        pts: np.ndarray,
        indices: Tuple[int, int, int, int],
        prefix: str,
        palm_forward: np.ndarray,
        palm_normal: np.ndarray,
        pinch_alpha: float = 0.0,
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles (abduction, MCP, PIP, DIP) for a finger (Index, Middle, Ring).
        Incorporates adaptive pinch flexion boost and DIP-PIP anatomical coupling.
        """
        mcp_idx, pip_idx, dip_idx, tip_idx = indices

        # Bone vectors
        v_meta = pts[mcp_idx] - pts[WRIST]      # Wrist -> MCP
        v_prox = pts[pip_idx] - pts[mcp_idx]    # MCP -> PIP
        v_inter = pts[dip_idx] - pts[pip_idx]   # PIP -> DIP
        v_dist = pts[tip_idx] - pts[dip_idx]    # DIP -> TIP

        # Joint 0: Abduction/Adduction
        if prefix == "ah_joint2":  # Middle finger is reference axis
            q0 = 0.0
        else:
            q0 = signed_abduction_angle(v_prox, palm_forward, palm_normal)
            if prefix == "ah_joint1":
                q0 = q0 * 0.80
            elif prefix == "ah_joint3":
                q0 = q0 * 0.80

        # Joint 1: MCP Flexion
        q1 = angle_between(v_meta, v_prox)

        # Joint 2: PIP Flexion
        q2 = angle_between(v_prox, v_inter)

        # Joint 3: DIP Flexion (raw)
        q3_raw = angle_between(v_inter, v_dist)

        # Apply DIP-PIP anatomical coupling: q3 blends between raw DIP and coupled PIP
        q3_coupled = (1.0 - self.w_couple) * q3_raw + self.w_couple * (q2 * self.couple_ratio)

        # Segment scaling from v6_tuning & v4 calibration
        q1_scaled = q1 * 1.15
        q2_scaled = q2 * 1.15
        q3_scaled = q3_coupled * 1.20

        # Adaptive pinch boost for Index finger
        if prefix == "ah_joint1" and pinch_alpha > 0.0:
            boost_factor = 1.0 + 0.35 * pinch_alpha
            q1_scaled *= boost_factor
            q2_scaled *= boost_factor
            q3_scaled *= boost_factor

        return {
            f"{prefix}0": q0,
            f"{prefix}1": q1_scaled,
            f"{prefix}2": q2_scaled,
            f"{prefix}3": q3_scaled,
        }

    def _compute_thumb_joints(
        self,
        pts: np.ndarray,
        palm_forward: np.ndarray,
        palm_normal: np.ndarray,
        pinch_alpha: float = 0.0,
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles for the Thumb (ah_joint00~03).
        Incorporates opposition calibration and adaptive pinch convergence.
        """
        cmc_idx, mcp_idx, ip_idx, tip_idx = THUMB_INDICES

        v_wrist_cmc = pts[cmc_idx] - pts[WRIST]
        v_cmc_mcp = pts[mcp_idx] - pts[cmc_idx]
        v_mcp_ip = pts[ip_idx] - pts[mcp_idx]
        v_ip_tip = pts[tip_idx] - pts[ip_idx]

        # Joint 00: Opposition / Rotation (towards palm center)
        opp_angle = angle_between(v_cmc_mcp, palm_normal)
        q00 = np.interp(opp_angle, [1.0, 2.2], [0.35, 1.25])

        # Joint 01: MCP Abduction / Elevation
        elev_angle = angle_between(v_cmc_mcp, palm_forward)
        q01 = np.interp(elev_angle, [0.3, 1.8], [-0.05, 0.95])

        # Joint 02: MCP Flexion
        q02 = angle_between(v_cmc_mcp, v_mcp_ip) * 1.20

        # Joint 03: IP Flexion
        q03 = angle_between(v_mcp_ip, v_ip_tip) * 1.25

        # Adaptive pinch adjustment: rotate thumb towards index tip when pinching
        if pinch_alpha > 0.0:
            q00 += 0.15 * pinch_alpha
            q02 += 0.18 * pinch_alpha
            q03 += 0.15 * pinch_alpha

        return {
            "ah_joint00": float(q00),
            "ah_joint01": float(q01),
            "ah_joint02": float(q02),
            "ah_joint03": float(q03),
        }

    def retarget_landmarks(self, landmarks_flat: List[float]) -> np.ndarray:
        """
        Converts 51-dim landmark array into 16-dim target joint angles with adaptive enhancements.
        """
        pts = np.array(landmarks_flat, dtype=np.float64).reshape((17, 3))

        # Palm coordinate system
        palm_forward = pts[9] - pts[WRIST]
        palm_transverse = pts[13] - pts[5]
        palm_normal = np.cross(palm_transverse, palm_forward)

        # 1. Compute Adaptive Pinch Alpha
        pinch_alpha = 0.0
        if self.enable_pinch_assist:
            # Distance between thumb tip (4) and index tip (8)
            dist_index = float(np.linalg.norm(pts[4] - pts[8]))
            if dist_index < self.pinch_d2:
                pinch_alpha = float(np.clip((self.pinch_d2 - dist_index) / (self.pinch_d2 - self.pinch_d1), 0.0, 1.0))

        # 2. Compute Joint Angles
        q_dict: Dict[str, float] = {}
        q_dict.update(self._compute_thumb_joints(pts, palm_forward, palm_normal, pinch_alpha=pinch_alpha))
        q_dict.update(self._compute_finger_joints(pts, INDEX_INDICES, "ah_joint1", palm_forward, palm_normal, pinch_alpha=pinch_alpha))
        q_dict.update(self._compute_finger_joints(pts, MIDDLE_INDICES, "ah_joint2", palm_forward, palm_normal, pinch_alpha=0.0))
        q_dict.update(self._compute_finger_joints(pts, RING_INDICES, "ah_joint3", palm_forward, palm_normal, pinch_alpha=0.0))

        # 3. Construct target array matching CONTROLLER_JOINT_ORDER and clamp limits
        target_angles = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            raw_val = q_dict[joint_name]
            min_lim, max_lim = JOINT_LIMITS[joint_name]
            target_angles[i] = np.clip(raw_val, min_lim, max_lim)

        # 4. Apply Exponential Moving Average (EMA) Low-Pass Filter
        if self._filtered_q is None:
            self._filtered_q = target_angles.copy()
        else:
            self._filtered_q = self.lp_alpha * target_angles + (1.0 - self.lp_alpha) * self._filtered_q

        return self._filtered_q.copy()

    def _landmarks_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 51:
            self.get_logger().warn(
                f"Received landmark array with unexpected length {len(msg.data)} (expected 51). Ignoring."
            )
            return

        # Clutch Check:
        # If Clutch is ENGAGED, ignore new landmarks and re-publish last held joint positions
        if self._clutch_engaged:
            if self._last_target_angles is not None:
                out_msg = Float64MultiArray()
                out_msg.data = self._last_target_angles.tolist()
                self._pub_joints.publish(out_msg)

                now = time.monotonic()
                if self.verbose and (now - self._last_log_time >= 1.0):
                    self._last_log_time = now
                    self.get_logger().info("[CLUTCH ENGAGED] ⏸ Target joints held at last position.")
            return

        # Calculate new target joint angles
        target_angles = self.retarget_landmarks(list(msg.data))
        self._last_target_angles = target_angles.copy()

        # Publish target joints
        out_msg = Float64MultiArray()
        out_msg.data = target_angles.tolist()
        self._pub_joints.publish(out_msg)
        self._seq += 1

        # Console logging
        now = time.monotonic()
        if self.verbose and (now - self._last_log_time >= 0.5 or self._seq == 1):
            self._last_log_time = now
            th = target_angles[0:4]
            ix = target_angles[4:8]
            md = target_angles[8:12]
            rg = target_angles[12:16]
            self.get_logger().info(
                f"\n[Frame #{self._seq}] Target Joint Angles (rad):\n"
                f"  Thumb  (ah_joint00~03): [{th[0]:+.3f}, {th[1]:+.3f}, {th[2]:+.3f}, {th[3]:+.3f}]\n"
                f"  Index  (ah_joint10~13): [{ix[0]:+.3f}, {ix[1]:+.3f}, {ix[2]:+.3f}, {ix[3]:+.3f}]\n"
                f"  Middle (ah_joint20~23): [{md[0]:+.3f}, {md[1]:+.3f}, {md[2]:+.3f}, {md[3]:+.3f}]\n"
                f"  Ring   (ah_joint30~33): [{rg[0]:+.3f}, {rg[1]:+.3f}, {rg[2]:+.3f}, {rg[3]:+.3f}]"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand V4 Advanced Kinematic Retargeting Node")
    parser.add_argument("--alpha", type=float, default=0.40, help="EMA low-pass filter coefficient (default: 0.40)")
    parser.add_argument("--no-pinch-assist", action="store_true", help="Disable adaptive pinch assistance")
    parser.add_argument("--quiet", action="store_true", help="Suppress periodic terminal joint logs")
    args = parser.parse_args()

    rclpy.init()
    node = KinematicRetargetingNode(
        verbose=not args.quiet,
        lp_alpha=args.alpha,
        enable_pinch_assist=not args.no_pinch_assist,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down retargeting node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
