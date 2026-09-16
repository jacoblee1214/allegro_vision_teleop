#!/usr/bin/env python3
"""
retargeting_node_v6.py — Advanced Kinematic Retargeting Node for 5-Finger Robot Hand (Allegro Hand V6).

Subscribes to:
  - `/allegro/vision/landmarks` (63-dim Float32MultiArray from vision_tracker_v6)
  - `/allegro/teleop_state` (JSON string containing Clutch, Record, Tag from vision_tracker_v6)

Calculates:
  - 20-DOF target joint angles [Thumb(4), Index(4), Middle(4), Ring(4), Pinky(4)]
  - Clutch Control: When Clutch is 'ENGAGED', freezes joint tracking and holds last commanded posture.
  - Adaptive Retargeting & Biomechanics:
      1. Thumb opposition & elevation synergy for grasping and scale-invariant pinch.
      2. DIP-PIP anatomical coupling for Index, Middle, Ring, Pinky (w_couple = 0.25) to prevent DIP flutter/hyperextension.
      3. 1st-order IIR Exponential Moving Average (EMA) low-pass filter (alpha = 0.40) for jitter elimination.

Publishes:
  - `/allegro/target_joints` (20-dim Float64MultiArray in CONTROLLER_JOINT_ORDER)

Usage:
    python3 retargeting_node_v6.py [--alpha 0.40] [--quiet]
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

# Dynamically import safety_utils_v6 from this script's directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_utils_v6 import CONTROLLER_JOINT_ORDER, JOINT_LIMITS, N_JOINTS

INPUT_TOPIC = "/allegro/vision/landmarks"
STATE_TOPIC = "/allegro/teleop_state"
OUTPUT_TOPIC = "/allegro/target_joints"

# Landmark indices (MediaPipe 21-point full hand)
# 0: Wrist
# 1~4: Thumb (1: CMC, 2: MCP, 3: IP, 4: TIP)
# 5~8: Index (5: MCP, 6: PIP, 7: DIP, 8: TIP)
# 9~12: Middle (9: MCP, 10: PIP, 11: DIP, 12: TIP)
# 13~16: Ring (13: MCP, 14: PIP, 15: DIP, 16: TIP)
# 17~20: Pinky (17: MCP, 18: PIP, 19: DIP, 20: TIP)
WRIST = 0
THUMB_INDICES = (1, 2, 3, 4)
INDEX_INDICES = (5, 6, 7, 8)
MIDDLE_INDICES = (9, 10, 11, 12)
RING_INDICES = (13, 14, 15, 16)
PINKY_INDICES = (17, 18, 19, 20)

QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Calculate angle in radians between two 3D vectors v1 and v2."""
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    if norm1 < 1e-6 or norm2 < 1e-6:
        return 0.0
    cos_theta = np.dot(v1, v2) / (norm1 * norm2)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return float(np.arccos(cos_theta))


def signed_abduction_angle(v_bone: np.ndarray, v_ref: np.ndarray, normal: np.ndarray) -> float:
    """Calculate signed angle of v_bone relative to v_ref around the palm normal axis."""
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
    def __init__(self, verbose: bool = True, lp_alpha: float = 0.40, hand_side: str = "left") -> None:
        super().__init__("kinematic_retargeting_node")
        self.verbose = verbose
        self.lp_alpha = lp_alpha
        self.hand_side = hand_side.lower()
        self.w_couple = 0.25
        self.couple_ratio = 0.85

        # Teleop Clutch state & filter history
        self._clutch_engaged: bool = False
        self._last_target_angles: Optional[np.ndarray] = None
        self._filtered_q: Optional[np.ndarray] = None
        self._seq = 0
        self._last_log_time = time.monotonic()

        # Subscriber: Vision Landmarks (63-dim)
        self._sub_lm = self.create_subscription(
            Float32MultiArray,
            INPUT_TOPIC,
            self._landmarks_callback,
            QOS_PROFILE,
        )

        # Subscriber: Teleop Operator State (Clutch, Record, Tag)
        self._sub_state = self.create_subscription(
            String,
            STATE_TOPIC,
            self._teleop_state_callback,
            QOS_PROFILE,
        )

        # Publisher: Target Joints (20-dim)
        self._pub = self.create_publisher(
            Float64MultiArray,
            OUTPUT_TOPIC,
            QOS_PROFILE,
        )

        self.get_logger().info(f"Allegro Hand V6 (5-Finger) Retargeting Node initialized.")
        self.get_logger().info(f"Subscribed to: {INPUT_TOPIC} & {STATE_TOPIC}")
        self.get_logger().info(f"Publishing target joint angles ({N_JOINTS} joints) to: {OUTPUT_TOPIC}")
        self.get_logger().info(f"Clutch Hold Control Active | EMA Filter alpha={self.lp_alpha:.2f}")

    def _teleop_state_callback(self, msg: String) -> None:
        """Parses teleop operator status and toggles clutch hold behavior & hand side."""
        try:
            state = json.loads(msg.data)
            new_clutch = state.get("clutch", False)
            if new_clutch != self._clutch_engaged:
                self._clutch_engaged = new_clutch
                if self._clutch_engaged:
                    self.get_logger().warn("[CLUTCH] ⏸ ENGAGED: Freezing 20-DOF robot hand posture (Hold).")
                else:
                    self.get_logger().info("[CLUTCH] ▶ RELEASED: Resuming live tracking.")

            # Dynamic hand model switching from UI
            new_hand = state.get("hand_side")
            if new_hand and new_hand in ("left", "right") and new_hand != self.hand_side:
                self.hand_side = new_hand
                self.get_logger().info(f"[UI HAND SWITCH] 🔄 Active Hand Model dynamically switched to: {self.hand_side.upper()} HAND")
        except Exception as e:
            self.get_logger().error(f"Failed to parse teleop_state message: {e}")

    def _compute_finger_joints(
        self,
        pts: np.ndarray,
        indices: Tuple[int, int, int, int],
        prefix: str,
        palm_forward: np.ndarray,
        palm_normal: np.ndarray,
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles (abduction, MCP, PIP, DIP) for a finger (Index, Middle, Ring, Pinky).
        Applies anatomical DIP-PIP coupling to stabilize fingertip orientation.
        """
        mcp_idx, pip_idx, dip_idx, tip_idx = indices

        # Bone vectors
        v_meta = pts[mcp_idx] - pts[WRIST]      # Wrist -> MCP
        v_prox = pts[pip_idx] - pts[mcp_idx]    # MCP -> PIP
        v_inter = pts[dip_idx] - pts[pip_idx]   # PIP -> DIP
        v_dist = pts[tip_idx] - pts[dip_idx]    # DIP -> TIP

        # Joint 0: Abduction/Adduction
        if prefix in ("joint2", "ah_joint2"):  # Middle finger is reference axis
            q0 = 0.0
        else:
            q0 = signed_abduction_angle(v_prox, palm_forward, palm_normal)
            if prefix in ("joint1", "ah_joint1"):
                q0 = q0 * 0.80
            elif prefix in ("joint3", "ah_joint3"):
                q0 = q0 * 0.80
            elif prefix in ("joint4", "ah_joint4"):
                q0 = q0 * 0.80

        # Joint 1: MCP Flexion
        q1 = angle_between(v_meta, v_prox)

        # Joint 2: PIP Flexion
        q2 = angle_between(v_prox, v_inter)

        # Joint 3: DIP Flexion (blend with coupled PIP)
        q3_raw = angle_between(v_inter, v_dist)
        q3_coupled = (1.0 - self.w_couple) * q3_raw + self.w_couple * (q2 * self.couple_ratio)

        # Apply calibration / sensitivity gains
        q1_scaled = q1 * 1.10
        q2_scaled = q2 * 1.05
        q3_scaled = q3_coupled * 1.10

        base_prefix = prefix.removeprefix("ah_")
        res = {
            f"{base_prefix}0": q0,
            f"{base_prefix}1": q1_scaled,
            f"{base_prefix}2": q2_scaled,
            f"{base_prefix}3": q3_scaled,
        }
        for k in list(res.keys()):
            res[f"ah_{k}"] = res[k]
        return res

    def _compute_thumb_joints(
        self,
        pts: np.ndarray,
        palm_forward: np.ndarray,
        palm_transverse: np.ndarray,
        palm_normal: np.ndarray,
        fingers_flexion: float = 0.0,
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles for the Thumb (joint00~03) using tuned adaptive kinematics:
        - joint00: Base Abduction / Opposition (0.05 rad open ~ 1.40 rad opposed across palm)
        - joint01: Elevation / Inward Swing (+0.12 rad extended ~ -0.75 rad inward towards palm)
        - joint02: MCP Flexion (scaled with v6_adaptive/v6_tuning gains for high fidelity)
        - joint03: IP Flexion (scaled for natural tip curl)
        - Dynamic Pinch Synergy: Active only during pinch gestures without locking free motion.
        - Natural Fist Synergy: Engages only when 4 fingers are deeply clenched (flexion > 0.9 rad).
        """
        cmc_idx, mcp_idx, ip_idx, tip_idx = THUMB_INDICES

        hand_size = np.linalg.norm(palm_forward) + 1e-6
        u_fwd = palm_forward / hand_size
        u_norm = palm_normal / (np.linalg.norm(palm_normal) + 1e-6)
        u_trans = palm_transverse / (np.linalg.norm(palm_transverse) + 1e-6)

        # Thumb bone vectors
        v_thumb_prox = pts[mcp_idx] - pts[cmc_idx]
        v_thumb_mid = pts[ip_idx] - pts[mcp_idx]
        v_thumb_dist = pts[tip_idx] - pts[ip_idx]
        v_thumb_ray = pts[tip_idx] - pts[cmc_idx]
        u_thumb_ray = v_thumb_ray / (np.linalg.norm(v_thumb_ray) + 1e-6)

        # 1. Base Opposition (joint00):
        # Rotation out of the lateral hand plane towards palm normal & index
        proj_norm = np.dot(u_thumb_ray, u_norm)
        proj_lat = np.dot(u_thumb_ray, -u_trans)
        opp_angle = np.arctan2(proj_norm, proj_lat)
        # Full dynamic sweep from spread open to deep opposition
        q00 = float(np.interp(opp_angle, [-0.25, 1.10], [0.05, 1.40]))

        # 2. Base Elevation / Inward Swing (joint01):
        # Outward when open (+0.12 rad), inward across palm when flexed (-0.75 rad)
        q01 = float(np.interp(proj_norm, [-0.15, 0.40], [0.12, -0.75]))

        # 3. Flexion angles (joint02, joint03) with tuned scales from v6_tuning & v6_adaptive
        q02_bone = angle_between(v_thumb_prox, v_thumb_mid)
        q03_bone = angle_between(v_thumb_mid, v_thumb_dist)

        # Sensitivity gains (boosts natural subtle thumb bending)
        q02 = q02_bone * 1.50
        q03 = q03_bone * 2.10

        # 4. Dynamic Pinch Synergy:
        # Distance from thumb tip (4) to index tip (8) and middle tip (12)
        d_pinch_index = np.linalg.norm(pts[4] - pts[8]) / hand_size
        d_pinch_middle = np.linalg.norm(pts[4] - pts[12]) / hand_size
        d_pinch = min(d_pinch_index, d_pinch_middle)
        pinch_factor = float(np.clip(1.0 - (d_pinch - 0.10) / (0.35 - 0.10), 0.0, 1.0))

        if pinch_factor > 0:
            q00 = (1.0 - pinch_factor * 0.70) * q00 + (pinch_factor * 0.70) * 1.25
            q01 = (1.0 - pinch_factor * 0.70) * q01 + (pinch_factor * 0.70) * (-0.55)
            q02 = max(q02, pinch_factor * 0.50)
            q03 = max(q03, pinch_factor * 0.70)

        # 5. Full Fist / Grasp Coupling:
        # ONLY activate when 4 fingers are actually folded (flexion > 0.90 rad) AND thumb is near palm center
        palm_center = (pts[0] + pts[5] + pts[17]) / 3.0
        d_palm = np.linalg.norm(pts[tip_idx] - palm_center) / hand_size
        if fingers_flexion > 0.90 and d_palm < 0.45:
            fist_w = float(np.clip((fingers_flexion - 0.90) / 0.35, 0.0, 1.0) * np.clip((0.45 - d_palm) / 0.15, 0.0, 1.0))
            q00 = (1.0 - fist_w) * q00 + fist_w * 1.45
            q01 = (1.0 - fist_w) * q01 + fist_w * (-1.15)
            q02 = max(q02, fist_w * 0.80)
            q03 = max(q03, fist_w * 1.00)

        if self.hand_side == "left":
            # Left Hand URDF (allegro_hand_v6_left.urdf) uses axis [0, 0, -1] for joint00.
            # Rotating inward towards the palm and fingers requires negative angle (-0.05 ~ -1.40 rad).
            # (Positive angle rotates outward away from palm).
            q00_out = -float(q00)  # Inverted: sweeps inward across palm towards fingers
            q01_out = -float(q01)  # Elevation/swing forward is POSITIVE (+0.10 ~ +0.80 rad) on left hand axis
            q02_out = float(q02)   # MCP flexion is POSITIVE (0.0 ~ 1.2 rad) -> curls forward
            q03_out = float(q03)   # IP tip curl is POSITIVE (0.0 ~ 1.3 rad) -> curls forward
        else:
            q00_out = float(q00)
            q01_out = float(q01)
            q02_out = float(q02)
            q03_out = float(q03)

        res = {
            "joint00": q00_out,
            "joint01": q01_out,
            "joint02": q02_out,
            "joint03": q03_out,
            "ah_joint00": q00_out,
            "ah_joint01": q01_out,
            "ah_joint02": q02_out,
            "ah_joint03": q03_out,
        }
        return res

    def retarget_landmarks(self, landmarks_flat: List[float]) -> np.ndarray:
        """
        Converts 63-dim flat landmark array into 20-dim target joint angles for Allegro Hand V6.
        """
        pts = np.array(landmarks_flat, dtype=np.float64).reshape((21, 3))

        palm_forward = pts[9] - pts[WRIST]
        palm_transverse = pts[17] - pts[5]
        if self.hand_side == "left":
            palm_normal = np.cross(palm_forward, palm_transverse)
        else:
            palm_normal = np.cross(palm_transverse, palm_forward)

        q_dict: Dict[str, float] = {}

        # 1. Compute 4 fingers (Index, Middle, Ring, Pinky) first
        q_dict.update(self._compute_finger_joints(pts, INDEX_INDICES, "joint1", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints(pts, MIDDLE_INDICES, "joint2", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints(pts, RING_INDICES, "joint3", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints(pts, PINKY_INDICES, "joint4", palm_forward, palm_normal))

        # 2. Real fingers MCP flexion average across 4 fingers
        fingers_flexion = float((q_dict["joint11"] + q_dict["joint21"] + q_dict["joint31"] + q_dict["joint41"]) / 4.0)

        # 3. Compute Thumb (joint00~03)
        q_dict.update(self._compute_thumb_joints(pts, palm_forward, palm_transverse, palm_normal, fingers_flexion))

        target_angles = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            raw_val = q_dict[joint_name]
            min_lim, max_lim = JOINT_LIMITS[joint_name]
            target_angles[i] = np.clip(raw_val, min_lim, max_lim)

        # Apply EMA Low-Pass Filter
        if self._filtered_q is None:
            self._filtered_q = target_angles.copy()
        else:
            self._filtered_q = self.lp_alpha * target_angles + (1.0 - self.lp_alpha) * self._filtered_q

        return self._filtered_q.copy()

    def _landmarks_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 63:
            self.get_logger().warn(
                f"Received landmark array with unexpected length {len(msg.data)} (expected 63). Ignoring."
            )
            return

        # Clutch Check:
        # If Clutch is ENGAGED, ignore new landmarks and re-publish last held joint positions
        if self._clutch_engaged:
            if self._last_target_angles is not None:
                out_msg = Float64MultiArray()
                out_msg.data = self._last_target_angles.tolist()
                self._pub.publish(out_msg)

                now = time.monotonic()
                if self.verbose and (now - self._last_log_time >= 1.0):
                    self._last_log_time = now
                    self.get_logger().info("[CLUTCH ENGAGED] ⏸ 20 target joints held at last position.")
            return

        target_angles = self.retarget_landmarks(list(msg.data))
        self._last_target_angles = target_angles.copy()

        out_msg = Float64MultiArray()
        out_msg.data = target_angles.tolist()
        self._pub.publish(out_msg)
        self._seq += 1

        now = time.monotonic()
        if self.verbose and (now - self._last_log_time >= 0.5 or self._seq == 1):
            self._last_log_time = now
            th = target_angles[0:4]
            ix = target_angles[4:8]
            md = target_angles[8:12]
            rg = target_angles[12:16]
            pk = target_angles[16:20]
            self.get_logger().info(
                f"\n[V6 Frame #{self._seq}] 20-DOF Target Joints (rad):\n"
                f"  Thumb  (00~03): [{th[0]:+.3f}, {th[1]:+.3f}, {th[2]:+.3f}, {th[3]:+.3f}]\n"
                f"  Index  (10~13): [{ix[0]:+.3f}, {ix[1]:+.3f}, {ix[2]:+.3f}, {ix[3]:+.3f}]\n"
                f"  Middle (20~23): [{md[0]:+.3f}, {md[1]:+.3f}, {md[2]:+.3f}, {md[3]:+.3f}]\n"
                f"  Ring   (30~33): [{rg[0]:+.3f}, {rg[1]:+.3f}, {rg[2]:+.3f}, {rg[3]:+.3f}]\n"
                f"  Pinky  (40~43): [{pk[0]:+.3f}, {pk[1]:+.3f}, {pk[2]:+.3f}, {pk[3]:+.3f}]"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand V6 Kinematic Retargeting Node")
    parser.add_argument("--alpha", type=float, default=0.40, help="EMA low-pass filter coefficient (default: 0.40)")
    parser.add_argument("--hand", type=str, default="left", choices=["left", "right"], help="Hand model side (default: left)")
    parser.add_argument("--quiet", action="store_true", help="Suppress periodic terminal joint logs")
    args = parser.parse_args()

    rclpy.init()
    node = KinematicRetargetingNode(verbose=not args.quiet, lp_alpha=args.alpha, hand_side=args.hand)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down V6 retargeting node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
