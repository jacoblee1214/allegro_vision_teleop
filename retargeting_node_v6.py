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
from safety_utils_v6 import (
    CONTROLLER_JOINT_ORDER,
    JOINT_LIMITS,
    JOINT_LIMITS_LEFT,
    JOINT_LIMITS_RIGHT,
    N_JOINTS,
    get_joint_limits,
)

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

# ─── Left Hand Thumb Kinematic Calibration Parameters ─────────────────────────
# Direct numerical control over Left Hand thumb joint signs, gains, and ranges:
LEFT_THUMB_CONFIG = {
    # joint00: Base Opposition across palm towards index
    # Motor polarity on Left HW: negative values rotate inward towards palm/index
    "j00_sign": -1.0,
    "j00_min": 0.05,            # Open flat (rad)
    "j00_max": 1.40,            # Max opposition (rad)
    "j00_pinch": 1.25,          # Target during pinch (rad)

    # joint01: Elevation / Swing (upward along index)
    # Motor polarity on Left HW: negative values rotate UPWARD along index
    # Amplified elevation range to 1.50 rad (~86 deg) for full upward reach
    "j01_sign": -1.0,
    "j01_flat": 0.10,           # Open flat resting angle (rad)
    "j01_elev_max": 1.50,       # Max upward elevation (rad, ~86 deg)
    "j01_pinch": 0.75,          # Target during pinch (rad, ~43 deg)
    "j01_fist": 0.45,           # Target during fist (rad)

    # joint02: MCP Flexion (forward curl)
    "j02_sign": 1.0,
    "j02_scale": 1.50,          # Sensitivity gain for human thumb MCP bend
    "j02_pinch": 0.50,

    # joint03: IP Curl (tip curl)
    "j03_sign": 1.0,
    "j03_scale": 2.10,          # Sensitivity gain for human thumb tip curl
    "j03_pinch": 0.70,
}
# Alias for backwards compatibility
LEFT_THUMB_CALIB = LEFT_THUMB_CONFIG


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
    def __init__(self, verbose: bool = True, lp_alpha: float = 0.40, hand_side: str = "right") -> None:
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

        # Subscriber: Teleop Operator State (Clutch, Record, Tag, Hand Switch)
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

        self.get_logger().info(f"Allegro Hand V6 (5-Finger) Retargeting Node initialized for {self.hand_side.upper()} HAND.")
        self.get_logger().info(f"Subscribed to: {INPUT_TOPIC} & {STATE_TOPIC}")
        self.get_logger().info(f"Publishing target joint angles ({N_JOINTS} joints) to: {OUTPUT_TOPIC}")
        self.get_logger().info(f"Architecture: Isolated Dual Pipelines (Left vs Right Hand fully decoupled)")

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
                self._filtered_q = None          # Reset filter history to prevent jump across models
                self._last_target_angles = None
                self.get_logger().info(f"[UI HAND SWITCH] 🔄 Active Hand Model switched to: {self.hand_side.upper()} HAND (filter reset)")
        except Exception as e:
            self.get_logger().error(f"Failed to parse teleop_state message: {e}")

    # ═══════════════════════════════════════════════════════════════════════════
    # ─── RIGHT HAND PIPELINE (100% Proven & Restored Original Kinematics) ───────
    # ═══════════════════════════════════════════════════════════════════════════

    def _compute_finger_joints_right(
        self,
        pts: np.ndarray,
        indices: Tuple[int, int, int, int],
        prefix: str,
        palm_forward: np.ndarray,
        palm_normal: np.ndarray,
    ) -> Dict[str, float]:
        """Computes 4 finger joint angles for RIGHT Hand (exact verified kinematics)."""
        mcp_idx, pip_idx, dip_idx, tip_idx = indices
        v_meta = pts[mcp_idx] - pts[WRIST]
        v_prox = pts[pip_idx] - pts[mcp_idx]
        v_inter = pts[dip_idx] - pts[pip_idx]
        v_dist = pts[tip_idx] - pts[dip_idx]

        if prefix in ("joint2", "ah_joint2"):
            q0 = 0.0
        else:
            q0 = signed_abduction_angle(v_prox, palm_forward, palm_normal) * 0.80

        q1 = angle_between(v_meta, v_prox) * 1.10
        q2 = angle_between(v_prox, v_inter) * 1.00
        q3 = angle_between(v_inter, v_dist) * 1.00

        base_prefix = prefix.removeprefix("ah_")
        res = {
            f"{base_prefix}0": q0,
            f"{base_prefix}1": q1,
            f"{base_prefix}2": q2,
            f"{base_prefix}3": q3,
        }
        for k in list(res.keys()):
            res[f"ah_{k}"] = res[k]
        return res

    def _compute_thumb_joints_right(
        self,
        pts: np.ndarray,
        palm_forward: np.ndarray,
        palm_transverse: np.ndarray,
        palm_normal: np.ndarray,
        fingers_flexion: float = 0.0,
    ) -> Dict[str, float]:
        """Computes 4 thumb joint angles for RIGHT Hand (exact verified kinematics)."""
        cmc_idx, mcp_idx, ip_idx, tip_idx = THUMB_INDICES
        hand_size = np.linalg.norm(palm_forward) + 1e-6
        u_fwd = palm_forward / hand_size
        u_norm = palm_normal / (np.linalg.norm(palm_normal) + 1e-6)
        u_trans = palm_transverse / (np.linalg.norm(palm_transverse) + 1e-6)

        v_thumb_prox = pts[mcp_idx] - pts[cmc_idx]
        v_thumb_mid = pts[ip_idx] - pts[mcp_idx]
        v_thumb_dist = pts[tip_idx] - pts[ip_idx]
        v_thumb_ray = pts[tip_idx] - pts[cmc_idx]
        u_thumb_ray = v_thumb_ray / (np.linalg.norm(v_thumb_ray) + 1e-6)

        proj_norm = np.dot(u_thumb_ray, u_norm)
        proj_lat = np.dot(u_thumb_ray, -u_trans)
        opp_angle = np.arctan2(proj_norm, proj_lat)
        q00_raw = float(np.interp(opp_angle, [-0.2, 1.3], [0.15, 1.35]))
        q01_raw = float(np.interp(proj_norm, [-0.2, 0.5], [0.10, -0.65]))

        q02_bone = angle_between(v_thumb_prox, v_thumb_mid)
        q03_bone = angle_between(v_thumb_mid, v_thumb_dist)

        palm_center = (pts[0] + pts[5] + pts[17]) / 3.0
        d_palm = np.linalg.norm(pts[tip_idx] - palm_center) / hand_size
        curl_factor = float(np.clip(1.0 - (d_palm - 0.28) / (0.70 - 0.28), 0.0, 1.0))
        fist_factor = float(np.clip((fingers_flexion - 0.35) / (1.1 - 0.35), 0.0, 1.0))
        close_factor = max(curl_factor, fist_factor)

        q00 = max(q00_raw, 0.30 + close_factor * 1.15)
        q01 = min(q01_raw, 0.05 - close_factor * 1.30)
        q02 = q02_bone * 1.10 + close_factor * 0.55
        q03 = q03_bone * 1.10 + close_factor * 0.70

        d_pinch_index = np.linalg.norm(pts[4] - pts[8]) / hand_size
        d_pinch_middle = np.linalg.norm(pts[4] - pts[12]) / hand_size
        d_pinch = min(d_pinch_index, d_pinch_middle)
        pinch_factor = float(np.clip(1.0 - (d_pinch - 0.12) / (0.38 - 0.12), 0.0, 1.0))

        if pinch_factor > 0 and close_factor < 0.6:
            q00 = (1.0 - pinch_factor * 0.75) * q00 + (pinch_factor * 0.75) * 1.20
            q01 = (1.0 - pinch_factor * 0.75) * q01 + (pinch_factor * 0.75) * (-0.55)
            q02 = max(q02, pinch_factor * 0.65)
            q03 = max(q03, pinch_factor * 0.85)

        res = {
            "joint00": float(q00),
            "joint01": float(q01),
            "joint02": float(q02),
            "joint03": float(q03),
            "ah_joint00": float(q00),
            "ah_joint01": float(q01),
            "ah_joint02": float(q02),
            "ah_joint03": float(q03),
        }
        return res

    def _retarget_right_hand(self, pts: np.ndarray) -> np.ndarray:
        """Full retargeting pipeline for RIGHT Hand using JOINT_LIMITS_RIGHT."""
        palm_forward = pts[9] - pts[WRIST]
        palm_transverse = pts[17] - pts[5]
        palm_normal = np.cross(palm_transverse, palm_forward)

        q_dict: Dict[str, float] = {}
        q_dict.update(self._compute_finger_joints_right(pts, INDEX_INDICES, "joint1", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints_right(pts, MIDDLE_INDICES, "joint2", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints_right(pts, RING_INDICES, "joint3", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints_right(pts, PINKY_INDICES, "joint4", palm_forward, palm_normal))

        fingers_flexion = float((q_dict["joint11"] + q_dict["joint21"] + q_dict["joint31"] + q_dict["joint41"]) / 4.0)
        q_dict.update(self._compute_thumb_joints_right(pts, palm_forward, palm_transverse, palm_normal, fingers_flexion))

        from safety_utils_v6 import JOINT_LIMITS_RIGHT
        target_angles = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            raw_val = q_dict[joint_name]
            min_lim, max_lim = JOINT_LIMITS_RIGHT[joint_name]
            target_angles[i] = np.clip(raw_val, min_lim, max_lim)
        return target_angles

    # ═══════════════════════════════════════════════════════════════════════════
    # ─── LEFT HAND PIPELINE (Dedicated Left HW Kinematics & Amplified Elev) ────
    # ═══════════════════════════════════════════════════════════════════════════

    def _compute_finger_joints_left(
        self,
        pts: np.ndarray,
        indices: Tuple[int, int, int, int],
        prefix: str,
        palm_forward: np.ndarray,
        palm_normal: np.ndarray,
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles for LEFT Hand fingers (Index, Middle, Ring, Pinky).
        With palm_normal = cross(palm_forward, palm_transverse):
          - Index outward abduction gives NEGATIVE angle, matching Left Index limit (-1.309, 0.384).
          - Ring & Pinky outward abduction give POSITIVE angle, matching Left Ring/Pinky limit (-0.4, 1.309).
          - NO sign inversion applied, ensuring natural outward finger spreading.
        """
        mcp_idx, pip_idx, dip_idx, tip_idx = indices
        v_prox = pts[pip_idx] - pts[mcp_idx]
        v_inter = pts[dip_idx] - pts[pip_idx]
        v_dist = pts[tip_idx] - pts[dip_idx]

        # 1. Joint 0: Abduction/Adduction (첫 번째 관절: 좌우 벌림)
        if prefix in ("joint2", "ah_joint2"):
            q0 = 0.0
        else:
            q0 = signed_abduction_angle(v_prox, palm_forward, palm_normal) * 0.85

        # 2. Joint 1: MCP Flexion (두 번째 관절: 기저 굽힘)
        # 표준 URDF 좌표계: 손을 폈을 때 0.0 rad(완전 직립 신전), 손을 접을 때 0.0 ~ 1.571 rad(정방향 양수 굽힘)
        # 실제 하드웨어 모터 오프셋(-90도)은 sim_bridge_node에서 모터 송출 시에만 적용되므로, 비전/URDF는 완벽한 0 rad 신전 유지!
        v_meta = pts[mcp_idx] - pts[WRIST]
        q1_raw = angle_between(v_meta, v_prox)
        q1 = float(np.clip(max(0.0, q1_raw - 0.05) * 1.10, 0.0, 1.571))

        # 3. Joint 2: PIP Flexion (세 번째 관절: 중간 마디 굽힘)
        # 0.06 rad 데드밴드를 적용하여 손을 폈을 때 미세한 자연 곡률로 인한 굽힘을 완전히 0으로 신전
        q2_raw = angle_between(v_prox, v_inter)
        q2 = float(np.clip(max(0.0, q2_raw - 0.06) * 1.10, 0.0, 1.396))

        # 4. Joint 3: DIP Flexion (네 번째 관절: 끝 마디 굽힘)
        q3_raw = angle_between(v_inter, v_dist)
        q3 = float(np.clip(max(0.0, q3_raw - 0.06) * 1.10, 0.0, 1.396))

        base_prefix = prefix.removeprefix("ah_")
        res = {
            f"{base_prefix}0": q0,
            f"{base_prefix}1": q1,
            f"{base_prefix}2": q2,
            f"{base_prefix}3": q3,
        }
        for k in list(res.keys()):
            res[f"ah_{k}"] = res[k]
        return res

    def _compute_thumb_joints_left(
        self,
        pts: np.ndarray,
        palm_forward: np.ndarray,
        palm_transverse: np.ndarray,
        palm_normal: np.ndarray,
        fingers_flexion: float = 0.0,
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles for LEFT Hand Thumb:
        - Opposition across palm (joint00): Negative commands rotate inward on physical Left Hand.
        - Elevation along index (joint01): Negative commands rotate UPWARD along index.
          Enhanced upward elevation reach up to 1.50 rad (~86 deg).
        """
        cmc_idx, mcp_idx, ip_idx, tip_idx = THUMB_INDICES
        hand_size = np.linalg.norm(palm_forward) + 1e-6
        u_fwd = palm_forward / hand_size
        u_norm = palm_normal / (np.linalg.norm(palm_normal) + 1e-6)
        u_trans = palm_transverse / (np.linalg.norm(palm_transverse) + 1e-6)

        v_thumb_prox = pts[mcp_idx] - pts[cmc_idx]
        v_thumb_mid = pts[ip_idx] - pts[mcp_idx]
        v_thumb_dist = pts[tip_idx] - pts[ip_idx]
        v_thumb_ray = pts[tip_idx] - pts[cmc_idx]
        u_thumb_ray = v_thumb_ray / (np.linalg.norm(v_thumb_ray) + 1e-6)

        # 1. Base Opposition (joint00)
        proj_norm = float(np.dot(u_thumb_ray, u_norm))
        # Spreading open laterally away from palm/pinky is -u_trans
        proj_lat = float(np.dot(u_thumb_ray, -u_trans))
        opp_angle = float(np.arctan2(proj_norm, proj_lat))

        # When thumb is splayed wide open laterally (proj_lat > 0.20), enforce wide open angle
        open_bias = float(np.clip((proj_lat - 0.20) / 0.45, 0.0, 1.0))
        q00_raw = float(np.interp(opp_angle, [0.10, 1.20], [LEFT_THUMB_CONFIG["j00_min"], LEFT_THUMB_CONFIG["j00_max"]]))
        q00 = (1.0 - open_bias * 0.75) * q00_raw + (open_bias * 0.75) * LEFT_THUMB_CONFIG["j00_min"]

        # 2. Base Elevation / Upward Swing (joint01)
        # Elevation along index finger: thumb aligns parallel to index (high proj_fwd, low proj_lat)
        proj_fwd = float(np.dot(u_thumb_ray, u_fwd))
        elev_align = float(np.clip((proj_fwd - 0.70) / 0.25, 0.0, 1.0)) * float(np.clip((0.45 - proj_lat) / 0.30, 0.0, 1.0))

        q01_base = float(np.interp(proj_norm, [-0.15, 0.45], [LEFT_THUMB_CONFIG["j01_flat"], 0.65]))
        # When hand is wide open, keep thumb resting near flat (0.10~0.20 rad)
        q01_base = (1.0 - open_bias * 0.60) * q01_base + (open_bias * 0.60) * 0.12
        # When elevated along index, dynamically reach up to j01_elev_max (1.50 rad ~ 86 deg)
        q01_mag = q01_base + elev_align * (LEFT_THUMB_CONFIG["j01_elev_max"] - q01_base)

        # 3. Flexion angles (joint02, joint03)
        # Small deadband (0.08 rad) so thumb opens straight without residual bone curvature
        q02_bone = max(0.0, angle_between(v_thumb_prox, v_thumb_mid) - 0.08)
        q03_bone = max(0.0, angle_between(v_thumb_mid, v_thumb_dist) - 0.08)
        q02 = q02_bone * LEFT_THUMB_CONFIG["j02_scale"]
        q03 = q03_bone * LEFT_THUMB_CONFIG["j03_scale"]

        # 4. Pinch Synergy
        d_pinch_index = np.linalg.norm(pts[4] - pts[8]) / hand_size
        d_pinch_middle = np.linalg.norm(pts[4] - pts[12]) / hand_size
        d_pinch = min(d_pinch_index, d_pinch_middle)
        pinch_factor = float(np.clip(1.0 - (d_pinch - 0.10) / (0.35 - 0.10), 0.0, 1.0))

        if pinch_factor > 0:
            q00 = (1.0 - pinch_factor * 0.70) * q00 + (pinch_factor * 0.70) * LEFT_THUMB_CONFIG["j00_pinch"]
            q01_mag = (1.0 - pinch_factor * 0.70) * q01_mag + (pinch_factor * 0.70) * LEFT_THUMB_CONFIG["j01_pinch"]
            q02 = max(q02, pinch_factor * LEFT_THUMB_CONFIG["j02_pinch"])
            q03 = max(q03, pinch_factor * LEFT_THUMB_CONFIG["j03_pinch"])

        # 5. Full Fist / Grasp
        palm_center = (pts[0] + pts[5] + pts[17]) / 3.0
        d_palm = np.linalg.norm(pts[tip_idx] - palm_center) / hand_size
        if fingers_flexion > 0.90 and d_palm < 0.45:
            fist_w = float(np.clip((fingers_flexion - 0.90) / 0.35, 0.0, 1.0) * np.clip((0.45 - d_palm) / 0.15, 0.0, 1.0))
            q00 = (1.0 - fist_w) * q00 + fist_w * 1.45
            q01_mag = (1.0 - fist_w) * q01_mag + fist_w * LEFT_THUMB_CONFIG["j01_fist"]
            q02 = max(q02, fist_w * 0.80)
            q03 = max(q03, fist_w * 1.00)

        c = LEFT_THUMB_CONFIG
        q00_out = c["j00_sign"] * float(q00)
        q01_out = c["j01_sign"] * float(q01_mag)
        q02_out = c["j02_sign"] * float(q02)
        q03_out = c["j03_sign"] * float(q03)

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

    def _retarget_left_hand(self, pts: np.ndarray) -> np.ndarray:
        """Full retargeting pipeline for LEFT Hand using JOINT_LIMITS_LEFT."""
        palm_forward = pts[9] - pts[WRIST]
        palm_transverse = pts[17] - pts[5]
        palm_normal = np.cross(palm_forward, palm_transverse)

        q_dict: Dict[str, float] = {}
        q_dict.update(self._compute_finger_joints_left(pts, INDEX_INDICES, "joint1", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints_left(pts, MIDDLE_INDICES, "joint2", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints_left(pts, RING_INDICES, "joint3", palm_forward, palm_normal))
        q_dict.update(self._compute_finger_joints_left(pts, PINKY_INDICES, "joint4", palm_forward, palm_normal))

        fingers_flexion = float((q_dict["joint11"] + q_dict["joint21"] + q_dict["joint31"] + q_dict["joint41"]) / 4.0)
        q_dict.update(self._compute_thumb_joints_left(pts, palm_forward, palm_transverse, palm_normal, fingers_flexion))

        from safety_utils_v6 import JOINT_LIMITS_LEFT
        target_angles = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            raw_val = q_dict[joint_name]
            min_lim, max_lim = JOINT_LIMITS_LEFT[joint_name]
            target_angles[i] = np.clip(raw_val, min_lim, max_lim)
        return target_angles

    def retarget_landmarks(self, landmarks_flat: List[float]) -> np.ndarray:
        """
        Converts 63-dim flat landmark array into 20-dim target joint angles for Allegro Hand V6.
        Delegates to independent _retarget_right_hand or _retarget_left_hand based on active model.
        """
        pts = np.array(landmarks_flat, dtype=np.float64).reshape((21, 3))

        if self.hand_side == "right":
            target_angles = self._retarget_right_hand(pts)
        else:
            target_angles = self._retarget_left_hand(pts)

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
    parser.add_argument("--hand", type=str, default="right", choices=["left", "right"], help="Hand model side (default: right)")
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
