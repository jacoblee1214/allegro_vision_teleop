#!/usr/bin/env python3
"""
retargeting_node.py — Kinematic Retargeting Node for 5-Finger Robot Hand (Allegro Hand V6).

Subscribes to `/allegro/vision/landmarks` (63-dim Float32MultiArray from vision_tracker),
calculates finger joint flexion and abduction angles using 3D phalanx bone vectors,
maps and clamps angles to Allegro Hand V6 joint limits, and publishes a 20-dim
Float64MultiArray in `[Thumb, Index, Middle, Ring, Pinky]` order to `/allegro/target_joints`.

Usage (inside container):
    source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash
    python3 /home/humble_ws/allegro_vision_teleop/retargeting_node.py
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray

# Dynamically import safety_utils_v6 from this script's directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_utils_v6 import CONTROLLER_JOINT_ORDER, JOINT_LIMITS, N_JOINTS

INPUT_TOPIC = "/allegro/vision/landmarks"
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
    # Project vectors onto the plane perpendicular to normal
    normal_unit = normal / (np.linalg.norm(normal) + 1e-8)
    v_bone_proj = v_bone - np.dot(v_bone, normal_unit) * normal_unit
    v_ref_proj = v_ref - np.dot(v_ref, normal_unit) * normal_unit

    norm_b = np.linalg.norm(v_bone_proj)
    norm_r = np.linalg.norm(v_ref_proj)
    if norm_b < 1e-6 or norm_r < 1e-6:
        return 0.0

    cos_ang = np.clip(np.dot(v_bone_proj, v_ref_proj) / (norm_b * norm_r), -1.0, 1.0)
    angle = np.arccos(cos_ang)

    # Determine sign using cross product with normal
    cross_prod = np.cross(v_ref_proj, v_bone_proj)
    if np.dot(cross_prod, normal_unit) < 0:
        angle = -angle

    return float(angle)


class KinematicRetargetingNode(Node):
    def __init__(self, verbose: bool = True) -> None:
        super().__init__("kinematic_retargeting_node")
        self.verbose = verbose
        self._seq = 0
        self._last_log_time = time.monotonic()

        # Subscriber: Vision Landmarks
        self._sub = self.create_subscription(
            Float32MultiArray,
            INPUT_TOPIC,
            self._landmarks_callback,
            QOS_PROFILE,
        )

        # Publisher: Target Joints
        self._pub = self.create_publisher(
            Float64MultiArray,
            OUTPUT_TOPIC,
            QOS_PROFILE,
        )

        self.get_logger().info(f"Subscribed to: {INPUT_TOPIC}")
        self.get_logger().info(f"Publishing target joint angles ({N_JOINTS} joints) to: {OUTPUT_TOPIC}")
        self.get_logger().info(f"Controller Joint Order: {CONTROLLER_JOINT_ORDER}")

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
        prefix is 'joint1' (Index), 'joint2' (Middle), 'joint3' (Ring), or 'joint4' (Pinky).
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
            # Angle relative to palm forward direction
            q0 = signed_abduction_angle(v_prox, palm_forward, palm_normal)
            if prefix in ("joint1", "ah_joint1"):
                # Index finger abduction scale
                q0 = q0 * 0.8
            elif prefix in ("joint3", "ah_joint3"):
                # Ring finger abduction scale
                q0 = q0 * 0.8
            elif prefix in ("joint4", "ah_joint4"):
                # Pinky finger abduction scale
                q0 = q0 * 0.8

        # Joint 1: MCP Flexion
        q1 = angle_between(v_meta, v_prox)

        # Joint 2: PIP Flexion
        q2 = angle_between(v_prox, v_inter)

        # Joint 3: DIP Flexion
        q3 = angle_between(v_inter, v_dist)

        # Apply calibration / sensitivity gains
        q1_scaled = q1 * 1.1
        q2_scaled = q2 * 1.0
        q3_scaled = q3 * 1.0

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
        Computes 4 joint angles for the Thumb (joint00~03):
        - When hand is open: thumb rests open sideways (q00~0.25, q01~0.30).
        - When fingers/thumb fold in (fist/grasp): thumb rotates INWARDS across palm (q00->1.45, q01->1.00, NOT downwards!).
        - When pinching: thumb opposition and elevation guide tips to meet accurately.
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
        # Rotation out of the lateral hand plane towards palm normal & index/pinky
        proj_norm = np.dot(u_thumb_ray, u_norm)
        proj_lat = np.dot(u_thumb_ray, -u_trans)  # positive when splayed outward
        opp_angle = np.arctan2(proj_norm, proj_lat)
        q00_raw = np.interp(opp_angle, [-0.2, 1.3], [0.15, 1.35])

        # 2. Base Elevation / Inward Swing (joint01):
        # On the physical Allegro Hand V6 hardware:
        # NEGATIVE values move the thumb INWARDS towards the palm.
        # POSITIVE values move the thumb DOWNWARDS away from the palm.
        # When thumb is open sideways: q01 is near 0 or slightly positive (0.0 ~ +0.10).
        # When thumb moves in front of palm: q01 goes negative (-0.3 ~ -0.7).
        q01_raw = np.interp(proj_norm, [-0.2, 0.5], [0.10, -0.65])

        # 3. Flexion angles (joint02, joint03)
        q02_bone = angle_between(v_thumb_prox, v_thumb_mid)
        q03_bone = angle_between(v_thumb_mid, v_thumb_dist)

        # 4. Palm Proximity & Hand Closure (Fist / Grasp):
        # When fingers are folded into palm, drive thumb NEGATIVE (INWARDS towards palm center, NOT downwards!)
        palm_center = (pts[0] + pts[5] + pts[17]) / 3.0
        d_palm = np.linalg.norm(pts[tip_idx] - palm_center) / hand_size
        curl_factor = float(np.clip(1.0 - (d_palm - 0.28) / (0.70 - 0.28), 0.0, 1.0))
        fist_factor = float(np.clip((fingers_flexion - 0.35) / (1.1 - 0.35), 0.0, 1.0))
        close_factor = max(curl_factor, fist_factor)

        # Drive thumb INWARD onto palm and folded fingers:
        q00 = max(q00_raw, 0.30 + close_factor * 1.15)  # reaches 1.45 rad (full opposition across palm)
        q01 = min(q01_raw, 0.05 - close_factor * 1.30)  # reaches -1.25 rad (deeply INWARD towards palm!)
        q02 = q02_bone * 1.1 + close_factor * 0.55
        q03 = q03_bone * 1.1 + close_factor * 0.70

        # 5. Scale-Invariant Pinch Synergy Coupling:
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
        }
        for k in list(res.keys()):
            res[f"ah_{k}"] = res[k]
        return res

    def retarget_landmarks(self, landmarks_flat: List[float]) -> np.ndarray:
        """
        Converts 63-dim flat landmark array into 20-dim target joint angles.
        Guarantees exact CONTROLLER_JOINT_ORDER [Thumb, Index, Middle, Ring, Pinky].
        """
        pts = np.array(landmarks_flat, dtype=np.float64).reshape((21, 3))

        # Palm coordinate system
        # Forward vector: Wrist (0) -> Middle MCP (9)
        palm_forward = pts[9] - pts[WRIST]
        # Transverse vector: Index MCP (5) -> Pinky MCP (17) (updated across full palm width)
        palm_transverse = pts[17] - pts[5]
        # Normal vector (pointing out from palm)
        palm_normal = np.cross(palm_transverse, palm_forward)

        # Dictionary to store raw calculated angles
        q_dict: Dict[str, float] = {}

        # 2. Index (joint10~13)
        q_dict.update(self._compute_finger_joints(pts, INDEX_INDICES, "joint1", palm_forward, palm_normal))

        # 3. Middle (joint20~23)
        q_dict.update(self._compute_finger_joints(pts, MIDDLE_INDICES, "joint2", palm_forward, palm_normal))

        # 4. Ring (joint30~33)
        q_dict.update(self._compute_finger_joints(pts, RING_INDICES, "joint3", palm_forward, palm_normal))

        # 5. Pinky (joint40~43)
        q_dict.update(self._compute_finger_joints(pts, PINKY_INDICES, "joint4", palm_forward, palm_normal))

        # Average flexion of 4 fingers to detect hand fold / fist
        fingers_flexion = (q_dict["joint11"] + q_dict["joint21"] + q_dict["joint31"] + q_dict["joint41"]) / 4.0

        # 1. Thumb (joint00~03)
        q_dict.update(self._compute_thumb_joints(pts, palm_forward, palm_transverse, palm_normal, fingers_flexion))

        # Construct final ordered array matching CONTROLLER_JOINT_ORDER and clamp to JOINT_LIMITS
        target_angles = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            raw_val = q_dict[joint_name]
            min_lim, max_lim = JOINT_LIMITS[joint_name]
            # Safety clamp within robot joint limits
            target_angles[i] = np.clip(raw_val, min_lim, max_lim)

        return target_angles

    def _landmarks_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 63:
            self.get_logger().warn(
                f"Received landmark array with unexpected length {len(msg.data)} (expected 63). Ignoring."
            )
            return

        target_angles = self.retarget_landmarks(list(msg.data))

        # Publish target joints
        out_msg = Float64MultiArray()
        out_msg.data = target_angles.tolist()
        self._pub.publish(out_msg)
        self._seq += 1

        # Console logging
        now = time.monotonic()
        if self.verbose and (now - self._last_log_time >= 0.25 or self._seq == 1):
            self._last_log_time = now
            th = target_angles[0:4]
            ix = target_angles[4:8]
            md = target_angles[8:12]
            rg = target_angles[12:16]
            pk = target_angles[16:20]
            self.get_logger().info(
                f"\n[Frame #{self._seq}] Target Joint Angles (rad):\n"
                f"  Thumb  (joint00~03): [{th[0]:+.3f}, {th[1]:+.3f}, {th[2]:+.3f}, {th[3]:+.3f}]\n"
                f"  Index  (joint10~13): [{ix[0]:+.3f}, {ix[1]:+.3f}, {ix[2]:+.3f}, {ix[3]:+.3f}]\n"
                f"  Middle (joint20~23): [{md[0]:+.3f}, {md[1]:+.3f}, {md[2]:+.3f}, {md[3]:+.3f}]\n"
                f"  Ring   (joint30~33): [{rg[0]:+.3f}, {rg[1]:+.3f}, {rg[2]:+.3f}, {rg[3]:+.3f}]\n"
                f"  Pinky  (joint40~43): [{pk[0]:+.3f}, {pk[1]:+.3f}, {pk[2]:+.3f}, {pk[3]:+.3f}]"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand Kinematic Retargeting Node")
    parser.add_argument("--quiet", action="store_true", help="Disable verbose console output")
    args = parser.parse_args()

    rclpy.init()
    node = KinematicRetargetingNode(verbose=not args.quiet)
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
