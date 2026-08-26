#!/usr/bin/env python3
"""
retargeting_node.py — Kinematic Retargeting Node for Allegro Hand V4.

Subscribes to `/allegro/vision/landmarks` (51-dim Float32MultiArray from vision_tracker),
calculates finger joint flexion and abduction angles using 3D phalanx bone vectors,
maps and clamps angles to Allegro Hand V4 joint limits, and publishes a 16-dim
Float64MultiArray in `[Thumb, Index, Middle, Ring]` order to `/allegro/target_joints`.

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

# Dynamically import safety_utils from this script's directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_utils import CONTROLLER_JOINT_ORDER, JOINT_LIMITS, N_JOINTS

INPUT_TOPIC = "/allegro/vision/landmarks"
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
        Computes 4 joint angles (abduction, MCP, PIP, DIP) for a finger (Index, Middle, Ring).
        prefix is 'ah_joint1' (Index), 'ah_joint2' (Middle), or 'ah_joint3' (Ring).
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
            # Angle relative to palm forward direction
            q0 = signed_abduction_angle(v_prox, palm_forward, palm_normal)
            if prefix == "ah_joint1":
                # Index finger abduction scale
                q0 = q0 * 0.8
            elif prefix == "ah_joint3":
                # Ring finger abduction scale
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
    ) -> Dict[str, float]:
        """
        Computes 4 joint angles for the Thumb (ah_joint00~03).
        """
        cmc_idx, mcp_idx, ip_idx, tip_idx = THUMB_INDICES

        v_wrist_cmc = pts[cmc_idx] - pts[WRIST]
        v_cmc_mcp = pts[mcp_idx] - pts[cmc_idx]
        v_mcp_ip = pts[ip_idx] - pts[mcp_idx]
        v_ip_tip = pts[tip_idx] - pts[ip_idx]

        # Joint 00: Opposition / Rotation (Towards palm center)
        # Measured as the angle of CMC-MCP vector relative to palm normal/plane
        opp_angle = angle_between(v_cmc_mcp, palm_normal)
        # Map opposition angle (~1.2 to 2.2 rad) to Allegro range [0.263, 1.396]
        q00 = np.interp(opp_angle, [1.0, 2.2], [0.35, 1.25])

        # Joint 01: MCP Abduction / Elevation
        elev_angle = angle_between(v_cmc_mcp, palm_forward)
        q01 = np.interp(elev_angle, [0.3, 1.8], [-0.05, 0.95])

        # Joint 02: MCP Flexion
        q02 = angle_between(v_cmc_mcp, v_mcp_ip) * 1.1

        # Joint 03: IP Flexion
        q03 = angle_between(v_mcp_ip, v_ip_tip) * 1.1

        return {
            "ah_joint00": float(q00),
            "ah_joint01": float(q01),
            "ah_joint02": float(q02),
            "ah_joint03": float(q03),
        }

    def retarget_landmarks(self, landmarks_flat: List[float]) -> np.ndarray:
        """
        Converts 51-dim flat landmark array into 16-dim target joint angles.
        Guarantees exact CONTROLLER_JOINT_ORDER [Thumb, Index, Middle, Ring].
        """
        pts = np.array(landmarks_flat, dtype=np.float64).reshape((17, 3))

        # Palm coordinate system
        # Forward vector: Wrist (0) -> Middle MCP (9)
        palm_forward = pts[9] - pts[WRIST]
        # Transverse vector: Index MCP (5) -> Ring MCP (13)
        palm_transverse = pts[13] - pts[5]
        # Normal vector (pointing out from palm)
        palm_normal = np.cross(palm_transverse, palm_forward)

        # Dictionary to store raw calculated angles
        q_dict: Dict[str, float] = {}

        # 1. Thumb (ah_joint00~03)
        q_dict.update(self._compute_thumb_joints(pts, palm_forward, palm_normal))

        # 2. Index (ah_joint10~13)
        q_dict.update(self._compute_finger_joints(pts, INDEX_INDICES, "ah_joint1", palm_forward, palm_normal))

        # 3. Middle (ah_joint20~23)
        q_dict.update(self._compute_finger_joints(pts, MIDDLE_INDICES, "ah_joint2", palm_forward, palm_normal))

        # 4. Ring (ah_joint30~33)
        q_dict.update(self._compute_finger_joints(pts, RING_INDICES, "ah_joint3", palm_forward, palm_normal))

        # Construct final ordered array matching CONTROLLER_JOINT_ORDER and clamp to JOINT_LIMITS
        target_angles = np.zeros(N_JOINTS, dtype=np.float64)
        for i, joint_name in enumerate(CONTROLLER_JOINT_ORDER):
            raw_val = q_dict[joint_name]
            min_lim, max_lim = JOINT_LIMITS[joint_name]
            # Safety clamp within Allegro V4 joint limits
            target_angles[i] = np.clip(raw_val, min_lim, max_lim)

        return target_angles

    def _landmarks_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 51:
            self.get_logger().warn(
                f"Received landmark array with unexpected length {len(msg.data)} (expected 51). Ignoring."
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
            self.get_logger().info(
                f"\n[Frame #{self._seq}] Target Joint Angles (rad):\n"
                f"  Thumb  (ah_joint00~03): [{th[0]:+.3f}, {th[1]:+.3f}, {th[2]:+.3f}, {th[3]:+.3f}]\n"
                f"  Index  (ah_joint10~13): [{ix[0]:+.3f}, {ix[1]:+.3f}, {ix[2]:+.3f}, {ix[3]:+.3f}]\n"
                f"  Middle (ah_joint20~23): [{md[0]:+.3f}, {md[1]:+.3f}, {md[2]:+.3f}, {md[3]:+.3f}]\n"
                f"  Ring   (ah_joint30~33): [{rg[0]:+.3f}, {rg[1]:+.3f}, {rg[2]:+.3f}, {rg[3]:+.3f}]"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro Hand Kinematic Retargeting Node")
    parser.add_argument("--quiet", action="store_true", help="Disable verbose console output")
    args = parser.parse_args()

    rclpy.init()
    node = KinematicRetargetingNode(verbose=not args.quiet)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down retargeting node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
