#!/usr/bin/env python3
"""
retargeting_node_v3.py — DexPilot retargeting node for the Allegro Hand V6 (20-DOF).

Drop-in replacement for `retargeting_node.py`: same subscriptions, same
`/allegro/target_joints` output in `CONTROLLER_JOINT_ORDER`, same clutch and
hand-switch behaviour, so `sim_bridge_node.py` and both UIs are unchanged.

What changed is the map itself. v1 reads bone angles off the MediaPipe skeleton
and writes them into joints through hand-tuned gains; it never asks where the
robot's fingertips land, so a pinch closes on screen and not on the hand. v2
solves for it, using the MANUS teleop project's DexPilot core, its V6 Force
URDF and its measured pad offsets and pinch distances. See
`dexpilot_retarget.py` for the measurements, and `tools/check_pinch_v2.py` to
re-run them.

Subscribes to:
  - `/allegro/vision/world_landmarks`  (63-dim, MediaPipe *world* landmarks, metres)
  - `/allegro/vision/landmarks`        (63-dim, normalised image landmarks — fallback)
  - `/allegro/teleop_state`            (JSON: clutch, hand_side)

Publishes:
  - `/allegro/target_joints`           (20-dim Float64MultiArray)

The world topic wins whenever it is live: normalised landmarks are squashed by
the image aspect ratio and carry only a weak relative depth, and DexPilot
matches 3D fingertip positions. Run `vision_tracker_v2.py`, `teleop_cockpit_v2.py`
or `teleop_dashboard_v2.py` to get it; the v1 producers publish only the
normalised topic and this node will say so.

Usage:
    python3 retargeting_node_v2.py [--hand right] [--alpha 0.25] [--scaling 1.6] [--quiet]

Needs dex_retargeting + pinocchio, which live in the MANUS project's venv.
`run_teleop_v2.sh` picks that interpreter automatically.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dexpilot_retarget import DexPilotUnavailable, VisionDexPilotRetargeter, config_path
from safety_utils import CONTROLLER_JOINT_ORDER, N_JOINTS, get_joint_limits

WORLD_TOPIC = "/allegro/vision/world_landmarks"
INPUT_TOPIC = "/allegro/vision/landmarks"
STATE_TOPIC = "/allegro/teleop_state"
OUTPUT_TOPIC = "/allegro/target_joints"

#: How long a world-landmark frame keeps the normalised topic locked out. Long
#: enough to ride out a few dropped detections, short enough that killing a v2
#: producer and starting a v1 one recovers on its own.
WORLD_HOLD_S = 1.0

QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class DexPilotRetargetingNode(Node):
    def __init__(
        self,
        verbose: bool = True,
        hand_side: str = "right",
        alpha: Optional[float] = None,
        scaling: Optional[float] = None,
        calibrate: int = 150,
        undo_mirror: bool = True,
        aspect: float = 480.0 / 640.0,
        v6f_root: Optional[str] = None,
    ) -> None:
        super().__init__("dexpilot_retargeting_node")
        self.verbose = verbose

        self.retargeter = VisionDexPilotRetargeter(
            hand_side=hand_side,
            root=v6f_root,
            scaling=scaling,
            calibrate_frames=calibrate,
            undo_mirror=undo_mirror,
            aspect=aspect,
            low_pass_alpha=alpha,
        )
        # The solver returns `target_joint_names` order. The bridge reads
        # CONTROLLER_JOINT_ORDER. Relying on the two happening to agree is how a
        # thumb command reaches a middle finger, so check it once, loudly.
        if self.retargeter.joint_order != list(CONTROLLER_JOINT_ORDER):
            raise RuntimeError(
                f"the DexPilot config's target_joint_names do not match "
                f"CONTROLLER_JOINT_ORDER:\n  solver: {self.retargeter.joint_order}\n"
                f"  bridge: {list(CONTROLLER_JOINT_ORDER)}"
            )

        self._clutch_engaged = False
        self._last_target_angles: Optional[np.ndarray] = None
        self._seq = 0
        self._last_log_time = time.monotonic()
        self._last_world_time = 0.0
        self._warned_normalised = False
        self._announced_world = False
        self._last_chirality_warn = 0.0

        self._sub_world = self.create_subscription(
            Float32MultiArray, WORLD_TOPIC, self._world_callback, QOS_PROFILE)
        self._sub_lm = self.create_subscription(
            Float32MultiArray, INPUT_TOPIC, self._landmarks_callback, QOS_PROFILE)
        self._sub_state = self.create_subscription(
            String, STATE_TOPIC, self._teleop_state_callback, QOS_PROFILE)
        self._pub = self.create_publisher(Float64MultiArray, OUTPUT_TOPIC, QOS_PROFILE)

        log = self.get_logger()
        log.info(f"DexPilot retargeting (V6, 20-DOF) up for the {self.retargeter.hand_side.upper()} hand.")
        log.info(f"  core    : {self.retargeter.root}")
        log.info(f"  config  : {config_path(self.retargeter.hand_side, self.retargeter.root)}")
        log.info(f"  urdf    : {self.retargeter.model.spec.urdf_path}")
        log.info(f"  scaling : {self.retargeter.scaling_factor:.3f}"
                 + ("" if scaling is not None else f" (re-measuring over the first {calibrate} frames)"))
        log.info(f"  mirror  : {'undoing the camera flip' if undo_mirror else 'raw (camera flip kept)'}")
        log.info(f"  in      : {WORLD_TOPIC} (preferred), {INPUT_TOPIC} (fallback)")
        log.info(f"  out     : {OUTPUT_TOPIC} ({N_JOINTS} joints)")

    # ── state ───────────────────────────────────────────────────────────────

    def _teleop_state_callback(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse teleop_state message: {exc}")
            return

        clutch = bool(state.get("clutch", False))
        if clutch != self._clutch_engaged:
            self._clutch_engaged = clutch
            if clutch:
                self.get_logger().warn("[CLUTCH] ⏸ ENGAGED: holding the last 20-DOF posture.")
            else:
                # The warm start is a previous *frame*, and the operator has
                # moved since. Dropping it stops the first live frame being
                # solved from a stale seed.
                self.retargeter.reset()
                self.get_logger().info("[CLUTCH] ▶ RELEASED: resuming live tracking.")

        side = state.get("hand_side")
        if side in ("left", "right") and side != self.retargeter.hand_side:
            self.retargeter.set_hand_side(side)
            self._last_target_angles = None
            self.get_logger().info(
                f"[UI HAND SWITCH] 🔄 {side.upper()} hand model loaded "
                f"(scaling {self.retargeter.scaling_factor:.3f})")

    # ── landmarks ───────────────────────────────────────────────────────────

    def _world_callback(self, msg: Float32MultiArray) -> None:
        self._last_world_time = time.monotonic()
        if not self._announced_world:
            self._announced_world = True
            self.get_logger().info(f"Metric world landmarks on {WORLD_TOPIC}; using them.")
        self._handle(msg, metric=True)

    def _landmarks_callback(self, msg: Float32MultiArray) -> None:
        if time.monotonic() - self._last_world_time < WORLD_HOLD_S:
            return          # a v2 producer is live; its metric topic wins
        if not self._warned_normalised:
            self._warned_normalised = True
            self.get_logger().warn(
                f"Only normalised landmarks on {INPUT_TOPIC} — a v1 producer. They are "
                f"squashed by the image aspect ratio and their depth is weak, so pinches "
                f"will be less accurate. Run vision_tracker_v2.py / teleop_cockpit_v2.py / "
                f"teleop_dashboard_v2.py, or ./run_teleop_v2.sh, for the metric topic.")
        self._handle(msg, metric=False)

    def _handle(self, msg: Float32MultiArray, metric: bool) -> None:
        if len(msg.data) != 3 * 21:
            self.get_logger().warn(
                f"Landmark array of length {len(msg.data)} (expected 63). Ignoring.")
            return

        if self._clutch_engaged:
            if self._last_target_angles is not None:
                out = Float64MultiArray()
                out.data = self._last_target_angles.tolist()
                self._pub.publish(out)
                now = time.monotonic()
                if self.verbose and now - self._last_log_time >= 1.0:
                    self._last_log_time = now
                    self.get_logger().info("[CLUTCH ENGAGED] ⏸ holding 20 target joints.")
            return

        try:
            target = self.retargeter(msg.data, metric=metric)
        except Exception as exc:
            self.get_logger().error(f"Retargeting failed on this frame: {exc}")
            return

        # DexPilot already clips to the config's limits, which are narrower than
        # the URDF's. This is the belt-and-braces pass against the same table the
        # bridge uses, so an out-of-range value can never leave this node.
        limits = get_joint_limits(self.retargeter.hand_side)
        for i, name in enumerate(CONTROLLER_JOINT_ORDER):
            lo, hi = limits[name]
            target[i] = np.clip(target[i], lo, hi)

        self._last_target_angles = target.copy()
        out = Float64MultiArray()
        out.data = target.tolist()
        self._pub.publish(out)
        self._seq += 1
        self._log(target)

    def _log(self, q: np.ndarray) -> None:
        now = time.monotonic()
        if self.retargeter.calibration_note:
            self.get_logger().info(f"[CALIBRATION] {self.retargeter.calibration_note}")
            self.retargeter.calibration_note = None
        if now - self._last_chirality_warn >= 10.0:
            note = self.retargeter.chirality_note()
            if note:
                self._last_chirality_warn = now
                self.get_logger().warn(f"[CHIRALITY] {note}")
        if not self.verbose or (now - self._last_log_time < 0.5 and self._seq != 1):
            return
        self._last_log_time = now
        self.get_logger().info(
            f"\n[DexPilot frame #{self._seq}] 20-DOF target joints (rad):\n"
            f"  Thumb  (00~03): [{q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f}]\n"
            f"  Index  (10~13): [{q[4]:+.3f}, {q[5]:+.3f}, {q[6]:+.3f}, {q[7]:+.3f}]\n"
            f"  Middle (20~23): [{q[8]:+.3f}, {q[9]:+.3f}, {q[10]:+.3f}, {q[11]:+.3f}]\n"
            f"  Ring   (30~33): [{q[12]:+.3f}, {q[13]:+.3f}, {q[14]:+.3f}, {q[15]:+.3f}]\n"
            f"  Pinky  (40~43): [{q[16]:+.3f}, {q[17]:+.3f}, {q[18]:+.3f}, {q[19]:+.3f}]")


class SmoothedDexPilotRetargetingNode(DexPilotRetargetingNode):
    """v3: the v2 node, with the landmarks cleaned up before they reach DexPilot.

    Matching fingertip *distances* is more sensitive to MediaPipe's per-frame noise than
    copying joint angles was, and the robot shows it as trembling. Measured on the MANUS
    glove recording with 3 mm of added noise, on a held-still hand (joint change per frame):

        output EMA 0.25 only (v2)            13.8 mrad
        + input median of 5                   8.5 mrad
        + input median of 5 and EMA 0.3       5.9 mrad   <- default

    The median throws away single-frame spikes; the EMA smooths what is left. Both run on
    the 21 landmarks, before the solver, so DexPilot sees a steadier hand.

    Also: when tracking drops for longer than `reset_gap` seconds the hand reappears
    somewhere else, and solving from the old warm start can leave DexPilot stuck short of
    a pinch (36 mm instead of 7 mm in that recording). The warm start is dropped then.
    """

    def __init__(self, *args, smooth: int = 5, input_ema: float = 0.3, reset_gap: float = 0.3, **kwargs):
        super().__init__(*args, **kwargs)
        self._smooth = max(1, int(smooth))
        self._input_ema = float(np.clip(input_ema, 0.0, 1.0))
        self._reset_gap = float(reset_gap)
        self._window: deque = deque(maxlen=self._smooth)
        self._smoothed: Optional[np.ndarray] = None
        self._last_input = 0.0
        self.get_logger().info(
            f"  input   : median of {self._smooth} frames + EMA {self._input_ema:.2f} on the landmarks, "
            f"reset after {self._reset_gap:.1f}s without a hand")

    def _handle(self, msg: Float32MultiArray, metric: bool) -> None:
        if len(msg.data) != 3 * 21:
            return super()._handle(msg, metric)
        now = time.monotonic()
        if self._last_input and now - self._last_input > self._reset_gap:
            self.retargeter.reset()
            self._window.clear()
            self._smoothed = None
        self._last_input = now

        points = np.asarray(msg.data, dtype=np.float64)
        self._window.append(points)
        filtered = np.median(np.stack(self._window), axis=0) if self._smooth > 1 else points
        if self._smoothed is None or self._input_ema <= 0.0:
            self._smoothed = filtered
        else:
            self._smoothed = self._input_ema * filtered + (1.0 - self._input_ema) * self._smoothed

        cleaned = Float32MultiArray()
        cleaned.data = [float(v) for v in self._smoothed]
        super()._handle(cleaned, metric)


def main() -> None:
    p = argparse.ArgumentParser(description="Allegro Hand V6 DexPilot retargeting node (v3: smoothed input)")
    p.add_argument("--hand", default="right", choices=["left", "right"], help="hand model side")
    p.add_argument("--alpha", type=float, default=None,
                   help="output low-pass alpha; default is the config's (0.25)")
    p.add_argument("--scaling", type=float, default=None,
                   help="fixed operator/robot finger-length ratio; default measures it")
    p.add_argument("--calibrate", type=int, default=150,
                   help="frames to measure the scaling over (0 = use the config's)")
    p.add_argument("--no-undo-mirror", dest="undo_mirror", action="store_false",
                   help="do not undo the producers' cv2.flip; use if the thumb goes the wrong way")
    p.add_argument("--aspect", type=float, default=480.0 / 640.0,
                   help="camera height/width, for the normalised-landmark fallback only")
    p.add_argument("--v6f-root", default=None,
                   help="the MANUS teleop checkout (default: $V6F_TELEOP_ROOT)")
    p.add_argument("--quiet", action="store_true", help="suppress the periodic joint log")
    p.add_argument("--smooth", type=int, default=5,
                   help="median filter over this many landmark frames (1 = off). Raise it if the hand trembles")
    p.add_argument("--input-ema", type=float, default=0.3,
                   help="EMA on the landmarks after the median (0 = off; lower = smoother and slower)")
    p.add_argument("--reset-gap", type=float, default=0.3,
                   help="drop DexPilot's warm start after this many seconds without a hand")
    args = p.parse_args()

    if args.scaling is None and args.calibrate == 0:
        args.scaling = None     # keep the config's fixed value, measure nothing

    rclpy.init()
    try:
        node = SmoothedDexPilotRetargetingNode(
            smooth=args.smooth,
            input_ema=args.input_ema,
            reset_gap=args.reset_gap,
            verbose=not args.quiet,
            hand_side=args.hand,
            alpha=args.alpha,
            scaling=args.scaling,
            calibrate=args.calibrate,
            undo_mirror=args.undo_mirror,
            aspect=args.aspect,
            v6f_root=args.v6f_root,
        )
    except DexPilotUnavailable as exc:
        print(f"[retargeting_node_v3] {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down the DexPilot retargeting node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
