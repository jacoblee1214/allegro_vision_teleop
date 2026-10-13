"""
retarget/model.py — DexPilot retargeting for the Allegro Hand V6, on public parts only.

The optimization is upstream `dex_retargeting` (MIT, github.com/dexsuite/dex-retargeting),
installed from PyPI. What this module adds is everything that makes it fit this hand:

* **Fingertips the optimizer can aim at.** It targets *links*, and the V6's distal links
  have their origin at the joint, 38 to 51 mm short of the end. Aiming there asks the
  solver to overlap two joints, and the robot's pinch never closes however hard the
  operator pinches. So the hand's URDF gets five massless links fixed at the measured
  fingertip centres, and the optimizer is pointed at those.
* **A pinch distance it can reach.** Upstream's `eta1` defaults to 0.1 mm, which asks two
  fingertip *centres* to meet -- they cannot, the fingers are 19 mm thick. Measured from
  the meshes, the centres are `pinch_distance` apart when the surfaces touch, and that is
  what a pinch closes to.
* **The operator's hand in the hand's own frame**, and the size difference between the
  two, which is measured per operator over the first frames.

The numbers live in `v6.yaml` and come from `tools/measure_fingertips.py`.

Usage:

    from retarget import V6DexPilot
    model = V6DexPilot.load("right")
    qpos = model(canonical)            # (21, 3) metres -> 20 joint angles
"""
from __future__ import annotations

import hashlib
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
CONFIG = Path(__file__).resolve().parent / "v6.yaml"

#: MANO/MediaPipe landmark numbers. The hand frame is built from the wrist and the
#: knuckles; the fingertips are what the optimizer is given.
WRIST = 0
INDEX_MCP, MIDDLE_MCP, RING_MCP = 5, 9, 13
#: Per finger, the landmarks from knuckle to fingertip. Their lengths added up are the
#: operator's finger length, which is what the robot is scaled against.
FINGER_LANDMARKS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
#: Which fingers the scale is measured from: the three that are straight in a flat hand
#: and whose landmarks MediaPipe places most consistently.
SCALE_FINGERS = ("index", "middle", "ring")


def to_canonical(points: np.ndarray) -> np.ndarray:
    """The operator's 21 landmarks in their own hand's frame.

    Origin at the wrist, z along the wrist-to-middle-knuckle line, x from the little finger
    knuckle towards the index, y out of the back of the hand. Built from the wrist
    and three knuckles rather than from the fingers, so it does not move as the fingers do.

    **This is the V6's own base_link frame**, which it has to be: the optimizer compares
    these vectors against robot link positions, and a frame rotated against the robot's
    would turn a pinch into some other gesture. Check it against the hand with
    `tools/check_pinch.py`, which poses the robot, feeds its own landmarks back in, and
    sees whether the fingertips land where they started.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(21, 3)
    z = pts[MIDDLE_MCP] - pts[WRIST]
    z /= np.linalg.norm(z)
    x = pts[INDEX_MCP] - pts[RING_MCP]
    x -= z * (x @ z)                      # the knuckles are not quite square to the fingers
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    frame = np.eye(4)
    frame[:3, :3] = np.array([x, y, z]).T
    frame[:3, 3] = pts[WRIST]
    homogeneous = np.concatenate([pts, np.ones((21, 1))], axis=1)
    return (homogeneous @ np.linalg.inv(frame).T)[:, :3]


def hand_length(canonical: np.ndarray) -> float:
    """Knuckle to fingertip, added up over the three fingers the scale is measured from."""
    total = 0.0
    for finger in SCALE_FINGERS:
        chain = canonical[list(FINGER_LANDMARKS[finger])]
        total += float(np.linalg.norm(np.diff(chain, axis=0), axis=1).sum())
    return total


@dataclass
class HandSpec:
    """What the retargeting knows about one hand. Read from v6.yaml."""

    side: str
    urdf_path: Path
    tip_links: dict[str, str]
    fingertips: dict[str, list[float]]
    pinch_distance: float
    base_link: str
    joint_order: list[str]
    scaling_factor: float
    project_dist: float
    escape_dist: float
    #: EMA on the solved joints, stepped once per vision frame. None leaves it off.
    low_pass_alpha: Optional[float] = None
    #: Robot knuckle-to-fingertip length, filled in once the kinematics are loaded.
    robot_length: float = field(default=0.0)

    @classmethod
    def load(cls, side: str, urdf_path: Optional[Path] = None,
             config: Path = CONFIG) -> "HandSpec":
        raw = yaml.safe_load(config.read_text(encoding="utf-8"))
        if side not in ("left", "right"):
            raise ValueError(f"side must be left or right, got {side!r}")
        hand, common = raw[side], raw["common"]
        return cls(
            side=side,
            urdf_path=Path(urdf_path or REPO / "urdf" / f"allegro_hand_v6_{side}.urdf"),
            tip_links=hand["tip_links"],
            fingertips=hand["fingertips"],
            pinch_distance=float(hand["pinch_distance"]),
            base_link=common["base_link"],
            joint_order=list(common["joint_order"]),
            scaling_factor=float(common["scaling"]),
            project_dist=float(common["project_dist"]),
            escape_dist=float(common["escape_dist"]),
            low_pass_alpha=common.get("low_pass_alpha"),
        )


def urdf_with_fingertips(spec: HandSpec) -> Path:
    """The hand's URDF with a massless link fixed at each measured fingertip.

    The optimizer can only aim at links, so the points it should aim at have to be links.
    Nothing else changes: the added links have no mass, no geometry and fixed joints, so
    the kinematics and the joint order are the hand's own.

    Written to a cache keyed by the URDF's contents and these offsets, so it is rebuilt
    when either changes and shared between runs when they do not.
    """
    source = spec.urdf_path.read_bytes()
    key = hashlib.sha256(source + repr(sorted(spec.fingertips.items())).encode()).hexdigest()[:16]
    out = Path(tempfile.gettempdir()) / f"avt-retarget-{os.getuid()}" / f"{spec.side}-{key}.urdf"
    if out.exists():
        return out

    robot = ET.fromstring(source)
    present = {link.get("name") for link in robot.iter("link")}
    for finger, parent in spec.tip_links.items():
        if parent not in present:
            raise ValueError(f"{spec.urdf_path.name} has no link {parent!r} for the {finger}")
        ET.SubElement(robot, "link", {"name": f"{finger}_tip"})
        joint = ET.SubElement(robot, "joint",
                              {"name": f"{finger}_tip_joint", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": f"{finger}_tip"})
        ET.SubElement(joint, "origin",
                      {"xyz": " ".join(f"{v:.6f}" for v in spec.fingertips[finger]),
                       "rpy": "0 0 0"})

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(ET.tostring(robot, encoding="utf-8"))
    return out


class V6DexPilot:
    """DexPilot retargeting for one V6 hand: 21 landmarks in, 20 joint angles out."""

    def __init__(self, spec: HandSpec):
        from dex_retargeting.optimizer import DexPilotOptimizer
        from dex_retargeting.optimizer_utils import LPFilter
        from dex_retargeting.robot_wrapper import RobotWrapper
        from dex_retargeting.seq_retarget import SeqRetargeting

        self.spec = spec
        self.robot = RobotWrapper(str(urdf_with_fingertips(spec)))

        missing = [j for j in spec.joint_order if j not in self.robot.dof_joint_names]
        if missing:
            raise ValueError(f"{spec.urdf_path.name} has no joints {missing}")

        self.optimizer = DexPilotOptimizer(
            robot=self.robot,
            target_joint_names=list(spec.joint_order),
            finger_tip_link_names=[f"{f}_tip" for f in
                                   ("thumb", "index", "middle", "ring", "pinky")],
            wrist_link_name=spec.base_link,
            project_dist=spec.project_dist,
            escape_dist=spec.escape_dist,
            # Two fingertip centres are this far apart when the surfaces touch. Upstream's
            # default of 0.1 mm would ask them to occupy the same point.
            eta1=spec.pinch_distance,
            scaling=spec.scaling_factor,
        )
        # The filter runs once per vision frame, so its lag is (1 - a) / a frames: at a
        # 30 Hz camera, 100 ms at 0.25 and 22 ms at 0.6. It is the largest single delay in
        # the chain, which is why it is worth setting rather than leaving to a default.
        alpha = spec.low_pass_alpha
        self.retargeting = SeqRetargeting(
            self.optimizer, has_joint_limits=True,
            lp_filter=LPFilter(float(alpha)) if alpha else None)
        self.spec.robot_length = self._robot_finger_length()

    # ── what the retargeting needs from the robot ───────────────────────────
    def _robot_finger_length(self) -> float:
        """Knuckle to fingertip on the robot, over the same three fingers as the operator.

        The knuckle is where the flexion joint is -- Link02, the child of joint_1, not
        Link01, which hangs off the abduction joint 25 mm behind it. That is the joint the
        operator's own MCP landmark sits on, and measuring from the wrong one would scale
        the whole hand by the difference.
        """
        self.robot.compute_forward_kinematics(np.zeros(self.robot.dof))
        total = 0.0
        for finger in SCALE_FINGERS:
            chain = [self.spec.tip_links[finger].replace("_Link04", f"_Link0{i}")
                     for i in (2, 3, 4)] + [f"{finger}_tip"]
            points = [self.robot.get_link_pose(self.robot.get_link_index(name))[:3, 3]
                      for name in chain]
            total += float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        return total

    @property
    def joint_order(self) -> list[str]:
        return list(self.spec.joint_order)

    # ── running ─────────────────────────────────────────────────────────────
    def __call__(self, canonical: np.ndarray) -> np.ndarray:
        """21 landmarks in the hand's own frame, metres, to 20 joint angles."""
        pts = np.asarray(canonical, dtype=np.float64).reshape(21, 3)
        origin, task = self.optimizer.target_link_human_indices
        return np.asarray(self.retargeting.retarget(pts[task] - pts[origin]),
                          dtype=np.float64)

    def reset(self) -> None:
        """Forget the last solution.

        Each frame starts from the one before, which is what keeps the hand steady. After
        a jump -- the tracking dropping and picking the hand up elsewhere -- that start is
        worse than useless and the solver stays in a pose the operator has left.
        """
        self.retargeting.reset()

    def set_scaling(self, scaling: float) -> None:
        self.spec.scaling_factor = float(scaling)
        self.optimizer.scaling = float(scaling)

    def calibrate(self, frames: np.ndarray) -> float:
        """Measure this operator's hand against the robot's, and return the scale.

        Finger length rather than a pose, so it does not matter what the operator's hand
        was doing while it was measured. Frames where tracking lost the hand come through
        as implausible lengths and are left out.
        """
        lengths = np.array([hand_length(f) for f in np.asarray(frames).reshape(-1, 21, 3)])
        usable = lengths[(lengths > 0.05) & (lengths < 0.5)]
        if usable.size < 5:
            raise ValueError(f"only {usable.size} usable frames to measure the hand from")
        self.set_scaling(self.spec.robot_length / float(np.median(usable)))
        return self.spec.scaling_factor

    @classmethod
    def load(cls, side: str, urdf_path: Optional[Path] = None,
             config: Path = CONFIG) -> "V6DexPilot":
        return cls(HandSpec.load(side, urdf_path, config))
