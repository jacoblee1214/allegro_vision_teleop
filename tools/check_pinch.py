#!/usr/bin/env python3
"""
check_pinch.py — does a pinch on the operator's hand close on the robot's?

That is the one thing the retargeting exists for and the one thing that cannot be read
off the code: whether the solved joint angles actually bring two fingertips together. v1
looked right on screen and left the robot's thumb 13 cm short, because nothing in it ever
asked where the fingertips ended up.

The check is a round trip. Pose the robot, read where its own knuckles and fingertips
land, shrink that to operator size, and hand those 21 points back to the retargeting as
if a camera had seen them. What comes out should put the fingertips back where they
started. No recorded data and no hand-drawn skeleton: the test hand is the robot's own,
so it is the right shape and the right chirality for whichever side is being checked.

    .venv-retarget/bin/python tools/check_pinch.py
    .venv-retarget/bin/python tools/check_pinch.py --hand left

Each row is one pose, from a closed pinch to an open hand. `error` is what the round trip
lost; a few millimetres is the solver's own tolerance, and a pinch is closed when the
surfaces are within a millimetre or two of touching.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from retarget import V6DexPilot, to_canonical  # noqa: E402

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
#: MANO's landmark numbers, per finger, from knuckle to fingertip.
LANDMARKS = {"thumb": (1, 2, 3, 4), "index": (5, 6, 7, 8), "middle": (9, 10, 11, 12),
             "ring": (13, 14, 15, 16), "pinky": (17, 18, 19, 20)}
#: How much smaller the operator's hand is than the robot's. Any value does: the
#: retargeting measures the ratio for itself, and this only has to be plausible.
OPERATOR_SCALE = 0.8


def robot_landmarks(model: V6DexPilot, qpos: np.ndarray) -> np.ndarray:
    """Where the robot's wrist, knuckles and fingertips are, as 21 MANO landmarks."""
    robot = model.robot
    full = np.zeros(robot.dof)
    for name, value in zip(model.joint_order, qpos):
        full[robot.dof_joint_names.index(name)] = value
    robot.compute_forward_kinematics(full)

    def at(link: str) -> np.ndarray:
        return robot.get_link_pose(robot.get_link_index(link))[:3, 3]

    points = np.zeros((21, 3))
    points[0] = at(model.spec.base_link)
    for finger in FINGERS:
        distal = model.spec.tip_links[finger]
        chain = [distal.replace("_Link04", f"_Link0{i}") for i in (2, 3, 4)] + [f"{finger}_tip"]
        for landmark, link in zip(LANDMARKS[finger], chain):
            points[landmark] = at(link)
    return points


def pinch_pose(model: V6DexPilot) -> np.ndarray:
    """A joint pose where the thumb and index fingertips meet.

    Found rather than written down: the V6's thumb reaches the index through a
    combination of all four of its joints, and a pose typed in by hand lands short.
    """
    from scipy.optimize import minimize

    robot = model.robot
    limits = {name: robot.joint_limits[i] for i, name in enumerate(robot.dof_joint_names)}
    moving = [j for j in model.joint_order if j.startswith(("joint0", "joint1"))]
    index = [model.joint_order.index(j) for j in moving]
    bounds = [tuple(limits[j]) for j in moving]

    def gap(values: np.ndarray) -> float:
        qpos = np.zeros(len(model.joint_order))
        qpos[index] = values
        return fingertip_gap(model, qpos)

    start = np.array([np.clip(0.0, lo, hi) for lo, hi in bounds])
    best = minimize(gap, start, method="L-BFGS-B", bounds=bounds)
    qpos = np.zeros(len(model.joint_order))
    qpos[index] = best.x
    return qpos


def fingertip_gap(model: V6DexPilot, qpos: np.ndarray) -> float:
    """How far apart the robot's thumb and index fingertips are, in this pose."""
    robot = model.robot
    full = np.zeros(robot.dof)
    for name, value in zip(model.joint_order, qpos):
        full[robot.dof_joint_names.index(name)] = value
    robot.compute_forward_kinematics(full)
    return float(np.linalg.norm(
        robot.get_link_pose(robot.get_link_index("thumb_tip"))[:3, 3]
        - robot.get_link_pose(robot.get_link_index("index_tip"))[:3, 3]))


def main() -> None:
    ap = argparse.ArgumentParser(description="round-trip check of the pinch")
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    args = ap.parse_args()

    model = V6DexPilot.load(args.hand)
    contact = model.spec.pinch_distance

    closed = pinch_pose(model)
    opened = np.zeros(len(model.joint_order))
    sample = robot_landmarks(model, opened)
    model.calibrate(np.stack([to_canonical(sample * OPERATOR_SCALE)] * 10))

    print(f"\n  {args.hand} hand, scale measured as {model.spec.scaling_factor:.3f}, "
          f"fingertips touch at {contact * 1e3:.1f} mm between centres\n")
    print(f"  {'pose':>13} {'asked for':>11} {'robot did':>11} {'error':>8} {'surfaces':>10}")
    for name, blend in (("pinch", 1.0), ("nearly closed", 0.85),
                        ("half open", 0.5), ("open hand", 0.0)):
        qpos = opened + blend * (closed - opened)
        asked = fingertip_gap(model, qpos)
        operator = to_canonical(robot_landmarks(model, qpos) * OPERATOR_SCALE)

        model.reset()
        for _ in range(15):               # let it settle, as it does frame to frame
            solved = model(operator)
        did = fingertip_gap(model, solved)
        print(f"  {name:>13} {asked * 1e3:8.1f} mm {did * 1e3:8.1f} mm "
              f"{(did - asked) * 1e3:+6.1f} mm {(did - contact) * 1e3:+7.1f} mm")
    print()


if __name__ == "__main__":
    main()
