#!/usr/bin/env python3
"""
check_pinch_v2.py — does the robot's pinch actually close?

Runs both retargeters over recorded human keypoints and measures, through the
URDF's forward kinematics, how far apart the robot's thumb and index **pads**
end up. That is the question the v1 retargeter never asks: it maps bone angles
to joint angles and the fingers follow on screen, but nothing checks where the
fingertips land, which is why a pinch that closes on the operator's hand leaves
the robot's thumb 13 cm away.

The pad points are not the URDF's distal link frames -- those sit 40-56 mm
behind the pad -- but the measured offsets in the MANUS project's DexPilot
config, the same points DexPilot retargets to.

Typical output (`human_99test4_right.npy`, right hand):

     frame  human[mm]   dex[mm]   geo[mm] | dexpilot thumb q00..03 | v1 thumb q00..03
      3105        2.2       6.0     129.3 | +0.48 -1.01 +0.84 +0.71 | +0.97 -0.40 +0.65 +1.07

Read the thumb's joint01: DexPilot asks for -1.01 rad, v1 for -0.40. v1 builds
that joint with `np.interp(proj_norm, [-0.2, 0.5], [0.10, -0.65])` and a pinch
target of -0.55, so it cannot reach -1.0 whatever the operator does.

Run it with the MANUS project's interpreter, which has dex_retargeting:

    $V6F_TELEOP_ROOT/.venv/bin/python tools/check_pinch_v2.py
"""
from __future__ import annotations

import argparse
import sys
import types
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from dexpilot_retarget import DEFAULT_V6F_ROOT, config_path, v6f_src_path  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--v6f-root", default=None, help="the MANUS teleop checkout")
    p.add_argument("--hand", default="right", choices=["left", "right"])
    p.add_argument("--data", default=None,
                   help="(N, 21, 3) .npy of canonical human keypoints "
                        "(default: the MANUS project's human_99test4_right.npy)")
    p.add_argument("--rows", type=int, default=5, help="pinched frames to show")
    args = p.parse_args()

    root = Path(args.v6f_root).expanduser() if args.v6f_root else DEFAULT_V6F_ROOT
    sys.path.insert(0, str(v6f_src_path(root)))
    try:
        from v6f_teleop.retarget.dexpilot import DexPilotModel
        from v6f_teleop.urdf_kin import Robot
    except ImportError as exc:
        print(f"[check_pinch_v2] {exc}\n"
              f"Run this with {root / '.venv' / 'bin' / 'python'}.", file=sys.stderr)
        return 2

    data = Path(args.data) if args.data else root / "data" / "human_99test4_right.npy"
    if not data.is_file():
        print(f"[check_pinch_v2] no keypoints at {data}", file=sys.stderr)
        return 2
    kp = np.load(data)
    if kp.ndim != 3 or kp.shape[1:] != (21, 3):
        print(f"[check_pinch_v2] expected (N, 21, 3), got {kp.shape}", file=sys.stderr)
        return 2
    if args.hand == "left":
        # The recordings are of a right hand; a left hand's canonical keypoints
        # are the right's with x negated.
        kp = kp * (-1.0, 1.0, 1.0)

    model = DexPilotModel.load(config_path(args.hand, root))
    robot = Robot.from_urdf(model.spec.urdf_path)
    order = model.joint_order

    def pad(q, finger):
        link, *xyz = model.spec.fingertip_offsets[finger]
        T = robot.link_transform(link, dict(zip(order, q)), base=model.spec.wrist_link_name)
        return T[:3, 3] + T[:3, :3] @ np.asarray(xyz, dtype=float)

    # v1's kinematics without ROS: its maths lives on the node class, so bind
    # the three methods onto a bare object rather than standing up a node.
    import retargeting_node as v1
    side = args.hand
    fake = types.SimpleNamespace(w_couple=0.25, couple_ratio=0.85, hand_side=side)
    for name in (f"_compute_finger_joints_{side}", f"_compute_thumb_joints_{side}",
                 f"_retarget_{side}_hand"):
        setattr(fake, name, v1.KinematicRetargetingNode.__dict__[name].__get__(fake))
    v1_retarget = getattr(fake, f"_retarget_{side}_hand")

    human = np.linalg.norm(kp[:, 4] - kp[:, 8], axis=1)
    closed = np.argsort(human)[:args.rows * 8:8]
    open_ = np.argsort(human)[len(human) // 2::max(1, len(human) // 8)][:4]

    print(f"data   : {data.name}  ({len(kp)} frames, {args.hand} hand)")
    print(f"urdf   : {model.spec.urdf_path}")
    print(f"scaling: {model.spec.scaling_factor:g}   eta1: {model.spec.eta1 * 1000:g} mm\n")
    print(f"{'frame':>6} {'human[mm]':>10} {'dex[mm]':>9} {'geo[mm]':>9} |"
          f" {'dexpilot thumb q00..03':^23} | {'v1 thumb q00..03':^23}")
    for i in list(closed) + list(open_):
        i = int(i)
        qd, qg = model(kp[i]), v1_retarget(kp[i])
        gd = np.linalg.norm(pad(qd, "thumb") - pad(qd, "index")) * 1000
        gg = np.linalg.norm(pad(qg, "thumb") - pad(qg, "index")) * 1000
        fmt = lambda q: " ".join(f"{v:+.2f}" for v in q[:4])   # noqa: E731
        print(f"{i:6d} {human[i] * 1000:10.1f} {gd:9.1f} {gg:9.1f} | {fmt(qd)} | {fmt(qg)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
