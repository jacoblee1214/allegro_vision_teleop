#!/usr/bin/env python3
"""
measure_fingertips.py — where each fingertip is and how round it is, from the meshes.

The retargeting matches the robot's fingertips to the operator's, so "fingertip" has to
mean the same thing on both sides, and the distal link's own origin is no use: it sits at
the joint, some 50 mm short of the end. Aim the optimizer there and it tries to overlap
two joints, which is why a pinch that closes on the operator's hand leaves the robot's
fingers apart.

The end of a V6 finger is a rounded cap, so this fits a sphere to it and takes:

* **the centre** as the fingertip -- the centre of the fingertip's curvature, which is
  also roughly where MediaPipe puts its own tip landmark, inside the finger rather than
  on the skin;
* **the radius**, because it says when two fingertips touch: their centres are then one
  radius plus the other apart. That sum is `eta1`, the distance the optimizer closes a
  pinch to, so it is measured here rather than guessed.

Fitting a sphere rather than averaging an end slice is what makes this stable: the centre
of an end slice slides along the finger as the slice gets thicker, while the fitted centre
moves about a millimetre over the same range and the sideways components barely move.

    ./tools/measure_fingertips.py                  # both hands, into retarget/v6.yaml form
    ./tools/measure_fingertips.py --hand left
    ./tools/measure_fingertips.py --cap 0.012      # fit over a deeper cap

Numbers go in retarget/v6.yaml. Re-run when the meshes change.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from teleop_cockpit_v2 import fast_load_stl  # noqa: E402

FINGERS = {"thumb": 1, "index": 2, "middle": 3, "ring": 4, "pinky": 5}
DEFAULT_CAP = 0.008      # metres of the far end the sphere is fitted to


def fit_sphere(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Least squares sphere through the points: |p - c|^2 = r^2, linear in (c, r^2 - |c|^2)."""
    a = np.hstack([2.0 * points, np.ones((len(points), 1))])
    solution = np.linalg.lstsq(a, (points ** 2).sum(axis=1), rcond=None)[0]
    centre = solution[:3]
    return centre, float(np.sqrt(solution[3] + centre @ centre))


def fingertip(mesh_path: Path, cap: float) -> tuple[np.ndarray, float]:
    verts = fast_load_stl(mesh_path)[0].astype(np.float64)
    # The cap is the vertices furthest from the link's own origin, which is the joint.
    # Distance, not a chosen axis: a link frame need not line up with its finger.
    radius = np.linalg.norm(verts, axis=1)
    cap_verts = np.unique(verts[radius > radius.max() - cap], axis=0)
    return fit_sphere(cap_verts)


def main() -> None:
    ap = argparse.ArgumentParser(description="fingertip centres and radii, from the meshes")
    ap.add_argument("--hand", choices=["right", "left", "both"], default="both")
    ap.add_argument("--cap", type=float, default=DEFAULT_CAP,
                    help=f"metres of the far end fitted (default {DEFAULT_CAP})")
    ap.add_argument("--mesh-dir", default=str(REPO / "meshes"))
    args = ap.parse_args()

    for side in (("right", "left") if args.hand == "both" else (args.hand,)):
        meshes = Path(args.mesh_dir) / ("left" if side == "left" else "")
        radii = {}
        print(f"\n  # {side}, measured from {meshes} with --cap {args.cap}")
        print("  fingertips:")
        for finger, k in FINGERS.items():
            centre, r = fingertip(meshes / f"Finger0{k}_Link04.STL", args.cap)
            radii[finger] = r
            print(f"    {finger}: [{centre[0]:+.5f}, {centre[1]:+.5f}, {centre[2]:+.5f}]"
                  f"   # {np.linalg.norm(centre) * 1e3:.1f} mm out, {r * 1e3:.1f} mm radius")
        others = [r for f, r in radii.items() if f != "thumb"]
        print(f"  pinch_distance: {radii['thumb'] + float(np.mean(others)):.4f}"
              f"   # thumb {radii['thumb'] * 1e3:.1f} mm + finger {np.mean(others) * 1e3:.1f} mm,"
              f" the gap at which two fingertips touch")


if __name__ == "__main__":
    main()
