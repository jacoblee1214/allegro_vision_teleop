#!/usr/bin/env python3
"""
dexpilot_retarget.py — DexPilot retargeting for the vision teleop pipeline (v2).

Why this exists
---------------
The v1 retargeter (`retargeting_node.py`) is a hand-tuned geometric map: it reads
bone angles off the MediaPipe skeleton and writes them into the robot's joints
through per-joint gains, interpolation tables and pinch/fist "synergy" terms. It
looks right on screen because the *fingers* follow, but nothing in it ever asks
where the robot's fingertips actually end up, so a pinch that closes on the
operator's hand does not close on the robot's.

Measured on `v6f_manuse_teleoperation/V6_Teleoperation/data/human_99test4_right.npy`,
on the frames where the operator's thumb and index pads are 2.2-2.5 mm apart
(see `tools/check_pinch_v2.py`):

    operator gap    v1 geometric      this (DexPilot)
        2.2 mm         129.3 mm            6.0 mm
        2.5 mm         130.8 mm            5.9 mm

and the joint that makes the difference is the thumb's second one:

    thumb joint01   v1: -0.40 rad      DexPilot: -1.01 rad

v1 cannot reach it. `_compute_thumb_joints_right` builds joint01 by
`np.interp(proj_norm, [-0.2, 0.5], [0.10, -0.65])` and pushes it towards -0.55
during a pinch, so the thumb stops roughly 0.6 rad short of where the pinch
needs it -- exactly the "the thumb should curl further in and doesn't" symptom.

What this does instead
----------------------
Reuses the retargeting stack from the MANUS teleop project
(`v6f_manuse_teleoperation/V6_Teleoperation`) unchanged: DexPilot solves, per
frame, for the joint vector that puts the robot's *fingertip pads* on the
operator's fingertips, against that project's own V6 Force URDF -- which is
byte-identical to `urdf/allegro_hand_v6_right.urdf` here -- with its measured
pad offsets (the URDF's distal frames sit 40-56 mm behind the pad), its pinch
distance `eta1`, and its joint limits. Nothing is copied or re-tuned; the config
files there are the single source of those numbers.

Input
-----
DexPilot wants **metric** keypoints in GeoRT's canonical hand frame. The
producers publish two things:

  `/allegro/vision/world_landmarks`  MediaPipe world landmarks, metres.  Use these.
  `/allegro/vision/landmarks`        normalised image landmarks.  Fallback only:
                                     x is over image width and y over height, so
                                     the frame is squashed by H/W (0.75 at
                                     640x480) before anything else goes wrong,
                                     and z is a weak relative depth. Corrected
                                     for aspect here, scale recovered by
                                     calibration, but the depth stays poor.

The producers mirror the camera image (`cv2.flip(frame, 1)`) for the operator's
benefit, which hands MediaPipe a mirrored hand. `undo_mirror` negates x to put
the chirality back before anything reads it.

Usage:
    r = VisionDexPilotRetargeter(hand_side="right")
    q = r(landmarks_63, metric=True)      # -> (20,) rad, CONTROLLER_JOINT_ORDER
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

REPO = Path(__file__).resolve().parent

#: Landmark layout MediaPipe Hands emits, which is already the MANO 21-point
#: order DexPilot expects. No relabelling needed.
N_LANDMARKS = 21

#: Fingertip / knuckle pairs used for the chirality check (see `chirality_score`).
_TIPS = (8, 12, 16, 20)
_MCPS = (5, 9, 13, 17)


class DexPilotUnavailable(RuntimeError):
    """Raised when the retargeting core cannot be imported by this interpreter."""


def config_path(hand_side: str, root: Path | str | None = None) -> Path:
    """The hand's measurements and tuning. One file, both sides."""
    side = hand_side.lower()
    if side not in ("left", "right"):
        raise ValueError(f"hand_side must be 'left' or 'right', got {hand_side!r}")
    return Path(root or REPO) / "retarget" / "v6.yaml"


def _import_core(root: Path | str | None = None):
    """Import the retargeting core, with an error that says what to do."""
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    try:
        from retarget import HandSpec, V6DexPilot, to_canonical
    except ImportError as exc:                                  # noqa: PERF203
        venv = REPO / ".venv-retarget" / "bin" / "python"
        raise DexPilotUnavailable(
            f"{exc}.\n"
            f"The retargeting needs dex_retargeting and pinocchio, which want numpy 2.x, "
            f"while mediapipe pins numpy 1.x -- they cannot share one environment, so "
            f"this node runs in its own.\n"
            f"Run it with:  {venv}\n"
            f"./tools/setup.sh creates that environment; run_teleop_v3.sh uses it."
        ) from exc
    return V6DexPilot, HandSpec, to_canonical


def chirality_score(canonical: np.ndarray) -> float:
    """Positive for a right hand in the canonical frame, negative for a left one.

    The canonical frame is the robot's: x from the little finger towards the index, z
    along the fingers, y out of the back of the hand. The two hands differ in where the
    thumb sits -- in front of the palm on one side and behind it on the other -- so the
    thumb's y against the other knuckles' says which hand this is, whatever the fingers
    are doing. Measured over a recorded right hand: mean +0.040, positive on every frame.
    """
    pts = np.asarray(canonical, dtype=np.float64).reshape(N_LANDMARKS, 3)
    palm = float(np.mean(pts[list(_MCPS), 1]))
    return float(palm - np.mean(pts[[2, 3, 4], 1]))


class VisionDexPilotRetargeter:
    """MediaPipe 21 keypoints -> 20 joint angles, via the DexPilot solver.

    Args:
        hand_side: which robot hand, and so which config/URDF. Switchable live.
        root: where retarget/v6.yaml lives. Defaults to this repository.
        scaling: fixed operator/robot finger-length ratio. `None` measures it
            from the first `calibrate_frames` frames, which is what the
            normalised-landmark path needs (its unit is arbitrary) and what the
            world-landmark path wants anyway, since MediaPipe's metric scale is
            only approximate and differs per operator.
        calibrate_frames: frames to average the measurement over. 0 disables it.
        undo_mirror: negate x, to undo the producers' `cv2.flip(frame, 1)`.
        aspect: image height / width, for the normalised fallback path only.
        low_pass_alpha: override the config's output filter (0.25 in both
            configs). This is an EMA on the solved joints, `y += a * (x - y)`,
            stepped **once per vision frame**, so its lag is
            `(1 - a) / a` frames -- at this machine's 30 Hz camera that is
            100 ms at 0.25 and 22 ms at 0.6. It is the largest single delay in
            the chain, which is why it is worth a flag.
    """

    def __init__(
        self,
        hand_side: str = "right",
        root: Path | str | None = None,
        scaling: Optional[float] = None,
        calibrate_frames: int = 150,
        undo_mirror: bool = True,
        aspect: float = 480.0 / 640.0,
        low_pass_alpha: Optional[float] = None,
    ) -> None:
        self._Model, self._Spec, self._to_canonical = _import_core(root)
        self.root = Path(root).expanduser() if root else REPO
        self.scaling = scaling
        self.calibrate_frames = max(0, int(calibrate_frames))
        self.undo_mirror = bool(undo_mirror)
        self.aspect = float(aspect)
        self.low_pass_alpha = low_pass_alpha

        self._models: dict[str, object] = {}
        self._calib: list[np.ndarray] = []
        self._calib_left = 0
        self._calibrated = scaling is not None
        #: The scaling actually in force: the fixed one, or the measurement once
        #: it lands. Held here rather than only on the models, so a side built
        #: later (a hand switch after calibration) starts on the same number.
        self._effective_scaling = scaling
        self._chirality: list[float] = []
        #: Set once a measurement lands; `None` until then.
        self.last_calibration: Optional[float] = None
        self.calibration_note: Optional[str] = None
        self.hand_side = ""
        self.set_hand_side(hand_side)

    # ── model management ────────────────────────────────────────────────────

    def _model(self, side: str):
        if side not in self._models:
            cfg = config_path(side, self.root)
            if not cfg.is_file():
                raise DexPilotUnavailable(f"no hand measurements at {cfg}")
            # The filter has to be in place before the solver is assembled, because
            # SeqRetargeting takes it at construction; changing it afterwards would mean
            # rebuilding the whole thing.
            spec = self._Spec.load(side, config=cfg)
            if self.low_pass_alpha is not None:
                spec.low_pass_alpha = float(self.low_pass_alpha)
            model = self._Model(spec)
            if self._effective_scaling is not None:
                model.set_scaling(float(self._effective_scaling))
            self._models[side] = model
        return self._models[side]

    def set_hand_side(self, side: str) -> None:
        """Switch hands. Each side has its own config and its own URDF, so there
        is nothing to mirror -- unlike a GeoRT checkpoint, which is trained on
        one hand and has to be fed the other one's keypoints negated."""
        side = side.lower()
        if side == self.hand_side:
            return
        self.hand_side = side
        self.model = self._model(side)
        self.model.reset()
        # The operator did not change, so a measurement already taken still
        # holds -- `_apply_calibration` pushed it onto every model. Only arm a
        # fresh one if none has landed yet.
        if self._calibrated:
            self._calib_left = 0
        else:
            self._calib_left = self.calibrate_frames
            self._calib.clear()

    @property
    def joint_order(self) -> list[str]:
        return list(self.model.joint_order)

    @property
    def scaling_factor(self) -> float:
        return float(self.model.spec.scaling_factor)

    def reset(self) -> None:
        """Forget the warm start and the filter. Use it after a clutch release."""
        self.model.reset()

    def recalibrate(self, frames: Optional[int] = None) -> None:
        """Measure the scaling again over the next `frames` frames."""
        self._calib.clear()
        self._calib_left = self.calibrate_frames if frames is None else max(0, int(frames))

    # ── the frame ───────────────────────────────────────────────────────────

    def to_canonical(self, landmarks: Iterable[float] | np.ndarray, metric: bool = True) -> np.ndarray:
        """MediaPipe landmarks -> (21, 3) metric keypoints in the canonical frame."""
        pts = np.asarray(landmarks, dtype=np.float64).reshape(N_LANDMARKS, 3)
        if not metric:
            # Normalised landmarks: x is over image width, y over image height,
            # z over width like x. Undo the y squash so the frame is isotropic;
            # the leftover global scale is what calibration is for.
            pts = pts * (1.0, self.aspect, 1.0)
        if self.undo_mirror:
            pts = pts * (-1.0, 1.0, 1.0)
        return self._to_canonical(pts)

    def __call__(self, landmarks: Iterable[float] | np.ndarray, metric: bool = True) -> np.ndarray:
        """(63,) or (21, 3) landmarks in, (20,) joint angles out, in
        CONTROLLER_JOINT_ORDER (= the config's `target_joint_names`)."""
        canonical = self.to_canonical(landmarks, metric=metric)

        if self._calib_left:
            self._calib.append(canonical)
            self._calib_left -= 1
            if self._calib_left == 0:
                self._apply_calibration()

        self._chirality.append(chirality_score(canonical))
        if len(self._chirality) > 300:
            del self._chirality[:-300]

        return np.asarray(self.model(canonical), dtype=np.float64)

    def _apply_calibration(self) -> None:
        was = self.scaling_factor
        self.last_calibration = self.model.calibrate(np.stack(self._calib))
        self._calibrated = True
        self._effective_scaling = self.last_calibration
        self._calib.clear()
        self.model.reset()
        # One operator, one hand: the other side's model should use it too.
        for side, model in self._models.items():
            if side != self.hand_side:
                model.set_scaling(self.last_calibration)
        self.calibration_note = (
            f"scaling {self.last_calibration:.3f} measured from {self.calibrate_frames} "
            f"frames (the config said {was:.3f})"
        )

    def chirality_note(self) -> Optional[str]:
        """A warning if the operator's hand looks like the other one, else None.

        Averaged over the last few hundred frames, because a single flat-handed
        frame carries almost no chirality at all.
        """
        if len(self._chirality) < 120:
            return None
        score = float(np.mean(self._chirality))
        if abs(score) < 0.004:
            return None
        looks = "right" if score > 0 else "left"
        if looks == self.hand_side:
            return None
        return (
            f"the tracked hand looks like a {looks.upper()} hand (chirality "
            f"{score:+.3f}) but the {self.hand_side.upper()} model is loaded. "
            f"Either switch the hand in the UI, or flip --undo-mirror."
        )
