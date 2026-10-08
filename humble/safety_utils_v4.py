"""
safety_utils.py — Allegro Hand v4 joint mapping, limits, and safety primitives.

Joint naming sources:
  - URDF: /home/humble_ws/src/allegro_hand_hardwares/v4/description/urdf/allegro_hand_description_right.xacro
  - SDK order: v4_hardware_interface.cpp sdk_ordered_joint_base_names
  - readme: /home/humble_ws/src/allegro_hand_hardwares/v4/hardware/readme.md

Key verified facts:
  HW URDF: joint00~03 = Thumb, joint10~13 = Index(ff), joint20~23 = Middle(mf), joint30~33 = Ring(rf)
  SDK internal order: [Index(0~3), Middle(4~7), Ring(8~11), Thumb(12~15)]
    → matches sim flat order (CLAUDE.md: ff=0~3, mf=4~7, rf=8~11, th=12~15) exactly.
  ROS2 controller command order (yaml): [Thumb(0~3), Index(4~7), Middle(8~11), Ring(12~15)]
    → does NOT match sim flat order. Use sim_flat_to_cmd_idx() to convert.
  Joint limits: HW URDF == sim MuJoCo right_hand.xml (verified identical).
"""
from __future__ import annotations

import numpy as np
from typing import NamedTuple

# ─── Controller command order (from ros2_controllers.yaml joints list) ────────
# Index into a 16-dim command vector sent to ForwardCommandController.
CONTROLLER_JOINT_ORDER: list[str] = [
    "ah_joint00", "ah_joint01", "ah_joint02", "ah_joint03",  # Thumb  (sim 12~15)
    "ah_joint10", "ah_joint11", "ah_joint12", "ah_joint13",  # Index  (sim 0~3)
    "ah_joint20", "ah_joint21", "ah_joint22", "ah_joint23",  # Middle (sim 4~7)
    "ah_joint30", "ah_joint31", "ah_joint32", "ah_joint33",  # Ring   (sim 8~11)
]
N_JOINTS: int = 16

# ─── Joint limits (rad) ────────────────────────────────────────────────────────
# HW URDF == sim MuJoCo limits (verified identical).
# Keys are HW joint names (ah_joint{N}{M}).
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    # Thumb (joint00~03)
    "ah_joint00": ( 0.263,  1.396),
    "ah_joint01": (-0.105,  1.163),
    "ah_joint02": (-0.189,  1.644),
    "ah_joint03": (-0.162,  1.719),
    # Index / ff (joint10~13)
    "ah_joint10": (-0.470,  0.470),
    "ah_joint11": (-0.196,  1.610),
    "ah_joint12": (-0.174,  1.709),
    "ah_joint13": (-0.227,  1.618),
    # Middle / mf (joint20~23)
    "ah_joint20": (-0.470,  0.470),
    "ah_joint21": (-0.196,  1.610),
    "ah_joint22": (-0.174,  1.709),
    "ah_joint23": (-0.227,  1.618),
    # Ring / rf (joint30~33)
    "ah_joint30": (-0.470,  0.470),
    "ah_joint31": (-0.196,  1.610),
    "ah_joint32": (-0.174,  1.709),
    "ah_joint33": (-0.227,  1.618),
}

# ─── Safety thresholds ─────────────────────────────────────────────────────────
TAU_MAX_NM: float = 0.7           # Nm hard limit per joint
JOINT_LIMIT_MARGIN: float = 0.05  # 5% of range → zero-torque cutoff near limit
TEMP_WARN_C: float = 60.0         # °C — log warning, continue
TEMP_STOP_C: float = 70.0         # °C — emergency stop (tentative; verify with Wonik)


# ─── Mapping functions ─────────────────────────────────────────────────────────

# N-digit to sim flat base: N=0(Thumb)→12, N=1(Index)→0, N=2(Middle)→4, N=3(Ring)→8
_N_TO_FLAT_BASE: dict[int, int] = {0: 12, 1: 0, 2: 4, 3: 8}


def hw_name_to_sim_flat(hw_name: str) -> int:
    """Convert 'ah_joint{N}{M}' → sim flat index (0~15)."""
    base = hw_name.removeprefix("ah_")
    assert base.startswith("joint") and len(base) == 7, f"Bad joint name: {hw_name!r}"
    N, M = int(base[5]), int(base[6])
    assert N in _N_TO_FLAT_BASE and 0 <= M <= 3, f"Out-of-range N={N} M={M}"
    return _N_TO_FLAT_BASE[N] + M


def sim_flat_to_hw_name(flat: int) -> str:
    """Convert sim flat index (0~15) → 'ah_joint{N}{M}'."""
    assert 0 <= flat <= 15, f"flat out of range: {flat}"
    if flat < 4:    N, M = 1, flat       # Index
    elif flat < 8:  N, M = 2, flat - 4  # Middle
    elif flat < 12: N, M = 3, flat - 8  # Ring
    else:           N, M = 0, flat - 12  # Thumb
    return f"ah_joint{N}{M}"


# Pre-computed: sim flat index → position in CONTROLLER_JOINT_ORDER command vector
_CTRL_ORDER_INDEX: dict[str, int] = {name: i for i, name in enumerate(CONTROLLER_JOINT_ORDER)}

def sim_flat_to_cmd_idx(flat: int) -> int:
    """Convert sim flat index → index in 16-dim ForwardCommandController command vector."""
    return _CTRL_ORDER_INDEX[sim_flat_to_hw_name(flat)]


def cmd_idx_to_sim_flat(cmd_idx: int) -> int:
    """Convert ForwardCommandController command vector index → sim flat index."""
    return hw_name_to_sim_flat(CONTROLLER_JOINT_ORDER[cmd_idx])


# ─── Safety primitives ─────────────────────────────────────────────────────────

def clamp_torques(tau: np.ndarray) -> tuple[np.ndarray, bool]:
    """Clamp (16,) torque vector to [-TAU_MAX_NM, TAU_MAX_NM]. Returns (clamped, did_clamp)."""
    clamped = np.clip(tau, -TAU_MAX_NM, TAU_MAX_NM)
    return clamped, bool(np.any(np.abs(tau) > TAU_MAX_NM))


def apply_joint_limit_cutoff(
    tau: np.ndarray,
    positions: dict[str, float],
) -> np.ndarray:
    """
    Zero torque on joints within JOINT_LIMIT_MARGIN (5%) of their limit.
    tau: (16,) in CONTROLLER_JOINT_ORDER.
    positions: {hw_joint_name: position_rad} from JointState.
    Returns modified tau.
    """
    tau = tau.copy()
    for i, name in enumerate(CONTROLLER_JOINT_ORDER):
        lo, hi = JOINT_LIMITS[name]
        margin = (hi - lo) * JOINT_LIMIT_MARGIN
        pos = positions.get(name)
        if pos is None:
            continue
        if pos <= lo + margin or pos >= hi - margin:
            tau[i] = 0.0
    return tau


class TempStatus(NamedTuple):
    warn: bool
    stop: bool
    hot_joints: list[str]


def check_temperatures(temps: dict[str, float]) -> TempStatus:
    """
    Check per-joint temperatures.
    Returns TempStatus(warn, stop, hot_joints).
    stop=True if any joint >= TEMP_STOP_C.
    warn=True if any joint >= TEMP_WARN_C (but no stop).
    """
    hot: list[str] = []
    stop = False
    for name, t in temps.items():
        if t >= TEMP_STOP_C:
            hot.append(f"{name}={t:.1f}°C")
            stop = True
        elif t >= TEMP_WARN_C:
            hot.append(f"{name}={t:.1f}°C")
    warn = bool(hot) and not stop
    return TempStatus(warn=warn, stop=stop, hot_joints=hot)


def safe_command(
    tau_raw: np.ndarray,
    positions: dict[str, float],
    temps: dict[str, float],
) -> tuple[np.ndarray, dict]:
    """
    Full safety pipeline: clamp → limit cutoff → temp check.
    Returns (safe_tau, info_dict).
    If temp stop triggered, safe_tau is all zeros.
    """
    info: dict = {}
    tau, clamped = clamp_torques(tau_raw)
    info["clamped"] = clamped
    tau = apply_joint_limit_cutoff(tau, positions)
    ts = check_temperatures(temps)
    info["temp_warn"] = ts.warn
    info["temp_stop"] = ts.stop
    info["hot_joints"] = ts.hot_joints
    if ts.stop:
        tau = np.zeros(N_JOINTS)
    return tau, info
