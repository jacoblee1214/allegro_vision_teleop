"""
safety_utils.py — Allegro Hand V6 / 5-Finger Robot Hand joint mapping, limits, and safety primitives (20-DOF).

Joint naming sources & convention:
  - V6 URDF: joint00~03 = Thumb, joint10~13 = Index, joint20~23 = Middle, joint30~33 = Ring, joint40~43 = Pinky
  - Controller command order: [Thumb(0~3), Index(4~7), Middle(8~11), Ring(12~15), Pinky(16~19)]
  - Sim flat order: [Index(0~3), Middle(4~7), Ring(8~11), Pinky(12~15), Thumb(16~19)]
"""
from __future__ import annotations

import numpy as np
from typing import NamedTuple

# ─── Controller command order (from ros2_controllers.yaml joints list) ────────
# Index into a 20-dim command vector sent to ForwardCommandController / JointGroupPositionController.
CONTROLLER_JOINT_ORDER: list[str] = [
    "joint00", "joint01", "joint02", "joint03",  # Thumb (0~3)
    "joint10", "joint11", "joint12", "joint13",  # Index (4~7)
    "joint20", "joint21", "joint22", "joint23",  # Middle (8~11)
    "joint30", "joint31", "joint32", "joint33",  # Ring (12~15)
    "joint40", "joint41", "joint42", "joint43",  # Pinky (16~19)
]
N_JOINTS: int = 20

# ─── Joint limits (rad) ────────────────────────────────────────────────────────
# Allegro Hand V6 limits (exact URDF-verified from allegro_hand_v6_right.urdf)
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    # Thumb (joint00~03): joint00=abduction/opposition, joint01=elevation/inward swing, joint02/03=flexion
    "joint00": (-0.035,  1.658),
    "joint01": (-1.658,  1.658),
    "joint02": (-0.175,  1.309),
    "joint03": (-0.175,  1.396),
    # Index (joint10~13): joint10=abduction, joint11=MCP flexion, joint12=PIP, joint13=DIP
    "joint10": (-0.384,  1.309),
    "joint11": (-0.070,  1.571),
    "joint12": (-0.175,  1.396),
    "joint13": (-0.175,  1.396),
    # Middle (joint20~23)
    "joint20": (-1.135,  1.135),
    "joint21": (-0.070,  1.571),
    "joint22": (-0.175,  1.396),
    "joint23": (-0.175,  1.396),
    # Ring (joint30~33)
    "joint30": (-1.309,  0.384),
    "joint31": (-0.070,  1.571),
    "joint32": (-0.175,  1.396),
    "joint33": (-0.175,  1.396),
    # Pinky (joint40~43)
    "joint40": (-1.309,  0.436),
    "joint41": (-0.070,  1.571),
    "joint42": (-0.175,  1.396),
    "joint43": (-0.175,  1.396),
}
# Alias with "ah_" prefix for backwards compatibility
for _k, _v in list(JOINT_LIMITS.items()):
    JOINT_LIMITS[f"ah_{_k}"] = _v

# ─── Safety thresholds ─────────────────────────────────────────────────────────
TAU_MAX_NM: float = 0.7           # Nm hard limit per joint
JOINT_LIMIT_MARGIN: float = 0.05  # 5% of range → zero-torque cutoff near limit
TEMP_WARN_C: float = 60.0         # °C — log warning, continue
TEMP_STOP_C: float = 70.0         # °C — emergency stop


# ─── Mapping functions ─────────────────────────────────────────────────────────

# N-digit to sim flat base: N=0(Thumb)→16, N=1(Index)→0, N=2(Middle)→4, N=3(Ring)→8, N=4(Pinky)→12
_N_TO_FLAT_BASE: dict[int, int] = {0: 16, 1: 0, 2: 4, 3: 8, 4: 12}


def hw_name_to_sim_flat(hw_name: str) -> int:
    """Convert 'joint{N}{M}' or 'ah_joint{N}{M}' → sim flat index (0~19)."""
    base = hw_name.removeprefix("ah_")
    assert base.startswith("joint") and len(base) == 7, f"Bad joint name: {hw_name!r}"
    N, M = int(base[5]), int(base[6])
    assert N in _N_TO_FLAT_BASE and 0 <= M <= 3, f"Out-of-range N={N} M={M}"
    return _N_TO_FLAT_BASE[N] + M


def sim_flat_to_hw_name(flat: int, prefix: str = "") -> str:
    """Convert sim flat index (0~19) → '{prefix}joint{N}{M}'."""
    assert 0 <= flat < N_JOINTS, f"flat out of range: {flat}"
    if flat < 4:    N, M = 1, flat       # Index
    elif flat < 8:  N, M = 2, flat - 4  # Middle
    elif flat < 12: N, M = 3, flat - 8  # Ring
    elif flat < 16: N, M = 4, flat - 12 # Pinky
    else:           N, M = 0, flat - 16 # Thumb
    return f"{prefix}joint{N}{M}"


# Pre-computed: sim flat index → position in CONTROLLER_JOINT_ORDER command vector
_CTRL_ORDER_INDEX: dict[str, int] = {name: i for i, name in enumerate(CONTROLLER_JOINT_ORDER)}
for _name in list(CONTROLLER_JOINT_ORDER):
    _CTRL_ORDER_INDEX[f"ah_{_name}"] = _CTRL_ORDER_INDEX[_name]

def sim_flat_to_cmd_idx(flat: int) -> int:
    """Convert sim flat index → index in 20-dim ForwardCommandController command vector."""
    return _CTRL_ORDER_INDEX[sim_flat_to_hw_name(flat)]


def cmd_idx_to_sim_flat(cmd_idx: int) -> int:
    """Convert ForwardCommandController command vector index → sim flat index."""
    return hw_name_to_sim_flat(CONTROLLER_JOINT_ORDER[cmd_idx])


# ─── Safety primitives ─────────────────────────────────────────────────────────

def clamp_torques(tau: np.ndarray) -> tuple[np.ndarray, bool]:
    """Clamp (20,) torque vector to [-TAU_MAX_NM, TAU_MAX_NM]. Returns (clamped, did_clamp)."""
    clamped = np.clip(tau, -TAU_MAX_NM, TAU_MAX_NM)
    return clamped, bool(np.any(np.abs(tau) > TAU_MAX_NM))


def apply_joint_limit_cutoff(
    tau: np.ndarray,
    positions: dict[str, float],
) -> np.ndarray:
    """
    Zero torque on joints within JOINT_LIMIT_MARGIN (5%) of their limit.
    tau: (20,) in CONTROLLER_JOINT_ORDER.
    positions: {joint_name: position_rad} from JointState.
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
