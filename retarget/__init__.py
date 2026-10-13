"""DexPilot retargeting for the Allegro Hand V6, on upstream `dex_retargeting` alone.

See model.py for what this adds to the upstream optimizer and why, and v6.yaml for the
measurements it needs. Nothing here is specific to vision: it takes 21 MANO landmarks,
which is what both MediaPipe and a glove produce.
"""
from .model import HandSpec, V6DexPilot, hand_length, to_canonical

__all__ = ["HandSpec", "V6DexPilot", "hand_length", "to_canonical"]
