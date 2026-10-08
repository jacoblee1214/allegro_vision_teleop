"""
tactile_panel.py — Allegro Hand V6 tactile pressure display (Wonik sensor-visualizer style).

The V6 driver publishes 18 absolute pressures in hPa on `<hardware node>/tactile_pressures`:

    0  1  2   thumb   tip, middle phalanx, proximal phalanx
    3  4  5   index   (same order)
    6  7  8   middle
    9 10 11   ring
   12 13 14   pinky
   15 16 17   palm    upper pad on the index side, upper pad on the pinky side, lower pad

This layout was measured on the right hand on 2026-10-08 by pressing each pad in turn; the
circle positions are in `assets/v6_tactile_layout.json` (the left hand uses mirrored positions).
On that right hand channel 14 (pinky, proximal) reads 0.

At rest each sensor reads the atmosphere (about 1013-1024 hPa, a little different per sensor),
so the panel shows **contact pressure**: the reading minus a per-channel baseline taken while
nothing touches the hand. Press T (or the Tare button) to take it again.

A channel that reads below 500 hPa is not measuring anything (a broken sensor or cable reads 0)
and is drawn grey instead of as zero pressure.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import QWidget

from std_msgs.msg import Float32MultiArray, Float64MultiArray

ASSETS = Path(__file__).resolve().parent / "assets"
N_CHANNELS = 18
DEAD_BELOW_HPA = 500.0       # a live sensor never reads this low: the atmosphere alone is ~1013
BASELINE_SAMPLES = 20        # readings averaged (median) into the resting baseline
FULL_SCALE_KPA = 40.0        # colour saturates here; a hard press reaches ~50 kPa (1500 hPa)
CONTACT_KPA = 1.0            # below this the circle stays dim (sensor noise is about ±0.1 kPa)
_TYPES = {
    "std_msgs/msg/Float64MultiArray": Float64MultiArray,
    "std_msgs/msg/Float32MultiArray": Float32MultiArray,
}


class TactileSource:
    """Finds the tactile topic, subscribes with whatever type it has, and keeps a baseline.

    The topic name depends on the hardware node's name, so it is discovered instead of
    hard-coded: any topic ending in ``tactile_pressures``. Pass ``topic`` to pin one.
    ROS callbacks arrive on the executor thread and the panel reads from the Qt thread,
    hence the lock.
    """

    def __init__(self, node, topic: Optional[str] = None):
        self.node = node
        self.wanted = topic
        self.topic: Optional[str] = None
        self._sub = None
        self._lock = threading.Lock()
        self._raw: Optional[np.ndarray] = None
        self._baseline: Optional[np.ndarray] = None
        self._samples: list[np.ndarray] = []
        self._stamp = 0.0
        self._count = 0
        self._rate = 0.0
        self._rate_t0 = time.monotonic()
        node.create_timer(1.0, self._discover)

    def _discover(self) -> None:
        if self._sub is not None:
            return
        for name, types in self.node.get_topic_names_and_types():
            if self.wanted and name != self.wanted:
                continue
            if not self.wanted and not name.endswith("tactile_pressures"):
                continue
            msg_type = next((_TYPES[t] for t in types if t in _TYPES), None)
            if msg_type is None:
                continue
            self._sub = self.node.create_subscription(msg_type, name, self._cb, 10)
            self.topic = name
            self.node.get_logger().info(f"Tactile pressures: subscribed to {name} ({types[0]})")
            return

    def _cb(self, msg) -> None:
        data = np.asarray(msg.data, dtype=float)
        if data.size != N_CHANNELS:
            return
        now = time.monotonic()
        with self._lock:
            self._raw = data
            self._stamp = now
            self._count += 1
            if now - self._rate_t0 >= 1.0:
                self._rate = self._count / (now - self._rate_t0)
                self._count, self._rate_t0 = 0, now
            if self._baseline is None:
                self._samples.append(data)
                if len(self._samples) >= BASELINE_SAMPLES:
                    self._baseline = np.median(np.stack(self._samples), axis=0)
                    self._samples = []

    def tare(self) -> None:
        """Take the resting baseline again. Do it with nothing touching the hand."""
        with self._lock:
            self._baseline = None
            self._samples = []

    def snapshot(self):
        """(contact kPa or None, dead mask, age in s, rate Hz, calibrating?)"""
        with self._lock:
            raw = None if self._raw is None else self._raw.copy()
            base = None if self._baseline is None else self._baseline.copy()
            age = time.monotonic() - self._stamp if self._stamp else float("inf")
            rate = self._rate
        if raw is None:
            return None, None, age, rate, False
        dead = raw < DEAD_BELOW_HPA
        if base is None:
            return np.zeros(N_CHANNELS), dead, age, rate, True
        kpa = np.clip((raw - base) / 10.0, 0.0, None)   # hPa -> kPa, contact only
        return kpa, dead, age, rate, False


def _heat(frac: float) -> QColor:
    """Wonik blue at rest -> cyan -> yellow -> red at full scale."""
    stops = [(0.0, (30, 90, 200)), (0.35, (0, 200, 230)), (0.7, (250, 210, 40)), (1.0, (240, 50, 40))]
    frac = float(np.clip(frac, 0.0, 1.0))
    for (f0, c0), (f1, c1) in zip(stops, stops[1:]):
        if frac <= f1:
            t = (frac - f0) / (f1 - f0)
            return QColor(*[int(a + (b - a) * t) for a, b in zip(c0, c1)])
    return QColor(*stops[-1][1])


class TactilePanel(QWidget):
    """The rendered V6 palm on black, with one circle per sensor coloured by contact pressure."""

    def __init__(self, source: TactileSource, hand_side: str = "right", parent=None):
        super().__init__(parent)
        self.source = source
        self.setMinimumSize(300, 340)
        self._layout = json.loads((ASSETS / "v6_tactile_layout.json").read_text(encoding="utf-8"))["layout"]
        self._images = {side: QPixmap(str(ASSETS / f"v6_tactile_bg_{side}.png")) for side in ("right", "left")}
        self.hand_side = hand_side
        self.show_channel_numbers = False

    def set_hand_side(self, side: str) -> None:
        self.hand_side = side
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(0, 0, 0))

        img = self._images.get(self.hand_side)
        if img is None or img.isNull():
            p.setPen(QColor(200, 200, 200))
            p.drawText(self.rect(), Qt.AlignCenter, "assets/v6_tactile_bg_*.png not found")
            return

        # The assets are already cropped to the hand (tools/render_tactile_bg.py + crop).
        src = QRectF(0, 0, img.width(), img.height())
        avail_w = self.width() - 40           # keep the colour scale on the right clear of the hand
        scale = min(avail_w / src.width(), self.height() / src.height())
        dw, dh = src.width() * scale, src.height() * scale
        dst = QRectF((avail_w - dw) / 2, (self.height() - dh) / 2, dw, dh)
        p.setOpacity(0.55)                    # dim the hand so the readings stand out
        p.drawPixmap(dst, img, src)
        p.setOpacity(1.0)

        kpa, dead, age, rate, calibrating = self.source.snapshot()
        stale = age > 1.0
        radius = max(8.0, 0.034 * dw)
        font = QFont("DejaVu Sans", max(7, int(radius * 0.62)))
        font.setBold(True)
        p.setFont(font)

        for s in self._layout[self.hand_side]:
            ch = s["channel"]
            x = dst.left() + s["u"] * img.width() * scale
            y = dst.top() + s["v"] * img.height() * scale
            if y > dst.bottom():
                continue
            center = QPointF(x, y)
            if kpa is None or stale:
                color, label = QColor(70, 70, 80), ""
            elif dead[ch]:
                color, label = QColor(90, 90, 90), "--"
            else:
                v = kpa[ch]
                color = _heat(v / FULL_SCALE_KPA) if v >= CONTACT_KPA else QColor(30, 90, 200, 140)
                label = f"{v:.1f}" if v >= CONTACT_KPA else ""
            r = radius * (1.0 + 0.6 * min(1.0, (kpa[ch] / FULL_SCALE_KPA) if (kpa is not None and not stale and not dead[ch]) else 0.0))
            p.setPen(QPen(QColor(255, 255, 255, 160), 1.5))
            p.setBrush(QBrush(color))
            p.drawEllipse(center, r, r)
            text = str(ch) if self.show_channel_numbers else label
            if text:
                p.setPen(QColor(255, 255, 255))
                p.drawText(QRectF(x - 3 * r, y - r, 6 * r, 2 * r), Qt.AlignCenter, text)

        # Status line
        p.setFont(QFont("DejaVu Sans", 9))
        if self.source.topic is None:
            status, color = "센서 토픽 대기 중 (…/tactile_pressures)", QColor(250, 180, 80)
        elif stale:
            status, color = f"수신 끊김 ({self.source.topic})", QColor(240, 90, 90)
        elif calibrating:
            status, color = "기준값 측정 중 — 손에 아무것도 닿지 않게 유지", QColor(250, 210, 40)
        else:
            n_dead = int(np.count_nonzero(dead))
            status = f"{rate:.0f} Hz · 단위 kPa (기준값 대비)" + (f" · 신호 없는 채널 {n_dead}개" if n_dead else "")
            color = QColor(150, 170, 200)
        p.setPen(color)
        p.drawText(QRectF(8, self.height() - 22, self.width() - 16, 18), Qt.AlignLeft | Qt.AlignVCenter, status)

        # Colour scale
        bar = QRectF(self.width() - 26, 30, 12, self.height() * 0.45)
        for i in range(int(bar.height())):
            p.setPen(_heat(1.0 - i / bar.height()))
            p.drawLine(QPointF(bar.left(), bar.top() + i), QPointF(bar.right(), bar.top() + i))
        p.setPen(QColor(200, 200, 210))
        p.setFont(QFont("DejaVu Sans", 8))
        p.drawText(QRectF(bar.left() - 34, bar.top() - 16, 50, 14), Qt.AlignRight, f"{FULL_SCALE_KPA:.0f}")
        p.drawText(QRectF(bar.left() - 34, bar.bottom() + 2, 50, 14), Qt.AlignRight, "0")
        p.end()
