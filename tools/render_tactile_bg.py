"""V6 손바닥 정면 렌더 (압력 패널 배경).

assets/v6_tactile_bg_{right,left}.png 를 다시 만들 때 쓴다. 센서 좌표(assets/v6_tactile_layout.json)는
실측으로 정한 값이라 이 스크립트가 만들지 않는다. 시점·크롭을 바꾸면 그 좌표도 다시 맞춰야 한다.
Humble 쪽 콕핏 렌더러(teleop_cockpit_v6_3.py)를 GPU 로 돌려 그린 뒤, 결과를 손 영역으로 잘라 쓴다.
경로는 환경변수로 바꾼다: COCKPIT_DIR (teleop_cockpit_v6_3.py 위치), OUT_DIR (출력 위치).
"""
import os
import sys, numpy as np
sys.path.insert(0, os.environ.get("COCKPIT_DIR", "/home/humble_ws/allegro_vision_teleop"))
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import teleop_cockpit_v6_3 as ck

class _Meta(type):
    def __getattr__(cls, name): return 0          # QPainter.Antialiasing 같은 클래스 상수
class _NoPainter(metaclass=_Meta):    # HUD 글자를 빼기 위해 QPainter 를 무력화
    def __init__(self, *a, **k): pass
    def __getattr__(self, name): return lambda *a, **k: None
ck.QPainter = _NoPainter

W, H = 900, 1000
OUT_DIR = os.environ.get("OUT_DIR", ".")

DIST = 0.74
for side, az, pan_x in (("right", 0.0, 0.035), ("left", 180.0, 0.035)):
    PAN = [pan_x, 0.0, 0.10]
    w = ck.RobotHand3DWidget(hand_side=side)
    w._draw_grid = lambda: None
    w.resize(W, H); w.show(); app.processEvents()
    w.azimuth, w.elevation, w.distance, w.pan = az, 0.0, DIST, list(PAN)
    # 시야각을 30도로 좁혀 원근 왜곡을 줄인다 (resizeGL 의 투영을 덮어쓴다)
    import OpenGL.GL as gl
    w.makeCurrent()
    f = 1/np.tan(np.radians(30)/2); near, far = 0.02, 5.0
    P = np.array([[f/(W/H),0,0,0],[0,f,0,0],[0,0,(far+near)/(near-far),2*far*near/(near-far)],[0,0,-1,0]], dtype=np.float32)
    w._proj = P.T.copy()
    gl.glClearColor(0.0, 0.0, 0.0, 1.0)
    for _ in range(3): app.processEvents(); w.grabFramebuffer()
    img = w.grabFramebuffer(); img.save(os.path.join(OUT_DIR, f"v6_tactile_bg_{side}.png"))
    w.close()

print("렌더 완료. 손 영역으로 잘라 assets/ 에 둔다 (센서 좌표는 그대로 유지).")
