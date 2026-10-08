"""V6 손바닥 정면 렌더 + 18개 압력센서 화면 좌표 계산 (원익 센서 시각화 스타일 배경)."""
import sys, json, numpy as np, yaml
sys.path.insert(0, "/home/humble_ws/allegro_vision_teleop")
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
CFG_DIR = "/home/humble_ws/allegro_grasp_retarget/external/v6f_teleop/configs"
FINGERS = [("thumb", 1), ("index", 2), ("middle", 3), ("ring", 4), ("pinky", 5)]

def project(pts, az, el, d, pan, w, h, fov=30.0):
    az, el = np.radians(az), np.radians(el)
    c = np.array([pan[0] + d*np.cos(el)*np.sin(az), pan[1] - d*np.cos(el)*np.cos(az), pan[2] + d*np.sin(el)])
    fwd = np.array(pan) - c; fwd /= np.linalg.norm(fwd)
    side = np.cross(fwd, [0, 0, 1.0]); side /= np.linalg.norm(side); u = np.cross(side, fwd)
    R = np.eye(4); R[0,:3], R[1,:3], R[2,:3] = side, u, -fwd
    T = np.eye(4); T[:3,3] = -c
    f = 1/np.tan(np.radians(fov)/2); near, far = 0.02, 5.0
    P = np.array([[f/(w/h),0,0,0],[0,f,0,0],[0,0,(far+near)/(near-far),2*far*near/(near-far)],[0,0,-1,0]])
    out = []
    for p in pts:
        v = P @ R @ T @ np.append(p, 1.0); v = v[:3]/v[3]
        out.append(((v[0]+1)/2, (1-v[1])/2))          # 0~1 정규화 화면 좌표
    return out

layout = {}
DIST = 0.74
for side, az, pan_x in (("right", 0.0, 0.035), ("left", 180.0, 0.035)):
    TIP = yaml.safe_load(open(f"{CFG_DIR}/v6f_{side}_dexpilot.yml"))["fingertip_offsets"]   # 손별 패드 오프셋
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
    img = w.grabFramebuffer(); img.save(f"/home/humble_ws/tactile_assets_tmp/v6_tactile_bg_{side}.png")

    model = w._urdf_model; tf = model.compute_link_transforms({})
    def centroid(link):
        v = model.link_meshes[link][0].reshape(-1, 3).astype(float)
        T = tf[link]; c = v.mean(axis=0)
        return T[:3,:3] @ c + T[:3,3]
    pts, names = [], []
    for name, k in FINGERS:
        link, xyz = TIP[name][0], np.asarray(TIP[name][1:], float)
        T = tf[link]; pts.append(T[:3,:3] @ xyz + T[:3,3]); names.append(f"{name}_tip")      # ch 3k-3 (손끝)
        pts.append(centroid(f"Finger0{k}_Link03")); names.append(f"{name}_mid")               # ch 3k-2 (가운데 마디, 가정)
        pts.append(centroid(f"Finger0{k}_Link02")); names.append(f"{name}_base")              # ch 3k-1 (기저 마디, 가정)
    palm = model.link_meshes["base_link"][0].reshape(-1, 3).astype(float)
    lo, hi = palm.min(axis=0), palm.max(axis=0)
    for fx in (0.28, 0.5, 0.72):                       # 손바닥 3개 (위치 가정: 손바닥 위쪽 가로 배치)
        pts.append(np.array([lo[0] + fx*(hi[0]-lo[0]), 0.0, lo[2] + 0.62*(hi[2]-lo[2])]))
        names.append(f"palm_{len(names)-15}")
    uv = project(pts, az, 0.0, DIST, PAN, W, H)
    layout[side] = [{"channel": i, "name": n, "u": round(float(u), 4), "v": round(float(v), 4)} for i, (n, (u, v)) in enumerate(zip(names, uv))]
    w.close()

json.dump({"image_size": [W, H], "note": "ch=MANUS haptics.yml 순서. 손끝=각 손가락 첫 채널(확인됨), mid/base 순서와 손바닥 위치는 가정", "layout": layout},
          open("/home/humble_ws/tactile_assets_tmp/v6_tactile_layout.json", "w"), ensure_ascii=False, indent=1)
print("렌더 및 좌표 계산 완료")
