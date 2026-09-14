# 5-Finger Robot Hand Vision Retargeting Upgrade Prompt & Handoff Specification

이 문서는 추후 **5지(5-Finger) 로봇 핸드(예: Wonik Allegro 5-finger, Shadow Hand, DEX-Hand 등 20-DOF 이상)**가 도입되었을 때, AI 어시스턴트에게 그대로 전달하여 현재 4지 시스템을 5지 시스템으로 즉시 확장/업그레이드할 수 있도록 작성된 **프롬프트 및 핸드오프 명세서**입니다.

---

## 📋 [AI에게 그대로 복사하여 전달할 프롬프트]

```markdown
<USER_REQUEST>
**[Role & Context]**
너는 현재 `/home/humble_ws/allegro_vision_teleop`에 구현된 4-DOF 손가락 기반의 '비전 리타기팅 원격 제어 시스템'을 **5지(5-Finger) 로봇 핸드 시스템**으로 확장 업그레이드해야 해.
기존 시스템의 아키텍처와 트러블슈팅 내역은 `allegro_vision_teleop/README.md` 및 `VISION_RETARGETING_REPORT.md`에 정리되어 있어.

**[5-Finger 업그레이드 요구사항]**

**Task 1: Vision Tracker 5지 전체 랜드마크 추출 (21개 포인트)**
- `vision_tracker.py`를 수정하여 기존에 버렸던 새끼손가락(17~20번) 랜드마크를 모두 포함할 것.
- MediaPipe의 전체 21개 3D 랜드마크(0~20번: 손목 0, 엄지 1~4, 검지 5~8, 중지 9~12, 약지 13~16, 새끼 17~20)를 추출.
- 총 21 * 3 = 63개의 float 값을 1차원 평탄화(Flatten)하여 `/allegro/vision/landmarks` (`Float32MultiArray`, 길이 63)로 발행할 것.
- OpenCV GUI 화면에서도 새끼손가락(17~20번)을 활성 색상(초록색)으로 정상 시각화할 것.

**Task 2: Kinematic Retargeting 5지 기구학 확장**
- `retargeting_node.py`를 수정하여 63차원 랜드마크 데이터를 (21, 3)으로 재구성(Reshape)할 것.
- 새끼손가락 관절 인덱스 `PINKY_INDICES = (17, 18, 19, 20)`를 정의하고, 검지/중지/약지와 동일하게 3D 뼈대 벡터 내적을 통해 Abduction, MCP, PIP, DIP 관절각을 계산할 것.
- 손바닥 좌표계 기준 벡터를 손목(0) -> 중지 MCP(9), 검지 MCP(5) -> 새끼 MCP(17)로 갱신하여 손바닥 평면 법선 벡터의 정확도를 높일 것.
- 5지 로봇 핸드의 관절 순서(예: Thumb 4 + Index 4 + Middle 4 + Ring 4 + Pinky 4 = 총 20 DOF)에 맞춰 배열을 구성하고 Joint Limit 클램핑을 적용할 것.
- `/allegro/target_joints` (`Float64MultiArray`, 길이 20)로 발행할 것.

**Task 3: Safety Utils & Controller Bridge 확장**
- `safety_utils.py`에 새끼손가락 관절(`ah_joint40~43` 등 로봇 규격에 맞춤) 명칭, 순서(`N_JOINTS = 20`), 관절 한계 범위를 등록할 것.
- `sim_bridge_node.py`가 20차원 관절 배열에 대해 16차원 대신 20차원 상태 배열로 Low-Pass EMA 필터링을 수행하여 `/allegro_hand_position_controller/commands`로 전달하도록 수정할 것.

**Task 4: 실행 및 검증**
- `./run_teleop.sh sim` 또는 `nodes` 모드로 실행하여 21개 랜드마크(63 floats)가 수신되고, 20개 관절 각도가 정상 변환 및 발행되는지 검증하고 결과를 보고해 줘.
</USER_REQUEST>
```

---

## 🛠 4지 $\rightarrow$ 5지 변환 기술 명세 (Technical Details)

### 1. 랜드마크 인덱스 매핑 변화

| 부위 | 4지 시스템 (기존) | 5지 시스템 (업그레이드) |
| :--- | :--- | :--- |
| **손목 (Wrist)** | 0 | 0 |
| **엄지 (Thumb)** | 1, 2, 3, 4 (CMC, MCP, IP, TIP) | 1, 2, 3, 4 (CMC, MCP, IP, TIP) |
| **검지 (Index)** | 5, 6, 7, 8 (MCP, PIP, DIP, TIP) | 5, 6, 7, 8 (MCP, PIP, DIP, TIP) |
| **중지 (Middle)** | 9, 10, 11, 12 (MCP, PIP, DIP, TIP) | 9, 10, 11, 12 (MCP, PIP, DIP, TIP) |
| **약지 (Ring)** | 13, 14, 15, 16 (MCP, PIP, DIP, TIP) | 13, 14, 15, 16 (MCP, PIP, DIP, TIP) |
| **새끼 (Pinky)** | **제외됨 (Discarded)** | **17, 18, 19, 20 (MCP, PIP, DIP, TIP)** |
| **총 토픽 차원** | **$17 \times 3 = 51$ floats** | **$21 \times 3 = 63$ floats** |
| **로봇 관절수** | **16 DOF** | **20 DOF (4 DOF $\times$ 5 fingers)** |

---

### 2. 주요 코드 수정 체크포인트

1. **`vision_tracker.py`**:
   - `NUM_POINTS = 21` ($21 \times 3 = 63$ floats)
   - 랜드마크 추출 루프: `range(NUM_POINTS)` 전체 순회
   - GUI 제외 색상 표시 로직 제거

2. **`retargeting_node.py`**:
   - `PINKY_INDICES = (17, 18, 19, 20)`
   - 손바닥 횡축 벡터: `palm_transverse = pts[17] - pts[5]` (검지 MCP에서 새끼 MCP)
   - `q_dict.update(self._compute_finger_joints(pts, PINKY_INDICES, "ah_joint4", ...))`

3. **`safety_utils.py` & `sim_bridge_node.py`**:
   - `CONTROLLER_JOINT_ORDER`: 새끼손가락 관절 4개 추가
   - `N_JOINTS = 20`
   - `JOINT_LIMITS`: 새끼손가락 4개 관절 한계 추가
   - EMA 필터 상태 배열 크기 20으로 자동 확장
