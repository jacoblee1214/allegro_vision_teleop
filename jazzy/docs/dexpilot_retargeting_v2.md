# v2 리타게팅 — 화면에서만 되던 핀칭을 실제로 되게

## 증상

비전 텔레옵에서 손가락을 집으면 화면(RViz·콕핏 3D)에서는 핀칭 자세가 나오는데
실물 V6 핸드는 집히지 않았습니다. **엄지가 안쪽으로 더 굽혀져야 하는데 거기까지
가지 않는 것**이 원인입니다.

## 원인

`retargeting_node.py`(v1)는 MediaPipe 뼈대에서 관절 각도를 읽어 게인·보간표·
핀치 시너지 항으로 로봇 관절값에 써 넣는 **기하학적 매핑**입니다. 손가락이
따라 움직이니 화면에서는 그럴듯하지만, **로봇 지문(fingertip pad)이 실제로
어디에 놓이는지는 한 번도 확인하지 않습니다.** 그래서 작업자 손에서 닫힌 핀치가
로봇에서는 닫히지 않습니다.

`tools/check_pinch_v2.py`가 URDF 순기구학으로 로봇 엄지·검지 **지문 사이 거리**를
직접 재 봅니다 (MANUS 프로젝트의 `human_99test4_right.npy`, 오른손):

```
 frame  human[mm]   dex[mm]   geo[mm] | dexpilot thumb q00..03  |    v1 thumb q00..03
  3105        2.2       6.0     129.3 | +0.48 -1.01 +0.84 +0.71 | +0.97 -0.40 +0.65 +1.07
  3090        2.5       5.9     130.8 | +0.49 -1.04 +0.85 +0.70 | +0.97 -0.40 +0.65 +1.03
```

작업자 지문이 2.2 mm까지 붙은 프레임에서 **v1은 로봇 지문을 129 mm 벌려 놓습니다.**

차이를 만드는 것은 엄지 두 번째 관절 `joint01` 하나입니다. 핀치에 필요한 값은
−1.01 rad인데 v1은 −0.40 rad까지만 갑니다. v1의 해당 코드는

```python
q01_raw = float(np.interp(proj_norm, [-0.2, 0.5], [0.10, -0.65]))   # 하한이 -0.65
...
q01 = (1.0 - pinch_factor * 0.75) * q01 + (pinch_factor * 0.75) * (-0.55)
```

이라서, 작업자가 무엇을 하든 −0.65 rad 아래로는 내려갈 수 없습니다. URDF 한계는
−1.658 rad이므로 하드웨어가 못 가는 것이 아니라 **매핑이 거기까지 명령하지 않는
것**입니다.

## v2가 하는 일

MANUS 텔레옵 프로젝트(`~/v6f_manuse_teleoperation/V6_Teleoperation`)의 리타게팅
코어를 **그대로** 씁니다. DexPilot이 매 프레임 최적화를 풀어, 로봇 지문이 작업자
지문 위치에 오도록 20개 관절값을 구합니다. 기하 매핑이 아니라 위치를 맞추는
문제이므로 "핀치가 닫히는가"가 목적함수 안에 들어 있습니다.

참조하는 것(복사하지 않고 그 경로를 그대로 읽습니다):

| 항목 | 경로 |
|---|---|
| 리타게팅 코어 | `src/v6f_teleop/retarget/` (DexPilot + 자체 확장) |
| 설정 | `src/v6f_teleop/configs/v6f_{right,left}_dexpilot.yml` |
| URDF | `urdf/v6_force/v6_force_v2/urdf/allegro_hand_v6_{right,left}.urdf` |
| 캐노니컬 프레임 | `src/v6f_teleop/hand/mano.py` |

> **URDF는 이 저장소의 `urdf/allegro_hand_v6_right.urdf`와 바이트 단위로 동일합니다**
> (왼손은 끝의 개행 하나만 다름). 즉 URDF가 틀렸던 것이 아니라, URDF가 허용하는
> 범위를 v1 매핑이 쓰지 않고 있었습니다.
>
> 설정 파일이 들고 있는 값들은 이 저장소에서 다시 튜닝하지 않습니다. 특히
> `fingertip_offsets`(distal 링크 원점이 지문보다 40~56 mm 뒤에 있어 필요한 보정),
> `eta1`(지문이 닿았다고 볼 거리, 6 mm), `joint_limit_overrides`(사람 MCP가 못 가는
> 벌림 범위 차단)는 MANUS 프로젝트에서 실측해 둔 값이고, 그쪽이 단일 출처입니다.

## 입력이 바뀌었습니다: 월드 랜드마크

DexPilot은 **미터 단위 3D 키포인트**가 필요합니다. v1 생산자들이 내보내던
`/allegro/vision/landmarks`는 MediaPipe의 **정규화 이미지 좌표**라서

- x는 영상 **폭**, y는 영상 **높이**로 나눈 값 → 640×480에서 y축이 0.75배로 눌려 있음
- z는 손목 기준의 약한 상대 깊이

라는 두 가지 문제가 있습니다. v2 생산자(`*_v2.py`)는 MediaPipe의
`multi_hand_world_landmarks`(미터)를 `/allegro/vision/world_landmarks`로 **추가**
발행합니다. 기존 토픽도 그대로 내보내므로 v1 노드와 다른 구독자들은 영향이
없습니다.

`retargeting_node_v2.py`는 월드 토픽이 살아 있으면 그쪽만 쓰고, v1 생산자만 떠
있으면 정규화 토픽으로 떨어지면서 경고를 한 번 남깁니다(그 경우 종횡비는
보정하지만 깊이 품질은 어쩔 수 없습니다).

## 좌우 반전과 스케일

- **반전**: 생산자들은 작업자를 위해 `cv2.flip(frame, 1)`로 영상을 좌우 반전합니다.
  그러면 MediaPipe가 보는 손의 좌우(chirality)가 뒤집히므로, v2는 x를 음수화해서
  되돌립니다(`--no-undo-mirror`로 끌 수 있음). 노드가 추적 중인 손의 chirality를
  수백 프레임 평균으로 감시해서, 불러온 손 모델과 어긋나면 경고합니다.
- **스케일**: DexPilot의 `scaling_factor`는 "로봇 손가락 길이 ÷ 작업자 손가락 길이"
  (관절~지문) 하나입니다. MediaPipe 월드 랜드마크의 미터 스케일은 근사값이고
  사람마다 오차가 다르므로, 기본값은 **세션 시작 후 150프레임으로 직접 측정**
  입니다. 뼈 길이의 합으로 재기 때문에 작업자가 특정 자세를 취할 필요는 없습니다.
  고정하려면 `--scaling 1.6`.

  > 측정은 **손이 카메라에 들어온 첫 150프레임**으로 이루어집니다. 시작하자마자
  > 손을 화면에 넣어 두세요. 오검출 프레임으로 측정되면 값이 틀어지는데, 그럴
  > 때는 `--scaling`으로 고정하거나 다시 띄우면 됩니다. 로그의
  > `[CALIBRATION] scaling ...` 줄에서 실제로 쓰인 값을 확인할 수 있고,
  > 1.5~2.2 사이면 정상 범위입니다.

## 파이썬 인터프리터가 둘인 이유

DexPilot은 `dex_retargeting` + `pinocchio`를 쓰고 이들은 numpy 2.x로 빌드되어
있습니다. 이 저장소의 `.venv`는 mediapipe 때문에 numpy 1.26.4로 고정입니다. 한
인터프리터가 둘을 동시에 만족시킬 수 없지만, 리타게팅 노드와 비전 노드는 ROS
토픽으로만 대화하는 **별개 프로세스**라서 각자 필요한 인터프리터를 쓰면 됩니다.

- 비전 생산자 / 브릿지 / 녹화 → `./.venv/bin/python` (기존 그대로)
- 리타게팅 노드 → `$V6F_TELEOP_ROOT/.venv/bin/python`

`run_teleop_v2.sh`가 알아서 고르고, 뜨기 전에 import 가능 여부를 확인합니다.

## 실행

```bash
./run_dashboard.sh real                   # 대시보드 + RViz2 (v2로 전환됨)
./run_cockpit_v2.sh real                  # 콕핏 + DexPilot
./run_teleop_v2.sh sim --hand right       # 로봇 없이 확인
./run_teleop_v2.sh real --scaling 1.6     # 손 크기 측정 대신 고정값
./run_teleop_v2.sh real --no-undo-mirror  # 엄지가 반대쪽으로 모아질 때만
```

**런처 중에서는 `run_dashboard.sh`만 v2로 넘겼습니다.** `run_dashboard_v2.sh`와
같은 파이프라인이고, `--v1`을 붙이면 예전 기하 매핑으로 돌아갑니다:

```bash
./run_dashboard.sh real --v1
```

`run_cockpit.sh`와 `run_teleop.sh`는 일부러 v1 그대로 두었습니다. 그쪽을 v2로
돌리려면 `run_cockpit_v2.sh` / `run_teleop_v2.sh`를 쓰면 됩니다.

## 새로 추가된 파일

| 파일 | 역할 |
|---|---|
| `dexpilot_retarget.py` | MANUS 코어 로딩, MediaPipe 21점 → 캐노니컬 프레임, 스케일 측정, chirality 점검 |
| `retargeting_node_v2.py` | DexPilot 리타게팅 ROS 노드 (`retargeting_node.py` 자리) |
| `vision_tracker_v2.py` | 헤드리스/단순 GUI 생산자 + 월드 랜드마크 |
| `teleop_cockpit_v2.py` | 콕핏 UI + 월드 랜드마크 |
| `teleop_dashboard_v2.py` | 대시보드 UI + 월드 랜드마크 |
| `run_teleop_v2.sh` | v2 통합 런처 (인터프리터 선택 포함) |
| `run_cockpit_v2.sh`, `run_dashboard_v2.sh` | UI별 런처 |
| `run_dashboard.sh` | **기존 파일 중 유일하게 수정**: v2 파이프라인으로 실행, `--v1`로 예전 동작 |
| `config/teleop_v2.env` | v2 전용 기본값 (`V6F_TELEOP_ROOT`, `SCALING`, `CALIBRATE`, `UNDO_MIRROR`) |
| `tools/check_pinch_v2.py` | 위 표를 다시 뽑는 검증 스크립트 |

`sim_bridge_node.py`, `safety_utils.py`, `dataset_recorder.py`, `launch/`,
`urdf/`는 바뀐 것이 없습니다. v2 노드도 같은 `/allegro/target_joints`를 같은
`CONTROLLER_JOINT_ORDER`로 내보냅니다(노드가 기동할 때 순서가 설정의
`target_joint_names`와 일치하는지 검사하고, 다르면 뜨지 않습니다).

## 확인하는 법

```bash
source /opt/ros/jazzy/setup.bash
$V6F_TELEOP_ROOT/.venv/bin/python tools/check_pinch_v2.py      # 위의 표
```

실행 중에는 리타게팅 노드 로그의 엄지 `joint01`을 보세요. 핀치할 때
**−1.0 rad 부근까지 내려가면** v2가 동작하는 것이고, −0.4 근처에서 멈추면 아직
v1 노드가 붙어 있는 것입니다.

## 제어 속도 / 지연

"제어속도를 올린다"는 두 가지가 섞여 있습니다. **갱신 주기(Hz)** 와 **지연(ms)**
인데, 이 시스템에서 병목은 서로 다른 곳에 있습니다. 아래는 이 PC에서 직접 측정한
값입니다(2026-10-01, HP True Vision 웹캠).

### 갱신 주기: 카메라가 천장입니다 — 30 Hz

리타게팅 노드는 랜드마크가 들어올 때만 발행하므로 `/allegro/target_joints`는
**카메라 프레임레이트를 넘을 수 없습니다.** 그런데 이 노트북 웹캠은

```
$ v4l2-ctl -d /dev/video0 --list-formats-ext
  MJPG 640x480  30.000 fps      YUYV 640x480  30.000 fps      (모든 해상도 동일)
```

**모든 포맷·해상도에서 30 fps가 최대입니다.** 즉 `--fps 60`은 이 카메라에서
아무 효과가 없습니다. 측정한 비전 루프:

| 구간 | 중앙값 |
|---|---|
| `cap.read()` (다음 프레임 대기) | 18.4 ms |
| MediaPipe `process()` | 14.7 ms |
| **루프 전체** | **33.0 ms → 30.3 Hz** |

`read()`가 18.4 ms 걸리는 건 33.3 ms 주기에서 처리 14.7 ms를 뺀 값과 정확히
일치합니다 — 카메라 대기가 맞고, MediaPipe는 병목이 아닙니다.

> `model_complexity=0`(lite 모델)도 재봤는데 **이 기계에서는 오히려 느립니다**
> (17.9 ms vs 14.6 ms). 손대지 마세요.

**주기를 올리는 유일한 방법은 60 fps 이상 카메라입니다.** RealSense D4xx의 RGB는
640×480@60을 지원하고, `tools/detect_camera.py`가 이미 RealSense를 우선 선택하게
되어 있습니다. 60 fps로 가면 주기가 2배가 되는 동시에 **아래의 프레임 기반 지연이
전부 절반**이 됩니다. 가장 구조적인 개선입니다.

### 지연: 필터가 대부분입니다

체인의 지연 예산 (30 fps 기준, EMA 그룹 지연 `(1-a)/a × dt`):

| 구간 | 지연 | 조정 방법 |
|---|---|---|
| 카메라 캡처 + 전송 | ~33 ms | 카메라 교체 외 방법 없음 |
| MediaPipe | ~15 ms | 없음 (위 참고) |
| DexPilot 솔버 | 2.5 ms | 무시 가능 |
| **DexPilot 출력 필터 (a=0.25 @30 Hz)** | **~100 ms** | **`--retarget-alpha`** |
| 브릿지 EMA (a=0.25 @100 Hz) | ~30 ms | `--alpha` |
| ros2_control + Modbus | ~10 ms | 아래 참고 |
| **합계** | **~190 ms** | |

**DexPilot 출력 필터 하나가 전체의 절반입니다.** 스텝 응답을 직접 측정한 값
(열린 손 → 핀치, 90% 도달까지):

| `--retarget-alpha` | 90% 도달 | 그룹 지연 |
|---|---|---|
| 0.25 (기본) | 9 프레임 = 300 ms | 100 ms |
| 0.40 | 5 프레임 = 167 ms | 50 ms |
| 0.60 | 3 프레임 = 100 ms | 22 ms |
| 0.80 | 2 프레임 = 67 ms | 8 ms |

```bash
./run_dashboard.sh real --retarget-alpha 0.6    # 반응 먼저
./run_dashboard.sh real --retarget-alpha 0.5 --alpha 0.35
```

주의: 이 필터는 MediaPipe의 프레임별 노이즈를 가리는 역할도 합니다. 올릴수록
손가락 떨림이 드러나므로 0.5~0.6부터 시도하고, 떨리면 내리세요.

브릿지 EMA(`--alpha`)는 30 Hz로 들어오는 계단 입력을 100 Hz로 부드럽게 펴는
역할이라 너무 올리면 모터에 계단이 그대로 갑니다. 리타게팅 alpha를 먼저 올리고,
브릿지는 0.35 근처까지만 올리는 편이 낫습니다.

### 명령 주기(100 Hz)를 올리는 건 마지막

`controller_manager`의 `update_rate: 100`
(`allegro_hand_v6_bringup/config/single_hand/ros2_controllers.yaml`)이 천장이고,
브릿지의 `--rate`를 그 위로 올려도 컨트롤러가 100 Hz로만 소비합니다. 입력이 애초에
30 Hz라 **여기를 올려도 새 정보가 늘지 않습니다.**

그래도 올리려면 먼저 Modbus가 버티는지 확인하세요. 드라이버가 매 사이클
FC03 읽기 1회 + 쓰기 1회를 동기로 돌고, 달성 주기를 `/diagnostics`에 직접
보고합니다:

```bash
ros2 topic echo /diagnostics | grep -A2 "Signal: "
#   Signal: read_states     Freq: ... Hz, Jitter: ... ms, RTT: ... ms
#   Signal: write_commands  Freq: ... Hz, RTT: ... ms
```

RTT 합이 사이클 주기(100 Hz → 10 ms)에 여유 있게 들어가야 합니다. 꽉 차 있으면
`update_rate`를 올리는 순간 사이클을 놓치고 지터가 커집니다.

### 요약: 이 순서로

1. `--retarget-alpha 0.6` — 공짜, 약 80 ms 절감, 지금 바로
2. `--alpha 0.35` — 약 12 ms 추가 절감
3. **60 fps 카메라(RealSense RGB)** — 주기 2배 + 프레임 기반 지연 전부 절반
4. `/diagnostics`로 Modbus 여유 확인 후에만 `update_rate` 상향

## 확인된 것

- **2026-09-30 실물 V6 핸드에서 핀칭 동작 확인.** 위 표의 수치는 녹화된 작업자
  키포인트 + URDF 순기구학으로 계산한 것이지만, 라이브 카메라 경로(월드 랜드마크
  품질, 반전 방향 `undo_mirror=true`, 세션 시작 시 측정되는 스케일)까지 실제
  하드웨어에서 동작합니다.
- `run_dashboard.sh`(v2 경로)도 기동 확인했습니다. 종료할 때 나오는
  `publisher's context is invalid` 트레이스백은 Qt 타이머가 rclpy 종료 뒤
  한 번 더 발행해서 생기는 것으로, v1 때부터 있던 것이고 v2와 무관합니다.

## 남은 것

- MANUS 프로젝트에는 DexPilot 위에 얹는 제스처·손바닥 접촉·자세 사전(prior)
  모듈(`gestures.py`, `palm_contact.py`, `posture.py`)이 더 있습니다. v2는 아직
  쓰지 않습니다. 핀치가 해결된 뒤 잡기 자세가 아쉬우면 다음 후보입니다.
