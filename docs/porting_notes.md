# 포팅 노트 — Humble/도커 → Jazzy 네이티브

원본: [jacoblee1214/allegro_vision_teleop](https://github.com/jacoblee1214/allegro_vision_teleop)
대상 환경: Ubuntu 24.04 · ROS 2 Jazzy · 컨테이너 없음 · 드라이버 워크스페이스 `~/v6f_manuse_teleoperation`

## 가져온 파일과 이름 변경

원본은 버전별 파일을 모두 남겨두는 구조입니다(`*_v6`, `*_v6_1`, `*_v6_2`, `*_v6_3`). 실행에 쓰이는 최신 조합만 가져오고 접미사를 없앴습니다.

| 원본 | 이 저장소 |
|---|---|
| `teleop_cockpit_v6_3.py` | `teleop_cockpit.py` |
| `teleop_dashboard_v6_1.py` | `teleop_dashboard.py` |
| `retargeting_node_v6.py` | `retargeting_node.py` |
| `sim_bridge_node_v6_1.py` | `sim_bridge_node.py` |
| `vision_tracker_v6.py` | `vision_tracker.py` |
| `dataset_recorder_v6_1.py` | `dataset_recorder.py` |
| `safety_utils_v6.py` | `safety_utils.py` |
| `check_hand.py` | `tools/check_hand.py` |
| `run_teleop_v6_3.sh` | `run_teleop.sh` (도커 위임 제거, 환경 파일 도입) |
| `wonik_patch_v6_1/launch/allegro_hand_v6_1.launch.py` | `launch/allegro_hand_bringup.launch.py` (패키지 복사 대신 저장소 안에서 실행) |

V4(4지, SocketCAN) 런처와 노드는 가져오지 않았습니다. 이 PC의 하드웨어는 V6 5지입니다.

## 코드 수정 지점

1. **경로**: `/home/humble_ws/...`, `~/humble_ws/...` 하드코딩 제거.
   - URDF: `$AVT_URDF_DIR` → `<저장소>/urdf` → `allegro_hand_v6_description` share 순으로 탐색
   - 메쉬: `$AVT_MESH_DIR` → `<저장소>/meshes`(심볼릭 링크) → 패키지 share 순으로 탐색
   - 녹화: `/home/humble_ws/pinn_hw/results/episodes` → `$AVT_EPISODE_DIR` 또는 `<저장소>/data/episodes`
2. **URDF 사본**: `urdf/allegro_hand_v6_{left,right}.urdf` 는 `~/v6f_manuse_teleoperation` 드라이버 패키지의 파일과 동일한지 diff로 확인 후 복사했습니다. 워크스페이스를 source 하지 않아도 UI가 뜨도록 하기 위한 사본입니다.
3. **드라이버 패키지 무수정**: 원본은 `wonik_patch_v6_1/`을 Wonik bringup 패키지에 복사해 재빌드해야 했습니다. 여기서는 같은 내용을 `launch/allegro_hand_bringup.launch.py` 하나로 대체해, 드라이버 패키지를 건드리지 않습니다.
4. **런처**: 호스트→컨테이너 자동 진입 로직, `mknod`로 `/dev/video*` 생성하는 부분, 전역 `LIBGL_ALWAYS_SOFTWARE=1` 제거. CycloneDDS는 설치 여부를 확인한 뒤에만 선택합니다(없을 때 `ros2`가 즉시 종료되기 때문).
5. **파이썬 환경**: `.venv --system-site-packages`. `mediapipe`/`PyOpenGL`/`opencv-contrib-python`/`scipy`만 pip, `rclpy`·`cv_bridge`·`PyQt5`는 apt. numpy는 1.26.4 고정(mediapipe 0.10.x가 numpy 2.x 비대응, pip numpy 2.x는 rclpy C 확장을 깨뜨림).
   `QOpenGLWidget`은 이 Qt 빌드에서 `PyQt5.QtWidgets`에 있으므로 `python3-pyqt5.qtopengl`은 의존성에 넣지 않았습니다.

## 검증 결과 (2026-09-27)

| 항목 | 결과 |
|---|---|
| `py_compile` (노드 7개 + `tools/check_hand.py` + launch) | 통과 |
| `.venv` import (`numpy 1.26.4`, `cv2 4.10.0`, `mediapipe 0.10.14`, `scipy 1.13.1`, `rclpy`, `cv_bridge`, `PyQt5`, `QOpenGLWidget`, `OpenGL.GL`, `ament_index_python`) | 전부 OK |
| `ros2 launch launch/allegro_hand_bringup.launch.py --show-args` | 인자 6개 정상 노출 |
| `hardware:=mock_components` 브링업 | `joint_state_broadcaster`, `allegro_hand_position_controller` 둘 다 **active**, `/joint_states` 발행, `robot_state_publisher` 초기화 성공 |
| `./run_teleop.sh sim --headless` | `/vision_tracker`, `/kinematic_retargeting_node`, `/sim_bridge_node` + 컨트롤러 기동. 토픽 `/allegro/vision/landmarks`, `/allegro/target_joints`, `/allegro/joint_states_urdf`, `/allegro/teleop_state`, `/allegro/camera/image_raw`, `/allegro_hand_position_controller/commands` 확인. `Ctrl+C`/SIGTERM에서 자식 프로세스 전부 정리됨 |
| 카메라 자동 감지 | `/dev/video0` (HP True Vision FHD) 선택 |

### 실물 하드웨어 검증 (2026-09-27, 오른손 실기)

| 항목 | 결과 |
|---|---|
| `./check_hand.sh` | 오른손 · 펌웨어 `0x0300` (v3.0) · 20축 엔코더 정상 |
| `./run_cockpit.sh real` — 손 종류 자동 감지 | 레지스터 `0x0071` → `right` 정확히 감지 |
| Modbus 하드웨어 인터페이스 | `AllegroHandV6System` 플러그인으로 `192.168.1.100:502` 연결 성공, `activate` 성공 |
| 컨트롤러 | `joint_state_broadcaster`, `allegro_hand_position_controller` 둘 다 **active** |
| 피드백 | `/allegro/joint_states_urdf` 100.0 Hz |
| 명령 ↔ 실물 추종 오차 | 평균 0.024 rad (1.4°), 최대 0.140 rad (8.0°) — 동작 중 위치 제어 추종 오차 범위 |
| 콕핏 UI | 카메라·MediaPipe·3D STL 손·20축 게이지 전부 정상, 실물 피드백 반영 |
| CycloneDDS | 설치 후 `rmw_cyclonedds_cpp`로 자동 선택됨 |

이 셋업은 오른손 전용입니다. 왼손은 연결하지 않습니다.

### 검증 중 발견해 고친 것

1. **Qt 플랫폼 플러그인 충돌** — `opencv-contrib-python` 휠이 import 시점에 `QT_QPA_PLATFORM_PLUGIN_PATH`를 자기 번들 Qt5(`cv2/qt/plugins`)로 **무조건** 덮어씁니다. 그 플러그인은 cv2 번들 Qt5에 링크돼 있어 apt PyQt5가 못 읽고, `QApplication` 생성이 `Could not load the Qt platform plugin "xcb"`로 죽습니다. 환경변수를 미리 export해도 cv2가 덮어쓰므로 소용없어서, `teleop_cockpit.py`·`teleop_dashboard.py`에서 cv2 import 직후 cv2가 설정한 값만 제거합니다. 두 파일은 PyQt5로 그리고 `cv2.imshow`를 쓰지 않으므로 잃는 기능은 없습니다(`vision_tracker.py`는 imshow를 쓰므로 건드리지 않음).
2. **`tools/setup.sh`의 `set -u`** — ROS 2 `setup.bash`가 `AMENT_TRACE_SETUP_FILES` 등 미설정 변수를 읽어서 nounset이 켜져 있으면 source 순간 스크립트가 죽습니다. source 구간마다 `set +u`/`set -u`로 감쌌고, 의존성 검증이 실패해도 마지막 안내가 나오도록 `set -e` 조기 종료도 없앴습니다.
3. **런처 종료 처리** — `teleop_cockpit.py`는 종료 시 rclpy 컨텍스트가 사라진 뒤 Qt 타이머가 한 번 더 publish하면서 멈추는 경우가 있고, `ros2 launch`는 자식 정리에 수 초가 걸립니다. 기존 cleanup은 0.5초 뒤 바로 SIGTERM을 보내서 `ros2 launch`를 죽여 `ros2_control_node`를 고아로 남겼습니다(**`real` 모드에서 런처 종료 후에도 손이 제어 상태로 남음**). SIGINT → 최대 8초 대기 → SIGTERM → SIGKILL 순서로 바꾸고, 브링업을 `setsid`로 별도 프로세스 그룹에 띄워 그룹 단위로 정리합니다. 수정 후 잔여 프로세스 0, 종료 12초.

4. **RViz 손 자세** — `world → base_link` 정적 변환이 Wonik 브링업과 같은 `pitch = -π/2`였습니다. `base_link`에서 +Z가 손가락 방향, +X가 엄지 방향이라 이 회전은 손가락을 world −X로 눕히고 엄지만 위로 세웁니다(팔에 장착한 손에는 맞지만 텔레옵 화면으로는 부적절). 항등 회전으로 바꿔 손이 선 자세가 되게 했습니다. 관절 0 자세에서 TF로 확인: 검지/중지/소지 끝 z = +0.183 / +0.196 / +0.164, 엄지 끝 x = +0.132.

5. **단축키가 먹통이 되는 문제** — `C`/`R`/`S`/`F`/`H`는 메인 윈도우의 `keyPressEvent`로만 처리됐습니다. 그런데 에피소드 목록 `QTableWidget`은 출력 가능한 키를 keyboard-search로 **소비**하기 때문에, 표를 한 번 클릭해 포커스가 가면 그 뒤로 모든 단축키가 조용히 죽습니다(시작 직후엔 버튼에 포커스가 있어 동작하다가 갑자기 안 되는 증상). 오프스크린 Qt 테스트로 재현 확인: 버튼/라벨 포커스에서는 메인 윈도우가 키를 받지만 표 포커스에서는 전혀 받지 못합니다. `Qt.ApplicationShortcut` 컨텍스트의 `QShortcut`으로 바꿔 포커스 위치와 무관하게 동작하게 했고(키를 먼저 소비하므로 `keyPressEvent`와 이중 실행되지 않음), 표는 `setFocusPolicy(Qt.NoFocus)`로 탭 순서에서도 뺐습니다.

   신호 경로 자체는 정상이었습니다. `/allegro/teleop_state`로 clutch를 직접 publish하면 리타게팅 노드가 `[CLUTCH] ⏸ ENGAGED`/`▶ RELEASED`로 정확히 반응하고, `--record`로 띄운 뒤 record→tag를 publish하면 301프레임(3.01 s @ 100 Hz) 에피소드가 JSON + manifest로 저장됩니다.

6. **IR 카메라가 선택되는 문제** — 원본의 카메라 자동감지는 "RealSense가 있으면 그것, 없으면 프레임이 나오는 첫 장치"입니다. 이 노트북은 카메라 모듈 하나에 노드 4개(`video0` RGB `MJPG`/`YUYV`, `video1` 메타데이터, `video2` IR `GREY`, `video3` 메타데이터)를 노출하고 **IR 센서도 정상 캡처됩니다**. 그래서 `video0`이 점유된 상태(이전 UI가 고아로 남은 경우 등)에서는 조용히 `video2`가 선택되고, 8-bit 흑백에 IR 조명 점멸이 겹쳐 화면이 번쩍입니다.

   인라인 셸 안의 파이썬을 `tools/detect_camera.py`로 분리하고, `v4l2-ctl --list-formats`로 **흑백 전용(FourCC가 `GREY`/`Y8`/`Y16` 등뿐인) 장치를 제외**하도록 했습니다. `--list`로 각 노드가 왜 선택/제외됐는지 볼 수 있습니다. 쓸 수 있는 컬러 카메라가 없으면 엉뚱한 장치로 떨어지지 않고 명확한 메시지와 함께 종료합니다.

   함께 고친 것: 런처의 고아 UI 정리가 조용히 `pkill`만 하던 것을, 어떤 PID를 정리했는지 알리고 다른 `run_teleop.sh`가 떠 있으면 경고하도록 바꿨습니다(두 세션이 `controller_manager`와 카메라를 공유하면 충돌). 터미널을 그냥 닫으면 트랩이 안 돌아 UI가 고아로 남는 것이 애초 원인이었습니다.

### 환경 메모

- `ros-jazzy-rmw-cyclonedds-cpp` 설치 완료. 런처가 자동으로 선택합니다.
- 유선 인터페이스에 NetworkManager 프로필 `allegro-hand`(192.168.1.10/24, enp129s0)를 만들어 뒀습니다. 런처 안의 `ip addr add`는 root가 필요해 일반 사용자로는 조용히 실패하므로, 주소가 없으면 `nmcli con up allegro-hand`로 올리세요.
- 검증 중 `/var/crash`에 `_opt_ros_jazzy_bin_ros2.crash`가 생겼는데, `timeout`으로 강제 종료한 `ros2 topic hz` / `ros2 control list_controllers` CLI가 CycloneDDS 종료 중 죽은 것입니다. 텔레옵 노드와는 무관합니다.
