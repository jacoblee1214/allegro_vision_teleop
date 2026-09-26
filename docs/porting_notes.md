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

1. **경로**: `/home/humble_ws/...`, `/home/jake/humble_ws/...` 하드코딩 제거.
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

아직 확인하지 않은 것: **실물 하드웨어(`real` 모드)**. 로봇이 연결되어 있지 않았습니다(`enp129s0` NO-CARRIER). 손 종류 자동 감지와 왼손 MCP 오프셋 경로는 실물에서 확인해야 합니다.

미설치 apt 패키지: `ros-jazzy-rmw-cyclonedds-cpp` (sudo 필요). 런처는 없으면 기본 rmw로 진행하므로 실행은 됩니다. `./tools/setup.sh`를 한 번 돌리면 설치됩니다.
