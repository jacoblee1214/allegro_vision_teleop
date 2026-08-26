# Allegro Hand V4 Vision Retargeting & Teleoperation

RGB 웹캠(MediaPipe)을 이용한 16-DOF **Allegro Hand V4** 로봇 핸드 실시간 비전 리타기팅 및 원격 제어(Teleoperation) ROS 2 패키지입니다.

---

## 📌 Architecture & Dataflow

```mermaid
flowchart LR
    A["Webcam (/dev/video0)"] -->|RGB Stream| B["vision_tracker.py<br/>(MediaPipe Hands)"]
    B -->|"/allegro/vision/landmarks<br/>(Float32MultiArray, 51-dim)"| C["retargeting_node.py<br/>(Kinematics & Vector Math)"]
    C -->|"/allegro/target_joints<br/>(Float64MultiArray, 16-dim)"| D["sim_bridge_node.py<br/>(EMA Low-Pass Filter)"]
    D -->|"/allegro_hand_position_controller/commands<br/>(Float64MultiArray, 16-dim)"| E["ros2_control_node"]
    E -->|Mock Components| F["RViz2 (3D Simulation)"]
    E -->|Physical Device (can0, 1Mbps)| G["Allegro Hand V4 (Real HW)"]
```

---

## 📌 Key Features
- **단일 웹캠 기반 실시간 손 추적**: MediaPipe Hands 기반으로 작업자의 손 3D 랜드마크 추출 (새끼손가락 제외, 17개 포인트 $\times$ 3D = 51차원 토픽 발행).
- **3D 기구학 리타기팅 (Kinematic Retargeting)**: 손가락 뼈대(Phalanx) 벡터 사이각 내적 계산을 통해 16개 관절 각도(Radian) 산출.
- **Allegro Hand V4 제어 순서 보장**: ForwardCommandController 요구 순서 `[Thumb, Index, Middle, Ring]` (`joint00~33`) 완벽 매핑 및 관절 한계(Joint Limits) 자동 클램핑.
- **지수 이동 평균(EMA) Low-Pass 필터**: 고주파 지터 억제 및 모터 보호를 위한 스무딩 필터 내장 ($\alpha=0.15$).
- **가상 시뮬레이션(RViz2) & 실물 하드웨어(CAN) 동시 지원**: 표준 ROS 2 토픽 통신으로 URDF나 3D 메쉬 파일 중복 없이 드라이버와 독립적으로 연동.

---

## 📋 Prerequisites
- **ROS 2**: Humble
- **Hardware/Driver**: Wonik Robotics 공식 Allegro Hand ROS 2 패키지 (`allegro_hand_ros2` / `allegro_hand_bringup`)
- **Camera**: 일반 USB/내장 RGB 웹캠 (`/dev/video0`)
- **CAN Adapter (실기 구동 시)**: Peak-System PCAN-USB 등 SocketCAN 지원 어댑터

---

## 📂 Repository Structure
```text
allegro_vision_teleop/
├── README.md                 # 프로젝트 개요 및 실행 매뉴얼
├── requirements.txt          # Python 필수 라이브러리 목록
├── run_teleop.sh             # 올인원 원클릭 실행 스크립트 (sim / real / nodes)
├── vision_tracker.py         # 웹캠 캡처 & MediaPipe 손 랜드마크 추출 노드
├── retargeting_node.py       # 3D 기구학 관절각 변환 노드
├── sim_bridge_node.py        # Low-Pass EMA 필터 적용 컨트롤러 브릿지 노드
└── safety_utils.py           # Allegro V4 관절 명칭, Joint Limits 및 매핑 유틸
```

---

## ⚙️ Installation
```bash
pip install -r requirements.txt
```

---

## 🚀 Quick Start (One-Click Launcher)

올인원 런처 [`run_teleop.sh`](run_teleop.sh)를 이용하면 단일 터미널에서 전체 파이프라인을 구동하고 `Ctrl+C`로 일괄 안전 종료할 수 있습니다.

### 1. 가상 시뮬레이션(RViz2) 모드
```bash
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "/home/humble_ws/allegro_vision_teleop/run_teleop.sh sim"
```

### 2. 실물 하드웨어(CAN) 모드
```bash
# 1) CAN0 인터페이스 활성화 (1 Mbps)
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up

# 2) 실기 원격 제어 실행 (필요 시 --alpha 및 --device 옵션 지정)
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "/home/humble_ws/allegro_vision_teleop/run_teleop.sh real --alpha 0.15"
```

### 3. 비전 노드 3개만 실행 (하드웨어/RViz가 이미 켜진 경우)
```bash
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "/home/humble_ws/allegro_vision_teleop/run_teleop.sh nodes"
```

> [!TIP]
> 화면 창 없이 백그라운드(Headless)로만 가볍게 돌리고 싶을 때는 `--no-gui` 옵션을 붙여서 실행할 수 있습니다:  
> `.../run_teleop.sh sim --no-gui`

---

## 🛠 Manual Multi-Terminal Guide

노드별 상세 로그를 각각 분리해서 확인하고 싶을 때 사용합니다.

```bash
# 터미널 1: 하드웨어 또는 시뮬레이션 실행
# (시뮬레이션): ros2_control_hardware_type:=mock_components
# (실물핸드):   ros2_control_hardware_type:=physical_device
docker exec -it -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_SOFTWARE=1 ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && ros2 launch allegro_hand_bringup allegro_hand.launch.py ros2_control_hardware_type:=mock_components use_sim_time:=false"

# 터미널 2: 비전 트래커 노드 실행
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && python3 /home/humble_ws/allegro_vision_teleop/vision_tracker.py"

# 터미널 3: 키네마틱 리타기팅 노드 실행
docker exec -it ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && python3 /home/humble_ws/allegro_vision_teleop/retargeting_node.py"

# 터미널 4: 컨트롤러 브릿지 노드 실행
docker exec -it ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && python3 /home/humble_ws/allegro_vision_teleop/sim_bridge_node.py --alpha 0.15"
```

---

## 📡 ROS 2 Topics Summary

| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `/allegro/vision/landmarks` | `std_msgs/msg/Float32MultiArray` | 17개 손 랜드마크 3D 좌표 (51-dim) |
| `/allegro/target_joints` | `std_msgs/msg/Float64MultiArray` | 기구학 변환된 16개 관절 목표 각도 (Radian) |
| `/allegro_hand_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | EMA 필터링된 최종 16개 관절 명령 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 현재 하드웨어/시뮬레이션 관절 상태 (100 Hz) |
