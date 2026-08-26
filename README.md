# Allegro Hand V4 Vision Retargeting & Teleoperation

RGB 웹캠(MediaPipe)을 이용한 16-DOF **Allegro Hand V4** 로봇 핸드 실시간 비전 리타기팅 및 원격 제어(Teleoperation) 패키지입니다.

---

## 📌 Features
- **단일 웹캠 기반 손 랜드마크 추출**: MediaPipe Hands 기반 3D 좌표 실시간 추적 (새끼손가락 제외, 17개 포인트 $\times$ 3D = 51-dim 토픽 발행).
- **기구학 리타기팅 (Kinematic Retargeting)**: 손가락 뼈대(Phalanx) 벡터 사이각 내적 계산을 통해 16개 관절 각도(Radian) 산출.
- **Allegro Hand V4 제어 순서 보장**: ForwardCommandController 요구 순서 `[Thumb, Index, Middle, Ring]` (`joint00~33`) 매핑 및 관절 한계(Joint Limits) 자동 클램핑.
- **지수 이동 평균(EMA) Low-Pass 필터**: 모터 손상 방지 및 부드러운 모션 추종을 위한 스무딩 필터 내장 ($\alpha=0.15$).
- **시뮬레이션(RViz2) 및 실물 하드웨어(CAN) 동시 지원**.

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

### 올인원 런처 (`run_teleop.sh`) 사용 시
```bash
# 1. 가상 시뮬레이션(RViz2) 전체 파이프라인 단일 실행:
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "/home/humble_ws/allegro_vision_teleop/run_teleop.sh sim"

# 2. 실물 하드웨어(CAN) 전체 파이프라인 단일 실행 (CAN0 활성화 후):
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "/home/humble_ws/allegro_vision_teleop/run_teleop.sh real --alpha 0.15"

# 3. 하드웨어/RViz가 이미 켜진 상태에서 3개 비전 노드만 일괄 실행:
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "/home/humble_ws/allegro_vision_teleop/run_teleop.sh nodes"
```

---

## 🛠 Manual Multi-Terminal Guide

### 1. 시뮬레이션 모드 (RViz2)
```bash
# 터미널 1: 가상 하드웨어 & RViz2 실행
docker exec -it -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_SOFTWARE=1 ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && ros2 launch allegro_hand_bringup allegro_hand.launch.py ros2_control_hardware_type:=mock_components use_sim_time:=false"

# 터미널 2: 비전 트래커 노드 실행
docker exec -it -e DISPLAY=$DISPLAY ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && python3 /home/humble_ws/allegro_vision_teleop/vision_tracker.py"

# 터미널 3: 키네마틱 리타기팅 노드 실행
docker exec -it ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && python3 /home/humble_ws/allegro_vision_teleop/retargeting_node.py"

# 터미널 4: 컨트롤러 브릿지 노드 실행
docker exec -it ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && python3 /home/humble_ws/allegro_vision_teleop/sim_bridge_node.py --alpha 0.15"
```

---

### 2. 실물 하드웨어 모드 (Physical Hand)
```bash
# CAN0 활성화 (1 Mbps)
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up

# 터미널 1: 실물 하드웨어 컨트롤러 구동
docker exec -it -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_SOFTWARE=1 ros_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /home/humble_ws/install/setup.bash && ros2 launch allegro_hand_bringup allegro_hand.launch.py ros2_control_hardware_type:=physical_device use_sim_time:=false"

# 터미널 2, 3, 4: 상기 시뮬레이션 모드와 동일하게 실행
```
