# 격리 병동 소독 · 폐기물 수거 협동 로봇 시스템

> 협동-3:디지털 트윈(NVIDIA Isaac Sim) 기반 로봇 자동화 시뮬레이션 시스템 구현 프로젝트
> [두산로보틱스] 지능형 로보틱스 엔지니어 · B그룹 4조 **불사조**

감염병 격리 병동에서 의료진이 직접 수행하던 **소독 분사**와 **폐기물 수거** 업무를
Nova Carter(모바일 베이스) + Doosan M0609(6축 협동로봇 팔)로 구성된 로봇 2대가 병렬 수행하고,
관리자는 병동 밖 웹 관제 대시보드(HMI)에서 모니터링·개입하는 시스템입니다.

전 과정이 Isaac Sim 시뮬레이션 환경에서 구현·검증되었습니다. (실기체 미사용)

---

## 목차

1. [시스템 설계 및 플로우차트](#1-시스템-설계-및-플로우차트)
2. [운영체제 환경](#2-운영체제-환경)
3. [사용한 장비 목록](#3-사용한-장비-목록)
4. [의존성](#4-의존성)
5. [사용 설명 — 실행 순서 및 스크립트](#5-사용-설명--실행-순서-및-스크립트)

부록 · [디렉터리 구조](#부록-a-디렉터리-구조) · [주요 인터페이스](#부록-b-주요-인터페이스) · [트러블슈팅](#부록-c-트러블슈팅)

---

## 1. 시스템 설계 및 플로우차트

### 1.1 시스템 아키텍처

![시스템 아키텍처](docs/system_architecture.png)

웹 관제(HMI) → FastAPI 브리지 → ROS2 DDS → Isaac Sim 이중 로봇까지,
**단일 ROS 도메인 위에서 namespace로 로봇만 분리한 4계층 구조**입니다.

| 계층 | 구성 요소 | 세부 |
|---|---|---|
| Application | 관제 HMI | React 18 + Vite + TypeScript |
| Service | FastAPI 백엔드 | Uvicorn `:8001` · REST `/api/commands/*` · WebSocket `/ws/robot/status` |
| Middleware | ROS2 Humble | `rmw_fastrtps_cpp` (FastDDS) · `ROS_DOMAIN_ID=151` · 화이트리스트 |
| ├ 브리지 | `robot_bridge.py` | ROS2 스레드 ↔ asyncio 큐 (Producer/Consumer) |
| ├ 자율주행 | Nav2 | AMCL · Costmap · BasicNavigator · `pointcloud_to_laserscan` |
| Control | `commander` 패키지 | 미션 노드 · `hmi_link` · `tf_relay` · `pc_reframe` |
| Simulation | Isaac Sim 5.1 | `19_dual_task_select_yolo_integrated.py` · USD/OmniGraph · 전역 단일 `/clock` |
| Robot (가상 HW) | 로봇 2대 | Nova Carter 베이스 + Doosan M0609 팔 + 분사노즐 / Surface Gripper |

**설계 원칙**

- 관제 단말과 로봇 로직을 REST/WebSocket 경계로 분리 → 관제 화면이 끊겨도 미션은 계속 수행
- 도메인은 하나로 두고 namespace(`/carter1`, `/carter2`)로 토픽만 분리 → `/clock`·map 공유 유지
- 사람 안전 2단 방어
  - **Nav2 주행 구간** — costmap 기반 경로 회피
  - **스크립트 `cmd_vel` 구간**(그리퍼·분사) — 회피가 물리적으로 불가하므로 **YOLO PersonGate → 제자리 정지**

### 1.2 로봇 역할 배정 — `task_select`

**두 로봇은 소독과 폐기물 수거를 모두 수행할 수 있습니다.**
역할이 하드웨어에 고정되어 있지 않고, HMI에서 선택한 모드에 따라 `task_select`로 결정됩니다.

| 모드 | 동작 |
|---|---|
| 통합 시작 | 두 로봇이 서로 다른 임무를 자동 배정받아 동시 수행 |
| 개별 선택 | 로봇별로 `소독 분사` 또는 `폐기물 수거`를 직접 지정 |
| 단일 로봇 모드 | 1대가 소독 전용 또는 수거 전용으로 동작 (Surface Gripper 노즐/그리퍼 전환) |

> 두 로봇이 **같은 모드를 동시에 선택하면 중복 검사에 걸려 알림창이 뜨고 재선택을 요구**합니다.
> 그래서 아키텍처 그림과 플로우차트에 표기된 `carter1`/`carter2`의 역할은
> **해당 예시 시나리오에서의 배정**이며, 고정된 사양이 아닙니다.

### 1.3 플로우차트

![플로우차트](docs/flowchart.png)

관제 대시보드 명령 하나로 소독 시퀀스와 폐기물 수거 시퀀스가 **독립 병렬 수행**되며,
어느 단계에서든 긴급정지 인터럽트가 개입 가능합니다.

**정상 시퀀스**

```
대시보드 접속 → 모드 선택 → 동일 모드 중복 검사 → 명령 전송
   → [소독 시퀀스]  ∥  [폐기물 수거 시퀀스]      ← 두 로봇 어느 쪽이든 수행 가능
   → 복귀 → 대기 (작업 큐 · 로그 초기화)
```

| 소독 시퀀스 | 폐기물 수거 시퀀스 |
|---|---|
| ① 대기 (도킹 스테이션) | ① 대기 (도킹 스테이션) |
| ② 복도 진입 · 전진 주행 | ② 전방 주행 (쓰레기물 위치 이동) |
| ③ 노즐 접촉 · 그리퍼 상승 | ③ 폐기물통 파지 |
| ④ 소독 분사 | ④ 수거함 이동 |
| ⑤ 유턴 재분사 (복도 끝 도달 → 반대편) | ⑤ 폐기물 투하 |
| ⑥ 복귀 (도킹 스테이션) | ⑥ 수거통 원위치 |
| ⑦ 대기 (완료) | ⑦ 복귀 → 대기 (완료) |

**분기 · 인터럽트 3종**

| 구분 | 조건 | 처리 |
|---|---|---|
| 중복 검사 | 두 로봇이 같은 모드 선택 | 알림창 출력 → 선택 취소 후 재선택 |
| 안전 인터럽트 | 사람 · 장애물 감지 | 내비게이션 구간: costmap 회피 / `cmd_vel` 구간: PersonGate 제자리 정지 |
| 수동 인터럽트 | 긴급정지 (통합 / 개별) | Nav2 goal 취소 + `cmd_vel` 0 → 수동 제어 활성화 → 동작 재개 또는 도킹 복귀 |

> **동작 재개는 처음부터가 아니라 중단 지점부터** 수행됩니다.
> 정지 시 단계 인덱스를 보존하기 때문입니다.

---

## 2. 운영체제 환경

| 항목 | 버전 / 사양 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| RMW | `rmw_fastrtps_cpp` (FastDDS) |
| `ROS_DOMAIN_ID` | **151** (모든 노드가 반드시 동일) |
| 시뮬레이터 | NVIDIA Isaac Sim 5.1.0 |
| Python | 3.10 (ROS2 Humble 기본) |
| Node.js | 18 이상 (프론트엔드) |
| 시각화 | RViz2 |
| 3D 파츠 설계 | Onshape (쓰레기통 USD) |
| 개발 도구 | VSCode, Cursor, Claude Code |
| 협업 | GitHub, Slack, Notion |

**호스트 구성**

- 실행 호스트 `rokey@isaacsim17`
- Isaac Sim · Nav2 · FastAPI · YOLO 연산 노드를 **단일 워크스테이션에 통합 배치**
  (개발·검증 단계이므로 분산 배치하지 않음)
- 모든 노드가 **동일한 `ROS_DOMAIN_ID(151)`을 공유해야 합니다.**
  도메인을 분리하면 `/clock`·map 공유가 깨집니다.

---

## 3. 사용한 장비 목록

### 3.1 물리 장비

| 장비 | 사양 | 수량 | 용도 |
|---|---|---|---|
| GPU 랩탑 | NVIDIA GeForce RTX 5080 (Laptop) | 2 | Isaac Sim 렌더링 · YOLO 추론 · ROS2 노드 실행 |

> 실물 로봇, 웹캠, 네트워크 스위치는 사용하지 않았습니다. 전 과정이 시뮬레이션 기반입니다.

### 3.2 시뮬레이션 내 로봇 (가상 하드웨어)

| 구성 | 모델 | 비고 |
|---|---|---|
| 모바일 베이스 | NVIDIA Nova Carter | 3D LiDAR · IMU · Odometry(ground truth) · 전방 스테레오 카메라 |
| 매니퓰레이터 | Doosan M0609 (6축 협동로봇) | URDF · RMPflow · Lula IK |
| 엔드이펙터 A | 커스텀 분사 노즐 | `m0609_with_nozzle.usd` · numpy 파티클 폴 |
| 엔드이펙터 B | Surface Gripper | Tool Changer 방식으로 교체 장착 |
| 카메라 센서 | RealSense D455 (시뮬레이션 카메라) | Nova Carter 정면·측면 · YOLO 학습 데이터 생성 |

> Nova Carter 섀시 질량은 매니퓰레이터 반작용 흡수를 위해
> Isaac Sim 상에서 **50kg → 150kg**으로 상향 설정했습니다.

---

## 4. 의존성

전체 목록은 [`requirements.txt`](requirements.txt) 참조.

ROS2 Humble과 Isaac Sim 5.1은 apt / Omniverse Launcher로 별도 설치하며,
아래는 추가 설치가 필요한 패키지입니다.

### 4.1 Python

```bash
pip install -r requirements.txt
```

| 구분 | 패키지 |
|---|---|
| Perception | `ultralytics` `torch` `torchvision` `opencv-python` `numpy(<2.0)` `Pillow` `PyYAML` |
| Backend | `fastapi` `uvicorn[standard]` `pydantic` `websockets` `python-multipart` |
| 수학 · 좌표 | `scipy` `transforms3d` |

> `sqlite3`는 Python 표준 라이브러리이므로 별도 설치가 필요 없습니다.
> `torch`는 RTX 5080에 맞는 CUDA 빌드로 설치해야 합니다. → <https://pytorch.org/get-started/locally/>

### 4.2 ROS2 패키지 (apt)

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-tf-transformations \
  ros-humble-rviz2 \
  python3-colcon-common-extensions
```

### 4.3 프론트엔드

```bash
cd hmi/frontend_v2
npm install     # react 18 · react-dom 18 · typescript · vite
```

---

## 5. 사용 설명 — 실행 순서 및 스크립트

### 5.1 배치 및 빌드

제출본은 **패키지 소스만** 포함되어 있습니다.
아래와 같이 워크스페이스에 배치한 뒤 빌드하세요.

```bash
# 1) 패키지 소스를 워크스페이스로 배치
mkdir -p ~/cobot3_ws/src
cp -r src/commander  ~/cobot3_ws/src/
cp -r src/perception ~/cobot3_ws/src/
cp -r isaacpjt hmi scripts ~/cobot3_ws/

# 2) 빌드
cd ~/cobot3_ws
colcon build --symlink-install

# 3) Nav2 관련 패키지(carter_navigation)는 별도 워크스페이스
cd ~/humble_ws
colcon build --symlink-install
```

### 5.2 FastDDS 화이트리스트 배치

프로파일이 없으면 `ros2 topic echo` / `hz`가 **전부 0으로 나오는 거짓 음성**이 발생합니다.

```bash
mkdir -p ~/.config/fastdds
cp config/fastdds_profile.xml ~/.config/fastdds/
```

> 경로가 `~/.ros`가 **아니라** `~/.config/fastdds`입니다.

### 5.3 환경 소싱

**모든 터미널에서 가장 먼저 실행합니다.** 누락 시 토픽이 보이지 않거나 미션이 동작하지 않습니다.

```bash
source ~/cobot3_ws/setup_env.sh
```

`setup_env.sh`가 처리하는 내용:

```bash
# 1) ROS2 기본 환경
source /opt/ros/humble/setup.bash

# 2) 워크스페이스 오버레이 (순서 중요: humble_ws → cobot3_ws)
source ~/humble_ws/install/setup.bash
source ~/cobot3_ws/install/setup.bash

# 3) DDS 설정
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.config/fastdds/fastdds_profile.xml

# 4) 도메인 (모든 노드 동일)
export ROS_DOMAIN_ID=151

# 5) 로그 버퍼링 해제 (헤드리스 실행 시 로그 유실 방지)
export PYTHONUNBUFFERED=1
```

> **거짓 음성 진단 팁** — 토픽이 안 잡히면 데몬이 오래된 설정을 물고 있을 수 있습니다.
> `ros2 daemon stop && ros2 daemon start`

### 5.4 원격 접속 설정 (선택)

다른 PC 브라우저에서 관제 화면에 접속하는 경우에만 설정합니다.

```bash
# 백엔드 — CORS 허용 origin
export HMI_CORS_ORIGIN="http://<접속할_PC_IP>:5174"
```

```bash
# 프론트엔드 — hmi/frontend_v2/.env
VITE_API_BASE=http://<서버_IP>:8001
VITE_WS_URL=ws://<서버_IP>:8001/ws/robot/status
```

> Vite는 **dev 서버 시작 시점**에 `.env` 값을 브라우저 JS에 고정 삽입합니다.
> `.env`를 수정했다면 **dev 서버를 반드시 재시작**하세요.

---

### 5.5 실행 순서

> ⚠️ **순서를 반드시 지켜야 합니다.**
> Isaac Sim이 `/clock`을 발행하기 전에 Nav2를 띄우면 `use_sim_time` 노드들이 시간을 받지 못해
> TF가 마비되고, Nav2가 `No map received` 상태로 제자리 회전합니다.

각 단계는 **별도 터미널**에서 실행하며, 모든 터미널에서 `setup_env.sh`를 먼저 소싱합니다.

```
① Isaac Sim (Play 확인) → ② Nav2 → ③ Backend → ④ Mission → ⑤ Vision → ⑥ Frontend
```

#### ① Isaac Sim — 시뮬레이션 및 로봇 제어

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
cd ~/cobot3_ws/isaacpjt/M0609
isaac_python 19_dual_task_select_yolo_integrated.py
```

씬 로딩 후 **Play 상태(`/clock` 발행 시작)를 확인한 뒤** 다음 단계로 넘어갑니다.

#### ② Nav2 — 자율주행 스택

```bash
source /opt/ros/humble/setup.bash
~/cobot3_ws/run_nav.sh
```

RViz2에서 두 로봇의 맵과 위치가 정상 표시되는지 확인합니다.

#### ③ Backend — FastAPI 관제 서버

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws/hmi/backend_v2
uvicorn main:app --host 0.0.0.0 --port 8001
```

`0.0.0.0` 바인딩이므로 같은 네트워크의 다른 PC에서도 접속할 수 있습니다.

#### ④ Mission — 미션 노드

```bash
source /opt/ros/humble/setup.bash
~/cobot3_ws/run_missions_19_hmi.sh
```

> **반드시 이 스크립트로만 실행하세요.**
> 스크립트 내부에서 `humble_ws`(carter_navigation 포함) 소싱과 `use_sim_time:=True` 설정을 함께 처리합니다.
> `ros2 run`으로 직접 실행하면 소싱이 빠져 goal timestamp가 wall time으로 발행되고,
> TF 변환에 실패해 로봇이 움직이지 않습니다.

#### ⑤ Vision — YOLO 사람 감지

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws
~/cobot3_ws/run_vision_19.sh
```

#### ⑥ Frontend — 관제 대시보드

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws/hmi/frontend_v2
npm install
npm run dev
```

브라우저에서 `http://<서버_IP>:5173` 으로 접속합니다.

### 5.6 사용 흐름

1. 관제 대시보드 접속
2. **통합 시작** 또는 로봇별 **개별 모드 선택**(소독 분사 / 폐기물 수거)
3. 두 로봇이 각자의 시퀀스를 병렬 수행 — 지도·작업 큐·로그·비전 패널로 실시간 확인
4. 필요 시 **긴급정지**(통합 / 개별) → 상황 해제 후 **동작 재개**(중단 지점부터) 또는 **도킹 복귀**
5. 수동 제어 모드에서 지도를 클릭해 이동 경로 지정 (복수 지점 예약 가능)

### 5.7 종료

역순으로 종료합니다.
Isaac Sim은 `Stop → Play` 재개를 지원하지 않으므로,
미션을 다시 돌리려면 **Isaac Sim과 미션 노드를 모두 재시작**해야 합니다.

---

## 부록 A. 디렉터리 구조

### 제출 패키지 구성

빌드 산출물(`build/`, `install/`, `log/`)과 워크스페이스 껍데기는 제외했습니다.

```
.
├── README.md
├── requirements.txt
├── setup_env.sh                   # 공통 환경 소싱 스크립트
├── docs/
│   ├── system_architecture.png
│   └── flowchart.png
├── config/
│   └── fastdds_profile.xml        # ~/.config/fastdds/ 에 배치
├── src/
│   ├── commander/                 # 미션 제어 패키지
│   │   └── commander/
│   │       ├── spray_waypoint_mission.py      # 소독 미션 노드
│   │       ├── trash_can_nav_pick_mission.py  # 폐기물 수거 미션 노드
│   │       ├── hmi_link.py                    # HMI 연동 레이어
│   │       ├── tf_relay.py                    # 프레임명 namespace 접두 부여
│   │       └── pc_reframe.py                  # 포인트클라우드 frame_id 정합
│   └── perception/                # YOLO 비전 패키지
│       ├── perception/
│       │   └── train_yolo11s_trash_can_person.py
│       └── models/
│           └── trash_can_person_yolo11s.pt
├── isaacpjt/
│   └── M0609/
│       ├── 19_dual_task_select_yolo_integrated.py   # 메인 Isaac Sim 스크립트
│       └── usd/                   # m0609_with_nozzle.usd 등
├── hmi/
│   ├── backend_v2/                # FastAPI
│   │   ├── main.py
│   │   ├── robot_bridge.py        # ROS2 ↔ asyncio 브리지
│   │   └── connection_manager.py  # WebSocket 브로드캐스트
│   └── frontend_v2/               # React 18 + Vite
│       ├── src/lib/apiClient.ts
│       └── .env
└── scripts/
    ├── run_nav.sh
    ├── run_missions_19_hmi.sh
    └── run_vision_19.sh
```

### 제출용 zip 생성

```bash
cd ~/cobot3_ws
zip -r 불사조_B그룹4조_소스코드.zip \
    README.md requirements.txt setup_env.sh \
    docs config src isaacpjt hmi scripts \
    -x "*/build/*" "*/install/*" "*/log/*" \
       "*/node_modules/*" "*/__pycache__/*" "*.pyc" \
       "*/.git/*" "*/.vscode/*"
```

---

## 부록 B. 주요 인터페이스

### B.1 통신 구간

| 구간 | 프로토콜 | 인터페이스 |
|---|---|---|
| 브라우저 → 백엔드 | HTTP / REST | `POST /api/commands/start-all`, `/start/{id}`, `/estop`<br>`GET /api/history`, `/api/errors` |
| 백엔드 → 브라우저 | WebSocket | `/ws/robot/status` (브로드캐스트, 3초 자동 재연결) |
| 백엔드 ↔ ROS2 | DDS | 발행 `/robot/command` (JSON)<br>구독 `/robot/{id}/process_state`, `/safety_event` |
| 미션 ↔ Nav2 | ROS2 Action | `NavigateToPose` — goal / cancel |
| Nav2 ↔ 로봇 | ROS2 Topic | `/carterN/cmd_vel`, `/carterN/scan`, `/carterN/odom`, `/carterN/tf` |
| Isaac ↔ 전체 | ROS2 Topic | `/clock`, `/front_3d_lidar/lidar_points`, `/carterN/front_stereo_camera/*` |

### B.2 적용 알고리즘

| 도메인 | 알고리즘 | 비고 |
|---|---|---|
| 위치추정 | AMCL | `alpha1~5` 0.2 → 0.05 (odom이 ground truth이므로 더 신뢰) |
| 전역 경로 | Nav2 Global Planner | `NavigateToPose` Action |
| 지역 제어 | DWB | 속도/가속 한계 하향 |
| 환경 인식 | Costmap 2D | `inflation_radius` 1.0 → 0.5, `cost_scaling_factor` 0.3 → 3.0 |
| 센서 변환 | PointCloud → LaserScan | `pointcloud_to_laserscan` |
| IK (반응형) | RMPflow | 이동 구간 전용 |
| IK (정밀) | LulaKinematicsSolver | 파지·분사 조준 전용 — 1회 해 산출 후 관절각 확정 |
| 궤적 생성 | Joint-space Ramp 보간 | `g_ramp_to_joint_positions` |
| 객체 인식 | YOLOv11s (person / small_trash_can) | RGB 4Hz · PersonGate |
| 정밀 도킹 | Nudge + One-Shot Discrete Control | 1회 측정 → 회전 → 직진, 최대 3회 |
| 임무 제어 | FSM + Generator 협조 스케줄링 | `g_task_select_mission` |

### B.3 YOLO 학습 성능

데이터셋 600장 (Isaac Sim Replicator 합성 궤도 카메라)

| 지표 | overall | person | small_trash_can |
|---|---|---|---|
| Precision | 0.988 | 0.9977 | 0.978 |
| Recall | 0.986 | 1.0 | 0.971 |
| mAP50 | 0.994 | 0.995 | 0.994 |
| mAP50-95 | 0.957 | 0.965 | 0.949 |

```bash
python3 src/perception/perception/train_yolo11s_trash_can_person.py
```

---

## 부록 C. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| Nav2 `No map received` + 제자리 회전 | Isaac Sim이 `/clock`을 발행하지 않아 `use_sim_time` 노드의 TF가 마비 | 실행 순서를 **Isaac Play → Nav2 → Mission**으로 고정 |
| `ros2 topic echo` / `hz`가 전부 0 | FastDDS 프로파일 환경 변수 누락 | `setup_env.sh` 소싱 후 `ros2 daemon stop && start` |
| 미션 실행해도 로봇이 안 움직임 | `use_sim_time` 미설정 → goal timestamp가 wall time | `run_missions_19_hmi.sh`로만 실행 |
| 통합 RViz에 로봇이 안 그려짐 | 두 로봇이 동일 프레임명 사용 → TF 충돌 | `tf_relay`가 `carterN/` 접두를 부여 |
| 포인트클라우드가 RViz에 안 뜸 | Isaac 발행 `frame_id`와 접두 프레임명 불일치 | `pc_reframe` 노드로 재발행 |
| 카메라 토픽이 두 로봇 간 분리 안 됨 | `camera_namespace` 상수에 연결되어 직접 Set이 무시됨 | `set_carter_namespace`에서 상수 값 자체를 수정 |
| 다른 PC 브라우저에서 400 에러 | `HMI_CORS_ORIGIN`에 해당 origin 미등록 | 실제 접속 origin으로 정확히 지정 |
| 로봇 위치가 갑자기 튐 (teleport) | AMCL `alpha` 과소신뢰 → 파티클 과확산 | `alpha1~5`를 0.05로 하향 |
| 두 로봇 동시 구동 시 프레임 드롭 | GPU 사용률 98% + 전력캡 · RealSense 동기 읽기 스톨 | `RENDER_EVERY=2`, 카메라 해상도 축소, `RS_PUBLISH_EVERY` 조정 |
| 좁은 복도에서 뺑뺑이 / 느림 | costmap inflation 과함 | `inflation_radius` 0.5, `cost_scaling_factor` 3.0 |
| 주행 흔들림 (wobble) | voxel_layer가 바닥 노이즈·로봇 팔을 유령 장애물로 인식 | 로컬 costmap plugins에서 voxel_layer 제거 |
| 미션 재실행 시 로봇 멈춤 | Isaac Sim은 `Stop → Play` 재개 미지원 | Isaac Sim과 미션 노드를 **모두 재시작** |

### 반복적으로 나타난 근본 원인 5가지

1. **`/clock` 미발행** — Nav2·costmap·localization 마비의 공통 원인
2. **Namespace / frame 접두 불일치** — tf, pointcloud, camera 세 곳에서 반복
3. **스폰 좌표 vs AMCL `initial_pose` 불일치** — 맵이나 스폰이 바뀔 때마다 재발
4. **반응형 IK(RMPflow)의 발산 경향** — 1회 IK 해 + ramp 보간으로 대체
5. **미검증 추정치** — "라이브 재확인 필요"로 표시해둔 값이 실제 실패 원인이 됨

> 다섯 가지 모두 **환경의 전제를 확인하지 않고 표준 설정을 그대로 쓴 것**에서 출발했습니다.
> 시뮬레이션 통합에서는 시간·프레임·좌표의 전제 확인이 코드 작성보다 먼저입니다.

---

## 팀 구성

| 이름 | 역할 | 담당 |
|---|---|---|
| 박채송 | 팀장 | Isaac Sim 로봇 제어 및 동작 구현 |
| 정태성 | 팀원 | YOLO 기반 Vision 인식 및 시스템 통합 |
| 김범준 | 팀원 | Isaac Sim 동작 고도화 및 파라미터 튜닝 |
| 하찬용 | 팀원 | HMI UI 구성 및 연동 |

**멘토** 손미란 강사님 · **개발 기간** 2026.07.15 ~ 07.29 (14일)
