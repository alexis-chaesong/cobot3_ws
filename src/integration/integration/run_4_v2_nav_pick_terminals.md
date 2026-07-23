# 4_v2 Nav2 + YOLO 게이트 — 실행 / 트러블슈팅 (실측 2026-07-23)

## 핵심 원인 요약

| 증상 | 진짜 원인 | 해결 |
|------|-----------|------|
| `Package 'carter_navigation' not found` | 패키지가 `cobot3_ws`가 아니라 `~/IsaacSim-ros_workspaces/humble_ws`에 있음 | T1/T3에서 humble_ws `install/setup.bash` source |
| `rclpy._rclpy_pybind11` / segfault (T2) | T2에서 `ros_set`(/opt/ros/humble, py3.10) 사용 | T2는 `ros_set` 금지. Isaac bridge py3.11 rclpy (스크립트 P5 가드) |
| `amcl/get_state ... waiting` (T3) | Isaac(T2)가 죽거나 아직 `/clock` 미발행 | T2 살아 있고 `/clock` 나온 뒤 T3 실행 |
| PhysX `velocity tensor (2,6)` + Killed | RealSense nested RigidBody | 4_v2가 attach 시 RB disable/strip (P7) |
| T1만 `cobot3_ws/install` source | `carter_navigation` 검색 경로에 없음 | humble_ws 추가 source |

## 워크스페이스 맵

```
/opt/ros/humble                          # 시스템 ROS (T1/T3만)
~/IsaacSim-ros_workspaces/humble_ws      # carter_navigation, nav2 params
~/cobot3_ws                              # commander, 4_v2 스크립트, map/usd
~/dev_ws/isaac_sim/.../python.sh         # Isaac Python 3.11 (T2만)
```

## 실행 순서 (반드시 이 순서)

### 터미널 1 — Nav2

```bash
source /opt/ros/humble/setup.bash                                          # ros_set
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash              # ★ 필수
source ~/cobot3_ws/install/setup.bash

# 확인
ros2 pkg prefix carter_navigation
# → .../IsaacSim-ros_workspaces/humble_ws/install/carter_navigation

ros2 launch carter_navigation carter_navigation.launch.py \
  map:=/home/rokey/cobot3_ws/src/assets/map/modified_hospital_map.yaml \
  params_file:=/home/rokey/IsaacSim-ros_workspaces/humble_ws/install/carter_navigation/share/carter_navigation/params/carter_navigation_params_hospital.yaml
```

정상: `/amcl`, `/controller_server` 등 기동. Isaac 전에는 `odom` TF 타임아웃 로그가 나올 수 있음(정상).

### 터미널 2 — Isaac 4_v2 (`ros_set` 금지)

```bash
cd ~/cobot3_ws

# /opt/ros 가 섞였다면 정리
export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
export LD_LIBRARY_PATH="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}"

isaac_python src/integration/integration/4_v2_mobile_manipulator_trash_can_nav_pick_test.py \
  --weights ~/cobot3_ws/src/perception/models/small_trash_can_yolo11s.pt \
  --yolo-conf 0.05 --yolo-frames 12 --yolo-min-hits 2
```

정상 로그 순서:
1. `[ROS] P5: re-exec with Isaac bridge py3.11 rclpy` (가끔)
2. `[LOAD] hospital environment=...`
3. `[YOLO] RealSense attached ... stripped nested RB/collision ops=...`
4. `[YOLO] camera RGB OK ...`
5. `[ROS] '/trash_can_nav_goal' 발행 + '/clock' 발행 + '/start_pick' 구독 시작`

다른 터미널에서 확인:
```bash
# clean env 권장 (Isaac LD_LIBRARY_PATH 없는 셸)
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
ros2 topic hz /clock          # ~20Hz
ros2 topic echo /trash_can_nav_goal --once
```

### 터미널 3 — commander (T2의 `/clock` 확인 후)

```bash
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source ~/cobot3_ws/install/setup.bash

ros2 run commander trash_can_nav_pick_mission_v2
```

정상:
```
Publishing Initial Pose
Nav2 is ready for use!
Navigating to goal: ...
남은 거리: ...
```

이후 T2:
```
[NAV:PICK] '/start_pick'=True 수신
[YOLO] gate ... hits=12/12 passed=True
[YOLO] gate PASS → 기존 4_ 파지 시퀀스 시작
```

## 이번 실측에서 확인된 것 (2026-07-23)

- T1: `carter_navigation`은 humble_ws source 후에만 발견됨
- T2: P5 re-exec + P7 strip → PhysX `(2,6)` 에러 **0회**, `/clock` ~19Hz
- T3: `Nav2 is ready` → 목표 주행 → `/start_pick`
- YOLO: hospital 씬에서도 **12/12 HIT**, gate PASS
- 파지: Nav 후 chassis 오차(~0.3m)로 gripper close 실패 경고가 날 수 있음  
  (Nav2/접근 기하 튜닝 이슈 — YOLO 게이트와는 별개)

## bashrc 권장 추가

```bash
alias ros_nav='source /opt/ros/humble/setup.bash; \
  source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash; \
  source ~/cobot3_ws/install/setup.bash'

alias isaac_ros_clean='
  export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ":" "\n" | grep -v "/opt/ros" | paste -sd:);
  export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ":" "\n" | grep -v "/opt/ros" | paste -sd:);
  export LD_LIBRARY_PATH="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}";
  echo "Isaac ROS env ready"'
```

- T1/T3: `ros_nav`
- T2: `isaac_ros_clean` 후 `isaac_python ...`

## Nav2 없이 YOLO+파지만

```bash
cd ~/cobot3_ws
isaac_python src/integration/integration/4_v2_mobile_manipulator_trash_can_nav_pick_test.py \
  --headless --skip-nav --pick-only \
  --weights ~/cobot3_ws/src/perception/models/small_trash_can_yolo11s.pt \
  --yolo-conf 0.05 --yolo-frames 10 --yolo-min-hits 2
```

## 터미널3 `주행 실패` + 무한 재시도 (고침)

증상: `남은 거리: 0.00 m` 직후 `주행 실패 → '/start_pick' 발행 안 함` 이 수백 구간 반복.

원인: Nav2 가 목표 근처에서 `TaskResult.FAILED` 를 내는 경우가 있는데, commander 가
`SUCCEEDED` 만 성공으로 처리함 → Isaac 은 `/start_pick` 을 못 받아 같은 goal 재발행.

해결: `commander` 를 재빌드해 두었음 (`NEAR_GOAL_M=0.75` 이하면 FAILED 여도 도착 간주).
터미널3 만 재시작:

```bash
# Ctrl+C 로 기존 commander 종료 후
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source ~/cobot3_ws/install/setup.bash
ros2 run commander trash_can_nav_pick_mission_v2
```

정상 시: `FAILED 이지만 목표 근접 ... → '/start_pick'=True 발행` 후 T2 가 파지/YOLO 로 진행.

※ 원본 `trash_can_nav_pick_mission` 은 건드리지 않음. 4_v2 전용은 `_v2` 실행 파일.

## RViz / Nav2 vs YOLO 구분

- Nav2 launch 의 RViz 는 맵·라이다·경로용. (환경에 따라 창이 2개 보일 수 있음 — 둘 다 같은 nav 설정이면 Nav2 용)
- Isaac Sim 뷰포트는 RViz가 아닙니다.

역할이 다릅니다:

| 창/토픽 | 역할 |
|---------|------|
| Nav2 RViz (`/scan`, costmap, path) | 라이다로 **어디까지 갈지** (주행) |
| `/yolo/annotated` (+ OpenCV 창) | 카메라+YOLO로 **쓰레기통을 봤는지** (파지 게이트) |

### RViz에서 YOLO 박스 보기
1. Nav2 RViz 창에서 **Add → By topic**
2. `/yolo/annotated` → **Image** 선택
3. 파지 직전 게이트 구간에서 `HIT/MISS`, detection box, `GATE PASS → PICK` 배너가 실시간으로 보입니다.
4. 상태 문자열: `ros2 topic echo /yolo/status`

`--headless` 없이 T2를 띄우면 OpenCV 창 `YOLO Trash-Can Detection (v2)` 도 같이 뜹니다.