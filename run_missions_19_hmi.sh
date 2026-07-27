#!/usr/bin/env bash
# =====================================================================
# [터미널 3-HMI, 19_ 전용] 19_dual_task_select_yolo_integrated.py(작업선택식+YOLO)용
#   ROS 미션노드 2개 — 둘 다 trash_can_nav_pick_mission 을 "순수 Nav2 goal 릴레이"로만 쓴다.
#   (역할고정 모델의 run_missions_hmi.sh 와 달리 spray_waypoint_mission 은 아예 안 씀 —
#    19_ 은 두 로봇 다 g_run_nav_leg 하나로 trash/spray 내비게이션을 전부 처리하므로,
#    ROS 쪽은 어느 작업인지 몰라도 되는 제네릭 goal↔start_pick 왕복만 하면 된다.)
#
#   ★ -p hmi_enable:=False ★ : 이 노드의 구식 HMI 발행(트래시 전용 고정 라벨 "폐기물통 파지"
#     등)을 끈다 — 19_ 자신이 /carterN/process_state 를 직접 발행하므로(publish_hmi_state,
#     19_ 소스 참고), 같은 토픽에 두 발행자가 붙어 라벨이 충돌/깜빡이는 걸 막기 위함.
#     ⚠ 알려진 제약 : hmi_enable:=False 면 이 노드의 estop 시 Nav2 goal 취소 훅도 같이 꺼진다.
#     즉 긴급정지 시 "Nav2 가 standoff 로 주행 중인 구간"은 취소되지 않고 계속 주행할 수 있음
#     (costmap 기반이라 장애물/사람은 여전히 피함). 19_ 자체 estop 게이팅(최종접근 cmd_vel,
#     스윕)은 정상 작동하므로 실제 파지/분사 등 물리 작업은 확실히 멈춘다 — HMI v2 계획 문서의
#     "알려진 제약" 참고, ROS 미션노드 코드는 이번 범위에서 안 고친다.
#
#   ★ carter2 시작pose 오버라이드 필수 ★ : CARTER_START_POSES 테이블의 carter2 기본값은
#     구버전(y=-0.0029517927591273807, yaw=0) 그대로라, 이 노드의 nav.setInitialPose() 가
#     AMCL 을 다시 그 틀린 위치로 되돌린다(params_2.yaml 의 AMCL 초기위치 수정이 무효화됨) —
#     반드시 -p start_y/-p start_yaw_deg 로 19_ 의 실제 스폰(C2_START_POSE)과 맞춰야 한다.
#     carter1 은 테이블에 이미 (18.5, 0.2317, 90.0) 로 올바르게 들어있어 오버라이드 불필요.
#
#   ★ 선행 : Isaac 19_ Play ▶(/clock) → run_nav.sh(Nav2 active + 두 로봇 localize) →
#     HMI v2 백엔드(hmi/backend_v2, 포트 8001) 실행 후 이 스크립트. 그다음 브라우저(5174)에서
#     로봇별 작업(trash/spray) 선택.
#   ⚠ 웹 비전 패널("사람 감지")은 이 스크립트가 안 켠다 — 별도 6번째 터미널로
#     `~/cobot3_ws/run_vision_19.sh` 를 반드시 같이 띄워야 carter1/carter2 화면이 표시됨
#     (안 띄우면 backend_v2 로 프레임이 전혀 안 들어와 프론트가 "재연결 중…"만 계속 반복함).
#   Ctrl+C 한 번이면 두 미션 모두 종료.
# =====================================================================
source /opt/ros/humble/setup.bash
source /home/rokey/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash

pids=()
cleanup() { echo; echo "[run_missions_19_hmi] 종료 — 미션 노드 정리"; kill "${pids[@]}" 2>/dev/null; }
trap cleanup INT TERM EXIT

# carter1 : CARTER_START_POSES 테이블에 이미 (18.5, 0.2317, 90.0) 있음 — 오버라이드 불필요.
ros2 run commander trash_can_nav_pick_mission --ros-args -p namespace:=carter1 -p use_sim_time:=True \
  -p hmi_enable:=False &
pids+=($!)

# carter2 : 테이블 기본값이 구버전이라 반드시 오버라이드(19_ C2_START_POSE 와 일치).
ros2 run commander trash_can_nav_pick_mission --ros-args -p namespace:=carter2 -p use_sim_time:=True \
  -p hmi_enable:=False -p start_y:=0.2287482072408726 -p start_yaw_deg:=90.0 &
pids+=($!)

echo "[run_missions_19_hmi] carter1=${pids[0]}  carter2=${pids[1]}  (둘 다 순수 Nav2 goal 릴레이, hmi_enable=False)"
echo "[run_missions_19_hmi] 19_ 이 /carterN/process_state 직접 발행 — 웹(5174)에서 로봇별 작업(trash/spray) 선택."
echo "[run_missions_19_hmi] Ctrl+C 로 둘 다 종료."
wait
