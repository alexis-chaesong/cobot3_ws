#!/usr/bin/env bash
# =====================================================================
# [터미널 3-HMI] run_missions.sh + 웹 HMI 게이트(wait_for_hmi_start:=True)
#   run_missions.sh 와 "완전히 동일한 환경/파라미터"를 쓴다. 다른 점은 단 하나:
#   두 미션이 실행 즉시 시작하지 않고, 웹 대시보드의 "통합 시작 / 개별 시작"(START)
#   버튼을 받을 때까지 '대기'로 멈춰 있다가 시작한다는 것.
#
#   ★ run_missions.sh 와 소스 3종 동일(특히 humble_ws) — 이게 빠지면 trash 미션의
#     BasicNavigator.waitUntilNav2Active() 가 carter2 Nav2 라이프사이클을 못 물고 멈춘다.
#   ★ 선행: Isaac Play ▶(/clock) → run_nav.sh(Nav2 active + 두 로봇 localize) → HMI 백엔드
#     실행 후 이 스크립트. 그다음 브라우저에서 통합 시작.
#   Ctrl+C 한 번이면 두 미션 모두 종료.
# =====================================================================
source /opt/ros/humble/setup.bash
source /home/rokey/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash

pids=()
cleanup() { echo; echo "[run_missions_hmi] 종료 — 미션 노드 정리"; kill "${pids[@]}" 2>/dev/null; }
trap cleanup INT TERM EXIT

# c1(소독) : ★16번(dual SG 툴체인지)★ 첫 웨이포인트 = 노즐 거치대 접근 pose(=carter1 홈 18.5,0,yaw0).
#   16_dual_sg_tool_changer_integrated.py 의 g_carter1_mission 은 "첫 start_sweep = 노즐 파지(툴체인지),
#   이후 start_sweep = 소독 스윕" 이라, 거치대 웨이포인트가 맨 앞에 없으면 carter1 이 벽에서 노즐을
#   잡으려다 실패한다. 뒤 두 개(18.8,8.0 / 18.5,18.5)가 기존 벽면 스윕 웨이포인트.
ros2 run commander spray_waypoint_mission     --ros-args -p namespace:=carter1 -p use_sim_time:=True \
  -p wait_for_hmi_start:=True -p dock_first:=True \
  -p sweep_x:="[18.5, 18.8, 18.5]" -p sweep_y:="[0.0, 8.0, 18.5]" -p sweep_yaw:="[0.0, 1.5708, -1.5708]" &
pids+=($!)

# c2(폐기물) : HMI 게이트만 추가
ros2 run commander trash_can_nav_pick_mission --ros-args -p namespace:=carter2 -p use_sim_time:=True \
  -p wait_for_hmi_start:=True &
pids+=($!)

echo "[run_missions_hmi] carter1(spray)=${pids[0]}  carter2(trash)=${pids[1]}"
echo "[run_missions_hmi] 두 미션 '대기' 상태 — 웹 '통합 시작'을 누르면 시작. Ctrl+C 로 둘 다 종료."
wait
