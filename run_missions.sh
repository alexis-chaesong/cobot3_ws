#!/usr/bin/env bash
# =====================================================================
# [터미널 3] 두 미션 동시 실행 (carter1=소독 sweep + carter2=폐기물 nav-pick)
#   Nav2 가 active 되고 두 로봇이 localize 된 뒤(RViz 에 두 로봇 축이 뜬 뒤) 실행.
#   Ctrl+C 한 번이면 두 미션 모두 정리하고 종료.
#   개별로 돌리고 싶으면 아래 두 줄을 각각 다른 터미널에서 실행하면 된다.
# =====================================================================
source /opt/ros/humble/setup.bash
source /home/rokey/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash

pids=()
cleanup() { echo; echo "[run_missions] 종료 — 미션 노드 정리"; kill "${pids[@]}" 2>/dev/null; }
trap cleanup INT TERM EXIT

# ★ use_sim_time:=True 필수 : Nav2·Isaac 이 sim time(/clock)인데 미션이 wall time 이면
#   goal.header.stamp 가 어긋나 bt_navigator 의 TF 변환 실패 → goal abort → 로봇이 안 움직임.
# c1 스윕 웨이포인트 : WP1=(18.8,8.0,+90°)[x를 기본 18.5 에서 +0.3 이동], WP2=(18.5,18.5,-90°)
ros2 run commander spray_waypoint_mission     --ros-args -p namespace:=carter1 -p use_sim_time:=True \
  -p sweep_x:="[18.8, 18.5]" -p sweep_y:="[8.0, 18.5]" -p sweep_yaw:="[1.5708, -1.5708]" &
pids+=($!)
ros2 run commander trash_can_nav_pick_mission --ros-args -p namespace:=carter2 -p use_sim_time:=True &
pids+=($!)

echo "[run_missions] carter1(spray)=${pids[0]}  carter2(trash)=${pids[1]}  — Ctrl+C 로 둘 다 종료"
wait
