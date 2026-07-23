#!/usr/bin/env bash
# =====================================================================
# [터미널 2] Nav2 멀티 + 통합 RViz + tf_relay×2 + initialpose 자동발행 (한 방)
#   ★ 반드시 Isaac(run_isaac.sh)에서 Play ▶ 를 눌러 /clock 이 흐른 뒤 실행할 것(6-1).
#   launch 가 아래를 모두 처리한다:
#     · map_server + amcl(carter1·carter2) + 통합 RViz 1개
#     · tf_relay_carter1 / tf_relay_carter2 (전역 /tf 중계 — 통합 RViz 표시용)
#     · 기동 12s·20s 후 /carter1·/carter2 initialpose 자동발행(localize 트리거)
#   토글 : use_tf_relay:=False / pub_initialpose:=False 로 끌 수 있음.
# =====================================================================
set -e
source /opt/ros/humble/setup.bash
source /home/rokey/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
MAP=/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation/maps/map/modified_hospital_map.yaml
exec ros2 launch carter_navigation \
  multiple_robot_carter_navigation_modified_hospital.launch.py \
  map:="$MAP" "$@"
