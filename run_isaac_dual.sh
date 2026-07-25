#!/usr/bin/env bash
# =====================================================================
# [터미널 1] Isaac Sim 듀얼 SG 툴체인저 통합 씬 (16_dual_sg_tool_changer_integrated.py)
#   창이 뜨면 GUI 에서 반드시 Play ▶ 를 눌러야 /clock 이 흐른다.
#   콘솔에 [SPAWN] c1/c2 좌표·[NS] 로그, carter1 은 이후 거치대 웨이포인트에서
#   [TOOLCHANGE] 노즐 파지 성공 로그가 뜨는지 확인.
#   ※ 연속 Play 전제(Stop→Play 재개 미지원) — 재시작은 이 스크립트 재실행.
#   환경변수 옵션 : ISAAC_HEADLESS=1(창없이) / LIVESTREAM=1(원격 노트북 WebRTC 스트리밍,
#   NVIDIA Isaac Sim WebRTC Streaming Client 필요).
# =====================================================================
set -e
ISAAC="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
# ROS2 브리지 라이브러리 경로(= .bashrc 의 isaac_ros alias 와 동일) — 이게 없으면
# isaacsim.ros2.bridge extension startup 이 실패하고, 연쇄로 rclpy import 도 실패한다.
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC/exts/isaacsim.ros2.bridge/humble/lib"
cd /home/rokey/cobot3_ws
exec "$ISAAC/python.sh" isaacpjt/M0609/16_dual_sg_tool_changer_integrated.py "$@"
