#!/usr/bin/env bash
# =====================================================================
# [터미널 5, 19_ 전용] YOLO 비전 뷰어 — 웹(HMI v2) 비전 패널 연동용.
#   19_dual_task_select_yolo_integrated.py 가 발행하는 우측 RealSense
#   (/carterN/realsense/color/image_raw) 를 구독해 YOLO 추론 →
#     (a) /carterN/person_alert (Bool)                : 19_ 사람회피 게이트
#     (b) /carterN/vision/annotated/compressed         : bbox 그린 프레임(--publish-annotated 기본 on)
#         → backend_v2(:8001) robot_bridge 가 구독 → GET /api/vision/{carter}/stream 로 MJPEG 중계
#         → frontend_v2(:5174) VisionFeedPanel 에 표시(carter1/carter2 토글).
#   ※ 이 뷰어를 안 띄우면 웹 비전 패널이 빈 화면이다(코드는 다 연결돼 있고, 프레임 공급원이 이 뷰어).
#     18_ 때 수동으로 띄우던 것을 19_ 실행 흐름에 스크립트로 편입한 것.
#
#   ★선행 순서★ : Isaac 19_ Play ▶(/clock) → run_nav.sh → backend_v2 기동(:8001) →
#     (이 스크립트) → 브라우저 http://localhost:5174 비전 패널.
#
#   ★로봇 범위★ : 19_ 은 두 로봇 모두 YOLO(우측 RealSense)를 가지므로 기본 carter1+carter2 둘 다
#     처리한다(프론트 비전 토글 양쪽 대응). CPU 추론이 무거워 끊김이 생기면 아래처럼 한 대만 :
#         VISION_ROBOTS=carter1 ~/cobot3_ws/run_vision_19.sh
#     또는 부하 완화 플래그를 그대로 덧붙여 실행(뒤 인자는 뷰어로 그대로 전달) :
#         ~/cobot3_ws/run_vision_19.sh --alternate            # 로봇/캠 시간분할
#         ~/cobot3_ws/run_vision_19.sh --no-window            # OpenCV 창 없이(웹만)
#   인자 기본값(사용자 라이브 검증값) : --device cpu --imgsz 320 --rate 4.
# =====================================================================
# 진단 env (비대화형 셸이라 .bashrc 안 탐 → 명시. 안 넣으면 토픽 수신 0 거짓음성, 인수인계서 15-1)
export ROS_DOMAIN_ID=151
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
# 시스템 ROS + 워크스페이스 오버레이 (뷰어는 시스템 python3+rclpy 사용 — Isaac env 는 섞지 말 것)
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash

ROBOTS="${VISION_ROBOTS:-carter1 carter2}"

cd /home/rokey/cobot3_ws
echo "[run_vision_19] YOLO 뷰어 시작 — robots=[$ROBOTS], RealSense(realsense/color/image_raw)"
echo "[run_vision_19] annotated 프레임 → backend_v2(:8001) → 웹(:5174) 비전 패널. Ctrl+C 로 종료."
exec python3 src/perception/perception/multi_robot_yolo_viewer.py \
  --robots $ROBOTS \
  --image-suffix realsense/color/image_raw \
  --device cpu --imgsz 320 --rate 4 \
  "$@"
