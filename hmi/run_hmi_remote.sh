#!/usr/bin/env bash
# =====================================================================
# [다른 노트북 전용] 19_ HMI(backend_v2 + frontend_v2)를 원격 노트북에서 실행.
#   메인 머신(Isaac 19_ / Nav2 / 미션 / YOLO 뷰어)과 같은 ROS2 도메인(151)으로 크로스머신 연결.
#
#   ★핵심★ : backend(rclpy)가 반드시 올바른 ROS env(도메인151 + rmw_fastrtps + whitelist)로 떠야
#            비전/명령이 오간다. 이 env 누락이 "비전 안 들어옴 / 명령 전달 안 됨"의 주원인
#            (셸에서 ros2 topic list 는 되는데 uvicorn 터미널엔 env 가 없던 경우).
#
#   전제(다른 노트북) :
#     · ROS2 humble 베이스 설치(std/geometry/sensor/tf2_msgs, tf2_ros, rclpy, rmw_fastrtps_cpp).
#       ※ commander·cobot3_ws 워크스페이스는 불필요(backend 는 표준 메시지만 사용).
#     · pip : fastapi uvicorn opencv-python  (backend 의존성)
#     · node/npm : frontend_v2 최초 1회 `npm install`
#     · $HOME/.ros/fastdds_whitelist.xml 에 ★이 노트북 LAN 인터페이스★ 포함(크로스머신 discovery).
#
#   사용 : hmi/ 폴더(backend_v2 + frontend_v2 + 이 스크립트)를 다른 노트북에 복사 후 ./run_hmi_remote.sh
#          Ctrl+C 한 번으로 backend/frontend 둘 다 종료.
# =====================================================================
set -euo pipefail

# ── ROS2 크로스머신 env (메인과 동일 도메인/RMW/whitelist) ──
export ROS_DOMAIN_ID=151
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# ★ cobot3_ws/install 은 소싱하지 않는다(commander 불필요) ★

HMI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[run_hmi_remote] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[run_hmi_remote] whitelist=$FASTRTPS_DEFAULT_PROFILES_FILE"
if [[ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
  echo "[run_hmi_remote][WARN] whitelist 파일이 없습니다 — 크로스머신 discovery 가 막힐 수 있음."
fi

# ── (선행 점검) 메인 머신 토픽이 실제로 보이는지 — 안 보이면 env/whitelist 문제 ──
echo "[run_hmi_remote] 메인 머신 토픽 점검(3초)…"
if timeout 3 ros2 topic list 2>/dev/null | grep -qE "process_state|vision/annotated"; then
  echo "[run_hmi_remote]  ✓ carterN 토픽 보임 — DDS 연결 OK"
else
  echo "[run_hmi_remote]  ⚠ carterN 토픽 안 보임 — 메인 머신이 실행 중인지 / 도메인·whitelist 확인 필요"
  echo "[run_hmi_remote]    (계속 진행하지만 비전/명령이 안 될 수 있음)"
fi

pids=()
cleanup() { echo; echo "[run_hmi_remote] 종료 — backend/frontend 정리"; kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup INT TERM EXIT

# ── backend_v2 (FastAPI + rclpy 브릿지, 포트 8001) ──
( cd "$HMI_DIR/backend_v2" && exec uvicorn main:app --host 0.0.0.0 --port 8001 ) &
pids+=($!)
echo "[run_hmi_remote] backend_v2  → http://0.0.0.0:8001  (pid ${pids[-1]})"

# ── frontend_v2 (Vite, 포트 5174) — backend 가 이 노트북 로컬이라 .env 의 localhost:8001 그대로 OK ──
( cd "$HMI_DIR/frontend_v2" && exec npm run dev ) &
pids+=($!)
echo "[run_hmi_remote] frontend_v2 → http://localhost:5174  (pid ${pids[-1]})"

echo "[run_hmi_remote] 브라우저에서 http://localhost:5174 열기. Ctrl+C 로 둘 다 종료."
wait
