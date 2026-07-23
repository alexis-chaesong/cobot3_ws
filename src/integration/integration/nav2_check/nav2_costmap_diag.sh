#!/usr/bin/env bash
# nav2_costmap_diag.sh
# ============================================================================
# 1단계 진단 : Nav2 "No map received" / local costmap Warn 의 근본 원인을
# 하나씩 좁힌다. (carter_navigation.launch.py + Nova_Carter_ROS 씬 기준)
#
# 대상 파이프라인(known-good 구성):
#   Isaac(Nova_Carter_ROS, ROS2 bridge ON, Play)  →  /clock, /chassis/odom,
#     /front_3d_lidar/lidar_points, TF(odom→base_link→front_3d_lidar)
#   carter_navigation.launch.py  →  pointcloud_to_laserscan(/scan),
#     amcl(map→odom), map_server(/map), nav2(global/local costmap), rviz
#
# 사용:
#   1) Isaac 씬 Play + ROS2 bridge Enable 확인
#   2) carter_navigation.launch.py map:=carter_hospital_navigation.yaml 로 bringup
#   3) source /opt/ros/humble/setup.bash
#      bash nav2_costmap_diag.sh
#
# 이 스크립트는 진단만 한다. 아무것도 수정하지 않는다.
# 마지막에 [원인 판정] 블록으로 어느 구간이 끊겼는지 알려준다.
# ============================================================================
# 주의: set -u 를 쓰면 ROS setup.bash 의 미설정 변수 참조에서 스크립트가 첫 출력 전에
# 조용히 죽는다(아무것도 안 뜸). 그래서 set -u 를 쓰지 않는다.
echo ">>> nav2_costmap_diag 시작 (이 줄이 안 보이면 실행 자체가 안 된 것)"
source /opt/ros/humble/setup.bash 2>/dev/null || true

ok(){   echo -e "  [ OK ] $1"; }
no(){   echo -e "  [FAIL] $1"; }
warn(){ echo -e "  [WARN] $1"; }
hdr(){ echo; echo "==================================================================="; echo " $1"; echo "==================================================================="; }

# 진단 결과 플래그 (마지막 판정에 사용)
CLOCK_OK=0; MAP_PUB=0; MAP_ACTIVE=0; TF_MAP_ODOM=0; TF_ODOM_BASE=0
GLOBAL_CM=0; LOCAL_CM=0; SCAN_OK=0

SCAN=${SCAN_TOPIC:-/scan}                 # carter_navigation 은 3D라이다→/scan
LIDAR_FRAME=${LIDAR_FRAME:-front_3d_lidar}

# ---------------------------------------------------------------------------
hdr "0) 노드 / 토픽 인벤토리"
# ---------------------------------------------------------------------------
NODES=$(ros2 node list 2>/dev/null)
TOPICS=$(ros2 topic list 2>/dev/null)
echo "  --- 실행 중 핵심 노드 ---"
for n in /map_server /amcl /controller_server /planner_server /bt_navigator \
         /global_costmap/global_costmap /local_costmap/local_costmap \
         /pointcloud_to_laserscan /lifecycle_manager_navigation /lifecycle_manager_localization; do
  echo "$NODES" | grep -qx "$n" && ok "$n" || no "$n 없음"
done
echo "  --- 관련 토픽 ---"
echo "$TOPICS" | grep -iE "clock|/map$|/map_|scan|costmap|odom|/tf" | sed 's/^/     /'

# ---------------------------------------------------------------------------
hdr "1) /clock 이 흐르는가 (use_sim_time 노드 전체가 여기 의존)"
# ---------------------------------------------------------------------------
if echo "$TOPICS" | grep -qx "/clock"; then
  T1=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null | grep -E "sec:" | head -2 | tr -dc '0-9\n' | paste -sd. -)
  sleep 1.0
  T2=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null | grep -E "sec:" | head -2 | tr -dc '0-9\n' | paste -sd. -)
  echo "     clock 표본1=$T1  표본2=$T2"
  if [ -n "$T1" ] && [ -n "$T2" ] && [ "$T1" != "$T2" ]; then
    ok "/clock 진행 중 (시뮬 시간 증가) → use_sim_time 노드 정상 급전 가능"; CLOCK_OK=1
  else
    no "/clock 이 멈춰있거나 0 고정 → Isaac clock publish 미동작 또는 Sim Play 안 됨"
    warn "이게 원인이면: map_server/amcl/costmap 전부 시간 못 받아 아무것도 안 뜬다."
  fi
  echo "     hz(3초):"; timeout 4 ros2 topic hz /clock 2>&1 | grep -E "average|rate" | head -1 | sed 's/^/       /'
else
  no "/clock 토픽 없음 → Isaac ROS2 bridge(isaacsim.ros2.bridge) 가 꺼졌거나 clock graph 미배선"
fi

# ---------------------------------------------------------------------------
hdr "2) /map : map_server 가 실제로 발행하는가 (QoS=transient_local 주의!)"
# ---------------------------------------------------------------------------
# /map 은 latched(transient_local) 라 hz 로는 주기 안 잡힘 → transient_local 로 echo 해야 받는다.
echo "  --- map_server 라이프사이클 상태 ---"
LC=$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null)
echo "     map_server: ${LC:-(응답없음)}"
echo "$LC" | grep -qi "active" && { ok "map_server = active (configure→activate 정상)"; MAP_ACTIVE=1; } \
                              || no "map_server 가 active 아님 → configure/activate 실패 (lifecycle_manager 로그 확인)"

echo "  --- /map 수신 시도 (QoS transient_local reliable) ---"
MAPHDR=$(timeout 5 ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
         | grep -E "width|height|resolution|frame_id")
if [ -n "$MAPHDR" ]; then
  ok "/map 수신됨 (transient_local 로는 정상 도착)"; MAP_PUB=1
  echo "$MAPHDR" | sed 's/^/       /'
  warn "→ 만약 RViz 에서만 'No map received' 라면: RViz Map 디스플레이 QoS Durability 를"
  warn "  Transient Local 로 바꿔야 한다 (Volatile 이면 latched 맵을 못 받음). costmap 자체는 정상."
else
  no "/map 을 transient_local 로도 못 받음 → map_server 가 발행 안 하는 중 (active 아님/yaml 로드 실패)"
fi

# ---------------------------------------------------------------------------
hdr "3) TF 체인 : map→odom(amcl) , odom→base_link(Isaac) , base_link→$LIDAR_FRAME"
# ---------------------------------------------------------------------------
echo "  --- map → odom (amcl 이 publish; localization 활성 시) ---"
OUT=$(timeout 4 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -E "Translation|Failure|Exception|does not exist" | head -2)
echo "${OUT:-     (없음)}" | sed 's/^/     /'
echo "$OUT" | grep -qi "Translation" && { ok "map→odom 존재 (amcl 살아있음)"; TF_MAP_ODOM=1; } \
                                     || no "map→odom 없음 → amcl 미활성 / initial_pose 미설정 (RViz 2D Pose Estimate 필요)"

echo "  --- odom → base_link (Isaac IsaacComputeOdometry) ---"
OUT=$(timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation|Failure|Exception|does not exist" | head -2)
echo "${OUT:-     (없음)}" | sed 's/^/     /'
echo "$OUT" | grep -qi "Translation" && { ok "odom→base_link 존재"; TF_ODOM_BASE=1; } \
                                     || no "odom→base_link 없음 → Isaac odom/TF graph 미배선 또는 Sim 정지"

echo "  --- base_link → $LIDAR_FRAME (Carter 내장 TF) ---"
timeout 4 ros2 run tf2_ros tf2_echo base_link "$LIDAR_FRAME" 2>&1 | grep -E "Translation|Failure|does not exist" | head -2 | sed 's/^/     /'

# ---------------------------------------------------------------------------
hdr "4) /scan : costmap obstacle layer 입력 (pointcloud_to_laserscan 출력)"
# ---------------------------------------------------------------------------
if echo "$TOPICS" | grep -qx "$SCAN"; then
  echo "     hz(4초):"; timeout 5 ros2 topic hz "$SCAN" 2>&1 | grep -E "average|rate" | head -1 | sed 's/^/       /'
  FR=$(timeout 4 ros2 topic echo "$SCAN" --once 2>/dev/null | grep -E "frame_id" | head -1)
  echo "     $FR"
  echo "$FR" | grep -q "$LIDAR_FRAME" && { ok "/scan frame_id 일치"; SCAN_OK=1; } || warn "/scan frame_id 가 $LIDAR_FRAME 아님 → TF/costmap 불일치 가능"
else
  no "$SCAN 없음 → pointcloud_to_laserscan 미실행 또는 /front_3d_lidar/lidar_points 안 나옴"
fi

# ---------------------------------------------------------------------------
hdr "5) costmap 출력 : global / local"
# ---------------------------------------------------------------------------
for CM in /global_costmap/costmap /local_costmap/costmap; do
  if echo "$TOPICS" | grep -qx "$CM"; then
    RES=$(timeout 6 ros2 topic echo "$CM" --once --qos-durability transient_local 2>/dev/null \
          | grep -E "width|height|resolution" | head -3 | tr '\n' ' ')
    if [ -n "$RES" ]; then
      ok "$CM 수신됨 : $RES"
      [ "$CM" = "/global_costmap/costmap" ] && GLOBAL_CM=1 || LOCAL_CM=1
    else
      no "$CM 토픽은 있으나 데이터 미수신 (costmap 노드 not activated / 입력 부족)"
    fi
  else
    no "$CM 토픽 없음 → 해당 costmap 노드 미활성"
  fi
done

# ---------------------------------------------------------------------------
hdr "[원인 판정]  (수정은 승인 후 — 여기선 원인만 특정)"
# ---------------------------------------------------------------------------
if [ "$CLOCK_OK" -eq 0 ]; then
  echo "  ▶ 1순위 원인 후보: /clock 미진행."
  echo "    Isaac 에서 ROS2 bridge(isaacsim.ros2.bridge) Enable + Sim Play + clock graph 확인."
  echo "    use_sim_time=True 노드 전체가 시간 못 받아 map/costmap 이 안 뜬다. 여기부터 고칠 것."
elif [ "$MAP_ACTIVE" -eq 0 ] || { [ "$MAP_PUB" -eq 0 ] && [ "$MAP_ACTIVE" -eq 1 ]; }; then
  echo "  ▶ 1순위 원인 후보: map_server 라이프사이클/맵 로드."
  echo "    lifecycle_manager_localization 로그, yaml_filename, 맵 png 경로 확인."
elif [ "$MAP_PUB" -eq 1 ] && [ "$GLOBAL_CM" -eq 0 ]; then
  if [ "$TF_MAP_ODOM" -eq 0 ]; then
    echo "  ▶ 1순위 원인 후보: TF map→odom 부재(amcl/initial_pose)."
    echo "    /map 은 발행되는데 global costmap 이 map 프레임으로 못 올라온다. RViz '2D Pose Estimate' 로 초기위치."
  else
    echo "  ▶ 1순위 원인 후보: global_costmap 노드 미활성(lifecycle) 또는 QoS."
  fi
elif [ "$MAP_PUB" -eq 1 ] && [ "$GLOBAL_CM" -eq 1 ]; then
  echo "  ▶ 데이터 파이프라인은 정상. RViz 만 'No map received' 라면 RViz 쪽 QoS 문제:"
  echo "    RViz Map/Costmap 디스플레이 QoS Durability = Transient Local 로 설정."
  echo "    (map_server·costmap 은 latched 라 Volatile 구독자는 못 받는다.)"
else
  echo "  ▶ 복합/부분 실패. 위 [FAIL] 항목을 위→아래 순서로 하나씩 해결(clock→map→TF→costmap)."
fi
echo "  요약 플래그: clock=$CLOCK_OK map_pub=$MAP_PUB map_active=$MAP_ACTIVE"\
"tf(map-odom)=$TF_MAP_ODOM tf(odom-base)=$TF_ODOM_BASE scan=$SCAN_OK global_cm=$GLOBAL_CM local_cm=$LOCAL_CM"
echo "==================================================================="
