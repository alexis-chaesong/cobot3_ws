#!/usr/bin/env bash
# check_pipeline.sh
# ============================================================================
# 작업 0 사전점검 : costmap 을 띄우기 전에, Isaac 쪽에서 벽 인식에 필요한
# 데이터(/scan, TF, /clock)가 실제로 나오는지 한 번에 확인한다.
#
# 사용 : (Isaac 씬을 Play 한 상태에서)
#   source /opt/ros/humble/setup.bash
#   bash check_pipeline.sh
# ============================================================================
set -u
source /opt/ros/humble/setup.bash 2>/dev/null

ok(){ echo -e "  [ OK ] $1"; }
no(){ echo -e "  [FAIL] $1"; }

echo "==================================================================="
echo " 0) Nav2 설치 확인"
echo "==================================================================="
if ros2 pkg prefix nav2_costmap_2d >/dev/null 2>&1; then
  ok "nav2_costmap_2d 설치됨 ($(ros2 pkg prefix nav2_costmap_2d))"
else
  no "nav2_costmap_2d 없음 → sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
fi

SCAN=${SCAN_TOPIC:-/front_2d_lidar/scan}   # Nova_Carter_ROS 내장 전방 2D 라이다
LIDAR_FRAME=${LIDAR_FRAME:-front_2d_lidar}

echo "==================================================================="
echo " 1) 토픽 존재 확인 ($SCAN /clock /tf)"
echo "==================================================================="
TOPICS=$(ros2 topic list 2>/dev/null)
for t in "$SCAN" /clock /tf /tf_static; do
  echo "$TOPICS" | grep -qx "$t" && ok "$t 존재" || no "$t 없음"
done
echo "  (참고) 관련 토픽 전체:"
echo "$TOPICS" | grep -iE "scan|clock|tf|odom|costmap|point|cmd_vel|twist" | sed 's/^/     /'

echo "==================================================================="
echo " 2) $SCAN 발행 주파수 & 샘플 (라이다가 벽을 보는가)"
echo "==================================================================="
echo "  --- ros2 topic hz $SCAN (5초) ---"
timeout 6 ros2 topic hz "$SCAN" 2>&1 | sed 's/^/     /' | head -6
echo "  --- $SCAN 1회 헤더/범위 ---"
timeout 4 ros2 topic echo "$SCAN" --once 2>/dev/null \
  | grep -E "frame_id|range_min|range_max|angle_min|angle_max" | sed 's/^/     /'
echo "  (frame_id 가 '$LIDAR_FRAME' 인지 확인 → costmap yaml/launch 와 일치해야 함)"

echo "==================================================================="
echo " 3) TF 체인 확인 (odom→base_link→$LIDAR_FRAME)"
echo "==================================================================="
echo "  --- odom → base_link ---"
timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation|At time|Failure|Exception" | head -4 | sed 's/^/     /'
echo "  --- base_link → $LIDAR_FRAME (Carter 내장 TF tree) ---"
timeout 4 ros2 run tf2_ros tf2_echo base_link "$LIDAR_FRAME" 2>&1 | grep -E "Translation|At time|Failure|Exception" | head -4 | sed 's/^/     /'
echo "  (둘 다 Translation 이 나오면 costmap 이 scan 을 odom 으로 변환 가능)"

echo "==================================================================="
echo " 4) costmap 출력 확인 (costmap 노드 실행 후 별도 실행)"
echo "==================================================================="
if echo "$TOPICS" | grep -q "/costmap/costmap"; then
  echo "  --- /costmap/costmap 에서 장애물 셀(값>0) 개수 ---"
  timeout 5 ros2 topic echo /costmap/costmap --once 2>/dev/null \
    | python3 -c "import sys,re;d=sys.stdin.read();import ast;\
m=re.search(r'data:\s*\[([^\]]*)\]',d,re.S);\
vals=[int(x) for x in m.group(1).replace('\n',' ').split(',') if x.strip().lstrip('-').isdigit()] if m else [];\
print('     총 셀:',len(vals),' 장애물(>0):',sum(1 for v in vals if v>0),' 최대값:',max(vals) if vals else 'NA')" 2>/dev/null \
    || echo "     (파싱 실패 - RViz 로 육안 확인 권장)"
  echo "  → 장애물(>0) 셀이 0 보다 크면 벽이 costmap 에 반영된 것"
else
  no "/costmap/costmap 없음 → 먼저 costmap_check.launch.py 를 실행하세요"
fi
echo "==================================================================="
echo " 점검 종료"
echo "==================================================================="
