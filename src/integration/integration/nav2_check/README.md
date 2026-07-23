# 작업 0 — Hospital 복도 벽이 Nav2 costmap 에 잡히는지 확인

맵(SLAM) 없이 **라이다 `/scan` 만으로 복도 벽이 local costmap 에 장애물로 뜨는지**
확인하기 위한 최소 검증 세트.

## 진단 요약 (정적 분석 결과)

| 항목 | 상태 | 근거 |
|------|------|------|
| 복도 벽 시각 메시 | ✅ 있음 | `hospital.usd` → `Props/Geo_M3_SideWall.usd` 등 |
| 복도 벽 물리 충돌체 | ✅ 있음 | 벽 메시 `Section0/1` 에 `PhysicsCollisionAPI`+`PhysicsMeshCollisionAPI`(approx=none) |
| 라이다/TF/odom/clock 발행 | ✅ **내장** | `Nova_Carter_ROS.usd` OmniGraph 에 이미 전부 있음(아래 표) |
| Nav2 스택 | ✅ 설치됨 | navigation2 / nav2-bringup |
| hospital+Carter 통합 씬 | ⚙️ 필요 | 아래 "2. Isaac 씬 준비" |

### Nova_Carter_ROS.usd 내장 ROS2 인터페이스 (배선 완료, 새로 짤 필요 없음)
| 데이터 | 토픽 | 프레임 | 노드 |
|--------|------|--------|------|
| 전방 2D 라이다(벽) | `/front_2d_lidar/scan` (LaserScan) | `front_2d_lidar` | ROS2RtxLidarHelper(laser_scan) |
| 후방 2D 라이다 | `/back_2d_lidar/scan` | `back_2d_lidar` | 〃 |
| 전방 3D 라이다 | `/front_3d_lidar/lidar_points` (PointCloud2) | `front_3d_lidar` | ROS2RtxLidarHelper(point_cloud) |
| Odometry | `/chassis/odom` | odom→base_link | ROS2PublishOdometry |
| TF | `/tf` : **odom→base_link→front_2d_lidar** (+ 전 센서) | — | RawTransformTree + TransformTree×3 |
| Clock | `/clock` | — | ROS2PublishClock |
| 주행 명령 | Twist 구독(cmd_vel) | — | ROS2SubscribeTwist |

- 프레임 일치 확인: `.../front_RPLidar/RPLidar_S2E` prim 의 `isaac:nameOverride="front_2d_lidar"`
  = LaserScan frame_id → **static TF 불필요**.

> 결론: **벽 에셋·센서 배선·Nav2 모두 준비됨.** 남은 건 hospital+Carter 를 한 씬에
> 놓고 ROS2 bridge 를 켠 뒤 Play → costmap 실행뿐.

## 실행 순서

### 1. Nav2 설치 — 완료됨 ✅
(navigation2, nav2-bringup 설치 확인)

### 2. Isaac 씬 준비 : hospital 복도 + Nova Carter
라이다/TF/odom/clock 그래프는 **Nova_Carter_ROS.usd 에 내장**돼 있으므로 OmniGraph
작업이 필요 없다. 아래 중 하나로 "복도 안에 Carter 가 있는 씬"을 만들고 **Play**:

- (권장) 이미 주행에 쓰던 씬(예: `Demo_scene_save.usd`, Carter+hospital 참조 포함)을 그대로 사용.
- 또는 GUI 에서 `hospital_hallway.usd` 를 열고 Nova_Carter_ROS 를 복도 안(벽 사이 빈
  공간)으로 드래그해 배치. Carter 가 벽에 겹치지 않게 놓을 것.

Play 전에 확인:
- **ROS2 Bridge 확장 켜기** : Window → Extensions → `isaacsim.ros2.bridge` Enable
  (내장 그래프가 발행되려면 필수). 씬에 ROS2 노드가 있으면 자동 활성화되기도 함.
- 라이다가 복도 벽 높이를 스캔하는지(전방 2D 라이다는 차체 낮은 위치의 수평 스캔).

Play 후 데이터 점검:
```bash
source /opt/ros/humble/setup.bash
bash /home/rokey/cobot3_ws/src/integration/integration/nav2_check/check_pipeline.sh
```
`/front_2d_lidar/scan` hz>0, `odom→base_link` 및 `base_link→front_2d_lidar` TF 가
나오면 준비 완료.

### 3. local costmap 실행 + 육안 확인
```bash
source /opt/ros/humble/setup.bash
NAV2_CHECK=/home/rokey/cobot3_ws/src/integration/integration/nav2_check

# 라이다 TF 가 Isaac 에서 나오면:
ros2 launch $NAV2_CHECK/costmap_check.launch.py rviz:=true

# base_link→라이다 TF 가 안 나오면 static TF 로 보완(frame 이름은 /scan frame_id 와 일치):
ros2 launch $NAV2_CHECK/costmap_check.launch.py rviz:=true \
    use_static_lidar_tf:=true lidar_frame:=front_2d_lidar lidar_z:=0.3
```
RViz 에서 **Map 디스플레이 → topic `/costmap/costmap`** 추가.

## ✅ 성공 판정
- RViz 의 `/costmap/costmap` 에 **복도 벽 라인이 장애물 셀(진한 색)로** 그려짐.
- 또는 `check_pipeline.sh` 의 4단계에서 **장애물(>0) 셀 개수 > 0**.

## ❌ 실패 시 원인별 대응
| 증상 | 원인 | 조치 |
|------|------|------|
| `/front_2d_lidar/scan` 없음 | ROS2 bridge 미활성 / 미Play / 씬에 Carter 그래프 없음 | bridge 확장 Enable, Play, 씬이 Nova_Carter_ROS 를 참조하는지 확인 |
| scan 은 나오나 costmap 비어있음 | TF 끊김(odom/base_link/front_2d_lidar) | `check_pipeline.sh` 3단계, frame_id 일치 확인 |
| costmap 이 로봇만 있고 벽 없음 | 라이다가 벽을 못 봄(높이/사거리) | 라이다 스캔 평면 높이, `obstacle_max_range`(기본12m) 확인 |
| 전부 정상인데 셀 안 뜸 | `use_sim_time`/`/clock` 불일치 | `/clock` 발행 확인, 모든 노드 `use_sim_time:=true` |
| 벽 일부만 뜸 | 라이다 사거리/각도 밖 | 정상. rolling costmap 은 로봇 주변만 표시 |

## 파일
- `local_costmap_check.yaml` — 맵 없는 local costmap(odom, /scan obstacle layer) 파라미터
- `costmap_check.launch.py` — costmap + lifecycle(+옵션 static TF/RViz)
- `check_pipeline.sh` — /scan·TF·/clock·costmap 사전점검
