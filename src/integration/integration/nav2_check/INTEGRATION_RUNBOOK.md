# Nav2 + 소독팔 통합 실행 런북 (hospital)

목표 : Nav2 가 베이스를 주행/localize 하고, waypoint 정지 시 팔이 소독(위아래+방사).
구성 = **10_carter_hospital_spray_nav.py**(Isaac 씬 + 팔, Nav2 가 /cmd_vel 로 주행)
     + **carter_navigation**(Nav2 bringup, hospital 맵)
     + **spray_waypoint_mission**(waypoint 순회 + /spray_active 트리거).

> "No map received" 의 핵심 원인은 대부분 **실행 순서**다. `use_sim_time:True` 인 Nav2
> 노드(map_server/amcl/costmap)는 `/clock` 이 흐르지 않으면 활성화에 실패한다.
> 그래서 **Isaac 을 먼저 Play(=/clock 발행) 한 뒤 Nav2 를 띄운다.** RViz Map QoS 는
> 이미 Transient Local 로 정상(carter_navigation.rviz) → RViz 설정은 건드릴 필요 없음.

---

## 순서 (반드시 이 순서)

### ① Isaac : 씬 + 팔 노드 실행 → Play
```bash
~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/M0609/10_carter_hospital_spray_nav.py
```
- GUI 가 뜨면 **Carter 가 복도 자유공간에 있는지** 확인. 벽에 박혀 있으면
  스크립트 상단 `CARTER_START_POSE` 를 조정하고 재실행.
- **Play ▶** 를 누른다. (이 순간부터 `/clock`, `/scan`, `/tf`, `/chassis/odom` 발행)

### ② 검증 : /clock 이 흐르는가 (게이트 — 여기서 막히면 아래 다 실패)
새 터미널:
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /clock          # 값이 올라오면 OK (멈춰있으면 Isaac Play/브리지 확인)
ros2 topic echo /clock --once # sec 이 0 이 아니고 증가하는지
```
- `/clock` 이 안 나오면 : Isaac 이 Play 상태인지, `10번`이 `isaacsim.ros2.bridge` 를
  enable 했는지 확인. **여기가 통과돼야 Nav2 가 뜬다.**

### ③ Nav2 bringup (hospital 맵을 반드시 명시!)
```bash
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=$(ros2 pkg prefix carter_navigation)/share/carter_navigation/maps/carter_hospital_navigation.yaml
```
- `map:=` 를 빼면 기본값이 **warehouse 맵**이라 엉뚱한 맵이 뜬다(반드시 지정).
- RViz 가 함께 뜬다. Fixed Frame = map.

### ④ 검증 : 맵/코스트맵이 실제로 오는가
```bash
bash ~/cobot3_ws/src/integration/integration/nav2_check/nav2_costmap_diag.sh
```
- 스크립트 마지막 **[원인 판정]** 을 본다.
  - `map_active=1 map_pub=1` 인데 RViz 만 비어 보이면 → RViz 에서 Map 체크/Fixed Frame 확인.
  - `map_pub=0` → map_server 미활성(주로 ②/clock 문제) → ①②로 복귀.
  - `global_cm=0` 인데 `map_pub=1` 이고 `tf(map-odom)=0` → localization(초기위치) 문제 → ⑤.

### ⑤ 초기위치 정렬 (localization) — 예전 "빙빙" 방지의 핵심
AMCL 기본 초기위치는 `(-6, -1, yaw π)`(map 프레임). Carter 실제 스폰과 다르면
로봇이 잘못 localize 되어 planner 가 헤매고 recovery 로 **제자리 회전**한다.
- 가장 확실한 방법 : RViz 상단 **2D Pose Estimate** 클릭 → 맵에서 로봇의 실제
  위치·방향에 맞춰 화살표를 찍는다. costmap 의 로봇이 벽·복도와 맞물리면 성공.
- TF 확인 :
```bash
ros2 run tf2_ros tf2_echo map odom       # Translation 나오면 amcl 살아있음
ros2 run tf2_ros tf2_echo map base_link  # 로봇이 map 상 어디에 있는지
```

### ⑥ 팔 단독 확인 (Nav2 와 무관하게 모션 정상인지)
```bash
ros2 topic pub -1 /spray_active std_msgs/msg/Bool "{data: true}"   # 위아래 소독 시작
# ... 팔이 움직이는지 확인 ...
ros2 topic pub -1 /spray_active std_msgs/msg/Bool "{data: false}"  # 접힌 자세로
```
- 움직이면 팔 파이프라인 정상. (10번은 True 구간에서만 와이프, False 면 stow.)

### ⑦ 통합 주행 + 소독
- 간단 확인 : RViz **Nav2 Goal** 로 복도 앞쪽을 한 점 찍어 주행되는지.
- 자동 순회 :
```bash
source ~/cobot3_ws/install/setup.bash
ros2 run commander spray_waypoint_mission --ros-args \
  -p waypoints_x:="[...]" -p waypoints_y:="[...]" \
  -p waypoints_yaw:="[...]" -p waypoints_spray:="[true, ...]" \
  -p spray_duration:=5.0
```
  waypoint 좌표는 map 프레임. RViz **Publish Point** 로 복도 지점을 클릭하면
  터미널에 좌표가 찍히니 그 값으로 채운다. (맵 범위 X[-50,27] Y[-6,36])

---

## 순서 요약 (한 줄)
Isaac 10번 Play → `/clock` 확인 → Nav2 bringup(map:=hospital) → diag 로 map 확인
→ 2D Pose Estimate 로 localize → `/spray_active` 로 팔 확인 → Nav2 Goal / spray_waypoint_mission.

## 자주 나는 증상 → 원인
| 증상 | 원인 | 조치 |
|---|---|---|
| RViz "No map received" | map_server 미발행(주로 /clock) | ②/clock → ③ 재bringup |
| costmap 안 뜸, 맵은 뜸 | map→odom TF 없음(초기위치) | ⑤ 2D Pose Estimate |
| 목표 주면 제자리 회전 | mislocalize / 초기위치 불일치 | ⑤ 재설정, CARTER_START_POSE 정렬 |
| 팔 안 움직임 | /spray_active 안 옴 | ⑥ 수동 pub 로 확인, mission 실행 |
| 주행 중 팔이 베이스 흔듦 | 주행 중 분사(옆으로 뻗음) | 10번은 주행 중 stow → 정지시만 분사(정상) |
