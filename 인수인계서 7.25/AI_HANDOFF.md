# cobot3_ws AI 인수인계서

> **대상**: 이어받을 AI/개발자  
> **작성**: 2026-07-25  
> **이 파일이 진입점.** perception 세부 상수·촬영 루프는 `PERCEPTION_HANDOFF.md`.  
> 멀티로봇 Nav2·소독 배경(장문): `~/Downloads/AI인수인계서 (1).txt` — 단 **env 숫자는 이 문서 우선**(DOMAIN **150**, FastDDS=`~/.config/net_loadtest/fastdds_whitelist.xml`).  
> 표기: **[개선됨]** 코드/데이터에 반영 · **[고칠점]** 아직 안 됨·불안정 · **[검증]** 파일/라이브 확인 · **[미검증]** 라이브 미확인

---

## 0. 한눈에

| | |
|--|--|
| **프로젝트** | 격리병동 소독·폐기물 **멀티로봇** (Isaac Sim 5.1 + ROS2 Humble + Nav2) |
| **워크스페이스** | `~/cobot3_ws` + Nav2용 `~/IsaacSim-ros_workspaces/humble_ws` |
| **로봇** | **carter1** 소독(+YOLO 사람 게이트) / **carter2** 폐기물(YOLO 없음, 라이다만) |
| **Isaac 메인** | `isaacpjt/M0609/14_1_multi_robot_yolotest.py` |
| **YOLO 가중치** | `src/perception/models/trash_can_person_yolo11s.pt` — **2026-07-25 15:15 학습 완료**(v2 600장 직후) **[검증]** |
| **다음 할 일** | 라이브 검출·`[AVOID]` · Nav2 goal timeout 완화 · **프레임 드랍 추가 완화 후 c2에도 YOLO** · `run_isaac.sh`→14_1 |

---

## 1. cobot3_ws 란

팀 **ROS2 Humble 워크스페이스**이자 Isaac 병원 시나리오 통합 루트.

- **하는 일**: Nova Carter + Doosan M0609로 복도 자율주행, 벽 stop-and-go 소독, 쓰레기통 파지·덤프.
- **여기 있는 것**: Isaac 스크립트(`isaacpjt/`), 미션·릴레이(`src/nova_carter/commander`), YOLO·데이터셋(`src/perception`), USD 자산(`src/assets`), `run_*.sh`.
- **밖에 있는 것**: Nav2 launch/params/맵 → `~/IsaacSim-ros_workspaces/humble_ws` (소실·복구 이력 있음 → 백업 권장).

런타임 역할:

| 프로세스 | 역할 |
|----------|------|
| Isaac `14_1` | 씬·물리·팔 FSM·`/clock`·카메라·스윕/`cmd_vel`·핸드오프 구독 |
| `run_nav.sh` | Nav2 + RViz + tf_relay + initialpose |
| `run_missions.sh` | c1 웨이포인트→`/carter1/start_sweep`, c2 파지/덤프 |
| `multi_robot_yolo_viewer.py` | 이미지→YOLO→`/carter1/person_alert` (시스템 python3) |

핸드오프는 **토픽**이다. `start_sweep.py` 같은 파일은 없다.

---

## 2. 핵심 파일 맵

### 2.1 실행·Isaac

| 파일 | 역할 | 상태 |
|------|------|------|
| `isaacpjt/M0609/14_1_multi_robot_yolotest.py` | ★현재 메인★ c1 front+side YOLO 게이트, c2 폐기물, 사람 에셋 | 사용 중 |
| `isaacpjt/M0609/14_multi_robot_yolotest.py` | 14_1 직전 계열 | 참고 |
| `isaacpjt/M0609/13_multi_robot_integrated.py` | YOLO 없는 통합 원본 | `run_isaac.sh`가 여기를 실행 **[고칠점]** |
| `run_isaac.sh` | Isaac 런처 | **13번 고정** — 14_1 쓸 때는 직접 `isaac_python` |
| `run_nav.sh` | Nav2 멀티 + RViz 등 | 사용 중 |
| `run_missions.sh` | c1/c2 미션 동시 | 사용 중 (`use_sim_time:=True`) |

### 2.2 Perception / YOLO

| 파일 | 역할 | 상태 |
|------|------|------|
| `src/perception/perception/multi_robot_yolo_viewer.py` | 검출 노드. front\|side, alternate, X닫기=종료 | **[개선됨]** |
| `src/perception/perception/capture_dataset.py` | HOSPITAL_BG + tight GT 촬영 | **[개선됨]** |
| `src/perception/perception/train_model.py` | YOLOv11s 파인튜닝 | **이미 돌림** (15:15) |
| `src/perception/datasets/trash_can_person_v2/` | 600장 + labels | **[검증]** 14:59 촬영 |
| `src/perception/models/trash_can_person_yolo11s.pt` | 뷰어가 쓰는 가중치 | **[검증]** 학습 반영됨 |
| `src/perception/models/runs/trash_can_person/` | 학습 산출(best/last, curves) | 참고 |
| `PERCEPTION_HANDOFF.md` | 촬영/GT/옵션 상세 | 보조 문서 |

### 2.3 미션·자산

| 파일 | 역할 |
|------|------|
| `src/nova_carter/commander/.../spray_waypoint_mission.py` | c1 → `/carter1/start_sweep` |
| `src/nova_carter/commander/.../trash_can_nav_pick_mission.py` | c2 파지/덤프 미션 |
| `src/assets/props/people.usd` | 사람 캐릭터 |
| `src/assets/scenes/...` / hospital USD | 촬영·시뮬 씬 |

### 2.4 문서

| 파일 | 역할 |
|------|------|
| **`AI_HANDOFF.md`** (본 문서) | 전체 진입점 · 개선/고칠점 · 반복 장애 |
| `PERCEPTION_HANDOFF.md` | YOLO/데이터셋 딥다이브 |
| `~/Downloads/AI인수인계서 (1).txt` | 역사·Nav2/스윕 장문 (env는 구버전일 수 있음) |

---

## 3. 개선점 (이미 반영됨)

### 3.1 데이터셋 · 학습

| 항목 | 내용 |
|------|------|
| **HOSPITAL_BG** | 객체를 병원 배경 위(`OBJ_SITE_XY=(14.0, 6.5)`)에서 촬영 → 회색배경 도메인 갭 완화 |
| **tight GT** | T-pose AABB 투영(박스 과대) → Replicator `bounding_box_2d_tight` + semantics. AABB는 폴백만 |
| **사람 거리** | 벽 침투 방지 `(0.8, 1.2, 1.8, 2.5)` m + 슬릿 GT 필터(`gt_is_degenerate`) |
| **`--smoke`** | 소수 프레임 검수 |
| **재촬영** | v2 **600장** 완료 (14:59) |
| **재학습** | `trash_can_person_yolo11s.pt` **갱신 완료** (15:15). **다시 train 돌릴 필요 없음**(데이터 또 바꾸기 전엔) |

### 3.2 Isaac `14_1` · 프레임 드랍

| 항목 | 내용 |
|------|------|
| c1 RealSense | `C1_RS_RESOLUTION=(320,240)`, `C1_RS_PUBLISH_EVERY=8` |
| 렌더 스로틀 | `RENDER_EVERY=2` (물리/clock은 매 스텝) |
| front YOLO | 기존 hawk 토픽 사용 — **추가 render product 없음** |
| `DIAG_AUTO_PLAY=1` | GUI Play 없이 타임라인 시작(진단용) |
| 사람 회피 골격 | alert → 분사중단·정지·`sweep_done` → Nav2 다음 점 |

### 3.3 YOLO 뷰어

| 항목 | 내용 |
|------|------|
| 듀얼 캠 | front hawk + side RealSense → OR로 `person_alert` |
| 부하 | 기본 `--device cpu`, `--imgsz 320`, `--rate 4`, **`--alternate`** |
| near | `--front-near-frac 0.22`, side `--near-frac 0.5` |
| 창 종료 | X / `q` / Esc → **프로세스 종료** (예전: 창만 닫히고 imshow가 재생성) |

### 3.4 운영 지식 (문서화됨)

| 항목 | 내용 |
|------|------|
| T1 vs T2–4 env | Isaac: `isaac_ros`+`isaac_python`만. ROS: `ros_set`(+`sh_set`). 섞으면 rclpy/LD 충돌 |
| DOMAIN | **150** (구문서 151 아님) |
| STANDBY 해석 | Isaac HB의 `start_sweep 기다림` ≠ 무조건 버그. 미션/Nav2 goal 실패를 먼저 볼 것 |
| 라이브 진단 | Cursor 셸은 `required_permissions: ["all"]` 필요 |

---

## 4. 고칠점 (아직 안 됐거나 불안정)

### 4.1 우선 (기능·안정)

| # | 고칠점 | 증상/이유 | 제안 |
|---|--------|-----------|------|
| 1 | **라이브 검출·회피 최종 검증** **[미검증]** | 학습은 끝났으나, 병원 씬에서 사람/다리 → alert → `[AVOID]` → 스윕종료 전 구간을 아직 “완료”로 못 봄 | `14_1`+뷰어로 확인. 실패 시 conf/near·시야부터 |
| 2 | **미션 Nav2 goal timeout → STANDBY 고착** **[검증 사례]** | 미션이 너무 빨리 goal을 내면 `Failed to send goal response (timeout)` → `start_sweep` 영영 안 옴 | Nav2/amcl 준비 대기 후 미션, 또는 goal retry |
| 3 | **`run_isaac.sh`가 13번** | YOLO/사람 회피 없이 뜸 | 14_1로 바꾸거나 문서대로 직접 실행 유지 |
| 4 | **프레임 드랍 잔여 (c2 YOLO의 전제조건)** | 2로봇+병원+hawk는 여전히 무거움. RS~2Hz대 가능. **여기에 c2 YOLO를 지금 얹으면 드랍이 더 악화** | 아래 §4.1a — **드랍을 먼저 줄인 뒤** c2 YOLO |

### 4.1a 예정: carter2(쓰레기통 파지)에도 YOLO — 단, 프레임 드랍 먼저

| | |
|--|--|
| **계획** | 폐기물 로봇 **carter2**에도 YOLO를 달 예정(쓰레기통 `small_trash_can` 검출·파지 보조 등). 지금은 **c1만** 뷰어/게이트 사용, c2는 Nav2+라이다만. |
| **제약** | c1 front+side만으로도 Isaac GPU·발행·뷰어 CPU가 빡빡함. **c2 스트림/추론을 지금 추가하면 프레임 드랍이 재발·악화**하기 쉬움. |
| **순서 (필수)** | ① §4.1 #4·§5.3으로 **드랍을 체감 가능한 수준까지 완화·재측정** → ② 그다음 c2 카메라 토픽·뷰어(`--robots carter1 carter2` 등)·필요 시 Isaac 측 게이트 추가. |
| **c2 붙일 때 힌트** | 기존 hawk만 쓰기(추가 RP 금지 우선) · 뷰어 `--alternate`를 로봇/캠까지 확장 · `--device cpu` 유지 · c2는 trash 위주·낮은 rate · 가능하면 c1과 시간 분할 추론. |

### 4.2 중·저우선

| # | 고칠점 | 비고 |
|---|--------|------|
| 5 | 구 인수인계서 env(151, `~/.ros/fastdds…`) | 이 문서/실 `.bashrc`와 불일치 — 구문서만 보면 DDS 깨짐 |
| 6 | 클래스 불균형 person ≫ trash | 필요 시 궤도/샘플 조정 후 **그때** 재학습 |
| 7 | c2 Stop→Play 재개 미지원 | 재시작=Isaac 프로세스 재실행 |
| 8 | Surface gripper 경고 다수 | 종종 이후 파지 성공. 치명적이지 않으나 노이즈 |
| 9 | humble_ws Nav2 세트 git 밖 | 소실 대비 백업 |
| 10 | 실인(애니메이션 포즈) 대응 | 지금은 T-pose 일치로 Isaac 검증엔 후순위 |

### 4.3 “재학습”에 대한 정리

- **지금 시점**: 재학습 **불필요**. v2 촬영 직후 학습 완료.
- **다시 돌릴 때**: `capture_dataset`로 데이터/GT를 또 바꾼 뒤, 또는 라이브에서 성능이 명백히 안 나올 때.

---

## 5. 14_1 실행 시 반복 장애 (증상 → 원인 → 대응)

### 5.1 c1/c2가 안 움직이고 대기

```
[HB] carter1 = STANDBY: ... start_sweep 기다림
[HB] carter2 = PICK/DUMP: ... start_pick 기다림
```

| 원인 | 대응 |
|------|------|
| Play 안 함 / `/clock` 정지 | GUI Play. `ros2 topic hz /clock` |
| 미션을 Nav2보다 빨리 | Play → Nav active·amcl → 미션. timeout이면 미션만 재시작 |
| `use_sim_time` 깨짐 | `run_missions.sh`의 True 유지 |
| DOMAIN/DDS/화이트리스트 | 150 + FastDDS 프로파일. Wi‑Fi IP가 whitelist 밖이면 이상 |
| 사람 스테이지 이동 | **보통 원인 아님**. goal/미션 로그를 먼저 |

### 5.2 c1이 배회·빙빙

| 원인 | 대응 |
|------|------|
| `/clock` 없음 | Play 먼저 |
| amcl/initialpose 불량 | RViz 포즈·자동 initialpose. 스폰 ≈ `(18.5, 0)` 계열 |
| 사람/장애물 costmap 우회 | 의도된 긴 경로일 수 있음 |
| 팔 반작용 yaw(구이슈) | 스윕 튜닝 쪽 — “배회”와 구분 |

### 5.3 프레임 드랍 / 버벅임

부하: 2로봇 + hawk 다수 + 병원 + RS + YOLO(+과거 GPU 경쟁) + Nav2/RViz.  
이미 줄임: RS 해상도·발행률, 뷰어 CPU·alternate, front는 기존 hawk.  
더 필요하면 §4.1 #4 레버.

### 5.4 YOLO 미검출 / 회피 안 됨

| 원인 | 대응 |
|------|------|
| 뷰어 미실행 | T4 필수. rqt ≠ 검출 |
| 시야 | side=우측(3시), front=전방. 도크에서 전방 사람만 있으면 side `det=0`일 수 있음 |
| near/conf 미달 | bbox 작거나 가장자리 |
| 옛 뷰어 프로세스 잔존 | `pkill -f multi_robot_yolo_viewer` 후 재실행 |

### 5.5 보통 무시해도 되는 로그

Frame name override · CameraInfo fisheye/fy=fx · gripper close 실패 후 성공 · weld disjointed · No adjacent samples

---

## 6. 표준 실행

```text
T1  cd ~/cobot3_ws
    isaac_ros
    isaac_python isaacpjt/M0609/14_1_multi_robot_yolotest.py
    → Play ▶   (또는 DIAG_AUTO_PLAY=1)
    ※ ros_set 하지 말 것

T2  ros_set && sh_set && ~/cobot3_ws/run_nav.sh
    ※ /clock 확인 후. isaac_ros LD 남기지 말 것

T3  (Nav·amcl 뜬 뒤) ~/cobot3_ws/run_missions.sh

T4  ros_set && python3 src/perception/perception/multi_robot_yolo_viewer.py
```

환경(실 `.bashrc`):

```bash
export ROS_DOMAIN_ID=150
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.config/net_loadtest/fastdds_whitelist.xml
```

진단:

```bash
ros2 topic hz /clock
ros2 topic hz /carter1/realsense/color/image_raw
ros2 topic echo /carter1/person_alert --once
# 미션/Nav 로그: Goal accepted / Failed to send goal response
```

---

## 7. 권장 다음 작업 순서

1. **라이브 검증**: 14_1 + 뷰어 — 병원 배경 사람 검출, `person_alert`, Isaac `[AVOID]`.  
2. **STANDBY 고착 완화**: 미션 기동 전 Nav2 준비 대기 / goal retry.  
3. **프레임 드랍 추가 완화·측정** (c1만으로 “플레이 가능” 수준).  
4. **그다음** carter2 YOLO (쓰레기통 파지 보조) — §4.1a. **드랍 전에 c2 YOLO 달지 말 것.**  
5. (선택) `run_isaac.sh` → `14_1`로 교체.  
6. 데이터가 바뀌거나 라이브가 명백히 실패할 때만 `capture` → `train_model` 재실행.

---

## 8. 짧은 교훈

1. Play = `/clock`. 없으면 전부가 이상해 보인다.  
2. Isaac 터미널과 ROS 터미널의 LD/소싱을 섞지 말 것.  
3. DOMAIN 150 + FastDDS 경로(`.config/net_loadtest/…`).  
4. rqt ≠ YOLO.  
5. STANDBY면 Isaac보다 **미션/Nav2 goal**을 보라.  
6. 촬영만으로는 모델이 안 바뀜 — 학습은 이미 반영됨. 다음은 라이브 검증.  
7. OpenCV 창 X는 (현재 코드에서) 프로세스 종료.
