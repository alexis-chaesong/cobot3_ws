# cobot3_ws — Perception / YOLO 인수인계 (보조)

> **전체 스택·개선/고칠점·반복 장애의 진입점**: [`AI_HANDOFF.md`](./AI_HANDOFF.md)  
> 이 문서는 데이터셋·학습·뷰어 **세부만** 적는다.  
> 작성: 2026-07-25

---

## 0. 현재 상태 (한 줄)

HOSPITAL_BG + tight GT로 **v2 600장 촬영(14:59) → `train_model` 학습 완료(15:15)**.  
가중치 `src/perception/models/trash_can_person_yolo11s.pt` 반영됨.  
**남은 것 = 라이브 검출·`[AVOID]` 검증** (재학습은 데이터 재변경 전까지 불필요).

---

## 1. 개선점 (perception)

| 항목 | 내용 |
|------|------|
| HOSPITAL_BG | `OBJ_SITE_XY=(14.0, 6.5)`, 병원 벽/바닥이 배경 |
| tight GT | `USE_TIGHT_BBOX_GT=True` — `bounding_box_2d_tight` (+ AABB 폴백) |
| 사람 거리 | `(0.8, 1.2, 1.8, 2.5)` + `gt_is_degenerate` |
| 트래시 궤도 | 2~3.5m (벽 클리핑 회피) |
| `--smoke` | 소수 프레임 검수 |
| 뷰어 듀얼캠 | front hawk + side RS, `--alternate`, CPU 기본 |
| 뷰어 종료 | 창 X → 프로세스 종료 |
| 14_1 RS 부하 | 320×240, publish every 8 |
| 학습 | v2 기준 `trash_can_person_yolo11s.pt` 갱신 **완료** |

---

## 2. 고칠점 (perception 관련)

| 항목 | 내용 |
|------|------|
| 라이브 검증 | 병원 씬에서 사람/다리 검출·alert·회피 전 구간 **미검증** |
| 프레임 드랍 잔여 | 필요 시 RENDER_EVERY / hawk↓ / `--no-window`. **c2 YOLO 전에 필수** |
| **예정: c2 YOLO** | 쓰레기통 파지 로봇에도 YOLO 예정. **지금 붙이면 드랍 악화 → 드랍 완화 후** (`AI_HANDOFF.md` §4.1a) |
| 클래스 불균형 | person ≫ trash — 조정 시에만 재촬영·재학습 |
| 실인 포즈 | T-pose 검증 후순위 |

운영(STANDBY, env, `run_isaac.sh`)은 **`AI_HANDOFF.md` §4·§5**.

---

## 3. 파이프라인 파일

```
capture_dataset.py   # Isaac: 촬영·GT  (isaac_python … --headless)
train_model.py       # 시스템 python3: 학습 → models/trash_can_person_yolo11s.pt
multi_robot_yolo_viewer.py  # 시스템 python3: 라이브 검출
14_1_multi_robot_yolotest.py  # Isaac: alert 구독·스윕 게이트
datasets/trash_can_person_v2/
models/trash_can_person_yolo11s.pt
models/runs/trash_can_person/   # 학습 로그·곡선
```

### 다시 돌릴 때만

```bash
# 데이터/GT를 바꾼 뒤에만
isaac_python src/perception/perception/capture_dataset.py --headless
# annotated 육안 검수 후
python3 src/perception/perception/train_model.py
```

### 라이브

```bash
python3 src/perception/perception/multi_robot_yolo_viewer.py
# 기본: device=cpu, imgsz=320, rate=4, alternate, front+side
```

---

## 4. capture / train 요지

- 클래스: `0 small_trash_can`, `1 person`
- 궤도 촬영 + 하드 네거티브(병원 빈 배경) + DomeLight
- `dataset.yaml` path는 train 시 절대경로 강제
- 출력 기본 imgsz 학습 480, 뷰어 추론 imgsz 320

상세 상수·교훈(hfov, semantics, rqt≠검출 등)은 이전 장문과 동일 — 운영·장애는 `AI_HANDOFF.md`로 이관.
