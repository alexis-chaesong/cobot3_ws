# 사람 검출 (카메라 기반 회피용) — 데이터 합성 → YOLOv11-small 학습 → 로봇 추론

## 0) 사람 에셋 구하기 (Isaac 콘텐츠 브라우저)
1. Isaac Sim GUI 실행 → **Content** 창.
2. `Isaac > People > Characters` (또는 `NVIDIA > Assets > Characters`) 로 이동.
   - 로컬에 없으면 Isaac 이 S3 에서 자동 다운로드. URL 패턴 예:
     `.../Assets/Isaac/5.1/Isaac/People/Characters/<이름>/<이름>.usd`
3. 캐릭터 우클릭 → **Copy URL Link** → `gen_person_yolo.py` 의 `PERSON_USDS` 리스트에 붙여넣기.
   다양성 위해 3~5명(성별·복장 다르게) 권장.

## 1) 데이터 생성 (Isaac Replicator, 헤드리스)
```bash
~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  ~/cobot3_ws/isaacpjt/replicator/gen_person_yolo.py
```
→ `~/cobot3_ws/datasets/person_synth/` 에 images/·labels/(YOLO)·data.yaml 생성.
- 첫 실행 후 확인/조정 : 사람 배치범위 `FREE_XY`, 카메라 범위 `CAM_POS`, 발이 바닥에 닿는지 `PERSON_Z`.
- bbox 필드명 오류 시 : `_to_yolo()` 주석대로 `bbox['data'].dtype.names` 출력해 이름만 맞춤.
- 라벨 몇 개 눈으로 검수(이미지에 박스 그려보기) 후 대량 생성 권장.

## 2) YOLOv11-small 학습 (Isaac 밖, GPU)
```bash
pip install ultralytics
yolo detect train model=yolo11s.pt data=~/cobot3_ws/datasets/person_synth/data.yaml \
  imgsz=640 epochs=100 batch=16
```
- 합성만으로 부족하면(sim-to-real gap) 실제 사람 이미지 소량 추가 + 도메인 랜덤화 강화.

## 3) 로봇 카메라 추론 → Nav2 회피 (다음 단계)
- 추론 대상 토픽 : `/carter1(2)/front_stereo_camera/left/image_raw` (namespace 분리됨).
- 검출 박스 → 카메라 내부파라미터 + depth/스테레오로 map 좌표 투영 → Nav2 costmap 장애물
  레이어(또는 사람 전용 keepout)로 넣어 회피. (별도 노드로 구현 예정.)
