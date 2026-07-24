#!/usr/bin/env python3
"""
gen_person_yolo.py — Isaac Replicator 로 "사람 검출" YOLO 학습 데이터 합성 생성
============================================================================
목적 : 병원 씬에 사람 에셋을 배치·랜덤화하며 카메라 이미지 + 정답 2D 박스를
       YOLO 포맷(labels *.txt = "class cx cy w h" 정규화)으로 바로 출력한다.
       → 그대로 YOLOv11-small 학습에 넣을 수 있다.

준비물(★필수) : PERSON_USDS 에 Isaac 콘텐츠 브라우저에서 구한 '사람 usd' URL 을 1개 이상 채운다.
   Isaac Sim GUI → Content 창 → Isaac > People > Characters (또는 NVIDIA > Assets > Characters)
   → 캐릭터 우클릭 → "Copy URL Link" → 아래 PERSON_USDS 리스트에 붙여넣기(여러 개면 다양성↑).

실행 : ~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
         /home/rokey/cobot3_ws/isaacpjt/replicator/gen_person_yolo.py
   (헤드리스로 빠르게 생성. 완료되면 OUT_DIR 에 images/ labels/ data.yaml 생성.)

출력 구조(YOLO 표준) :
   OUT_DIR/
     images/train/*.png  images/val/*.png
     labels/train/*.txt  labels/val/*.txt     # 각 줄: 0 cx cy w h  (0=person, 정규화)
     data.yaml                                 # YOLOv11 학습에 이 파일 지정

학습(Isaac 밖, Ultralytics) :
   pip install ultralytics
   yolo detect train model=yolo11s.pt data=<OUT_DIR>/data.yaml imgsz=640 epochs=100

주의 : 라이브 미검증 스켈레톤. 첫 실행 후 (a) 사람 배치범위 FREE_XY, (b) 카메라 pose 범위,
   (c) bbox 어노테이터 필드명(아래 _to_yolo 주석)만 실제 출력 보고 미세조정할 것.
============================================================================
"""
import os
import random

from isaacsim import SimulationApp
# 데이터 생성은 창 없이(headless) 빠르게. RTX 필요(카메라 렌더).
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import numpy as np
from PIL import Image
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, Gf, Sdf
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.semantics import add_update_semantics

# ─────────────────────────────── 설정(노브) ───────────────────────────────
HOSPITAL_USD = ("/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/navigation/"
                "carter_navigation/maps/map/modified_hospital.usd")   # 배경(병원)
# ★여기에 콘텐츠 브라우저에서 복사한 사람 usd URL 을 채운다(1개 이상, 여러 개 권장).
PERSON_USDS = [
    # "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/People/Characters/<이름>/<이름>.usd",
]
OUT_DIR = "/home/rokey/cobot3_ws/datasets/person_synth"

N_FRAMES     = 2000          # 총 프레임 수
VAL_EVERY    = 10            # 10프레임당 1장 val (10% 검증셋)
IMG_W, IMG_H = 640, 400      # 로봇 카메라 축소해상도와 동일(640x400)
N_PERSON     = 3             # 한 프레임 최대 사람 수(매 프레임 0~N 명 랜덤 노출)
RT_SUBFRAMES = 24            # RTX 수렴용 서브프레임(↑ 화질↑·속도↓)
WARMUP       = 10            # 초기 워밍업 스텝(첫 프레임 검은화면 방지)

# 사람 배치 자유공간(map xy) — 병원 복도/방의 빈 곳. 첫 실행 후 씬 보고 조정.
FREE_XY_MIN = (13.0, 4.0)
FREE_XY_MAX = (22.0, 17.0)
# 카메라(로봇 시점 흉내) 배치 범위 : 위치 + 높이. look_at 은 사람/자유공간 중심으로.
CAM_POS_MIN = (14.0, 0.0, 0.4)
CAM_POS_MAX = (21.0, 8.0, 1.3)
PERSON_Z    = 0.0            # 바닥 높이(에셋 원점 기준. 발이 바닥에 안 닿으면 조정)
# ───────────────────────────────────────────────────────────────────────────

assert PERSON_USDS, "★ PERSON_USDS 가 비었습니다 — 콘텐츠 브라우저에서 사람 usd URL 을 채우세요."

for sub in ("images/train", "images/val", "labels/train", "labels/val"):
    os.makedirs(os.path.join(OUT_DIR, sub), exist_ok=True)

stage = omni.usd.get_context().get_stage()
add_reference_to_stage(HOSPITAL_USD, "/World/Hospital")

# 사람 인스턴스 N_PERSON 개 생성 + "person" 시맨틱 라벨(→ bbox 어노테이터가 이 클래스만 박스 출력)
person_paths = []
for i in range(N_PERSON):
    path = f"/World/Person_{i}"
    src = PERSON_USDS[i % len(PERSON_USDS)]
    prim = add_reference_to_stage(src, path)
    add_update_semantics(prim, "person")
    person_paths.append(path)

# 플레인 USD 카메라 + 렌더프로덕트(우리가 pose 를 직접 조작하려고 rep.create.camera 대신 USD 카메라 사용)
CAM_PATH = "/World/DataCam"
UsdGeom.Camera.Define(stage, CAM_PATH)
render_product = rep.create.render_product(CAM_PATH, (IMG_W, IMG_H))

# 조명(랜덤화용)
LIGHT_PATH = "/World/DataLight"
UsdGeom.Xform.Define(stage, LIGHT_PATH)  # placeholder; distant light 는 rep 로 만들어도 됨
distant_light = rep.create.light(light_type="distant", intensity=1000, position=(0, 0, 20))

# 어노테이터 : RGB + 2D tight bounding box
rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
bbox_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
rgb_annot.attach(render_product)
bbox_annot.attach(render_product)


def _set_pose(path, pos, yaw_deg, visible=True):
    prim = stage.GetPrimAtPath(path)
    img = UsdGeom.Imageable(prim)
    (img.MakeVisible if visible else img.MakeInvisible)()
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)))
    m.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.AddTransformOp().Set(m)


def _look_at(cam_path, eye, target):
    prim = stage.GetPrimAtPath(cam_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    eye_v = Gf.Vec3d(*eye); tgt_v = Gf.Vec3d(*target)
    # USD 카메라는 -Z 를 바라봄 → look_at 행렬 구성
    m = Gf.Matrix4d(1.0)
    m.SetLookAt(eye_v, tgt_v, Gf.Vec3d(0, 0, 1))
    xf.AddTransformOp().Set(m.GetInverse())


def _rand_xy(lo, hi):
    return (random.uniform(lo[0], hi[0]), random.uniform(lo[1], hi[1]))


def _to_yolo(bbox_data):
    """bounding_box_2d_tight get_data() → YOLO 라인 리스트. 사람만 라벨했으므로 전부 class 0.
    필드명은 replicator 버전에 따라 x_min/y_min/x_max/y_max (structured array). 첫 실행 후
    print(data['data'].dtype.names) 로 확인해 필요시 이름만 맞추면 됨."""
    lines = []
    arr = bbox_data.get("data")
    if arr is None or len(arr) == 0:
        return lines
    for row in arr:
        x0, y0, x1, y1 = int(row["x_min"]), int(row["y_min"]), int(row["x_max"]), int(row["y_max"])
        if x1 <= x0 or y1 <= y0:
            continue
        # 화면 밖 클램프
        x0 = max(0, min(IMG_W - 1, x0)); x1 = max(0, min(IMG_W, x1))
        y0 = max(0, min(IMG_H - 1, y0)); y1 = max(0, min(IMG_H, y1))
        cx = (x0 + x1) / 2.0 / IMG_W
        cy = (y0 + y1) / 2.0 / IMG_H
        bw = (x1 - x0) / IMG_W
        bh = (y1 - y0) / IMG_H
        if bw <= 0 or bh <= 0:
            continue
        lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    return lines


# 워밍업(초기 검은 프레임 방지)
for _ in range(WARMUP):
    rep.orchestrator.step(rt_subframes=RT_SUBFRAMES)

saved = 0
for i in range(N_FRAMES):
    # ── 도메인 랜덤화 : 사람 수/위치/방향 ──
    n_show = random.randint(0, N_PERSON)          # 0~N 명(사람 없는 음성 프레임도 포함)
    for k, path in enumerate(person_paths):
        if k < n_show:
            _set_pose(path, (*_rand_xy(FREE_XY_MIN, FREE_XY_MAX), PERSON_Z),
                      random.uniform(0, 360), visible=True)
        else:
            _set_pose(path, (0, 0, -50), 0, visible=False)   # 화면 밖 + 숨김
    # ── 카메라 pose 랜덤화(로봇 시점 흉내) ──
    eye = (random.uniform(CAM_POS_MIN[0], CAM_POS_MAX[0]),
           random.uniform(CAM_POS_MIN[1], CAM_POS_MAX[1]),
           random.uniform(CAM_POS_MIN[2], CAM_POS_MAX[2]))
    tgt = (*_rand_xy(FREE_XY_MIN, FREE_XY_MAX), PERSON_Z + 0.9)
    _look_at(CAM_PATH, eye, tgt)

    # 렌더 + 어노테이션 수집
    rep.orchestrator.step(rt_subframes=RT_SUBFRAMES)
    rgb = rgb_annot.get_data()          # HxWx4 uint8
    bbox = bbox_annot.get_data()
    yolo_lines = _to_yolo(bbox)

    split = "val" if (i % VAL_EVERY == 0) else "train"
    name = f"person_{i:06d}"
    Image.fromarray(rgb[:, :, :3]).save(os.path.join(OUT_DIR, "images", split, name + ".png"))
    with open(os.path.join(OUT_DIR, "labels", split, name + ".txt"), "w") as f:
        f.write("\n".join(yolo_lines))
    saved += 1
    if i % 100 == 0:
        print(f"[GEN] {i}/{N_FRAMES}  (split={split}, boxes={len(yolo_lines)})")

# data.yaml (YOLOv11 학습용)
with open(os.path.join(OUT_DIR, "data.yaml"), "w") as f:
    f.write(f"path: {OUT_DIR}\n")
    f.write("train: images/train\n")
    f.write("val: images/val\n")
    f.write("names:\n  0: person\n")

print(f"[DONE] {saved} 프레임 생성 → {OUT_DIR}  (data.yaml 포함)")
simulation_app.close()
