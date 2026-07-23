"""
realsense_yolov11s_dataset_capture.py
-------------------------------------
mobile_manipulator_with_yolov11s_v1.usd 에서 RealSense D455 + YOLOv11s 연동 및
small_trash_can_body 다각도 데이터셋을 생성한다.

구성:
  A) RealSense 라이브: 손목 카메라 RGB -> YOLOv11s (images/realsense_live)
  B) 다각도 DB: Replicator look-at 카메라로 쓰레기통 궤도 촬영 (images/raw)
     + 월드 AABB 투영 GT 라벨 (labels/)
     + YOLO annotated / detections

저장 경로 (고정):
  <ws>/src/perception/datasets/small_trash_can_v1/

실행:
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \\
      src/perception/perception/realsense_yolov11s_dataset_capture.py [--headless]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_WS_ROOT = _THIS_DIR.parents[2]
_DEFAULT_MODEL = str(_WS_ROOT / "src" / "perception" / "models" / "yolo11s.pt")
DEFAULT_OUT_DIR = _WS_ROOT / "src" / "perception" / "datasets" / "small_trash_can_v1"
ASSETS_DIR = _WS_ROOT / "src" / "assets"

_PARSER = argparse.ArgumentParser(description="RealSense D455 + YOLOv11s trash-can dataset capture")
_PARSER.add_argument("--headless", action="store_true")
_PARSER.add_argument("--model", type=str, default=_DEFAULT_MODEL)
_PARSER.add_argument("--conf", type=float, default=0.25)
_PARSER.add_argument("--out", type=str, default="")
_PARSER.add_argument("--settle-steps", type=int, default=8)
_ARGS, _UNKNOWN = _PARSER.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _ARGS.headless})

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.robot.manipulators.manipulators import SingleManipulator  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from ultralytics import YOLO  # noqa: E402

USD_PATH = str(ASSETS_DIR / "scenes" / "mobile_manipulator_with_yolov11s_v1.usd")
ARTICULATION_ROOT_PATH = "/World/Nova_Carter_ROS/chassis_link"
ARM_USD_ROOT = "/World/m0609"
EE_LINK_NAME = "link_6"
TRASH_CAN_PRIM_PATH = "/World/small_trash_can_body"
REALSENSE_COLOR_PATH = (
    "/World/m0609/link_5/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"
)
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
CLASS_NAME = "small_trash_can"
CLASS_ID = 0

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0
CAMERA_RESOLUTION = (640, 480)

STOW_JOINTS_DEG = [-90.0, 15.0, 110.0, -40.0, 90.0, 0.0]
REALSENSE_LIVE_POSES_DEG = [
    ("live_front", [-90.0, 70.0, 65.0, -110.0, 90.0, 0.0]),
    ("live_left", [-60.0, 68.0, 62.0, -108.0, 100.0, 10.0]),
    ("live_right", [-120.0, 68.0, 62.0, -108.0, 80.0, -10.0]),
]

# RealSense 앞에서 쓰레기통을 옮겨 다각도 확보 (실제 손목 카메라 데이터)
TRASH_OFFSETS = [
    (0.0, 0.0),
    (0.12, 0.0),
    (-0.12, 0.0),
    (0.0, 0.12),
    (0.0, -0.12),
    (0.10, 0.10),
    (-0.10, 0.10),
]
TRASH_YAWS_DEG = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]

ORBIT_VIEWS = []
for dist in (1.2, 1.6, 2.0):
    for elev in (20.0, 35.0, 50.0):
        for az in range(0, 360, 45):
            ORBIT_VIEWS.append((f"d{dist:.2f}_e{int(elev)}_a{az:03d}", float(az), float(elev), float(dist)))


def find_prim_path_by_name(root_path: str, name: str) -> str | None:
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def set_drive_gains(stage, root_path: str) -> None:
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive.GetTargetPositionAttr().Set(0.0)
                n += 1
    print(f"[INFO] drive gains set: {n}", flush=True)


def _smoothstep(a: float) -> float:
    return a * a * (3.0 - 2.0 * a)


def ramp_arm(world, robot, dof_names, joints_deg, steps: int = 160):
    start = robot.get_joint_positions().copy()
    target = start.copy()
    idxs = [dof_names.index(n) for n in ARM_JOINT_NAMES]
    for i, deg in zip(idxs, joints_deg):
        target[i] = math.radians(deg)
    for s in range(steps):
        world.step(render=True)
        alpha = _smoothstep((s + 1) / steps)
        wp = start + alpha * (target - start)
        robot.apply_action(ArticulationAction(joint_positions=wp[idxs], joint_indices=idxs))
    return target


def hold_arm(world, robot, dof_names, joint_positions, steps: int):
    idxs = [dof_names.index(n) for n in ARM_JOINT_NAMES]
    action = ArticulationAction(joint_positions=np.asarray(joint_positions)[idxs], joint_indices=idxs)
    for _ in range(steps):
        world.step(render=True)
        robot.apply_action(action)


def resolve_realsense_path(preferred: str) -> str:
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(preferred).IsValid():
        return preferred
    # GUI 에서 보이던 Visual/ 하위 경로 폴백
    alt = preferred.replace("/RSD455/Camera_", "/RSD455/Visual/Camera_")
    if stage.GetPrimAtPath(alt).IsValid():
        return alt
    root = stage.GetPrimAtPath(f"{ARM_USD_ROOT}/link_5/realsense_d455")
    cands = []
    if root.IsValid():
        for prim in Usd.PrimRange(root):
            if prim.GetTypeName() == "Camera":
                name = prim.GetName().lower()
                score = (("color" in name) * 3) + (("ov9782" in name))
                if any(k in name for k in ("depth", "left", "right", "pseudo")):
                    score -= 2
                cands.append((score, str(prim.GetPath())))
    if not cands:
        raise RuntimeError(f"RealSense color camera not found: {preferred}")
    cands.sort(reverse=True)
    return cands[0][1]


def get_world_aabb(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    mn = np.array([rng.GetMin()[i] for i in range(3)], dtype=float)
    mx = np.array([rng.GetMax()[i] for i in range(3)], dtype=float)
    return mn, mx


def quat_wxyz_from_yaw_deg(yaw_deg: float) -> np.ndarray:
    half = math.radians(yaw_deg) * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def set_trash_pose(trash: SingleXFormPrim, base_xy: np.ndarray, z: float, yaw_deg: float, offset_xy):
    pos = np.array([base_xy[0] + offset_xy[0], base_xy[1] + offset_xy[1], z], dtype=float)
    trash.set_world_pose(position=pos, orientation=quat_wxyz_from_yaw_deg(yaw_deg))


def spherical_eye(center: np.ndarray, az_deg: float, el_deg: float, dist: float) -> np.ndarray:
    az, el = math.radians(az_deg), math.radians(el_deg)
    return np.array(
        [
            center[0] + dist * math.cos(el) * math.cos(az),
            center[1] + dist * math.cos(el) * math.sin(az),
            max(0.08, center[2] + dist * math.sin(el)),
        ],
        dtype=float,
    )


def capture_rgb_camera(camera: Camera, world, tries: int = 25) -> np.ndarray | None:
    for _ in range(tries):
        world.step(render=True)
        rgb = camera.get_rgb()
        if rgb is None:
            continue
        arr = np.asarray(rgb)
        if arr.ndim == 3 and arr.size and np.max(arr) > 0:
            return np.clip(arr, 0, 255).astype(np.uint8) if arr.dtype != np.uint8 else arr
    return None


def capture_rgb_replicator(eye, look_at, resolution=CAMERA_RESOLUTION) -> np.ndarray | None:
    """Replicator look-at 카메라로 한 장 촬영 (RGBA -> RGB uint8)."""
    cam = rep.functional.create.camera(position=tuple(map(float, eye)), look_at=tuple(map(float, look_at)))
    rp = rep.create.render_product(cam, resolution)
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach(rp)
    # sync step
    rep.orchestrator.step(rt_subframes=4)
    data = annot.get_data()
    annot.detach()
    rp.destroy()
    # 임시 카메라 prim 정리
    stage = omni.usd.get_context().get_stage()
    cam_path = str(cam.GetPath()) if hasattr(cam, "GetPath") else None
    if cam_path and stage.GetPrimAtPath(cam_path).IsValid():
        stage.RemovePrim(cam_path)
    if data is None:
        return None
    arr = np.asarray(data)
    if arr.ndim != 3 or arr.shape[0] == 0:
        return None
    rgb = arr[:, :, :3]
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    return rgb


def project_aabb_opencv(eye, look_at, prim_path: str, img_w: int, img_h: int, hfov_deg: float = 70.0):
    """간단한 핀홀 투영으로 GT 박스 생성 (Replicator 프레임용)."""
    mn, mx = get_world_aabb(prim_path)
    corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])], dtype=float)
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(look_at, dtype=float)
    forward = target - eye
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(forward, up)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    # camera coords: x right, y down, z forward (OpenCV)
    fx = (img_w * 0.5) / math.tan(math.radians(hfov_deg) * 0.5)
    fy = fx
    cx, cy = img_w * 0.5, img_h * 0.5
    pts = []
    for p in corners:
        rel = p - eye
        x = np.dot(rel, right)
        y = -np.dot(rel, up)
        z = np.dot(rel, forward)
        if z <= 1e-4:
            continue
        u = fx * (x / z) + cx
        v = fy * (y / z) + cy
        pts.append([u, v])
    if len(pts) < 2:
        return None
    pts = np.asarray(pts)
    x0 = float(np.clip(pts[:, 0].min(), 0, img_w - 1))
    y0 = float(np.clip(pts[:, 1].min(), 0, img_h - 1))
    x1 = float(np.clip(pts[:, 0].max(), 0, img_w - 1))
    y1 = float(np.clip(pts[:, 1].max(), 0, img_h - 1))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    bw, bh = x1 - x0, y1 - y0
    area = (bw * bh) / float(img_w * img_h)
    if area < 0.005 or area > 0.85:
        return None
    cxn, cyn = ((x0 + x1) * 0.5) / img_w, ((y0 + y1) * 0.5) / img_h
    return {
        "xyxy": [x0, y0, x1, y1],
        "yolo_xywh": [cxn, cyn, bw / img_w, bh / img_h],
        "area_frac": area,
        "line": f"{CLASS_ID} {cxn:.6f} {cyn:.6f} {bw / img_w:.6f} {bh / img_h:.6f}",
    }


def project_aabb_camera_api(camera: Camera, prim_path: str, img_w: int, img_h: int):
    mn, mx = get_world_aabb(prim_path)
    corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])], dtype=float)
    pts = camera.get_image_coords_from_world_points(corners)
    if pts is None:
        return None
    pts = np.asarray(pts, dtype=float)
    valid = np.isfinite(pts).all(axis=1)
    if valid.sum() < 2:
        return None
    pts = pts[valid]
    x0, y0 = float(np.clip(pts[:, 0].min(), 0, img_w - 1)), float(np.clip(pts[:, 1].min(), 0, img_h - 1))
    x1, y1 = float(np.clip(pts[:, 0].max(), 0, img_w - 1)), float(np.clip(pts[:, 1].max(), 0, img_h - 1))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    bw, bh = x1 - x0, y1 - y0
    area = (bw * bh) / float(img_w * img_h)
    if area < 0.005 or area > 0.90:
        return None
    cxn, cyn = ((x0 + x1) * 0.5) / img_w, ((y0 + y1) * 0.5) / img_h
    return {
        "xyxy": [x0, y0, x1, y1],
        "yolo_xywh": [cxn, cyn, bw / img_w, bh / img_h],
        "area_frac": area,
        "line": f"{CLASS_ID} {cxn:.6f} {cyn:.6f} {bw / img_w:.6f} {bh / img_h:.6f}",
    }


def run_yolo(model: YOLO, bgr: np.ndarray, conf: float):
    result = model.predict(source=bgr, conf=conf, verbose=False)[0]
    dets = []
    for box in result.boxes:
        cid = int(box.cls.item())
        dets.append(
            {
                "class_id": cid,
                "class_name": result.names.get(cid, str(cid)),
                "confidence": float(box.conf.item()),
                "xyxy": [float(v) for v in box.xyxy[0].tolist()],
            }
        )
    return dets, result.plot()


def prepare_dirs(out_root: Path) -> dict[str, Path]:
    dirs = {
        "root": out_root,
        "raw": out_root / "images" / "raw",
        "annotated": out_root / "images" / "annotated",
        "live": out_root / "images" / "realsense_live",
        "labels": out_root / "labels",
        "detections": out_root / "detections",
    }
    for p in dirs.values():
        p.mkdir(parents=True, exist_ok=True)
    for key in ("raw", "annotated", "live", "labels", "detections"):
        for f in dirs[key].glob("*"):
            if f.is_file():
                f.unlink()
    (out_root / "dataset.yaml").write_text(
        f"path: {out_root.as_posix()}\ntrain: images/raw\nval: images/raw\nnames:\n  0: {CLASS_NAME}\n",
        encoding="utf-8",
    )
    return dirs


def save_sample(dirs, stem, bgr, annotated, gt, detections, meta_extra, meta_f, live=False):
    raw_path = (dirs["live"] if live else dirs["raw"]) / f"{stem}.jpg"
    ann_path = (dirs["live"] if live else dirs["annotated"]) / (f"{stem}_ann.jpg" if live else f"{stem}.jpg")
    label_path = dirs["labels"] / f"{stem}.txt"
    det_path = dirs["detections"] / f"{stem}.json"
    cv2.imwrite(str(raw_path), bgr)
    if gt is not None:
        x0, y0, x1, y1 = map(int, gt["xyxy"])
        cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(
            annotated, f"GT:{CLASS_NAME}", (x0, max(0, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
        )
        label_path.write_text(gt["line"] + "\n", encoding="utf-8")
    else:
        label_path.write_text("", encoding="utf-8")
    cv2.imwrite(str(ann_path), annotated)
    det_path.write_text(json.dumps(detections, indent=2), encoding="utf-8")
    meta_f.write(
        json.dumps(
            {
                "view_name": stem,
                "image": str(raw_path.relative_to(dirs["root"])),
                "annotated": str(ann_path.relative_to(dirs["root"])),
                "label": str(label_path.relative_to(dirs["root"])),
                "detections_file": str(det_path.relative_to(dirs["root"])),
                "gt_bbox": gt,
                "yolo_detections": detections,
                **meta_extra,
            }
        )
        + "\n"
    )
    meta_f.flush()


def main() -> int:
    out_root = Path(_ARGS.out) if _ARGS.out else DEFAULT_OUT_DIR
    dirs = prepare_dirs(out_root)
    print(f"[INFO] output = {out_root}", flush=True)
    model = YOLO(_ARGS.model)
    print(f"[INFO] YOLO = {_ARGS.model}", flush=True)

    omni.usd.get_context().open_stage(USD_PATH)
    for _ in range(60):
        simulation_app.update()

    world = World(physics_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    for path, label in (
        (TRASH_CAN_PRIM_PATH, "trash"),
        (ARM_USD_ROOT, "arm"),
        (ARTICULATION_ROOT_PATH, "articulation"),
    ):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"{label} missing: {path}")

    set_drive_gains(stage, ARM_USD_ROOT)
    ee = find_prim_path_by_name(ARM_USD_ROOT, EE_LINK_NAME)
    rs_path = resolve_realsense_path(REALSENSE_COLOR_PATH)
    print(f"[INFO] RealSense = {rs_path}", flush=True)

    robot = world.scene.add(
        SingleManipulator(prim_path=ARTICULATION_ROOT_PATH, name="mm", end_effector_prim_path=ee, gripper=None)
    )
    trash = SingleXFormPrim(prim_path=TRASH_CAN_PRIM_PATH, name="trash")
    rs_cam = Camera(prim_path=rs_path, name="realsense_color", resolution=CAMERA_RESOLUTION, frequency=30)

    world.reset()
    robot.initialize()
    rs_cam.initialize()
    dof_names = list(robot.dof_names)

    mn, mx = get_world_aabb(TRASH_CAN_PRIM_PATH)
    center = 0.5 * (mn + mx)
    base_xy = center[:2].copy()
    base_z = float(center[2])
    print(f"[INFO] trash center={center}", flush=True)

    (dirs["root"] / "session.json").write_text(
        json.dumps(
            {
                "created_at": datetime.now(timezone.utc).isoformat(),
                "usd_path": USD_PATH,
                "realsense_prim": rs_path,
                "yolo_model": _ARGS.model,
                "class_name": CLASS_NAME,
                "resolution": list(CAMERA_RESOLUTION),
                "orbit_views": len(ORBIT_VIEWS),
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    saved_orbit = 0
    saved_live = 0
    saved_rs_db = 0
    skipped = 0

    with (dirs["root"] / "metadata.jsonl").open("w", encoding="utf-8") as meta_f:
        # A) RealSense + YOLO 라이브
        print("[INFO] RealSense live samples...", flush=True)
        for pose_name, joints in REALSENSE_LIVE_POSES_DEG:
            target = ramp_arm(world, robot, dof_names, joints)
            hold_arm(world, robot, dof_names, target, steps=25)
            rgb = capture_rgb_camera(rs_cam, world)
            if rgb is None:
                skipped += 1
                continue
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            h, w = bgr.shape[:2]
            gt = project_aabb_camera_api(rs_cam, TRASH_CAN_PRIM_PATH, w, h)
            dets, ann = run_yolo(model, bgr, _ARGS.conf)
            save_sample(
                dirs, f"rs_{pose_name}", bgr, ann, gt, dets,
                {"source": "realsense_live", "joints_deg": joints}, meta_f, live=True,
            )
            saved_live += 1
            print(f"[INFO] live {pose_name}: yolo={len(dets)}", flush=True)

        # A2) RealSense DB: 고정 자세 + 쓰레기통 이동/회전
        print("[INFO] RealSense multi-view via trash relocation...", flush=True)
        target = ramp_arm(world, robot, dof_names, REALSENSE_LIVE_POSES_DEG[0][1])
        hold_arm(world, robot, dof_names, target, steps=20)
        idx = 0
        for ox, oy in TRASH_OFFSETS:
            for yaw in TRASH_YAWS_DEG:
                set_trash_pose(trash, base_xy, base_z, yaw, (ox, oy))
                hold_arm(world, robot, dof_names, target, steps=10)
                rgb = capture_rgb_camera(rs_cam, world)
                if rgb is None:
                    skipped += 1
                    continue
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                h, w = bgr.shape[:2]
                gt = project_aabb_camera_api(rs_cam, TRASH_CAN_PRIM_PATH, w, h)
                if gt is None:
                    skipped += 1
                    continue
                dets, ann = run_yolo(model, bgr, _ARGS.conf)
                stem = f"rsdb_{idx:03d}_ox{ox:+.2f}_oy{oy:+.2f}_yaw{int(yaw):03d}"
                save_sample(
                    dirs, stem, bgr, ann, gt, dets,
                    {
                        "source": "realsense_trash_motion",
                        "offset_xy": [ox, oy],
                        "yaw_deg": yaw,
                        "joints_deg": REALSENSE_LIVE_POSES_DEG[0][1],
                    },
                    meta_f,
                )
                saved_rs_db += 1
                idx += 1
                if saved_rs_db <= 3 or saved_rs_db % 10 == 0:
                    print(f"[INFO] rsdb {saved_rs_db}: {stem} frac={gt['area_frac']:.3f}", flush=True)
        set_trash_pose(trash, base_xy, base_z, 0.0, (0.0, 0.0))

        # B) Replicator 궤도 촬영
        print("[INFO] stow + orbit replicator capture...", flush=True)
        stow = ramp_arm(world, robot, dof_names, STOW_JOINTS_DEG)
        hold_arm(world, robot, dof_names, stow, steps=15)
        mn, mx = get_world_aabb(TRASH_CAN_PRIM_PATH)
        center = 0.5 * (mn + mx)

        for i, (name, az, el, dist) in enumerate(ORBIT_VIEWS):
            eye = spherical_eye(center, az, el, dist)
            try:
                rgb = capture_rgb_replicator(eye, center)
            except Exception as exc:
                print(f"[WARN] replicator fail {name}: {exc}", flush=True)
                skipped += 1
                continue
            if rgb is None:
                skipped += 1
                continue
            # 물리/렌더 동기
            for _ in range(_ARGS.settle_steps):
                world.step(render=True)

            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            h, w = bgr.shape[:2]
            gt = project_aabb_opencv(eye, center, TRASH_CAN_PRIM_PATH, w, h)
            if gt is None:
                skipped += 1
                continue
            dets, ann = run_yolo(model, bgr, _ARGS.conf)
            stem = f"{i:03d}_{name}"
            save_sample(
                dirs, stem, bgr, ann, gt, dets,
                {
                    "source": "replicator_orbit",
                    "azimuth_deg": az,
                    "elevation_deg": el,
                    "distance_m": dist,
                    "camera_eye": eye.tolist(),
                },
                meta_f,
            )
            saved_orbit += 1
            if saved_orbit <= 3 or saved_orbit % 15 == 0:
                print(f"[INFO] orbit {saved_orbit}: {stem} frac={gt['area_frac']:.3f}", flush=True)

    print(
        f"[DONE] orbit={saved_orbit} realsense_db={saved_rs_db} live={saved_live} "
        f"skipped={skipped} -> {out_root}",
        flush=True,
    )
    return 0 if (saved_orbit + saved_rs_db + saved_live) > 0 else 1


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr, flush=True)
        import traceback

        traceback.print_exc()
        code = 1
    finally:
        simulation_app.close()
    sys.exit(code)
