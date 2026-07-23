"""
yolo_trash_can_pick_place.py
----------------------------
RealSense D455 + YOLOv11s 가중치로 small_trash_can 을 인식한 뒤,
Surface Gripper 로 잡고 들어올려 Nova Carter 를 world +X 로 이동시킨 다음
내려놓는다.

흐름:
  1) 카메라 워밍업 + YOLO 인식 게이트 (감지될 때만 파지 진행)
  2) 검증된 관절 자세로 접근 -> creep + Surface Gripper 파지
  3) 들어올리기 -> j1 tuck
  4) 바퀴 구동으로 chassis world +X 이동
  5) 파지 자세 복귀 -> gripper open (내려놓기)

카메라 동작 확인 전용:
  .../python.sh src/perception/perception/yolo_trash_can_pick_place.py --check-camera

전체 시퀀스:
  .../python.sh src/perception/perception/yolo_trash_can_pick_place.py

옵션:
  --headless
  --weights PATH          (기본: models/small_trash_can_yolo11s.pt)
  --conf FLOAT            YOLO confidence (기본 0.20)
  --drive-x FLOAT         world +X 이동 거리 m (기본 0.40)
  --force-pick            YOLO 미검출이어도 파지 시퀀스 강제 실행
  --check-camera          카메라/YOLO 프리뷰만 저장하고 종료
  --check-frames N        check-camera 시 저장 프레임 수 (기본 20)
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_WS_ROOT = _THIS_DIR.parents[2]
_DEFAULT_WEIGHTS = str(_WS_ROOT / "src" / "perception" / "models" / "small_trash_can_yolo11s.pt")
_FALLBACK_WEIGHTS = str(_WS_ROOT / "src" / "perception" / "models" / "yolo11s.pt")
ASSETS_DIR = _WS_ROOT / "src" / "assets"
RMPFLOW_DIR = str(_WS_ROOT / "src" / "integration" / "integration" / "rmpflow")
CAMERA_CHECK_DIR = _WS_ROOT / "src" / "perception" / "datasets" / "camera_check"

_PARSER = argparse.ArgumentParser(description="YOLO trash-can detect -> pick -> drive +X -> place")
_PARSER.add_argument("--headless", action="store_true")
_PARSER.add_argument("--weights", type=str, default=_DEFAULT_WEIGHTS)
_PARSER.add_argument("--conf", type=float, default=0.05)
_PARSER.add_argument("--drive-x", type=float, default=0.40, help="world +X travel distance (m)")
_PARSER.add_argument("--force-pick", action="store_true")
_PARSER.add_argument("--check-camera", action="store_true")
_PARSER.add_argument("--check-frames", type=int, default=20)
_ARGS, _UNKNOWN = _PARSER.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _ARGS.headless})

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper  # noqa: E402
from isaacsim.robot.manipulators.manipulators import SingleManipulator  # noqa: E402
from isaacsim.robot.surface_gripper._surface_gripper import acquire_surface_gripper_interface  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from ultralytics import YOLO  # noqa: E402

if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

USD_PATH = str(ASSETS_DIR / "scenes" / "mobile_manipulator_with_yolov11s_v1.usd")
ARTICULATION_ROOT_PATH = "/World/Nova_Carter_ROS/chassis_link"
ARM_USD_ROOT = "/World/m0609"
EE_LINK_NAME = "link_6"
TRASH_CAN_PRIM_PATH = "/World/small_trash_can_body"
SURFACE_GRIPPER_PATH = f"{ARM_USD_ROOT}/{EE_LINK_NAME}/mop_surface_gripper"
REALSENSE_COLOR_CANDIDATES = [
    "/World/m0609/link_5/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color",
    "/World/m0609/link_5/realsense_d455/RSD455/Visual/Camera_OmniVision_OV9782_Color",
]
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
TARGET_CLASS = "small_trash_can"

NO_GRIPPER_URDF_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

# 관측 자세(인식) / 파지 자세(픽 테스트에서 검증됨)
SURVEY_JOINTS_DEG = [-90.0, 70.0, 65.0, -110.0, 90.0, 0.0]
TARGET_JOINTS_DEG = [-90.0, 101.0, 50.0, -94.0, 91.8, -1.1]
TUCK_J1_DEG = -170.0

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0
CAMERA_RESOLUTION = (640, 480)

LIFT_OFFSET = np.array([0.0, 0.0, 0.60])
POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600
SETTLE_STEPS = 60
GRASP_HOLD_STEPS = 60
JOINT_RAMP_STEPS = 300
CREEP_STEP_SIZE = 0.005
CREEP_MAX_STEPS = 40
CREEP_SETTLE_STEPS = 5

DRIVE_WHEEL_VELOCITY = 5.0
DRIVE_MAX_STEPS = 900
DRIVE_STOP_SETTLE_STEPS = 60


def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def get_prim_world_position(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    translation = omni.usd.get_world_transform_matrix(prim).ExtractTranslation()
    return np.array([translation[0], translation[1], translation[2]], dtype=float)


def get_world_orientation_wxyz(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    quat = omni.usd.get_world_transform_matrix(prim).ExtractRotationQuat()
    imag = quat.GetImaginary()
    return np.array([quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float)


def rotate_vector_by_quat(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    w = q_wxyz[0]
    qv = np.asarray(q_wxyz[1:4], dtype=float)
    v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def set_drive_gains(stage, root_path: str):
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


def ramp_to_joint_positions(world, robot, dof_names, arm_joint_names, target_joints_deg, ramp_steps: int):
    start = robot.get_joint_positions().copy()
    target = start.copy()
    idxs = [dof_names.index(n) for n in arm_joint_names if n in dof_names]
    for idx, deg in zip(idxs, target_joints_deg):
        target[idx] = np.radians(deg)
    for step in range(ramp_steps):
        world.step(render=True)
        alpha = _smoothstep((step + 1) / ramp_steps)
        wp = start + alpha * (target - start)
        robot.apply_action(ArticulationAction(joint_positions=wp[idxs], joint_indices=idxs))
    return target


def resolve_camera_path() -> str:
    stage = omni.usd.get_context().get_stage()
    for path in REALSENSE_COLOR_CANDIDATES:
        if stage.GetPrimAtPath(path).IsValid():
            return path
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
        raise RuntimeError("RealSense color camera not found")
    cands.sort(reverse=True)
    return cands[0][1]


def resolve_weights(path: str) -> str:
    p = Path(path)
    if p.is_file():
        return str(p)
    if Path(_FALLBACK_WEIGHTS).is_file():
        print(f"[WARN] weights missing ({path}), fallback to {_FALLBACK_WEIGHTS}", flush=True)
        return _FALLBACK_WEIGHTS
    raise FileNotFoundError(f"YOLO weights not found: {path}")


def capture_rgb(camera: Camera, world, tries: int = 40) -> np.ndarray | None:
    for _ in range(tries):
        world.step(render=True)
        rgb = camera.get_rgb()
        if rgb is None:
            continue
        arr = np.asarray(rgb)
        if arr.ndim == 3 and arr.size and np.max(arr) > 0:
            return np.clip(arr, 0, 255).astype(np.uint8) if arr.dtype != np.uint8 else arr
    return None


def _pick_best_det(result, conf: float):
    best = None
    for box in result.boxes:
        cls_id = int(box.cls.item())
        name = result.names.get(cls_id, str(cls_id))
        score = float(box.conf.item())
        if score < conf:
            continue
        xyxy = [float(v) for v in box.xyxy[0].tolist()]
        # 커스텀 가중치: small_trash_can / COCO 폴백: cup 등
        ok = (name == TARGET_CLASS) or (name in {"cup", "bottle", "bowl", "vase", "toilet"})
        if not ok:
            continue
        area = max(0.0, xyxy[2] - xyxy[0]) * max(0.0, xyxy[3] - xyxy[1])
        cand = {"class_name": name, "confidence": score, "xyxy": xyxy, "class_id": cls_id, "area": area}
        if best is None or (cand["confidence"], cand["area"]) > (best["confidence"], best["area"]):
            best = cand
    return best


def detect_trash_can(model: YOLO, bgr: np.ndarray, conf: float, fallback_model: YOLO | None = None):
    result = model.predict(source=bgr, conf=conf, verbose=False)[0]
    best = _pick_best_det(result, conf)
    annotated = result.plot()
    if best is None and fallback_model is not None:
        fb = fallback_model.predict(source=bgr, conf=max(conf, 0.15), verbose=False)[0]
        best = _pick_best_det(fb, max(conf, 0.15))
        annotated = fb.plot()
        result = fb
    return best, annotated, result


def create_plug_at_world_pos(trash_can_path: str, world_pos: np.ndarray, plug_name: str = "grip_plug") -> str:
    stage = omni.usd.get_context().get_stage()
    trash_can_prim = stage.GetPrimAtPath(trash_can_path)
    desired = Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
    local_pos = omni.usd.get_world_transform_matrix(trash_can_prim).GetInverse().Transform(desired)
    plug_path = f"{trash_can_path}/{plug_name}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(plug_path)
    plug_xform = UsdGeom.Xform.Define(stage, plug_path)
    plug_xform.AddTranslateOp().Set(local_pos)
    plug_xform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return plug_path


def _ramp_steps_for_distance(distance: float, max_linear_speed: float) -> int:
    raw = distance / max_linear_speed / PHYSICS_DT
    return int(np.clip(round(raw), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def move_to_pose(world, robot, rmpflow, tool0_path, target_position, target_orientation, label: str):
    start = get_prim_world_position(tool0_path)
    dist = float(np.linalg.norm(target_position - start))
    ramp_steps = _ramp_steps_for_distance(dist, MAX_EE_LINEAR_SPEED)
    print(f"[INFO] {label}: dist={dist:.3f}m ramp={ramp_steps}", flush=True)
    for step in range(ramp_steps):
        world.step(render=True)
        alpha = _smoothstep((step + 1) / ramp_steps)
        waypoint = start + alpha * (target_position - start)
        robot.apply_action(
            rmpflow.forward(
                target_end_effector_position=waypoint,
                target_end_effector_orientation=target_orientation,
            )
        )
    for step in range(MAX_APPROACH_STEPS):
        world.step(render=True)
        robot.apply_action(
            rmpflow.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=target_orientation,
            )
        )
        if np.linalg.norm(get_prim_world_position(tool0_path) - target_position) < POSITION_TOLERANCE:
            print(f"[INFO] {label} 도달", flush=True)
            return True
    print(f"[WARN] {label} 미수렴", flush=True)
    return False


def hold_pose(world, robot, rmpflow, target_position, target_orientation, steps: int):
    for _ in range(steps):
        world.step(render=True)
        robot.apply_action(
            rmpflow.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=target_orientation,
            )
        )


def drive_world_plus_x(world, robot, dof_names, target_dx: float):
    """바퀴 직진으로 chassis world +X 가 target_dx 이상 증가할 때까지 이동."""
    left_idx = dof_names.index("joint_wheel_left")
    right_idx = dof_names.index("joint_wheel_right")
    start = get_prim_world_position(ARTICULATION_ROOT_PATH)
    print(f"[INFO] drive +X start chassis={start}, target_dx={target_dx:.3f}m", flush=True)

    # 진행 방향: chassis local +X 가 world +X 와 같은 부호면 +vel, 아니면 -vel
    stage = omni.usd.get_context().get_stage()
    mat = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(ARTICULATION_ROOT_PATH))
    local_x = np.array([mat.TransformDir((1, 0, 0))[i] for i in range(3)], dtype=float)
    local_x /= np.linalg.norm(local_x)
    wheel_sign = 1.0 if local_x[0] >= 0.0 else -1.0
    vel = DRIVE_WHEEL_VELOCITY * wheel_sign

    drive_vel = np.zeros(len(dof_names))
    drive_vel[left_idx] = vel
    drive_vel[right_idx] = vel
    drive_action = ArticulationAction(joint_velocities=drive_vel)

    reached = False
    for step in range(DRIVE_MAX_STEPS):
        world.step(render=True)
        robot.apply_action(drive_action)
        now = get_prim_world_position(ARTICULATION_ROOT_PATH)
        dx = float(now[0] - start[0])
        if dx >= target_dx:
            reached = True
            print(f"[INFO] +X 도달: dx={dx:.3f}m step={step}", flush=True)
            break

    stop = ArticulationAction(joint_velocities=np.zeros(len(dof_names)))
    for _ in range(DRIVE_STOP_SETTLE_STEPS):
        world.step(render=True)
        robot.apply_action(stop)

    end = get_prim_world_position(ARTICULATION_ROOT_PATH)
    dx = float(end[0] - start[0])
    print(f"[RESULT] drive done: dx={dx:.3f}m dy={end[1]-start[1]:.3f}m reached={reached}", flush=True)
    return dx, reached


def run_camera_check(
    world, robot, dof_names, camera, model, out_dir: Path, frames: int, conf: float,
    fallback_model: YOLO | None = None,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw").mkdir(exist_ok=True)
    (out_dir / "annotated").mkdir(exist_ok=True)

    print("[CHECK] moving to survey pose for camera check...", flush=True)
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, SURVEY_JOINTS_DEG, 180)
    for _ in range(30):
        world.step(render=True)

    ok_frames = 0
    detections = 0
    mean_brightness = []
    records = []

    for i in range(frames):
        rgb = capture_rgb(camera, world)
        if rgb is None:
            print(f"[CHECK] frame {i}: NO RGB", flush=True)
            continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        bright = float(np.mean(bgr))
        mean_brightness.append(bright)
        det, ann, _ = detect_trash_can(model, bgr, conf, fallback_model)
        stem = f"check_{i:03d}"
        cv2.imwrite(str(out_dir / "raw" / f"{stem}.jpg"), bgr)
        cv2.imwrite(str(out_dir / "annotated" / f"{stem}.jpg"), ann)
        ok_frames += 1
        if det is not None:
            detections += 1
        records.append({"frame": i, "brightness": bright, "detection": det})
        print(
            f"[CHECK] frame {i}: shape={bgr.shape} brightness={bright:.1f} "
            f"det={None if det is None else (det['class_name'], round(det['confidence'], 3))}",
            flush=True,
        )

    summary = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "camera_ok": ok_frames > 0,
        "frames_requested": frames,
        "frames_ok": ok_frames,
        "detections": detections,
        "mean_brightness": float(np.mean(mean_brightness)) if mean_brightness else 0.0,
        "weights": _ARGS.weights,
        "output_dir": str(out_dir),
        "records": records,
    }
    (out_dir / "camera_check_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print("\n========== CAMERA CHECK RESULT ==========", flush=True)
    print(f" RGB frames OK : {ok_frames}/{frames}", flush=True)
    print(f" YOLO detects  : {detections}/{ok_frames}", flush=True)
    print(f" mean brightness: {summary['mean_brightness']:.1f} (0=black, ~128=normal)", flush=True)
    print(f" saved to      : {out_dir}", flush=True)
    print("  - raw/*.jpg        : 카메라 원본", flush=True)
    print("  - annotated/*.jpg  : YOLO 박스 오버레이", flush=True)
    print("  - camera_check_summary.json", flush=True)
    if ok_frames == 0:
        print("[CHECK] FAIL: 카메라 RGB 를 못 받았습니다.", flush=True)
    elif summary["mean_brightness"] < 5:
        print("[CHECK] WARN: 이미지가 거의 검정입니다 (렌더/노출 문제 가능).", flush=True)
    else:
        print("[CHECK] PASS: 카메라가 동작합니다. annotated 이미지로 YOLO도 확인하세요.", flush=True)
    print("=========================================\n", flush=True)
    return summary


def main() -> int:
    weights = resolve_weights(_ARGS.weights)
    print(f"[INFO] YOLO weights = {weights}", flush=True)
    model = YOLO(weights)
    fallback_model = None
    if Path(weights).resolve() != Path(_FALLBACK_WEIGHTS).resolve() and Path(_FALLBACK_WEIGHTS).is_file():
        fallback_model = YOLO(_FALLBACK_WEIGHTS)
        print(f"[INFO] COCO fallback weights = {_FALLBACK_WEIGHTS}", flush=True)

    print(f"[INFO] open stage: {USD_PATH}", flush=True)
    omni.usd.get_context().open_stage(USD_PATH)
    for _ in range(60):
        simulation_app.update()

    world = World(physics_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    for path, label in (
        (TRASH_CAN_PRIM_PATH, "trash"),
        (ARM_USD_ROOT, "arm"),
        (ARTICULATION_ROOT_PATH, "articulation"),
        (SURFACE_GRIPPER_PATH, "surface gripper"),
    ):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"{label} missing: {path}")

    set_drive_gains(stage, ARM_USD_ROOT)
    ee_prim_path = find_prim_path_by_name(ARM_USD_ROOT, EE_LINK_NAME)
    tool0_path = find_prim_path_by_name(ARM_USD_ROOT, "tool0")
    if ee_prim_path is None or tool0_path is None:
        raise RuntimeError("link_6/tool0 not found")
    cam_path = resolve_camera_path()
    print(f"[INFO] camera={cam_path}", flush=True)

    robot = world.scene.add(
        SingleManipulator(
            prim_path=ARTICULATION_ROOT_PATH,
            name="m0609_mobile_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=None,
        )
    )
    camera = Camera(prim_path=cam_path, name="realsense_color", resolution=CAMERA_RESOLUTION, frequency=30)

    world.reset()
    robot.initialize()
    camera.initialize()
    dof_names = list(robot.dof_names)
    for name in ARM_JOINT_NAMES + ["joint_wheel_left", "joint_wheel_right"]:
        if name not in dof_names:
            raise RuntimeError(f"missing dof: {name}")

    home = robot.get_joint_positions().copy()
    for name in ARM_JOINT_NAMES:
        home[dof_names.index(name)] = 0.0
    robot.set_joint_positions(home)
    for _ in range(15):
        world.step(render=True)

    # ── 카메라 확인 모드 ─────────────────────────────────────────
    if _ARGS.check_camera:
        summary = run_camera_check(
            world, robot, dof_names, camera, model, CAMERA_CHECK_DIR, _ARGS.check_frames, _ARGS.conf,
            fallback_model=fallback_model,
        )
        if not _ARGS.headless:
            print("[INFO] 창을 닫으면 종료됩니다.", flush=True)
            while simulation_app.is_running():
                world.step(render=True)
        return 0 if summary["camera_ok"] else 1

    # ── 인식 게이트 ──────────────────────────────────────────────
    print("[INFO] survey pose + YOLO detect...", flush=True)
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, SURVEY_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    det = None
    annotated = None
    for attempt in range(15):
        rgb = capture_rgb(camera, world)
        if rgb is None:
            continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        det, annotated, _ = detect_trash_can(model, bgr, _ARGS.conf, fallback_model)
        if det is not None:
            print(
                f"[DETECT] attempt={attempt} class={det['class_name']} "
                f"conf={det['confidence']:.3f} xyxy={det['xyxy']}",
                flush=True,
            )
            break
        print(f"[DETECT] attempt={attempt}: no trash can yet", flush=True)

    preview_dir = _WS_ROOT / "src" / "perception" / "datasets" / "pick_place_preview"
    preview_dir.mkdir(parents=True, exist_ok=True)
    if annotated is not None:
        cv2.imwrite(str(preview_dir / "detect_before_pick.jpg"), annotated)
        print(f"[INFO] detection preview -> {preview_dir / 'detect_before_pick.jpg'}", flush=True)

    if det is None and not _ARGS.force_pick:
        print(
            "[ERROR] YOLO 가 쓰레기통을 인식하지 못했습니다. "
            "--force-pick 으로 강제 실행하거나 --check-camera 로 카메라/가중치를 확인하세요.",
            flush=True,
        )
        return 2
    if det is None and _ARGS.force_pick:
        print("[WARN] --force-pick: YOLO 미검출이지만 파지를 진행합니다.", flush=True)

    # ── RMPflow / Gripper ────────────────────────────────────────
    rmpflow = RMPFlowController(
        name="yolo_trash_pick_controller",
        robot_articulation=robot,
        urdf_path=NO_GRIPPER_URDF_PATH,
    )
    base_link_prim = stage.GetPrimAtPath(f"{ARM_USD_ROOT}/base_link")
    base_matrix = omni.usd.get_world_transform_matrix(base_link_prim)
    bt = base_matrix.ExtractTranslation()
    bq = base_matrix.ExtractRotationQuat()
    bi = bq.GetImaginary()
    rmpflow.rmp_flow.set_robot_base_pose(
        robot_position=np.array([bt[0], bt[1], bt[2]]),
        robot_orientation=np.array([bq.GetReal(), bi[0], bi[1], bi[2]]),
    )

    surface_gripper = SurfaceGripper(
        end_effector_prim_path=ee_prim_path,
        surface_gripper_path=SURFACE_GRIPPER_PATH,
    )
    surface_gripper.initialize()

    # ── 파지 ─────────────────────────────────────────────────────
    print("[INFO] approach grasp pose...", flush=True)
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    grasp_position = get_prim_world_position(tool0_path)
    grasp_orientation = get_world_orientation_wxyz(tool0_path)

    move_dir = rotate_vector_by_quat(grasp_orientation, np.array([0.0, 0.0, 1.0]))
    move_dir /= np.linalg.norm(move_dir)
    current_target = grasp_position.copy()
    gripped_ok = False
    for creep_step in range(CREEP_MAX_STEPS):
        current_target = current_target + move_dir * CREEP_STEP_SIZE
        for _ in range(CREEP_SETTLE_STEPS):
            world.step(render=True)
            robot.apply_action(
                rmpflow.forward(
                    target_end_effector_position=current_target,
                    target_end_effector_orientation=grasp_orientation,
                )
            )
        surface_gripper.close()
        if surface_gripper.is_closed():
            gripped_ok = True
            print(f"[INFO] gripped at creep step {creep_step + 1}", flush=True)
            break

    grasp_position = current_target
    plug_path = create_plug_at_world_pos(TRASH_CAN_PRIM_PATH, get_prim_world_position(tool0_path))
    if gripped_ok:
        gripped = acquire_surface_gripper_interface().get_gripped_objects(SURFACE_GRIPPER_PATH)
        print(f"[CHECKPOINT] CLOSED, gripped={gripped}", flush=True)
    else:
        print("[CHECKPOINT] grasp failed within creep range", flush=True)

    # ── 들어올리기 ───────────────────────────────────────────────
    lift_target = grasp_position + LIFT_OFFSET
    move_to_pose(world, robot, rmpflow, tool0_path, lift_target, grasp_orientation, "lift")
    hold_pose(world, robot, rmpflow, lift_target, grasp_orientation, GRASP_HOLD_STEPS)
    gap = float(np.linalg.norm(get_prim_world_position(plug_path) - get_prim_world_position(tool0_path)))
    print(f"[RESULT] after lift gripper-plug gap={gap:.4f}m", flush=True)

    # ── tuck ─────────────────────────────────────────────────────
    current = robot.get_joint_positions()
    tuck_deg = []
    for name in ARM_JOINT_NAMES:
        idx = dof_names.index(name)
        tuck_deg.append(TUCK_J1_DEG if name == "joint_1" else float(np.degrees(current[idx])))
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, tuck_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)

    # ── world +X 주행 ────────────────────────────────────────────
    dx, reached = drive_world_plus_x(world, robot, dof_names, _ARGS.drive_x)

    # ── 내려놓기 ─────────────────────────────────────────────────
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    surface_gripper.open()
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    place_pos = get_prim_world_position(TRASH_CAN_PRIM_PATH)
    print(
        f"[RESULT] placed trash at {place_pos}, gripper_closed={surface_gripper.is_closed()}, "
        f"drive_dx={dx:.3f}m",
        flush=True,
    )

    # 파지 후 한 장 더 찍어 카메라 동작 재확인
    rgb = capture_rgb(camera, world)
    if rgb is not None:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        _, ann, _ = detect_trash_can(model, bgr, _ARGS.conf, fallback_model)
        cv2.imwrite(str(preview_dir / "after_place.jpg"), ann)
        print(f"[INFO] after-place preview -> {preview_dir / 'after_place.jpg'}", flush=True)

    print("[DONE] YOLO detect -> pick -> +X drive -> place 완료.", flush=True)
    if not _ARGS.headless:
        print("[INFO] 창을 닫으면 종료됩니다.", flush=True)
        while simulation_app.is_running():
            world.step(render=True)
    return 0 if (gripped_ok and reached) else 1


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
