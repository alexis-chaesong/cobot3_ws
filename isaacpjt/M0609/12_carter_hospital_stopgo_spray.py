"""
12_carter_hospital_stopgo_spray.py
====================================================================
자체주행(Nav2 없음) stop-and-go 소독 : hospital 복도를 앞으로 조금 이동 →
멈춰서 위아래+방사 소독 1회 → 다시 이동 … 을 반복한다. (스크린샷2 동작)

왜 이 파일인가
  - 10_carter_hospital_spray_nav.py : 씬 결합(hospital+Carter+노즐 팔)은 좋지만
    "주행을 Nav2 에 위임" → Nav2/RViz 가 안 뜨면 (a) 팔이 안 움직이고
    (/spray_active 가 영영 False) (b) 베이스가 제자리 회전(Nav2 rotate recovery).
  - 그래서 이 파일은 Nav2·/cmd_vel·/spray_active 를 전부 걷어내고, 9 번의
    stop-and-go + 11 번의 heading-hold(CoM 보정) 직진을 직접 휠 제어로 통합했다.

구성 = 10 번 씬 + 9 번 stop-and-go + 11 번 직진보정
  - 씬       : 10_carter_hospital_spray_nav.py build_scene 그대로 (hospital 참조 +
               Carter payload @ CARTER_START_POSE + 팔 chassis 병합 + 안전 바닥).
  - 팔 모션  : 9/11 의 q_of_s (Lula IK 하단/상단 aim → 위아래 왕복 + J5 플릭 + J1 90°).
  - 주행     : 직접 휠 제어. MOVE 구간에서 11 번의 heading-hold(KP_YAW/KP_LAT) +
               CoM 피드포워드(KFF_COM)로 "빙빙" 없이 직진.

동작 시퀀스 (stop-and-go)
  INIT : 팔 [0,0,0,0,0,0] → j1 오른쪽 90°(옆 벽 겨냥). 베이스 정지.
  WIPE : 베이스 정지(휠=0), 팔이 제자리에서 위아래 1회 소독(올라갔다 내려오기).
  MOVE : 팔은 하단 자세로 고정, 베이스가 MOVE_DISTANCE 만큼 직진(heading-hold).
  → WIPE↔MOVE 반복, 누적 FORWARD_DISTANCE 도달 시 정지.

실행
  ~/isaacsim/python.sh 12_carter_hospital_stopgo_spray.py  → GUI 에서 Carter 가
  복도 자유공간에 있는지 확인(아니면 CARTER_START_POSE 조정) → Play ▶

튜닝
  - CARTER_START_POSE : 복도 안 자유공간 + 옆에 벽이 오도록 (GUI 로 확인)
  - MOVE_DISTANCE / FORWARD_DISTANCE / FORWARD_SPEED : 이동/총거리/속도
  - J1_OFFSET(±) : 겨냥할 벽 방향(-90°=오른쪽), Z_LOW/Z_HIGH : 위아래 높이
  - KP_YAW/KP_LAT ↑ : 더 곧게(진동하면 ↓), KFF_COM 부호 : CoM 보정 방향
====================================================================
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg

_THIS_DIR = Path(__file__).resolve().parent

# ────────────────────────────────────────────────────────────────
#  A. 경로 / 상수
# ────────────────────────────────────────────────────────────────
HOSPITAL_USD = "/home/rokey/cobot3_ws/src/integration/integration/hospital_hallway.usd"
NOZZLE_USD = "/home/rokey/cobot3_ws/src/integration/integration/m0609_with_nozzle.usd"
CARTER_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
              "Assets/Isaac/5.1/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd")

ROBOT_PRIM_PATH = "/World/m0609"
EE_FRAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/link_6"
BASE_LINK_PATH = f"{ROBOT_PRIM_PATH}/base_link"
ROOT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/root_joint"

CARTER_PRIM_PATH = "/World/Nova_Carter_ROS"
CHASSIS_LINK_PATH = f"{CARTER_PRIM_PATH}/chassis_link"
MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)     # chassis 기준 팔 장착
CHASSIS_MASS = 150.0

# ★ Carter 스폰 : 복도 자유공간 + 옆에 소독할 벽이 오도록 (GUI 에서 확인/조정)
CARTER_START_POSE = dict(x=0.0, y=0.0, z=0.05, yaw_deg=0.0)

URDF_PATH = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
NOZZLE_OFFSET = 0.1392

# --- 벽 겨냥 (베이스 로컬) : 8/9/11 과 동일 ---
WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80

# --- 팔 시작 시퀀스 / 와이프 ---
J1_INDEX = 0
J1_OFFSET = -np.pi / 2       # 오른쪽 90° → 옆 벽 소독 (+면 왼쪽)
J5_INDEX = 4
J5_FLICK = -0.5
S_CRUISE = 0.90
S_ACCEL = 2.00
S_HOLD_STEPS = 6
STROKES_PER_WIPE = 2         # "한번 위아래" = 올라가기(1) + 내려오기(2)
MAX_JOINT_STEP = 0.04        # 관절 목표 rate-limit [rad/step] (급전환 완화)

# --- Nova Carter stop-and-go 직진 주행 + CoM 보정 ---
WHEEL_JOINT_NAMES = ["joint_wheel_left", "joint_wheel_right"]
WHEEL_RADIUS = 0.14
WHEEL_BASE = 0.4132
FORWARD_DISTANCE = 10.0       # 전체 전진 목표 거리 [m]
MOVE_DISTANCE = 0.50          # 한 번 이동 거리 [m] (이동 중 팔은 하단 고정)
FORWARD_SPEED = 0.20          # 전진 속도 [m/s]
BASE_CALIB_STEPS = 20         # 첫 이동에서 휠 방향 자동보정
# 직진 유지 게인 (11 번과 동일)
KP_YAW = 2.5                  # 헤딩 오차 → 각속도 [1/s]
KP_LAT = 1.2                  # 측면 이탈 → 각속도 [1/(m·s)]
KFF_COM = 0.010               # CoM 모멘트 피드포워드 (부호 반대면 - 로)
W_MAX = 1.0                   # 각속도 명령 상한 [rad/s]

ARM_LINK_NAMES = ["base_link", "link_1", "link_2", "link_3",
                  "link_4", "link_5", "link_6", "nozzle_base_link"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


# ────────────────────────────────────────────────────────────────
#  B. 유틸 - 수학 (8/9/10/11 과 동일)
# ────────────────────────────────────────────────────────────────
def matrix_to_quat_wxyz(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2; w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S; y = (R[0, 2] - R[2, 0]) / S; z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S; x = 0.25 * S; y = (R[0, 1] + R[1, 0]) / S; z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S; x = (R[0, 1] + R[1, 0]) / S; y = 0.25 * S; z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S; x = (R[0, 2] + R[2, 0]) / S; y = (R[1, 2] + R[2, 1]) / S; z = 0.25 * S
    q = np.array([w, x, y, z]); return q / np.linalg.norm(q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def spray_orientation_quat():
    R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    return matrix_to_quat_wxyz(R)


def local_to_world(base_pos, base_quat, p_local):
    return base_pos + quat_to_matrix(base_quat) @ np.asarray(p_local)


def aim_link6_world(base_pos, base_quat, ori_quat, z):
    world_offset = quat_to_matrix(ori_quat) @ np.array([0.0, 0.0, NOZZLE_OFFSET])
    return local_to_world(base_pos, base_quat, [WALL_X, AIM_Y, z]) - world_offset


def read_world_pose(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation(); q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ────────────────────────────────────────────────────────────────
#  C. 씬 구성 (10 번과 동일 : hospital + Carter + 노즐 팔 병합)
# ────────────────────────────────────────────────────────────────
def build_scene(my_world):
    stage = omni.usd.get_context().get_stage()

    # (1) /World + hospital 환경
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(HOSPITAL_USD)      # → /World/hospital

    # (2) Nova Carter (payload) at CARTER_START_POSE
    carter_prim = stage.DefinePrim(CARTER_PRIM_PATH, "Xform")
    carter_prim.GetPayloads().AddPayload(CARTER_URL)
    xc = UsdGeom.XformCommonAPI(carter_prim)
    xc.SetTranslate(Gf.Vec3d(CARTER_START_POSE["x"], CARTER_START_POSE["y"], CARTER_START_POSE["z"]))
    xc.SetRotate(Gf.Vec3f(0.0, 0.0, float(CARTER_START_POSE["yaw_deg"])))

    for _ in range(120):
        simulation_app.update()
    print(f"[LOAD] hospital={HOSPITAL_USD}\n[LOAD] carter @ {CARTER_START_POSE}")

    chassis = stage.GetPrimAtPath(CHASSIS_LINK_PATH)
    if not chassis.IsValid():
        print(f"[FATAL] {CHASSIS_LINK_PATH} 없음 — Carter 로드 실패")
        return False
    UsdPhysics.MassAPI.Apply(chassis).CreateMassAttr(CHASSIS_MASS)

    # (3) 노즐 팔 + chassis 위 정렬 배치 (chassis 실제 world 변환 기준 = 10 번 방식)
    world_prim.GetReferences().AddReference(NOZZLE_USD)        # → /World/m0609
    for _ in range(20):
        simulation_app.update()
    arm_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not arm_prim.IsValid():
        print(f"[FATAL] {ROBOT_PRIM_PATH} 없음 — 노즐 팔 로드 실패")
        return False
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    offset_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET)
    arm_m = offset_m * chassis_m
    xf = UsdGeom.Xformable(arm_prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(arm_m)

    # (4) 병합 : root_joint 를 world→chassis 로 재연결 + 팔 ArticulationRoot 제거
    rj = stage.GetPrimAtPath(ROOT_JOINT_PATH)
    if not rj.IsValid():
        print(f"[FATAL] {ROOT_JOINT_PATH} 없음")
        return False
    rj.RemoveAppliedSchema("PhysicsArticulationRootAPI")
    rj.RemoveAppliedSchema("PhysxArticulationAPI")
    fj = UsdPhysics.FixedJoint(rj)
    fj.CreateBody0Rel().SetTargets([Sdf.Path(CHASSIS_LINK_PATH)])
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(MOUNT_OFFSET))
    fj.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(BASE_LINK_PATH))
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(CHASSIS_LINK_PATH))

    # (5) 안전 바닥 (hospital 바닥 collision 누락 대비)
    my_world.scene.add_default_ground_plane(z_position=0.0)

    for _ in range(10):
        simulation_app.update()
    print("[SCENE] hospital + Carter + 노즐 팔 병합 장착 완료")
    return True


def tune_arm_drives():
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        for dt in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dt)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                n += 1
    print(f"[PHYSICS] arm drive tuned: {n}")


def find_articulation_root(root_path):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return root_path
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return root_path


def read_arm_link_masses():
    """각 팔 링크 질량[kg] (physics:mass). CoM 계산용."""
    stage = omni.usd.get_context().get_stage()
    masses = {}
    for name in ARM_LINK_NAMES:
        prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/{name}")
        if not prim.IsValid():
            continue
        attr = prim.GetAttribute("physics:mass")
        if attr and attr.Get():
            masses[name] = float(attr.Get())
    return masses


def arm_com_in_chassis(masses, chassis_pos, chassis_R):
    """팔 무게중심을 chassis 프레임에서 본 좌표 + 총질량. com_local[1]=측면(y) 쏠림."""
    tot = 0.0
    acc = np.zeros(3)
    for name, m in masses.items():
        pos, _ = read_world_pose(f"{ROBOT_PRIM_PATH}/{name}")
        acc += m * pos
        tot += m
    com_world = acc / max(tot, 1e-6)
    com_local = chassis_R.T @ (com_world - chassis_pos)
    return com_local, tot


# ────────────────────────────────────────────────────────────────
#  D. 사다리꼴 위상 왕복기 (9 번 : strokes 카운트 포함)
# ────────────────────────────────────────────────────────────────
class Sweeper:
    def __init__(self, lo, hi, cruise, accel, dt, hold_steps=6):
        self.lo, self.hi = lo, hi
        self.cruise, self.accel, self.dt = cruise, accel, dt
        self.hold_steps = hold_steps
        self.reset_bottom()

    def reset_bottom(self):
        """바닥(lo)에서 위로 출발 : 한 번의 위아래 시작 지점."""
        self.s = self.lo; self.dir = 1.0; self.v = 0.0
        self.hold = 0; self.strokes = 0

    def step(self):
        if self.hold > 0:
            self.hold -= 1
            if self.hold == 0:
                self.dir *= -1.0
            return self.s
        end = self.lo if self.dir < 0 else self.hi
        dr = abs(end - self.s)
        v_decel = np.sqrt(max(0.0, 2.0 * self.accel * dr))
        self.v = min(self.cruise, self.v + self.accel * self.dt, v_decel)
        self.s += self.dir * self.v * self.dt
        if dr <= 2e-3:
            self.s = end; self.v = 0.0
            self.hold = self.hold_steps
            self.strokes += 1              # 끝단 도달 = 스트로크 1회
        return self.s


def wheel_action(wheel_idx, linear_speed, angular_speed=0.0):
    """unicycle → 좌/우 휠 각속도."""
    omega_l = (2.0 * linear_speed - angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    omega_r = (2.0 * linear_speed + angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    return ArticulationAction(joint_velocities=np.array([omega_l, omega_r]),
                              joint_indices=np.array(wheel_idx))


# ────────────────────────────────────────────────────────────────
#  E. 메인
# ────────────────────────────────────────────────────────────────
def main():
    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
    if not build_scene(my_world):
        simulation_app.close(); return
    tune_arm_drives()
    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)

    art_root = find_articulation_root(CARTER_PRIM_PATH)
    robot = SingleArticulation(prim_path=art_root, name="carter_m0609")
    robot.initialize()
    dof_names = list(robot.dof_names)
    try:
        arm_idx = np.array([dof_names.index(n) for n in ARM_JOINT_NAMES])
    except ValueError as e:
        print(f"[FATAL] 팔 관절 미발견: {e}"); simulation_app.close(); return
    wheel_idx = [dof_names.index(n) for n in WHEEL_JOINT_NAMES if n in dof_names]
    if len(wheel_idx) < 2:
        wheel_idx = [i for i, n in enumerate(dof_names) if "wheel" in n.lower() and "caster" not in n.lower()][:2]
    if len(wheel_idx) < 2:
        print("[FATAL] 휠 dof 미발견"); simulation_app.close(); return
    print(f"[ART] arm_idx={arm_idx.tolist()} wheel_idx={wheel_idx} (전체 dof {len(dof_names)})")

    masses = read_arm_link_masses()
    print(f"[COM] 팔 링크 질량 {masses}  총 {sum(masses.values()):.2f} kg")

    # ── 하단/상단 aim IK (9/11 과 동일) ──
    base_pos, base_quat = read_world_pose(BASE_LINK_PATH)
    ori_quat = spray_orientation_quat()
    ik = mg.LulaKinematicsSolver(robot_description_path=DESCRIPTION_PATH, urdf_path=URDF_PATH)
    ik.set_robot_base_pose(base_pos, base_quat)
    q_low, ok_lo = ik.compute_inverse_kinematics(
        EE_FRAME, aim_link6_world(base_pos, base_quat, ori_quat, Z_LOW), ori_quat,
        position_tolerance=0.005, orientation_tolerance=0.05)
    q_high, ok_hi = ik.compute_inverse_kinematics(
        EE_FRAME, aim_link6_world(base_pos, base_quat, ori_quat, Z_HIGH), ori_quat,
        warm_start=(q_low if ok_lo else None),
        position_tolerance=0.005, orientation_tolerance=0.05)
    if not (ok_lo and ok_hi):
        print(f"[FATAL] aim IK 실패 (low={ok_lo}, high={ok_hi}) → Z_LOW/Z_HIGH/WALL_X 조정")
        simulation_app.close(); return
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high); q_half = 0.5 * (q_high - q_low)
    print(f"[AIM] q_low={np.round(q_low,3)}  q_high={np.round(q_high,3)}")

    def q_of_s(s):                       # 9/11 과 동일 (J1_OFFSET 포함)
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    Q_ZERO = np.zeros(6)                          # 초기 자세 [0,0,0,0,0,0]

    robot.set_joint_positions(Q_ZERO, joint_indices=arm_idx)
    q_applied = Q_ZERO.copy()                     # rate-limiter 상태
    for _ in range(20):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    print(f"\n[RUN] Play ▶ : (위아래 1회 소독 → {MOVE_DISTANCE:.2f} m 직진) 반복, "
          f"총 {FORWARD_DISTANCE:.1f} m.\n")

    # ── 상태 ──
    was_playing = False
    phase = "INIT"                # INIT → WIPE ↔ MOVE → (도달 시 정지)
    q_hold = q_of_s(-1.0)         # MOVE 중 팔 고정 자세(하단)
    # 직진 기준 (첫 WIPE 진입 시 캡처, 이후 전 구간 공유 → 한 직선 유지)
    global_start = None; forward0 = None; target_yaw = 0.0; heading_ready = False
    move_start_prog = 0.0
    fwd_sign = 1.0; calibrated = False; drive_steps = 0
    reached = False; step_i = 0; cycle = 0; progress = 0.0

    while simulation_app.is_running():
        my_world.step(render=True)
        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            robot.set_joint_positions(Q_ZERO, joint_indices=arm_idx)
            q_applied = Q_ZERO.copy()
            sweeper.reset_bottom()
            phase = "INIT"; heading_ready = False; calibrated = False
            fwd_sign = 1.0; reached = False; step_i = 0; cycle = 0
            print("[RUN] 시작: 초기 자세 [0,0,0,0,0,0] → j1 오른쪽 90°")
        was_playing = is_playing
        if not is_playing:
            continue

        chassis_pos, chassis_quat = read_world_pose(CHASSIS_LINK_PATH)
        chassis_R = quat_to_matrix(chassis_quat)
        step_i += 1

        # 진행 거리(직진 기준 캡처 후)
        if heading_ready:
            progress = float(np.dot(chassis_pos - global_start, forward0))

        # ── 총거리 도달 : 정지하고 팔은 제자리 소독 계속 ──
        if heading_ready and not reached and progress >= FORWARD_DISTANCE:
            reached = True
            print(f"[INFO] 총 {FORWARD_DISTANCE:.1f} m 도달(progress={progress:.2f}) → 정지, 팔은 계속 소독")
        if reached:
            robot.apply_action(wheel_action(wheel_idx, 0.0))
            q_target = q_of_s(sweeper.step())
            q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))
            continue

        # ─────────────────────────── 상태기계 ───────────────────────────
        if phase == "INIT":
            # 팔 [0,0,0,0,0,0] → j1 오른쪽 90°. 베이스 정지.
            q_target = np.zeros(6); q_target[J1_INDEX] = J1_OFFSET
            robot.apply_action(wheel_action(wheel_idx, 0.0))
            q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))
            if abs(q_applied[J1_INDEX] - J1_OFFSET) < 0.02:
                # 직진 기준 캡처 (이후 전 구간 이 헤딩으로 직진 보정)
                global_start = chassis_pos.copy()
                forward0 = chassis_R @ np.array([1.0, 0.0, 0.0])
                forward0 = forward0 / (np.linalg.norm(forward0) + 1e-9)
                target_yaw = np.arctan2(forward0[1], forward0[0])
                heading_ready = True
                phase = "WIPE"; sweeper.reset_bottom()
                print("[PHASE] j1 90° 완료 → WIPE(위아래 소독) 시작")
            continue

        if phase == "WIPE":
            # 베이스 정지, 팔이 제자리에서 위아래 1회.
            robot.apply_action(wheel_action(wheel_idx, 0.0))
            q_target = q_of_s(sweeper.step())
            q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))
            if sweeper.strokes >= STROKES_PER_WIPE:      # 위아래 1회 완료
                phase = "MOVE"; move_start_prog = progress; drive_steps = 0
                q_hold = q_of_s(sweeper.s)               # 하단 자세로 고정 이동
                print(f"[PHASE] WIPE 완료 → MOVE ({MOVE_DISTANCE:.2f} m 직진)")

        else:  # MOVE : 팔 하단 고정, 직진(heading-hold + CoM 보정)
            q_applied = q_applied + np.clip(q_hold - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))

            # heading-hold(직진 유지) + CoM 모멘트 피드포워드
            com_local, arm_mass = arm_com_in_chassis(masses, chassis_pos, chassis_R)
            com_y = com_local[1]
            d = chassis_pos - global_start
            left_dir = np.array([-forward0[1], forward0[0], 0.0])
            lateral = float(np.dot(d, left_dir))
            fwd_now = chassis_R @ np.array([1.0, 0.0, 0.0])
            yaw_now = np.arctan2(fwd_now[1], fwd_now[0])
            yaw_err = wrap_pi(yaw_now - target_yaw)
            w_fb = -(KP_YAW * yaw_err + KP_LAT * lateral)
            w_ff = -KFF_COM * (arm_mass * com_y)
            w_cmd = float(np.clip(w_fb + w_ff, -W_MAX, W_MAX))

            robot.apply_action(wheel_action(wheel_idx, fwd_sign * FORWARD_SPEED, w_cmd))
            drive_steps += 1
            moved = progress - move_start_prog

            # 첫 이동에서 휠 방향 자동보정
            if not calibrated and drive_steps >= BASE_CALIB_STEPS:
                if moved < -0.01:
                    fwd_sign = -fwd_sign
                    print(f"[INFO] 후진 감지(moved={moved:.3f}) → 휠 부호 반전")
                calibrated = True
                move_start_prog = progress               # 보정 후 거리 재측정
            elif calibrated and moved >= MOVE_DISTANCE:
                cycle += 1
                print(f"[CYCLE {cycle}] 이동 +{moved:.2f} m (누적 {progress:.2f} m) → 다음 위아래")
                phase = "WIPE"; sweeper.reset_bottom()

            if drive_steps % 60 == 0:
                print(f"  [MOVE t={step_i*PHYSICS_DT:5.1f}s] prog={progress:5.2f}m "
                      f"CoM_y={com_y:+.3f} yaw_err={np.degrees(yaw_err):+5.1f}° "
                      f"lat={lateral:+.3f} w={w_cmd:+.2f}")

    print("\n" + "=" * 60)
    print("[SUMMARY]")
    print(f"  진행 거리 : 목표 {FORWARD_DISTANCE:.1f} m / 실제 {progress:.2f} m ({cycle} 사이클)")
    print(f"  직진 보정 : KP_YAW={KP_YAW} KP_LAT={KP_LAT} KFF_COM={KFF_COM}")
    print("=" * 60)
    simulation_app.close()


if __name__ == "__main__":
    main()
