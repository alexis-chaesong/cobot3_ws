"""
9_carter_wall_spray.py
====================================================================
8_wall_spray.py (M0609 소독 노즐 : 벽면 위아래 + 방사 스윕) 를 그대로 가져와,
자율주행 Nova Carter 위에 팔을 얹고 앞으로 ~30 m 주행시키면서 동시에 소독
분사 모션을 수행한다.

── 8_wall_spray.py 에서 그대로 유지 ────────────────────────────────
  - Lula IK 로 하단/상단 aim 자세(q_low, q_high) 계산 (벽면 저/고 수평 겨냥).
  - 위상 s∈[-1,+1] 왕복 : q(s)=mid+half*s, joint_5 에 스트로크 플릭(J5_FLICK).
  - 관절공간 직접 제어(드라이브 위치추종). RMPFlow 불필요.
    → 관절 목표는 "베이스(팔) 기준" 이므로, 팔을 Carter 에 얹으면 모션이
      그대로 주행에 실린다(별도 base pose 동기 불필요).

── 추가된 것 : Nova Carter 통합 + 주행 ─────────────────────────────
  (1) 씬을 코드로 구성 :
        /World/m0609            ← m0609_with_nozzle.usd (8_wall_spray 와 동일)
        /World/Nova_Carter_ROS  ← Nova_Carter_ROS.usd (payload)
      팔의 root_joint(base_link 를 world 에 고정하던 FixedJoint)를 chassis_link
      로 재연결하고 팔의 ArticulationRoot 를 제거 → Carter 하나의 articulation
      으로 병합(= mobile_manipulator_v2.usd 와 동일한 방식). 그래서 Carter 가
      움직이면 팔 전체가 함께 전진한다.
  (2) 주행 : 좌/우 휠에 속도 명령(unicycle)으로 앞(베이스 +X)으로 ~30 m 전진.

  ※ 병합 articulation 이라 팔 관절은 dof 이름(joint_1..6)으로 인덱스를 찾아
    적용한다(8_wall_spray 의 full_q[:6] 대신). IK/스윕 로직 자체는 동일.

── 동작 시퀀스 (stop-and-go) ───────────────────────────────────────
  0) 시작 시 joint_1 을 오른쪽 90°(J1_OFFSET) 돌려 옆 벽을 겨냥.
  1) [WIPE] 제자리에서 위아래 1회 소독(올라갔다 내려오기). 주행 정지.
  2) [MOVE] 팔은 정지한 채 앞으로 MOVE_DISTANCE(1 m) 이동.
  3) 1)~2) 를 총 FORWARD_DISTANCE(30 m) 까지 반복.

튜닝
  - J1_OFFSET      : 시작 베이스 회전 (기본 -90°=오른쪽, +면 왼쪽)
  - Z_LOW / Z_HIGH : 위아래 겨냥 높이 (베이스 로컬)
  - S_CRUISE/S_ACCEL : 위아래 속도(클수록 빠름), STROKES_PER_WIPE : 왕복 횟수
  - J5_FLICK       : 손목 플릭량(창문 닦기식). 방향 반대면 부호 반전.
  - MOVE_DISTANCE / FORWARD_DISTANCE / FORWARD_SPEED : 이동/총거리/속도
====================================================================
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg

_THIS_DIR = Path(__file__).resolve().parent
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

# ────────────────────────────────────────────────────────────────
#  A. 경로 / 상수
# ────────────────────────────────────────────────────────────────
NOZZLE_USD = "/home/rokey/cobot3_ws/src/integration/integration/m0609_with_nozzle.usd"
HALLWAY_USD = "/home/rokey/cobot3_ws/src/integration/integration/hospital_hallway.usd"
CARTER_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
              "Assets/Isaac/5.1/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd")

ROBOT_PRIM_PATH = "/World/m0609"
EE_FRAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/link_6"
BASE_LINK_PATH = f"{ROBOT_PRIM_PATH}/base_link"
ROOT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/root_joint"

CARTER_PRIM_PATH = "/World/Nova_Carter_ROS"
CHASSIS_LINK_PATH = f"{CARTER_PRIM_PATH}/chassis_link"
# chassis_link 기준 팔 장착 오프셋 (mobile_manipulator_v2.usd 검증값)
MOUNT_OFFSET = Gf.Vec3f(-0.2317, 0.0, 0.5773)
CHASSIS_MASS = 150.0     # 팔 무게로 인한 전복 방지

URDF_PATH = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")

NOZZLE_OFFSET = 0.1392   # link_6 원점 → nozzle_tcp (link_6 +Z)

# --- 벽 / 겨냥 (베이스 로컬) : 8_wall_spray 와 동일 ---
WALL_X = 0.575           # 벽까지 정면 거리 [m]
AIM_Y = 0.0
Z_LOW = 0.12             # 하단 겨냥 높이 [m]
Z_HIGH = 0.80            # 상단 겨냥 높이 [m]

# --- 베이스 회전 (joint_1) : 시작 시 오른쪽 90° 돌려 옆 벽을 소독 ---
J1_INDEX = 0
J1_OFFSET = -np.pi / 2   # 오른쪽 90° (왼쪽으로 돌리려면 +np.pi/2)

# --- 손목 플릭 (joint_5) : 8_wall_spray 와 동일 ---
J5_INDEX = 4
J5_FLICK = -0.5          # 스트로크 동기 손목 플릭 [rad] (방향 반대면 부호 반전)

# --- 위상 s 왕복 (사다리꼴) : 더 빠르게 ---
S_CRUISE = 0.90          # [1/s]  (기존 0.40 → 약 2배 빠름)
S_ACCEL = 2.00
S_HOLD_STEPS = 6         # 끝단 정지유지(짧게)
STROKES_PER_WIPE = 2     # "한번 위아래" = 올라갔다(1) 내려오기(2)

# --- Nova Carter 주행 (stop-and-go) ---
WHEEL_JOINT_NAMES = ["joint_wheel_left", "joint_wheel_right"]
WHEEL_RADIUS = 0.14
WHEEL_BASE = 0.4132
FORWARD_DISTANCE = 30.0  # 전체 전진 목표 거리 [m]
MOVE_DISTANCE = 0.25      # 한 번 이동 거리 [m] (이동 중 팔은 정지)
FORWARD_SPEED = 0.35     # 주행 속도 [m/s]
BASE_CALIB_STEPS = 20

# --- 드라이브 게인 (팔) ---
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

PHYSICS_DT = 1.0 / 60.0
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


# ────────────────────────────────────────────────────────────────
#  B. 유틸 - 수학 (8_wall_spray 와 동일)
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
    """노즐 분사축(link_6 +Z) → 베이스 +X(벽) 수평."""
    R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    return matrix_to_quat_wxyz(R)


def local_to_world(base_pos, base_quat, p_local):
    return base_pos + quat_to_matrix(base_quat) @ np.asarray(p_local)


def aim_link6_world(base_pos, base_quat, ori_quat, z):
    """노즐 TCP 를 (WALL_X, AIM_Y, z) 수평으로 두는 link_6 월드 목표."""
    world_offset = quat_to_matrix(ori_quat) @ np.array([0.0, 0.0, NOZZLE_OFFSET])
    return local_to_world(base_pos, base_quat, [WALL_X, AIM_Y, z]) - world_offset


# ────────────────────────────────────────────────────────────────
#  C. 유틸 - 씬 구성 (m0609 + Nova Carter 병합)
# ────────────────────────────────────────────────────────────────
def read_world_pose(prim_path):
    """prim 의 현재 월드 (pos np3, quat wxyz np4)."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation(); q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def build_scene():
    """병원 복도 + m0609 노즐 팔 + Nova Carter 를 로드하고, 팔을 Carter 에 병합 장착."""
    stage = omni.usd.get_context().get_stage()

    # (0) 병원 복도 에셋
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    hallway_prim = stage.DefinePrim("/World/hospital_hallway", "Xform")
    hallway_prim.GetReferences().AddReference(HALLWAY_USD)
    # 복도 원점 기준으로 Carter 시작 위치를 맞춰야 하면 여기서 translate 조정
    # UsdGeom.XformCommonAPI(hallway_prim).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))

    # (1) 노즐 팔 (기존 그대로)
    world_prim.GetReferences().AddReference(NOZZLE_USD)

    # (2) Nova Carter (payload)
    carter_prim = stage.DefinePrim(CARTER_PRIM_PATH, "Xform")
    carter_prim.GetPayloads().AddPayload(CARTER_URL)
    UsdGeom.XformCommonAPI(carter_prim).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    # ↑ 복도 시작 지점(입구)의 실제 좌표로 바꿔야 함. 원점이 복도 중간이면 로봇이 벽에 스폰될 수 있음.

    # 로드 대기 (Carter 는 원격 에셋이라 시간이 걸릴 수 있음)
    for _ in range(120):
        simulation_app.update()
    print(f"[LOAD] arm={NOZZLE_USD}\n[LOAD] carter={CARTER_PRIM_PATH}")

    chassis = stage.GetPrimAtPath(CHASSIS_LINK_PATH)
    if not chassis.IsValid():
        print(f"[FATAL] {CHASSIS_LINK_PATH} 없음 — Carter 로드 실패")
        return False

    # (3) 차체 질량 ↑ (전복 방지)
    mass_api = UsdPhysics.MassAPI.Apply(chassis)
    mass_api.CreateMassAttr(CHASSIS_MASS)

    # (4) 팔을 Carter 차체 위로 이동 (초기 정렬 → root_joint 형성 시 튕김 방지)
    arm_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    UsdGeom.XformCommonAPI(arm_prim).SetTranslate(
        Gf.Vec3d(MOUNT_OFFSET[0], MOUNT_OFFSET[1], MOUNT_OFFSET[2]))

    # (5) 병합 : 팔 root_joint 를 world → chassis_link 로 재연결 + 팔 ArticulationRoot 제거
    rj = stage.GetPrimAtPath(ROOT_JOINT_PATH)
    if not rj.IsValid():
        print(f"[FATAL] {ROOT_JOINT_PATH} 없음 — 팔 구조 확인 필요")
        return False
    rj.RemoveAppliedSchema("PhysicsArticulationRootAPI")
    rj.RemoveAppliedSchema("PhysxArticulationAPI")
    fj = UsdPhysics.FixedJoint(rj)
    fj.CreateBody0Rel().SetTargets([Sdf.Path(CHASSIS_LINK_PATH)])
    fj.CreateLocalPos0Attr().Set(MOUNT_OFFSET)
    fj.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # (6) 팔 base_link ↔ chassis 충돌 무시
    base_prim = stage.GetPrimAtPath(BASE_LINK_PATH)
    fp = UsdPhysics.FilteredPairsAPI.Apply(base_prim)
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(CHASSIS_LINK_PATH))

    for _ in range(10):
        simulation_app.update()
    print("[SCENE] 팔을 Carter chassis 에 병합 장착 완료")
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
            resolved = str(prim.GetPath())
            print(f"[INFO] articulation root under {root_path} -> {resolved}")
            return resolved
    return root_path


# ────────────────────────────────────────────────────────────────
#  D. 사다리꼴 위상 왕복기 (8_wall_spray 와 동일)
# ────────────────────────────────────────────────────────────────
class Sweeper:
    def __init__(self, lo, hi, cruise, accel, dt, hold_steps=24):
        self.lo, self.hi = lo, hi
        self.cruise, self.accel, self.dt = cruise, accel, dt
        self.hold_steps = hold_steps
        self.reset_bottom()

    def reset_bottom(self):
        """바닥(lo)에서 위로 출발 : 한 번의 위아래(올라갔다 내려오기) 시작 지점."""
        self.s = self.lo
        self.dir = 1.0
        self.v = 0.0
        self.hold = 0
        self.strokes = 0

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
            self.s = end
            self.v = 0.0
            self.hold = self.hold_steps
            self.strokes += 1          # 끝단 도달 = 스트로크 1회 완료
        return self.s


# ────────────────────────────────────────────────────────────────
#  E. Carter 주행 유틸
# ────────────────────────────────────────────────────────────────
def wheel_action(wheel_idx, linear_speed, angular_speed=0.0):
    """unicycle → 좌/우 휠 각속도 ArticulationAction."""
    omega_l = (2.0 * linear_speed - angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    omega_r = (2.0 * linear_speed + angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    return ArticulationAction(
        joint_velocities=np.array([omega_l, omega_r]),
        joint_indices=np.array(wheel_idx),
    )


# ────────────────────────────────────────────────────────────────
#  F. 메인
# ────────────────────────────────────────────────────────────────
def main():
    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)

    if not build_scene():
        simulation_app.close()
        return
    tune_arm_drives()

    # # 바닥 (Carter 주행용)
    # my_world.scene.add_default_ground_plane()

    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)

    # 병합 articulation 연결 (Carter root = 팔+휠 전체)
    art_root = find_articulation_root(CARTER_PRIM_PATH)
    robot = SingleArticulation(prim_path=art_root, name="carter_m0609")
    robot.initialize()
    dof_names = list(robot.dof_names)
    print(f"[ART] dof_names={dof_names}")

    # 팔/휠 dof 인덱스 확보
    try:
        arm_idx = np.array([dof_names.index(n) for n in ARM_JOINT_NAMES])
    except ValueError as e:
        print(f"[FATAL] 팔 관절 dof 미발견: {e}")
        simulation_app.close()
        return
    wheel_idx = []
    for name in WHEEL_JOINT_NAMES:
        if name in dof_names:
            wheel_idx.append(dof_names.index(name))
    if len(wheel_idx) < 2:
        cand = [i for i, n in enumerate(dof_names) if "wheel" in n.lower() and "caster" not in n.lower()]
        wheel_idx = cand[:2]
    print(f"[ART] arm_idx={arm_idx.tolist()}  wheel_idx={wheel_idx}")
    if len(wheel_idx) < 2:
        print("[FATAL] Carter 휠 dof 미발견")
        simulation_app.close()
        return

    # ── 팔 베이스 pose (Carter 위, 상승) 로 하단/상단 aim IK ──
    base_pos, base_quat = read_world_pose(BASE_LINK_PATH)
    print(f"[BASE] link base world pos={np.round(base_pos,3)} quat={np.round(base_quat,3)}")
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
        print(f"[FATAL] aim IK 실패 (low={ok_lo}, high={ok_hi}). Z_LOW/Z_HIGH/WALL_X 조정.")
        simulation_app.close()
        return
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high)
    q_half = 0.5 * (q_high - q_low)
    print(f"[AIM] q_low ={np.round(q_low,3)}")
    print(f"[AIM] q_high={np.round(q_high,3)}")

    def q_of_s(s):
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s          # 손목 플릭 (창문 닦기식)
        q[J1_INDEX] += J1_OFFSET             # 베이스 오른쪽 90° 회전 → 옆 벽 소독
        return q

    # 초기(중앙) 자세 세팅
    robot.set_joint_positions(q_of_s(0.0), joint_indices=arm_idx)
    for _ in range(30):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    print(f"\n[RUN] Play ▶ : (위아래 1회 소독 → {MOVE_DISTANCE:.0f} m 이동) 을 반복하며 "
          f"총 {FORWARD_DISTANCE:.0f} m 진행합니다.\n")

    was_playing = False
    chassis0 = None
    forward0 = None
    sign = 1.0
    calibrated = False
    reached = False
    phase = "WIPE"           # WIPE: 팔 위아래(정지 주행) / MOVE: 이동(팔 정지)
    move_start = 0.0
    hold_s = -1.0            # MOVE 중 팔이 정지해 있을 위상
    drive_steps = 0
    step_i = 0
    cycle = 0
    progress = 0.0

    while simulation_app.is_running():
        my_world.step(render=True)
        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            sweeper.reset_bottom()
            robot.set_joint_positions(q_of_s(sweeper.s), joint_indices=arm_idx)
            chassis0, cq0 = read_world_pose(CHASSIS_LINK_PATH)
            forward0 = quat_to_matrix(cq0) @ np.array([1.0, 0.0, 0.0])   # 차체 +X = 전진
            forward0 = forward0 / (np.linalg.norm(forward0) + 1e-9)
            sign = 1.0; calibrated = False; reached = False
            phase = "WIPE"; drive_steps = 0; step_i = 0; cycle = 0
            print("[RUN] 소독 시작 (위아래 → 이동 반복)")
        was_playing = is_playing

        if not is_playing:
            continue

        chassis_pos, _ = read_world_pose(CHASSIS_LINK_PATH)
        progress = float(np.dot(chassis_pos - chassis0, forward0))
        step_i += 1

        # 총 목표 거리 도달 → 정지하고 팔은 제자리에서 계속 소독
        if not reached and progress >= FORWARD_DISTANCE:
            reached = True
            print(f"[INFO] 총 {FORWARD_DISTANCE:.0f} m 도달(progress={progress:.2f}) → 주행 종료, 팔은 계속 소독")
        if reached:
            robot.apply_action(wheel_action(wheel_idx, 0.0))
            robot.apply_action(ArticulationAction(joint_positions=q_of_s(sweeper.step()),
                                                  joint_indices=arm_idx))
            continue

        if phase == "WIPE":
            # 팔 : 위아래 1회 (정지한 채 소독), 휠 정지
            robot.apply_action(wheel_action(wheel_idx, 0.0))
            s = sweeper.step()
            robot.apply_action(ArticulationAction(joint_positions=q_of_s(s), joint_indices=arm_idx))
            if sweeper.strokes >= STROKES_PER_WIPE:      # 한 번 위아래 완료
                phase = "MOVE"; move_start = progress; drive_steps = 0
                hold_s = s                               # 끝난 자세 그대로 정지 이동
        else:  # MOVE : 팔 정지, 앞으로 MOVE_DISTANCE 이동
            robot.apply_action(ArticulationAction(joint_positions=q_of_s(hold_s), joint_indices=arm_idx))
            robot.apply_action(wheel_action(wheel_idx, sign * FORWARD_SPEED))
            drive_steps += 1
            moved = progress - move_start
            if not calibrated:
                if drive_steps >= BASE_CALIB_STEPS:      # 첫 이동에서 휠 방향 보정
                    if moved < 0:
                        sign = -sign
                        print(f"[INFO] 후진 감지(moved={moved:.3f}) → 휠 부호 반전")
                    calibrated = True
                    move_start = progress                # 보정 후부터 거리 재측정
            elif moved >= MOVE_DISTANCE:
                cycle += 1
                print(f"[CYCLE {cycle}] 이동 완료(+{moved:.2f} m, 누적 {progress:.2f} m) → 다음 위아래")
                phase = "WIPE"; sweeper.reset_bottom()

        if step_i % 120 == 0:
            print(f"  [t={step_i*PHYSICS_DT:6.1f}s] phase={phase}  progress={progress:6.2f}m  cycle={cycle}")

    print("\n" + "=" * 60)
    print("[SUMMARY]")
    print(f"  진행 거리     : 목표 {FORWARD_DISTANCE:.0f} m / 실제 {progress:.2f} m ({cycle} 사이클)")
    print(f"  베이스 회전   : joint_1 {np.degrees(J1_OFFSET):+.0f}°")
    print(f"  위아래 겨냥   : Z {Z_LOW:.2f}~{Z_HIGH:.2f} m + 손목플릭 {J5_FLICK:+.2f} rad")
    print("=" * 60)
    simulation_app.close()


if __name__ == "__main__":
    main()
