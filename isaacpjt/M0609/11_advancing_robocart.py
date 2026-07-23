"""
11_advancing_robocart.py
====================================================================
9_carter_wall_spray 기반. 두 가지 과제 추가:

[과제 1] 팔 시작 시퀀스
  - 초기 자세 = [0,0,0,0,0,0].
  - 동작 시작 순간 "최초 1회" joint_1 을 오른쪽 90° 회전(J1_OFFSET).
  - 그 이후 위아래+방사 와이프는 9_carter_wall_spray 와 동일(q_of_s + Sweeper).

[과제 2] 무게중심(CoM) 보정 직진 주행
  - 팔에 의해 옆으로 쏠린 무게중심을 매 스텝 실시간 계산(각 링크 질량×위치).
  - 그 쏠림(위치·하중)을 반영해 좌/우 휠 출력을 조절 → 직진을 유지.
    · 피드백(heading-hold) : 초기 헤딩/직선에서 벗어난 오차를 좌우륜 차등으로 보정.
    · 피드포워드(CoM)      : 팔 CoM 측면 모멘트(질량×옆거리)를 미리 상쇄.
  → Nav2 없이 직접 휠 제어로 "직진하며 소독".

메모 : 이건 9 계열 standalone(직접 휠 제어). Nav2 통합본은 10 번.
실행 : ./python.sh 11_advancing_robocart.py  → Play ▶
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
MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)
CHASSIS_MASS = 150.0

URDF_PATH = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
NOZZLE_OFFSET = 0.1392

# --- 벽 겨냥 (베이스 로컬) : 9 와 동일 ---
WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80

# --- 팔 시작 시퀀스 / 와이프 (9 와 동일) ---
J1_INDEX = 0
J1_OFFSET = -np.pi / 2       # 오른쪽 90° (최초 1회 회전 + 와이프 중 유지)
J5_INDEX = 4
J5_FLICK = -0.5
S_CRUISE = 0.90
S_ACCEL = 2.00
S_HOLD_STEPS = 6
MAX_JOINT_STEP = 0.04        # stow/전환 rate-limit

# --- Nova Carter 직진 주행 + CoM 보정 ---
WHEEL_JOINT_NAMES = ["joint_wheel_left", "joint_wheel_right"]
WHEEL_RADIUS = 0.14
WHEEL_BASE = 0.4132
ADVANCE_DISTANCE = 8.0       # 직진 목표 거리 [m]
FORWARD_SPEED = 0.20         # 전진 속도 [m/s] (와이프 중이라 다소 느리게)
BASE_CALIB_STEPS = 20        # 휠 방향 자동보정
# 직진 유지 게인
KP_YAW = 2.5                 # 헤딩 오차 → 각속도 [1/s]
KP_LAT = 1.2                 # 측면 이탈 → 각속도 [1/(m·s)]
KFF_COM = 0.010              # CoM 모멘트(질량×옆거리) 피드포워드 게인 (부호 반대면 - 로)
W_MAX = 1.0                  # 각속도 명령 상한 [rad/s]

ARM_LINK_NAMES = ["base_link", "link_1", "link_2", "link_3",
                  "link_4", "link_5", "link_6", "nozzle_base_link"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


# ────────────────────────────────────────────────────────────────
#  B. 유틸 - 수학 (9 와 동일)
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


# ────────────────────────────────────────────────────────────────
#  C. 씬 구성 (9 와 동일 : GroundPlane + Carter + 노즐 팔 병합)
# ────────────────────────────────────────────────────────────────
def build_scene(my_world):
    stage = omni.usd.get_context().get_stage()

    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(NOZZLE_USD)     # → /World/m0609

    carter_prim = stage.DefinePrim(CARTER_PRIM_PATH, "Xform")
    carter_prim.GetPayloads().AddPayload(CARTER_URL)
    UsdGeom.XformCommonAPI(carter_prim).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    for _ in range(120):
        simulation_app.update()

    chassis = stage.GetPrimAtPath(CHASSIS_LINK_PATH)
    if not chassis.IsValid():
        print(f"[FATAL] {CHASSIS_LINK_PATH} 없음 — Carter 로드 실패")
        return False
    UsdPhysics.MassAPI.Apply(chassis).CreateMassAttr(CHASSIS_MASS)

    arm_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    UsdGeom.XformCommonAPI(arm_prim).SetTranslate(
        Gf.Vec3d(MOUNT_OFFSET[0], MOUNT_OFFSET[1], MOUNT_OFFSET[2]))

    rj = stage.GetPrimAtPath(ROOT_JOINT_PATH)
    if not rj.IsValid():
        print(f"[FATAL] {ROOT_JOINT_PATH} 없음"); return False
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

    my_world.scene.add_default_ground_plane()
    for _ in range(10):
        simulation_app.update()
    print("[SCENE] GroundPlane + Carter + 노즐 팔 병합 완료")
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
    """각 팔 링크의 질량[kg] 딕셔너리 (physics:mass). CoM 계산용."""
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
    """팔 전체 무게중심을 chassis 프레임에서 본 좌표 + 총질량.
    반환 com_local[1] = 측면(y) 쏠림. (+y=좌, -y=우)"""
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
#  D. 사다리꼴 위상 왕복기 (9 와 동일)
# ────────────────────────────────────────────────────────────────
class Sweeper:
    def __init__(self, lo, hi, cruise, accel, dt, hold_steps=6):
        self.lo, self.hi = lo, hi
        self.cruise, self.accel, self.dt = cruise, accel, dt
        self.hold_steps = hold_steps
        self.reset_bottom()

    def reset_bottom(self):
        self.s = self.lo; self.dir = 1.0; self.v = 0.0; self.hold = 0

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
            self.s = end; self.v = 0.0; self.hold = self.hold_steps
        return self.s


def wheel_action(wheel_idx, linear_speed, angular_speed=0.0):
    omega_l = (2.0 * linear_speed - angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    omega_r = (2.0 * linear_speed + angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    return ArticulationAction(joint_velocities=np.array([omega_l, omega_r]),
                              joint_indices=np.array(wheel_idx))


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


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
    print(f"[ART] arm_idx={arm_idx.tolist()} wheel_idx={wheel_idx}")

    masses = read_arm_link_masses()
    print(f"[COM] 팔 링크 질량 {masses}  총 {sum(masses.values()):.2f} kg")

    # ── 하단/상단 aim IK (9 와 동일) ──
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
        print(f"[FATAL] aim IK 실패 (low={ok_lo}, high={ok_hi})"); simulation_app.close(); return
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high); q_half = 0.5 * (q_high - q_low)

    def q_of_s(s):                       # 9 와 동일 (J1_OFFSET 포함)
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    Q_ZERO = np.zeros(6)                          # 과제1: 초기 자세 [0,0,0,0,0,0]
    Q_INIT_ROT = np.zeros(6); Q_INIT_ROT[J1_INDEX] = J1_OFFSET   # j1 만 -90°

    robot.set_joint_positions(Q_ZERO, joint_indices=arm_idx)
    q_applied = Q_ZERO.copy()
    for _ in range(20):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    print("\n[RUN] Play ▶ : [0,0,0,0,0,0] → j1 오른쪽 90° → 위아래 와이프 + CoM 보정 직진.\n")
    was_playing = False
    phase = "INIT"          # INIT(j1 90°) → WIPE(와이프+직진)
    start_pos = None; forward0 = None; target_yaw = 0.0
    fwd_sign = 1.0; calibrated = False; reached = False
    step_i = 0; progress = 0.0

    while simulation_app.is_running():
        my_world.step(render=True)
        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            robot.set_joint_positions(Q_ZERO, joint_indices=arm_idx)
            q_applied = Q_ZERO.copy()
            sweeper.reset_bottom()
            phase = "INIT"; calibrated = False; reached = False
            fwd_sign = 1.0; step_i = 0
            print("[RUN] 시작: 초기 자세 [0,0,0,0,0,0]")
        was_playing = is_playing
        if not is_playing:
            continue

        chassis_pos, chassis_quat = read_world_pose(CHASSIS_LINK_PATH)
        chassis_R = quat_to_matrix(chassis_quat)

        # ── 팔 목표 ──
        if phase == "INIT":
            q_target = Q_INIT_ROT                 # 과제1: j1 만 오른쪽 90°
            robot.apply_action(wheel_action(wheel_idx, 0.0))     # 회전 중 베이스 정지
            if abs(q_applied[J1_INDEX] - J1_OFFSET) < 0.02:
                phase = "WIPE"
                start_pos = chassis_pos.copy()
                forward0 = chassis_R @ np.array([1.0, 0.0, 0.0])
                forward0 = forward0 / (np.linalg.norm(forward0) + 1e-9)
                target_yaw = np.arctan2(forward0[1], forward0[0])
                step_i = 0
                print("[PHASE] j1 90° 완료 → 와이프 + 직진 시작")
        else:  # WIPE : 9 의 와이프 + CoM 보정 직진
            q_target = q_of_s(sweeper.step())

        # 관절 rate-limit 적용
        q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))

        # ── 과제2: CoM 보정 직진 (WIPE 단계에서만) ──
        if phase == "WIPE":
            com_local, arm_mass = arm_com_in_chassis(masses, chassis_pos, chassis_R)
            com_y = com_local[1]                  # 측면 쏠림(+좌/-우)

            d = chassis_pos - start_pos
            progress = float(np.dot(d, forward0))
            left_dir = np.array([-forward0[1], forward0[0], 0.0])   # forward 의 좌측
            lateral = float(np.dot(d, left_dir))                    # +좌 이탈
            yaw_now = np.arctan2((chassis_R @ np.array([1.0, 0, 0]))[1],
                                 (chassis_R @ np.array([1.0, 0, 0]))[0])
            yaw_err = wrap_pi(yaw_now - target_yaw)

            # 피드백(직진 유지) + 피드포워드(CoM 모멘트 상쇄)
            w_fb = -(KP_YAW * yaw_err + KP_LAT * lateral)
            w_ff = -KFF_COM * (arm_mass * com_y)
            w_cmd = float(np.clip(w_fb + w_ff, -W_MAX, W_MAX))

            step_i += 1
            if not reached and abs(progress) < ADVANCE_DISTANCE:
                robot.apply_action(wheel_action(wheel_idx, fwd_sign * FORWARD_SPEED, w_cmd))
            else:
                if not reached:
                    print(f"[INFO] {ADVANCE_DISTANCE:.1f} m 도달(progress={progress:.2f}) → 정지")
                reached = True
                robot.apply_action(wheel_action(wheel_idx, 0.0))

            # 휠 방향 자동보정(첫 구간 진행이 음수면 뒤집음)
            if not calibrated and step_i == BASE_CALIB_STEPS:
                if progress < -0.01:
                    fwd_sign = -1.0
                    print(f"[INFO] 후진 감지 → 휠 부호 반전")
                calibrated = True

            if step_i % 60 == 0:
                print(f"  [t={step_i*PHYSICS_DT:5.1f}s] prog={progress:5.2f}m  "
                      f"CoM_y={com_y:+.3f}m(질량{arm_mass:.1f})  yaw_err={np.degrees(yaw_err):+5.1f}°  "
                      f"lat={lateral:+.3f}m  w={w_cmd:+.2f}")

    print("\n" + "=" * 60)
    print("[SUMMARY]")
    print(f"  직진 거리   : 목표 {ADVANCE_DISTANCE:.1f} m / 실제 {progress:.2f} m")
    print(f"  CoM 보정    : KP_YAW={KP_YAW} KP_LAT={KP_LAT} KFF_COM={KFF_COM}")
    print("=" * 60)
    simulation_app.close()


if __name__ == "__main__":
    main()
