"""
11_advancing_robocart_1.py
====================================================================
10_carter_hospital_spray_nav.py (Nav2 통합 주행 + 병원 배경) 과
11_advancing_robocart.py (팔 시작 시퀀스 + CoM 보정 직진) 를 "단일 모드"로 병합.
(이전 버전의 DRIVE_MODE 토글은 제거 — Nav2 주행과 CoM 보정이 항상 동시에 동작한다.)

병합 원칙
  1) 10 의 Nav2 / mapping 기능은 절대 훼손하지 않는다.
     → Nav2 의 localization(AMCL)/costmap/global·local planner 는 전혀 손대지
       않는다. 이 스크립트는 Nav2 의 "최종 속도 출력"에 CoM 트림(trim)만 얹는
       릴레이 노드로 동작 — Nav2 자신의 velocity_smoother 노드와 동일한 패턴.
  2) 11 에서 푼 두 과제는 반드시 유지한다.
     [과제1] 팔 시작 시퀀스 : 초기자세 [0]*6 → 동작 시작 "최초 1회" joint_1
             오른쪽 90°(J1_OFFSET) → 이후 위아래+방사 와이프(q_of_s + Sweeper).
     [과제2] CoM 보정 직진   : 팔 각 링크 질량×위치로 무게중심을 매 스텝 실시간
             계산 → 측면 쏠림을 좌우 휠 차등 출력(피드백+피드포워드)으로 상쇄.
  3) 배경은 항상 hospital_hallway.usd (10 과 동일 로드 방식).

  ── 어떻게 "동시에" 성립시켰나 (cmd_vel 릴레이) ─────────────────────
  Nav2 와 이 스크립트가 같은 /cmd_vel 을 동시에 발행하면 서로 덮어써서 충돌한다.
  그래서 Nav2 의 controller_server 출력을 최종 /cmd_vel 이 아니라 중간 토픽
  CMD_VEL_NAV_TOPIC(/cmd_vel_nav) 으로 remap 하고, 이 스크립트가 그것을 구독해
  CoM 보정을 더한 뒤 최종 CMD_VEL_TOPIC(/cmd_vel) 으로 재발행한다 — Carter 는
  항상 /cmd_vel 만 구독(10 과 동일). 이 패턴은 Nav2 자체 bringup 의
  velocity_smoother 노드가 하는 일과 완전히 동일해서 Nav2 내부 로직은 전혀
  건드리지 않는다.
    ⚠ 전제조건 : Nav2 bringup 이 controller_server 의 cmd_vel 출력을
      "/cmd_vel_nav" 로 remap 해서 발행해야 한다(그래야 이 스크립트가 받는다).
      이미 velocity_smoother 를 쓰고 있다면 그 입력 토픽 이름을 맞추면 된다.

  ── CoM 보정이 Nav2 의 회전 의도와 충돌하지 않는 이유 ────────────────
  과제2 원안(11)의 heading-hold 피드백(KP_YAW/KP_LAT)은 "고정된 목표 헤딩"을
  기준으로 하므로, Nav2 가 코너를 돌 때 그대로 적용하면 Nav2 의 회전 명령과
  싸운다. 그래서 Nav2 가 "직진 의도"(|angular.z| < STRAIGHT_DEADBAND)일 때만
  heading-hold 를 얹고, Nav2 가 실제로 회전 명령을 낼 때는 heading-hold 를 끄고
  CoM 피드포워드(팔의 물리적 질량 비대칭 상쇄)만 항상 더한다. 즉:
    - 회전 중        : angular.z_out = angular.z_nav2 + CoM_피드포워드
    - 직진 의도일 때 : angular.z_out = angular.z_nav2 + heading-hold + CoM_피드포워드
  linear.x 는 Nav2 값을 그대로 통과(속도 자체는 건드리지 않음).

실행
  1) 이 스크립트 실행(python.sh) → GUI 에서 Carter 위치 확인 후 Play ▶
  2) 별도 터미널 : Nav2 bringup (carter_navigation ... map:=carter_hospital_navigation.yaml)
     ※ controller_server cmd_vel 출력이 /cmd_vel_nav 로 나가도록 remap 되어 있어야 함.
  3) 별도 터미널 : ros2 run commander spray_waypoint_mission (또는 RViz Nav2 Goal)
====================================================================
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ROS2 bridge 확장 활성화 (Carter 내장 그래프 발행 + Nav2 구동에 필수)
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg

import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

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

# ★ Carter 스폰 : 복도 자유공간 + AMCL initial_pose 와 반드시 일치시킬 것 (GUI 에서 확인/조정)
CARTER_START_POSE = dict(x=0.0, y=0.0, z=0.05, yaw_deg=0.0)

URDF_PATH = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
NOZZLE_OFFSET = 0.1392

# --- 벽 겨냥 (베이스 로컬) : 8/9_wall_spray 와 동일 ---
WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80

# --- 팔 시작 시퀀스(과제1) / 와이프 : 9, 11 과 동일 ---
J1_INDEX = 0
J1_OFFSET = -np.pi / 2       # 오른쪽 90° (최초 1회 회전 + 이후 와이프 중 유지)
J5_INDEX = 4
J5_FLICK = -0.5

S_CRUISE = 0.90
S_ACCEL = 2.00
S_HOLD_STEPS = 6

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# --- /spray_active 게이트 (10 과 동일) ---
SPRAY_TOPIC = "/spray_active"
SPRAY_ALWAYS_ON = False
# "분사 OFF(=대기/주행)" 시 팔을 접어 측면 CoM 치우침을 없애는 자세.
# (과제1 의 진짜 최초 자세는 Q_START=zero 이며, STOW_Q 는 "최초 1회 활성화 이후"
#  재차 OFF 될 때만 쓰인다 — 10 의 베이스 안정화 의도를 그대로 보존.)
STOW_Q = np.array([0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
# stow↔분사 급전환 완화 : 관절 목표를 스텝당 이 값 이하로만 이동(베이스 흔들림 방지). 10/11 공통.
MAX_JOINT_STEP = 0.04    # rad / physics step

# --- Nav2 cmd_vel 릴레이 + CoM 보정 (과제2) ---
CMD_VEL_NAV_TOPIC = "/cmd_vel_nav"   # Nav2 controller_server 의 "원본" 출력 (remap 필요)
CMD_VEL_TOPIC = "/cmd_vel"           # Carter 가 실제로 구독하는 최종 토픽 (10 과 동일)
STRAIGHT_DEADBAND = 0.05             # [rad/s] Nav2 명령 |angular.z| 가 이 이하면 "직진 의도"로 간주
KP_YAW = 2.5                         # 헤딩 오차 → 각속도(angular.z) [1/s]   (직진 의도일 때만)
KP_LAT = 1.2                         # 측면 이탈 → 각속도(angular.z) [1/(m·s)] (직진 의도일 때만)
KFF_COM = 0.010                      # CoM 모멘트(질량×옆거리) 피드포워드 게인 → angular.z (항상)
W_TRIM_MAX = 1.0                     # 보정분(트림) 자체의 상한 [rad/s] (Nav2 명령에 "더해지는" 양)

# --- CoM 계산용 팔 링크 목록 (11 과 동일, 매 스텝 항상 실시간 계산) ---
ARM_LINK_NAMES = ["base_link", "link_1", "link_2", "link_3",
                  "link_4", "link_5", "link_6", "nozzle_base_link"]


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
#  C. 씬 구성 : hospital + Carter + 노즐 팔 (10 과 동일, 규칙4)
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

    # (3) 노즐 팔 + chassis 위 정렬 배치
    world_prim.GetReferences().AddReference(NOZZLE_USD)        # → /World/m0609 (defaultPrim=World)
    for _ in range(20):
        simulation_app.update()
    arm_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not arm_prim.IsValid():
        print(f"[FATAL] {ROBOT_PRIM_PATH} 없음 — 노즐 팔 로드 실패")
        return False
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    offset_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET)
    arm_m = offset_m * chassis_m                                # 팔 world = (chassis 기준 MOUNT_OFFSET)
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
    """각 팔 링크의 질량[kg] 딕셔너리 (physics:mass). CoM 계산용. (11 과 동일)"""
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
    """팔 전체 무게중심을 chassis 프레임에서 본 좌표 + 총질량. (11 과 동일)
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
#  D. 사다리꼴 위상 왕복기 (9/10/11 과 동일)
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
        print(f"[FATAL] 팔 관절 dof 미발견: {e}"); simulation_app.close(); return

    print(f"[ART] arm_idx={arm_idx.tolist()}  (전체 dof {len(dof_names)}개)")
    # 휠 조인트는 이 스크립트가 직접 명령하지 않는다(항상 /cmd_vel 경유) → dof 조회 불필요.

    masses = read_arm_link_masses()
    print(f"[COM] 팔 링크 질량 {masses}  총 {sum(masses.values()):.2f} kg")

    # ── 팔 하단/상단 aim IK (분사 모션용, 9/10/11 과 동일) ──
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

    def q_of_s(s):
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    # ── 과제1 : 진짜 초기 자세는 [0,0,0,0,0,0] ──
    Q_START = np.zeros(6)
    Q_INIT_ROT = np.zeros(6); Q_INIT_ROT[J1_INDEX] = J1_OFFSET   # j1 만 오른쪽 90°

    robot.set_joint_positions(Q_START, joint_indices=arm_idx)
    q_applied = Q_START.copy()
    for _ in range(20):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    # ── ROS2 : /spray_active 구독(팔 게이트) + /cmd_vel_nav 구독·/cmd_vel 발행(주행 릴레이) ──
    rclpy.init()
    ros_node = rclpy.create_node("carter_arm_spray_controller")

    spray_state = {"active": False}
    ros_node.create_subscription(
        Bool, SPRAY_TOPIC, lambda m: spray_state.__setitem__("active", bool(m.data)), 10)

    nav_cmd = {"v": 0.0, "w": 0.0}

    def _on_cmd_vel_nav(msg):
        nav_cmd["v"] = float(msg.linear.x)
        nav_cmd["w"] = float(msg.angular.z)

    ros_node.create_subscription(Twist, CMD_VEL_NAV_TOPIC, _on_cmd_vel_nav, 10)
    cmd_vel_pub = ros_node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    print(f"[ROS] '{SPRAY_TOPIC}' 구독(팔 게이트) / '{CMD_VEL_NAV_TOPIC}' 구독→CoM 트림→'{CMD_VEL_TOPIC}' 발행(주행 릴레이)")
    print("\n[RUN] Play ▶ : Nav2 가 경로를 계획(/cmd_vel_nav), 이 스크립트가 CoM 보정을 얹어 /cmd_vel 로 전달.\n"
          "       /spray_active True 구간에서 팔이 [0]→j1 90°(최초1회)→와이프.\n")

    was_playing = False
    have_started = False       # 최초 활성화 이전(=진짜 초기자세) 여부
    j1_rotated_once = False    # 과제1 : "최초 1회" j1 회전 완료 여부
    prev_active = None

    # 직진(heading-hold) 추적 상태 : Nav2 가 "직진 의도"일 때만 기준을 다시 잡는다.
    tracking_straight = False
    straight_start_pos = None
    forward0 = None
    target_yaw = 0.0
    step_i = 0
    com_y_last = 0.0; arm_mass_last = 0.0

    while simulation_app.is_running():
        my_world.step(render=True)
        rclpy.spin_once(ros_node, timeout_sec=0.0)     # /spray_active, /cmd_vel_nav 갱신 (비차단)

        is_playing = my_world.is_playing()
        if is_playing and not was_playing:
            robot.set_joint_positions(Q_START, joint_indices=arm_idx)
            q_applied = Q_START.copy()
            sweeper.reset_bottom()
            have_started = False; j1_rotated_once = False; prev_active = None
            tracking_straight = False; straight_start_pos = None; forward0 = None
            nav_cmd["v"] = 0.0; nav_cmd["w"] = 0.0
            step_i = 0
            print("[RUN] 시작: 초기 자세 [0,0,0,0,0,0]")
        was_playing = is_playing
        if not is_playing:
            continue

        chassis_pos, chassis_quat = read_world_pose(CHASSIS_LINK_PATH)
        chassis_R = quat_to_matrix(chassis_quat)

        # ── 팔 활성 여부 : /spray_active 게이트 (10 과 동일) ──
        active = True if SPRAY_ALWAYS_ON else spray_state["active"]
        if active and not prev_active:
            sweeper.reset_bottom()
            print("[SPRAY] ON → 팔 동작 시작")
        elif (not active) and prev_active:
            print("[SPRAY] OFF → 대기 자세로")
        prev_active = active

        # ── 과제1 : 팔 시작 시퀀스 ──
        if active:
            have_started = True
            if not j1_rotated_once:
                q_target = Q_INIT_ROT
                if abs(q_applied[J1_INDEX] - J1_OFFSET) < 0.02:
                    j1_rotated_once = True
                    sweeper.reset_bottom()
                    print("[ARM] joint_1 90° 완료(최초 1회) → 위아래+방사 와이프 시작")
            else:
                q_target = q_of_s(sweeper.step())
        else:
            q_target = STOW_Q if have_started else Q_START

        q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))

        # ── 과제2 : CoM 매 스텝 실시간 계산 ──
        com_local, arm_mass = arm_com_in_chassis(masses, chassis_pos, chassis_R)
        com_y_last, arm_mass_last = com_local[1], arm_mass

        # ── 주행 릴레이 : Nav2(/cmd_vel_nav) 에 "틀어진 정도"만 더해 /cmd_vel 로 재발행 ──
        #   linear.x 는 Nav2 값 그대로 통과(속도 자체는 이 스크립트가 정하지 않음).
        #   angular.z 는 Nav2 값 + CoM 피드포워드(항상) + heading-hold(직진 의도일 때만).
        step_i += 1
        nav_v, nav_w = nav_cmd["v"], nav_cmd["w"]

        if abs(nav_w) < STRAIGHT_DEADBAND:
            if not tracking_straight:                      # 직진 구간 진입 : 기준 재설정
                straight_start_pos = chassis_pos.copy()
                forward0 = chassis_R @ np.array([1.0, 0.0, 0.0])
                forward0 = forward0 / (np.linalg.norm(forward0) + 1e-9)
                target_yaw = np.arctan2(forward0[1], forward0[0])
                tracking_straight = True
            d = chassis_pos - straight_start_pos
            left_dir = np.array([-forward0[1], forward0[0], 0.0])
            lateral = float(np.dot(d, left_dir))
            yaw_now = np.arctan2((chassis_R @ np.array([1.0, 0, 0]))[1],
                                 (chassis_R @ np.array([1.0, 0, 0]))[0])
            yaw_err = wrap_pi(yaw_now - target_yaw)
            w_fb = -(KP_YAW * yaw_err + KP_LAT * lateral)
        else:
            tracking_straight = False                       # Nav2 가 회전 중 : heading-hold 끔(Nav2 회전과 안 싸움)
            yaw_err = 0.0; lateral = 0.0
            w_fb = 0.0

        w_ff = -KFF_COM * (arm_mass * com_y_last)            # CoM 피드포워드 : 항상 적용
        w_trim = float(np.clip(w_fb + w_ff, -W_TRIM_MAX, W_TRIM_MAX))

        twist = Twist()
        twist.linear.x = nav_v
        twist.angular.z = nav_w + w_trim
        cmd_vel_pub.publish(twist)

        if step_i % 120 == 0:
            print(f"  [t={step_i*PHYSICS_DT:6.1f}s] nav(v={nav_v:+.2f},w={nav_w:+.2f})  "
                  f"straight={tracking_straight}  yaw_err={np.degrees(yaw_err):+5.1f}°  lat={lateral:+.3f}m  "
                  f"CoM_y={com_y_last:+.3f}m(질량{arm_mass:.1f})  trim={w_trim:+.2f}  out_w={twist.angular.z:+.2f}")

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
