"""
10_carter_hospital_spray_nav.py
====================================================================
통합 단독 스크립트 : hospital_hallway 환경 + Nova Carter(ROS 구동) + m0609 노즐 팔.
  - 베이스 주행 : Nav2 가 /cmd_vel 로 구동(= Carter 내장 Twist 구독). 이 스크립트는
    휠을 직접 명령하지 않는다.
  - 팔 분사    : /spray_active(Bool) 를 구독해 True 일 때만 위아래+방사 소독 모션.
  - 센서/ TF / clock : Nova_Carter_ROS 내장 OmniGraph 가 ROS2 bridge 로 발행
    (/front_2d_lidar/scan, /front_3d_lidar/lidar_points, /chassis/odom, /tf, /clock).

역할 분리
  Nav2(터미널)          → /cmd_vel → Carter 휠  (+ /scan·/tf 로 localize)
  spray_waypoint_mission → NavigateToPose + /spray_active
  이 스크립트           → /spray_active 구독 → 팔만 제어 (휠 제어 없음)

실행
  1) 이 스크립트 실행(python.sh) → GUI 에서 Carter 위치가 복도 안인지 확인 후 Play ▶
  2) 별도 터미널 : Nav2 bringup (carter_navigation ... map:=carter_hospital_navigation.yaml)
  3) 별도 터미널 : ros2 run commander spray_waypoint_mission (또는 RViz Nav2 Goal)

튜닝
  - CARTER_START_POSE : Carter 스폰 (복도 자유공간 + AMCL 초기위치와 일치시킬 것!)
  - SPRAY_SIDE/J1_OFFSET, Z_LOW/HIGH, J5_FLICK, S_CRUISE : 8/9_wall_spray 와 동일
====================================================================
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ROS2 bridge 확장 활성화 (Carter 내장 그래프 발행 + Nav2 구동에 필수)
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg

import rclpy
from std_msgs.msg import Bool

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

J1_INDEX = 0
J1_OFFSET = -np.pi / 2       # 오른쪽 90° → 옆 벽 소독
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

SPRAY_TOPIC = "/spray_active"
# True  : /spray_active 무시하고 "항상" 위아래 와이프 (= 9_carter_wall_spray 와 동일 동작)
# False : /spray_active True 구간에서만 와이프 (정지-분사 : 주행 중엔 stow, waypoint 정지시 분사)
SPRAY_ALWAYS_ON = False

# 주행 중 stow 자세 : 팔을 접어 "중앙·낮게" → 측면 CoM 치우침 제거 → 베이스 안정.
# (분사 자세 q_of_s 는 J1_OFFSET 로 옆으로 뻗어 주행 중이면 베이스를 돌려버림)
STOW_Q = np.array([0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
# stow↔분사 급전환 완화 : 관절 목표를 스텝당 이 값 이하로만 이동(정지 중 베이스 흔들림 방지).
# 와이프 자체 속도(≈0.02rad/step)보다 커서 와이프는 안 느려지고, 큰 전환(joint_1 90°)만 완만해짐.
MAX_JOINT_STEP = 0.04    # rad / physics step


# ────────────────────────────────────────────────────────────────
#  B. 유틸 - 수학 (8/9_wall_spray 와 동일)
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
#  C. 씬 구성 : hospital + Carter + 노즐 팔
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


# ────────────────────────────────────────────────────────────────
#  D. 사다리꼴 위상 왕복기 (9_wall_spray 와 동일)
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

    # ── 팔 하단/상단 aim IK (분사 모션용) ──
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

    q_stow = STOW_Q                      # 분사 OFF(주행 중) : 접힌 중앙 자세로 베이스 안정
    robot.set_joint_positions(q_stow, joint_indices=arm_idx)
    q_applied = q_stow.copy()            # rate-limiter 상태(현재 명령 관절값)
    for _ in range(20):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    # ── ROS2 : /spray_active 구독 (팔 분사 게이트) ──
    rclpy.init()
    ros_node = rclpy.create_node("carter_arm_spray_controller")
    spray_state = {"active": False}
    ros_node.create_subscription(
        Bool, SPRAY_TOPIC, lambda m: spray_state.__setitem__("active", bool(m.data)), 10)
    print(f"[ROS] '{SPRAY_TOPIC}' 구독 시작 (True 시 분사 모션 ON)")

    print("\n[RUN] Play ▶ : Nav2 가 베이스를 주행, /spray_active True 구간에서 팔이 소독.\n")
    was_playing = False
    prev_active = None

    while simulation_app.is_running():
        my_world.step(render=True)
        rclpy.spin_once(ros_node, timeout_sec=0.0)     # /spray_active 갱신 (비차단)

        is_playing = my_world.is_playing()
        if is_playing and not was_playing:
            sweeper.reset_bottom()
            robot.set_joint_positions(q_stow, joint_indices=arm_idx)
            q_applied = q_stow.copy()
        was_playing = is_playing
        if not is_playing:
            continue

        active = True if SPRAY_ALWAYS_ON else spray_state["active"]
        if active != prev_active:
            print(f"[SPRAY] {'ON → 위아래+방사 시작' if active else 'OFF → 접힌 자세로'}")
            prev_active = active
            if active:
                sweeper.reset_bottom()

        # 팔만 제어 (휠 명령 없음 = 주행은 Nav2). stow↔분사 전환은 rate-limit 로 완만하게.
        q_target = q_of_s(sweeper.step()) if active else q_stow
        q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
