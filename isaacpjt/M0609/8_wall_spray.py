"""
8_wall_spray.py
====================================================================
두산 M0609 + 소독 분사 노즐 : 벽면 "최대 세로 범위" 방사형 스윕.

목표 : 병원 벽면 소독 분사. 창문/칠판 닦기식 "스트로크 + 손목 플릭".
  - 팔이 벽 아래→위로 한 스트로크로 크게 왕복(joint_2,3)하는 동안,
    그 스트로크에 물려서 joint_5(손목 pitch)가 진행 방향으로 쭉 벌어짐.
    → 위로 갈수록 손목이 위로 열리고, 아래로 갈수록 아래로 열려
      스트로크 끝에서 노즐이 최대로 벌어지는 넓은 상하 분사 패턴.

원리
  - Lula IK 로 하단/상단 aim 자세(q_low, q_high)를 구함
    (노즐이 벽면 낮은 곳/높은 곳을 수평으로 겨냥).
  - 위상 s∈[-1,+1] 사다리꼴 왕복:
        q(s) = mid + half*s          # 전 관절 보간 (주로 joint_2,3)
        q(s)[joint_5] += FLICK*s     # 스트로크에 물린 손목 플릭(창문 닦기식)
  - s=+1 : 상단 자세 + 손목 위로 벌림  → 벽면 최상단 분사
    s=-1 : 하단 자세 + 손목 아래로 벌림 → 벽면 최하단 분사
  - joint-space 직접 제어(드라이브 위치추종). RMPFlow 불필요, 추종오차 극소.

튜닝
  - Z_LOW / Z_HIGH : 상하 스윕 겨냥 높이 (도달 범위 내)
  - J5_FLICK       : 스트로크에 물린 손목 플릭량 (클수록 손목을 더 크게 벌림)
====================================================================
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys

import numpy as np
import omni.usd
from pxr import Usd, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
import isaacsim.robot_motion.motion_generation as mg

_THIS_DIR = Path(__file__).resolve().parent
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

# ────────────────────────────────────────────────────────────────
#  A. 경로 / 상수
# ────────────────────────────────────────────────────────────────
USD_PATH = "/home/rokey/cobot3_ws/src/integration/integration/m0609_with_nozzle.usd"
ROBOT_PRIM_PATH = "/World/m0609"
EE_FRAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/link_6"

URDF_PATH = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")

NOZZLE_OFFSET = 0.1392   # link_6 원점 → nozzle_tcp (link_6 +Z)

# --- 벽 / 병진 겨냥 (베이스 로컬) ---
WALL_X = 0.575           # 벽까지 정면 거리 [m]
AIM_Y = 0.0              # 좌우 중앙
Z_LOW = 0.12             # 병진 하단 겨냥 높이 [m]
Z_HIGH = 0.80            # 병진 상단 겨냥 높이 [m]

# --- 손목 플릭 (joint_5 : 스윕 스트로크에 동기 / 창문·칠판 닦기식) ---
J5_INDEX = 4
J5_FLICK = -0.5          # 스윕 방향 동기 손목 플릭량 [rad]
                         #   아래(s=-1)→위(s=+1) 스트로크에서 손목을 위로 쭉 벌림.
                         #   s 와 함께 증가 → 스트로크 끝(위/아래)에서 최대로 열림.
                         #   방향이 반대로 보이면 부호를 (-) 로 뒤집을 것.
                         #   과하면 하단에서 노즐이 바닥밑을 향할 수 있음(SUMMARY 확인).

# --- 위상 s 왕복 속도 (사다리꼴) ---
S_CRUISE = 0.40          # [1/s]  (full -1→+1 약 5s)
S_ACCEL = 0.80           # [1/s^2]
S_HOLD_STEPS = 24        # 끝단 정지유지

# --- 드라이브 게인 ---
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

PHYSICS_DT = 1.0 / 60.0
JOINT_LOWER = np.array([-6.2832, -6.2832, -2.618, -6.2832, -6.2832, -6.2832])
JOINT_UPPER = np.array([6.2832, 6.2832, 2.618, 6.2832, 6.2832, 6.2832])


# ────────────────────────────────────────────────────────────────
#  B. 유틸
# ────────────────────────────────────────────────────────────────
def load_usd():
    from pxr import UsdGeom
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(USD_PATH)
    for _ in range(20):
        simulation_app.update()
    print(f"[LOAD] {USD_PATH}")


def tune_drives():
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
    print(f"[PHYSICS] drive tuned: {n}")


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


def wall_hit_z(ee_pos, ee_quat, base_pos):
    """link_6 자세의 분사 ray 가 벽평면(베이스 로컬 X=WALL_X)에 닿는 높이[z]."""
    zaxis = quat_to_matrix(ee_quat) @ np.array([0.0, 0.0, 1.0])
    tcp = np.asarray(ee_pos) + NOZZLE_OFFSET * zaxis
    wall_x_world = base_pos[0] + WALL_X
    if abs(zaxis[0]) < 1e-6:
        return None
    t = (wall_x_world - tcp[0]) / zaxis[0]
    return (tcp + t * zaxis)[2]


# ────────────────────────────────────────────────────────────────
#  C. 사다리꼴 위상 왕복기 (s ∈ [lo, hi])
# ────────────────────────────────────────────────────────────────
class Sweeper:
    def __init__(self, lo, hi, cruise, accel, dt, hold_steps=24):
        self.lo, self.hi = lo, hi
        self.cruise, self.accel, self.dt = cruise, accel, dt
        self.hold_steps = hold_steps
        self.s = 0.5 * (lo + hi)
        self.dir = 1.0
        self.v = 0.0
        self.hold = 0

    def reset_center(self):
        self.s = 0.5 * (self.lo + self.hi)
        self.dir = 1.0
        self.v = 0.0
        self.hold = 0

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
        return self.s


# ────────────────────────────────────────────────────────────────
#  D. 메인
# ────────────────────────────────────────────────────────────────
def main():
    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)

    load_usd()
    tune_drives()

    robot = my_world.scene.add(
        SingleManipulator(prim_path=ROBOT_PRIM_PATH, name="m0609",
                          end_effector_prim_path=EE_LINK_PATH, gripper=None)
    )
    my_world.reset()
    robot.initialize()

    base_pos, base_quat = robot.get_world_pose()
    base_pos = np.asarray(base_pos); base_quat = np.asarray(base_quat)
    print(f"[BASE] world pos={np.round(base_pos,3)}  quat={np.round(base_quat,3)}")

    ori_quat = spray_orientation_quat()

    # ── D-1. 하단/상단 aim 자세 IK ──────────────────────────────
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
        raise RuntimeError(f"aim IK 실패 (low={ok_lo}, high={ok_hi}). Z_LOW/Z_HIGH/WALL_X 조정.")
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high)
    q_half = 0.5 * (q_high - q_low)
    print(f"[AIM] q_low ={np.round(q_low,3)}")
    print(f"[AIM] q_high={np.round(q_high,3)}")
    movers = [f"joint_{i+1}:{d:+.2f}" for i, d in enumerate(q_high - q_low) if abs(d) > 0.05]
    print(f"[AIM] 주 이동 관절: {movers}  + joint_5 스트로크 동기 플릭 ±{J5_FLICK}rad")

    def q_of_s(s):
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s          # 스트로크 방향으로 손목을 쭉 벌림(플릭)
        return q                             #   위로 갈수록 joint_5 ↑ (창문 닦기식)

    # 초기 자세(중앙) 세팅 후 홀드
    full_q = np.zeros(robot.num_dof)
    full_q[:6] = q_of_s(0.0)
    robot.set_joint_positions(full_q)
    for _ in range(30):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    # ── 진단 ───────────────────────────────────────────────────
    wall_z_min, wall_z_max = 1e9, -1e9
    limit_hit = False
    step_i = 0

    print("\n[RUN] Play 를 누르면 (상하 스윕 + joint_5 스트로크 동기 플릭) 합성 동작을 시작합니다.\n")
    was_playing = False

    while simulation_app.is_running():
        my_world.step(render=True)
        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            robot.set_joint_positions(full_q)
            sweeper.reset_center()
        was_playing = is_playing

        if not is_playing:
            continue

        # (1) 상하 스윕 + 스트로크 동기 손목 플릭 : 위상 s → 관절 목표
        #     (q_of_s 안에서 joint_5 가 s 와 함께 벌어짐 → 창문 닦기식 플릭)
        s = sweeper.step()
        q_target = np.zeros(robot.num_dof)
        q_target[:6] = q_of_s(s)

        # (2) joint-space 직접 명령
        robot.apply_action(ArticulationAction(joint_positions=q_target))

        # (3) 진단: 벽면 분사 높이
        ee_pos, ee_quat = robot.end_effector.get_world_pose()
        zw = wall_hit_z(ee_pos, ee_quat, base_pos)
        if zw is not None:
            wall_z_min = min(wall_z_min, zw)
            wall_z_max = max(wall_z_max, zw)

        q_now = robot.get_joint_positions()[:6]
        near = (q_now <= JOINT_LOWER + 0.02) | (q_now >= JOINT_UPPER - 0.02)
        if near.any():
            limit_hit = True

        step_i += 1
        if step_i % 60 == 0 and zw is not None:
            print(f"  [t={step_i*PHYSICS_DT:6.1f}s] s={s:+.2f}  wall_z={zw:.3f}m  "
                  f"cover=[{wall_z_min:.3f},{wall_z_max:.3f}]  limit_near={'Y' if near.any() else 'n'}")

    print("\n" + "=" * 60)
    print("[SUMMARY]")
    print(f"  병진 겨냥 Z      : {Z_LOW:.2f} ~ {Z_HIGH:.2f} m  (joint 2,3,5 보간)")
    print(f"  손목 플릭        : joint_5 ±{J5_FLICK} rad (스트로크 동기)")
    print(f"  벽면 분사 커버 Z : {wall_z_min:.3f} ~ {wall_z_max:.3f} m "
          f"(세로 {100*(wall_z_max-wall_z_min):.1f} cm)")
    print(f"  관절리밋 근접    : {'발생' if limit_hit else '없음'}")
    print("=" * 60)
    simulation_app.close()


if __name__ == "__main__":
    main()
