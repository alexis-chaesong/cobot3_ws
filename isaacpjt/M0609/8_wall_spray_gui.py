"""
8_wall_spray_gui.py  (Isaac Sim GUI / Script Editor 전용)
====================================================================
Script Editor 에 붙여넣고 Ctrl+Enter 로 실행.

이 버전에서 고친 것
  * setup() 의 예외를 traceback 으로 출력 (이전엔 asyncio 가 삼켜서
    "[PHYSICS] drive tuned: 6" 이후 아무 로그도 안 나왔음).
  * 재실행에 안전: 기존 physics callback 제거 후 재등록.
  * 시작 자세 유지: 로드 직후 정면(벽 바라보는) IK 자세로 관절+드라이브
    타깃을 모두 세팅하고, Play 후 잠깐(warmup) 그 자세를 홀드 →
    엔드이펙터(link_6) 축과 노즐 축이 처음부터 일치한 채 시작. 튕김/꺾임 없음.
  * 그 다음 위-아래 왕복 스캔 시작.

멈춤:  stop()   (Script Editor 에 입력)
표준 standalone 실행을 원하면 8_wall_spray.py 를 python.sh 로.
====================================================================
"""

import sys
import asyncio
import traceback
import numpy as np

import omni.timeline
import omni.usd
from pxr import Usd, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
import isaacsim.robot_motion.motion_generation as mg

M0609_DIR = "/home/rokey/cobot3_ws/isaacpjt/M0609"
RMPFLOW_DIR = M0609_DIR + "/rmpflow"
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController

ROBOT_PRIM_PATH = "/World/m0609"
EE_FRAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/link_6"
URDF_PATH = RMPFLOW_DIR + "/m0609_isaac_sim.urdf"
DESCRIPTION_PATH = RMPFLOW_DIR + "/m0609_description.yaml"

NOZZLE_OFFSET = 0.1392
WALL_X, SCAN_Y = 0.575, 0.0
PROBE_Z_TOP, PROBE_Z_BOTTOM, PROBE_STEP = 1.30, 0.05, 0.02
Z_MARGIN = 0.03
CRUISE_SPEED, ACCEL = 0.08, 0.15
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8
PHYSICS_DT = 1.0 / 60.0
WARMUP_STEPS = 60          # Play 후 정면 자세 홀드 (약 1초)
PHYS_CB = "wall_spray_cb"

STATE = {}


# ── 수학 ──────────────────────────────────────────────────────────
def matrix_to_quat_wxyz(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2; w = .25 * S
        x = (R[2, 1] - R[1, 2]) / S; y = (R[0, 2] - R[2, 0]) / S; z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S; x = .25 * S; y = (R[0, 1] + R[1, 0]) / S; z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S; x = (R[0, 1] + R[1, 0]) / S; y = .25 * S; z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S; x = (R[0, 2] + R[2, 0]) / S; y = (R[1, 2] + R[2, 1]) / S; z = .25 * S
    q = np.array([w, x, y, z]); return q / np.linalg.norm(q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# 분사축(link_6 +Z = 노즐 축) -> 베이스 +X(벽)
ORI_QUAT = matrix_to_quat_wxyz(np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]]))


def link6_world(base_pos, base_quat, z):
    Rw = quat_to_matrix(ORI_QUAT)
    world_off = Rw @ np.array([0, 0, NOZZLE_OFFSET])
    tcp_world = np.asarray(base_pos) + quat_to_matrix(base_quat) @ np.array([WALL_X, SCAN_Y, z])
    return tcp_world - world_off


class Sweeper:
    def __init__(self, z_lo, z_hi, cruise, accel, dt, hold_steps=24):
        self.lo, self.hi, self.cruise, self.accel, self.dt = z_lo, z_hi, cruise, accel, dt
        self.hold_steps = hold_steps
        self.z = 0.5 * (z_lo + z_hi); self.dir = -1.0; self.v = 0.0; self.hold = 0

    def step(self):
        if self.hold > 0:
            self.hold -= 1
            if self.hold == 0:
                self.dir *= -1.0
            return self.z
        end = self.lo if self.dir < 0 else self.hi
        dr = abs(end - self.z)
        v_decel = np.sqrt(max(0.0, 2.0 * self.accel * dr))
        self.v = min(self.cruise, self.v + self.accel * self.dt, v_decel)
        self.z += self.dir * self.v * self.dt
        if dr <= 2e-3:
            self.z = end; self.v = 0.0; self.hold = self.hold_steps
        return self.z


def tune_drives():
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        for dt in ("angular", "linear"):
            d = UsdPhysics.DriveAPI.Get(prim, dt)
            if d:
                d.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                d.GetDampingAttr().Set(DRIVE_DAMPING)
                d.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                n += 1
    print(f"[PHYSICS] drive tuned: {n}")


def on_physics_step(step_size):
    ctrl = STATE.get("ctrl")
    if ctrl is None:
        return
    if STATE["warmup"] > 0:              # 시작 정면 자세 홀드
        STATE["warmup"] -= 1
        z = STATE["z_c"]
    else:
        z = STATE["sweeper"].step()      # 위-아래 왕복
    tgt = link6_world(STATE["base_pos"], STATE["base_quat"], z)
    action = ctrl.forward(target_end_effector_position=tgt,
                          target_end_effector_orientation=ORI_QUAT)
    STATE["robot"].apply_action(action)


async def setup():
    try:
        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
            print(f"[ERROR] {ROBOT_PRIM_PATH} 없음. m0609 로봇 씬을 먼저 여세요.")
            return

        tune_drives()

        # 이전 실행에서 남은 (물리 컨텍스트가 없는) World 싱글톤을 재사용하면
        # reset_async 안에서 get_physics_context() 가 None 이라 warm_start 에서 죽는다.
        # → 항상 싱글톤을 정리하고 새로 만들어 물리 컨텍스트를 보장.
        if World.instance() is not None:
            World.clear_instance()
        world = World(stage_units_in_meters=1.0)
        if world.get_physics_context() is None:
            print("[ERROR] physics context 생성 실패")
            return
        STATE["ctrl"] = None
        print("[SETUP] World ready (fresh), physics_context OK")

        robot = world.scene.get_object("m0609")
        if robot is None:
            robot = world.scene.add(SingleManipulator(
                prim_path=ROBOT_PRIM_PATH, name="m0609",
                end_effector_prim_path=EE_LINK_PATH, gripper=None))
        print("[SETUP] robot registered")

        await world.reset_async()
        robot.initialize()
        print("[SETUP] world reset + robot initialized, num_dof =", robot.num_dof)

        base_pos, base_quat = robot.get_world_pose()
        base_pos = np.asarray(base_pos); base_quat = np.asarray(base_quat)
        print(f"[BASE] pos={np.round(base_pos,3)} quat={np.round(base_quat,3)}")

        # 도달 가능한 Z 범위 IK 실측
        ik = mg.LulaKinematicsSolver(robot_description_path=DESCRIPTION_PATH, urdf_path=URDF_PATH)
        ik.set_robot_base_pose(base_pos, base_quat)
        reach, warm = [], None
        for z in np.arange(PROBE_Z_TOP, PROBE_Z_BOTTOM - 1e-9, -PROBE_STEP):
            q, ok = ik.compute_inverse_kinematics(
                EE_FRAME, link6_world(base_pos, base_quat, z), ORI_QUAT,
                warm_start=warm, position_tolerance=0.005, orientation_tolerance=0.05)
            if ok:
                reach.append(z); warm = q
        if not reach:
            print("[ERROR] 도달 가능한 Z 없음. WALL_X 조정 필요.")
            return
        z_hi = max(reach) - Z_MARGIN; z_lo = min(reach) + Z_MARGIN
        z_c = 0.5 * (z_lo + z_hi)
        print(f"[IK] 도달 Z {min(reach):.3f}~{max(reach):.3f} → 스캔 {z_lo:.3f}~{z_hi:.3f} "
              f"(세로 {100*(z_hi-z_lo):.1f}cm)")

        # 정면 자세 IK → 관절 + 드라이브 타깃 모두 세팅 (Play 튕김 방지, 축 정렬 유지)
        qc, ok = ik.compute_inverse_kinematics(
            EE_FRAME, link6_world(base_pos, base_quat, z_c), ORI_QUAT,
            warm_start=warm, position_tolerance=0.005, orientation_tolerance=0.05)
        if not ok:
            print("[ERROR] 중앙 IK 실패")
            return
        fq = np.zeros(robot.num_dof); fq[:6] = qc[:6]
        robot.set_joint_positions(fq)
        robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=fq))
        print(f"[INIT] 정면 자세 세팅 q={np.round(qc[:6],3)} (link_6 축 ‖ 노즐 축 → +X)")

        # RMPFlow + Sweeper
        ctrl = RMPFlowController(name="wall_spray_rmpflow", robot_articulation=robot,
                                 physics_dt=PHYSICS_DT, end_effector_frame_name=EE_FRAME)
        ctrl.reset()

        STATE.update(dict(robot=robot, ctrl=ctrl, base_pos=base_pos, base_quat=base_quat,
                          sweeper=Sweeper(z_lo, z_hi, CRUISE_SPEED, ACCEL, PHYSICS_DT),
                          z_c=z_c, warmup=WARMUP_STEPS))

        world.add_physics_callback(PHYS_CB, on_physics_step)
        await world.play_async()
        print("[RUN] 시작: 정면 자세 홀드 후 벽면 세로 스캔 왕복. 멈춤: stop()")

    except Exception:
        print("[EXCEPTION] setup 실패 ↓↓↓")
        traceback.print_exc()


def stop():
    world = World.instance()
    if world and world.physics_callback_exists(PHYS_CB):
        world.remove_physics_callback(PHYS_CB)
    STATE["ctrl"] = None
    omni.timeline.get_timeline_interface().stop()
    print("[STOP] 콜백 제거 + 정지")


asyncio.ensure_future(setup())
