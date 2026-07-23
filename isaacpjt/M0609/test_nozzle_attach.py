"""
test_nozzle_attach.py — 분사노즐 attach 무결성 자동 점검 (작업 2)
====================================================================
목적 : 9_carter_wall_spray 의 로봇에서 노즐(nozzle_base_link)이 팔 끝(link_6)에
"고정 조인트 tool0_to_nozzle" 로 붙어 있는데, 위아래+방사 소독 모션(특히 joint_5
손목 플릭)의 빠른 가속에도 노즐이 link_6 기준으로 어긋나지 않는지 자동 검증한다.

방법
  - 정지 상태에서 T_rel0 = (link_6 기준 nozzle 상대 pose) 를 기준값으로 기록.
  - 팔을 빠르게 스윕+플릭시키며 매 스텝 T_rel 을 측정, 기준값과의
    위치/자세 편차 최대값을 추적.
  - 편차가 허용오차(POS_TOL, ANG_TOL) 이내면 PASS.

메모 : base(Carter) 주행은 팔 전체를 강체로 함께 옮기므로 link_6↔nozzle 상대
자세에 주는 응력이 미미하다. 실제 attach 를 흔드는 것은 팔 관절(특히 손목)의
각가속이므로, 고정팔 상태에서 빠른 관절 모션으로 최악 조건을 재현한다.

실행 : (Isaac Sim python.sh 로)
  ./python.sh test_nozzle_attach.py
  → 콘솔에 [PASS]/[FAIL] 과 최대 편차 출력.
====================================================================
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pathlib import Path

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator

NOZZLE_USD = "/home/rokey/cobot3_ws/src/integration/integration/m0609_with_nozzle.usd"
ROBOT_PRIM_PATH = "/World/m0609"
LINK6_PATH = f"{ROBOT_PRIM_PATH}/link_6"
NOZZLE_PATH = f"{ROBOT_PRIM_PATH}/nozzle_base_link"
EE_LINK_PATH = LINK6_PATH

PHYSICS_DT = 1.0 / 60.0
JOINT_LOWER = np.array([-6.2832, -6.2832, -2.618, -6.2832, -6.2832, -6.2832])
JOINT_UPPER = np.array([6.2832, 6.2832, 2.618, 6.2832, 6.2832, 6.2832])

# 허용오차 (고정 조인트라면 거의 0 이어야 함; 솔버 미세 컴플라이언스 여유)
POS_TOL = 0.005   # [m] 5mm
ANG_TOL = 1.0     # [deg]

N_WARMUP = 60
N_TEST = 900      # 15초


def read_world_pose(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation(); q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def relative_pose(parent_path, child_path):
    """child 를 parent 프레임에서 본 (상대위치 np3, 상대회전행렬 3x3)."""
    p_pos, p_quat = read_world_pose(parent_path)
    c_pos, c_quat = read_world_pose(child_path)
    R_p = quat_to_matrix(p_quat)
    rel_pos = R_p.T @ (c_pos - p_pos)
    rel_R = R_p.T @ quat_to_matrix(c_quat)
    return rel_pos, rel_R


def rot_angle_deg(R):
    """회전행렬 R 의 회전각[deg]."""
    ang = np.arccos(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return np.degrees(ang)


def main():
    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)

    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(NOZZLE_USD)
    for _ in range(30):
        simulation_app.update()

    if not stage.GetPrimAtPath(NOZZLE_PATH).IsValid():
        print(f"[FAIL] {NOZZLE_PATH} 없음 — 노즐 prim 경로 확인 필요")
        simulation_app.close(); return

    robot = my_world.scene.add(
        SingleManipulator(prim_path=ROBOT_PRIM_PATH, name="m0609",
                          end_effector_prim_path=EE_LINK_PATH, gripper=None))
    my_world.reset()
    robot.initialize()
    my_world.play()

    # 정지 자세로 워밍업 후 기준 상대 pose 기록
    q_home = np.zeros(robot.num_dof)
    q_home[:6] = np.array([0.0, -0.5, 1.2, 0.0, 1.0, 0.0])
    robot.set_joint_positions(q_home)
    for _ in range(N_WARMUP):
        my_world.step(render=False)

    rel_pos0, rel_R0 = relative_pose(LINK6_PATH, NOZZLE_PATH)
    print(f"[REF] 노즐 상대위치(link_6 기준)={np.round(rel_pos0,4)} m  |rel|={np.linalg.norm(rel_pos0):.4f}")

    # 빠른 스윕 + 손목 플릭으로 attach 응력 인가
    max_pos_dev = 0.0
    max_ang_dev = 0.0
    t = 0.0
    for i in range(N_TEST):
        t += PHYSICS_DT
        q = np.zeros(robot.num_dof)
        # 여러 관절 동시 고속 왕복 (특히 joint_5 손목 플릭)
        q[:6] = np.array([
            0.6 * np.sin(2 * np.pi * 0.5 * t),
            -0.5 + 0.5 * np.sin(2 * np.pi * 0.6 * t),
            1.2 + 0.4 * np.sin(2 * np.pi * 0.7 * t),
            0.8 * np.sin(2 * np.pi * 1.1 * t),
            1.0 + 0.9 * np.sin(2 * np.pi * 1.5 * t),   # 손목 플릭(빠름)
            1.2 * np.sin(2 * np.pi * 1.3 * t),
        ])
        robot.apply_action(ArticulationAction(joint_positions=q))
        my_world.step(render=False)

        rel_pos, rel_R = relative_pose(LINK6_PATH, NOZZLE_PATH)
        pos_dev = np.linalg.norm(rel_pos - rel_pos0)
        ang_dev = rot_angle_deg(rel_R0.T @ rel_R)
        max_pos_dev = max(max_pos_dev, pos_dev)
        max_ang_dev = max(max_ang_dev, ang_dev)
        if (i + 1) % 150 == 0:
            print(f"  [t={t:5.1f}s] 최대편차 pos={max_pos_dev*1000:.2f}mm ang={max_ang_dev:.3f}deg")

    print("\n" + "=" * 56)
    ok = (max_pos_dev <= POS_TOL) and (max_ang_dev <= ANG_TOL)
    print(f"[{'PASS' if ok else 'FAIL'}] 노즐 attach 무결성")
    print(f"  최대 위치 편차 : {max_pos_dev*1000:.3f} mm  (허용 {POS_TOL*1000:.1f} mm)")
    print(f"  최대 자세 편차 : {max_ang_dev:.3f} deg  (허용 {ANG_TOL:.1f} deg)")
    print("  → tool0_to_nozzle 고정 조인트가 고속 모션에도 노즐을 단단히 유지" if ok
          else "  → 편차 큼: 고정 조인트/드라이브 게인/질량 확인 필요")
    print("=" * 56)
    simulation_app.close()


if __name__ == "__main__":
    main()
