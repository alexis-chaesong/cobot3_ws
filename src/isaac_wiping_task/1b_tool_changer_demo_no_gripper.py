"""
1b_tool_changer_demo_no_gripper.py
-----------------------------------
Part 1 실험용 변형: RG2 그리퍼 없이 M0609 팔(link_6/tool0)에 Surface Gripper를
직접 부착했을 때, 거치대 선반과의 콜리전 문제가 사라지는지 검증하기 위한 스크립트.

배경: 1_tool_changer_demo.py에서는 Surface Gripper를 RG2의 gripper_body에 붙였는데,
gripper_body 원점이 손가락 끝단이 아니라 그리퍼 "몸통 중앙"에 있어서 실제 목표
지점(handle)까지 접근하려면 RG2 전체 부피(손가락+몸통)가 거치대 선반과 부딪히는
문제가 있었다. 이 스크립트는 RG2를 아예 빼고(doosan-robot2/urdf/m0609_isaac_sim.urdf
기반 자산 사용) link_6에 바로 Surface Gripper를 붙여서, 콜리전 부피가 줄어들면
문제가 해결되는지 확인한다.

주의: 이건 어디까지나 "콜리전이 그리퍼 부피 때문이었는가"를 확인하기 위한 진단용
스크립트다. 원래 Part 1 요구사항("화면상 RG2가 걸레를 잡은 것처럼 보여야 함")은
이 구성으로는 만족되지 않는다 — RG2 자체가 없기 때문이다. 진단 결과에 따라
1_tool_changer_demo.py 쪽의 local_pos0(그리퍼 손가락 끝단 오프셋)를 조정하는 방향으로
되돌아갈 수도 있다.

headless로 직접 확인한 이 자산의 구조:
  - 루트 프림: "/m0609" (이 USD에도 /World가 없음)
  - base_link, link_1..link_6만 있고 그리퍼 관련 링크(quick_changer/gripper_body/
    fingers)는 전혀 없다.
  - tool0은 link_6에 중첩된 프레임이며 RigidBodyAPI가 없다(정상). link_6과 tool0의
    world position이 사실상 동일하다 — 그리퍼가 없으니 당연히 오프셋도 없다.
  - base_link, link_6 world position은 m0609_with_gripper.usd와 동일한 팔 기구학이라
    STAND_POSITION=(0.6,0,0.3) 등 기존에 검증한 좌표를 그대로 재사용할 수 있다.

실행 방법:
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh 1b_tool_changer_demo_no_gripper.py
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
_WS_ROOT = _THIS_DIR.parents[1]  # cobot3_ws
M0609_DIR = _WS_ROOT / "isaacpjt" / "M0609"
RMPFLOW_DIR = str(_WS_ROOT / "src" / "integration" / "integration" / "rmpflow")  # 공유 rmpflow 컨트롤러 위치로 이동됨
for p in (str(_THIS_DIR), RMPFLOW_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import surface_gripper_utils  # noqa: E402
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402
from tool_changer import ToolChangerController  # noqa: E402

# ─────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────
# 그리퍼가 없는 순수 M0609 팔 자산 (doosan-robot2/urdf/m0609_isaac_sim.urdf 기반).
USD_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim" / "m0609_isaac_sim.usd")  # src/doosan-robot2로 이동됨
ROBOT_PRIM_PATH = "/m0609"
EE_LINK_NAME = "link_6"

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

# 그리퍼가 없으므로 Surface Gripper를 link_6(팔의 마지막 rigid body)에 직접 부착한다.
# tool0은 RigidBodyAPI가 없어(순수 프레임) 부착점으로 쓸 수 없다.
RG2_FINGERTIP_LINK_NAME = "link_6"

# 거치대 선반(FixedCuboid)을 없애고 걸레를 바닥(기본 ground plane)에 바로 놓는다.
# 선반 모서리와의 충돌(팔이 옆으로 튕겨나가는 문제)을 근본적으로 피하기 위함이다 —
# 평평한 바닥은 선반처럼 튀어나온 모서리가 없다. STAND_POSITION.z=0은 "바닥 표면"을
# 뜻하고, 걸레 pad/handle 좌표는 여기에 상대적으로 계산된다 (create_placeholder_mop 참고).
STAND_POSITION = np.array([0.5, 0.0, 0.0])
STAND_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])  # (w, x, y, z)

# 그리퍼가 없으니 link_6(=Surface Gripper 부착점)과 IK 목표 프레임(tool0)이 사실상
# 같은 위치다 — fingertip_offset_from_ik_frame 보정이 필요 없다(0으로 둔다).
FINGERTIP_OFFSET_FROM_TOOL0 = np.zeros(3)

EE_OFFSET = np.array([0.0, 0.0, 0.15])

# 원래 이 값을 0.015m로 띄워서 걸레 표면에 안 닿게 하려 했는데, 그러면 Surface
# Gripper의 grip 감지 축(gripper_body/link_6의 로컬 Z, world 기준과 방향이 다를 수
# 있음)에서 오히려 더 멀어져 그립이 실패하는 것을 headless로 확인했다. 거치대를
# 선반(FixedCuboid, 모서리가 있어 팔이 옆으로 튕겨나가는 원인이었음)에서 평평한
# 바닥(ground plane)으로 바꾸고 나니, handle 좌표에 정확히 맞춰도(모서리가 없어)
# 더 이상 옆으로 미끄러지는 문제가 재현되지 않아 0으로 되돌렸다.
GRASP_APPROACH_CLEARANCE = np.zeros(3)

PHYSICS_DT = 1.0 / 60.0

POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400

# 전체 동작이 너무 빨라(급가속/급정지) 파지한 걸레가 흔들기 도중 미끄러지는 문제가
# 있었다. RMPflow.forward()는 그 tick에 준 target으로 즉시 끌어당기는 stateless
# attractor라 별도 감속 로직이 없으므로, 목표를 한 번에 주지 않고 매 tick 조금씩
# (smoothstep 이징으로) 옮겨가며 넘겨서 속도를 직접 제한한다.
MAX_EE_LINEAR_SPEED = 0.10  # m/s, 접근/복귀 램프 속도
MIN_INTERP_STEPS = 60  # 1.0s @ 60Hz — 아주 짧은 이동도 눈에 보이게 램프되도록 하는 하한
MAX_INTERP_STEPS = 600  # 10.0s @ 60Hz — 예상외로 먼 거리라도 데모가 무한정 걸리지 않도록 하는 상한

SHAKE_MAX_LINEAR_SPEED = 0.08  # m/s — 걸레를 쥔 상태라 좀 더 조심스럽게
SHAKE_HOLD_STEPS = 30  # 0.5s — 각 흔들기 웨이포인트 도착 후 수렴 대기 없이 잠깐만 정지


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
    matrix = omni.usd.get_world_transform_matrix(prim)
    translation = matrix.ExtractTranslation()
    return np.array([translation[0], translation[1], translation[2]])


def create_placeholder_mop(stage, world):
    """걸레 placeholder를 거치대 선반이 아니라 바닥(ground plane) 위에 바로 놓는다.
    STAND_POSITION.z = 0 (바닥 표면)을 기준으로 pad가 그 위에 얹힌다.
    """
    mop_pad_path = "/World/Mop/pad"
    pad_half_height = 0.01
    pad_position = STAND_POSITION + np.array([0.0, 0.0, pad_half_height])
    world.scene.add(
        DynamicCuboid(
            prim_path=mop_pad_path,
            name="mop_pad",
            position=pad_position,
            scale=np.array([0.15, 0.25, pad_half_height * 2.0]),
            color=np.array([0.6, 0.6, 0.9]),
            mass=0.2,
        )
    )
    handle_path = f"{mop_pad_path}/handle"
    handle_xform = UsdGeom.Xform.Define(stage, handle_path)
    handle_xform.AddTranslateOp().Set((0.0, 0.0, pad_half_height))
    return handle_path


def set_drive_gains(stage, root_path: str):
    drive_count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive_count += 1
    print(f"[INFO] 관절 드라이브 강성/댐핑 설정 완료: {drive_count}개")


def _smoothstep(alpha: float) -> float:
    """velocity가 양 끝(alpha=0, 1)에서 0이 되는 이징 커브. 선형보간과 달리 램프가
    끝나고 정지할 때 급정지가 생기지 않는다."""
    return alpha * alpha * (3.0 - 2.0 * alpha)


def _ramp_steps_for_distance(distance: float, max_linear_speed: float) -> int:
    """이동 거리를 최대 선속도로 나눠 필요한 step 수를 구하고, 너무 짧거나
    너무 길지 않게 clip한다."""
    raw_steps = distance / max_linear_speed / PHYSICS_DT
    return int(np.clip(round(raw_steps), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def _ramp_ee_target(world, robot, rmpflow, tool0_path, target_position, target_orientation, ramp_steps: int):
    """매 physics tick마다 목표를 현재 위치에서 target_position까지 smoothstep
    이징으로 조금씩 옮겨가며 RMPflow에 넘긴다. 한 번에 먼 목표를 그대로 주면
    RMPflow가 급가속/급정지로 반응하는 것을 막기 위함이다. orientation은 호출
    구간 내내 값이 바뀌지 않으므로(이 스크립트의 모든 호출부에서 그렇다) 보간하지
    않고 그대로 고정해서 넘긴다.
    """
    start_position = get_prim_world_position(tool0_path)
    for step in range(ramp_steps):
        world.step(render=True)
        alpha = (step + 1) / ramp_steps
        waypoint = start_position + _smoothstep(alpha) * (target_position - start_position)
        action = rmpflow.forward(
            target_end_effector_position=waypoint,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)


def move_to_pose(
    world, robot, rmpflow, tool0_path, target_position, target_orientation, label: str,
    max_linear_speed: float = MAX_EE_LINEAR_SPEED,
):
    start_position = get_prim_world_position(tool0_path)
    distance = float(np.linalg.norm(target_position - start_position))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(
        f"[INFO] {label} 목표 {target_position} 로 이동 시작 "
        f"(start={start_position}, dist={distance:.3f}m, ramp_steps={ramp_steps})..."
    )
    _ramp_ee_target(world, robot, rmpflow, tool0_path, target_position, target_orientation, ramp_steps)

    for step in range(MAX_APPROACH_STEPS):
        world.step(render=True)
        ee_position = get_prim_world_position(tool0_path)
        action = rmpflow.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)
        if ToolChangerController.is_at_pose(ee_position, target_position, POSITION_TOLERANCE):
            print(f"[INFO] {label} 도달 (settle step={step}, ee={ee_position})")
            return True
    final_pos = get_prim_world_position(tool0_path)
    print(
        f"[WARN] {label} 이동이 {MAX_APPROACH_STEPS} step 내에 목표 허용오차 이내로 수렴하지 못했습니다. "
        f"final_ee={final_pos} dist={np.linalg.norm(final_pos - target_position):.4f}"
    )
    return False


def shake_test(world, robot, rmpflow, tool0_path, tool_changer, center_position, orientation):
    """파지 후 걸레가 실제로 안 떨어지는지 확인하기 위해 EE를 좌우/전후로 흔든다.

    각 웨이포인트는 move_to_pose와 동일한 smoothstep 램프로 부드럽게 이동하되,
    move_to_pose의 장시간 수렴-대기(MAX_APPROACH_STEPS)는 쓰지 않는다 — 목적이
    "정확한 도달"이 아니라 "흔드는 동안 안 떨어지는지"이므로, 도착 후 짧게만
    (SHAKE_HOLD_STEPS) 정지한 뒤 바로 다음 방향으로 넘어간다.
    """
    offsets = [
        np.array([0.06, 0.0, 0.0]),
        np.array([-0.06, 0.0, 0.0]),
        np.array([0.0, 0.06, 0.0]),
        np.array([0.0, -0.06, 0.0]),
        np.array([0.0, 0.0, 0.0]),  # 중앙으로 복귀
    ]
    for i, offset in enumerate(offsets):
        target = center_position + offset
        start_position = get_prim_world_position(tool0_path)
        distance = float(np.linalg.norm(target - start_position))
        ramp_steps = _ramp_steps_for_distance(distance, SHAKE_MAX_LINEAR_SPEED)
        print(f"[INFO] 흔들기 {i + 1}/{len(offsets)} -> {target} (ramp_steps={ramp_steps})")
        _ramp_ee_target(world, robot, rmpflow, tool0_path, target, orientation, ramp_steps)

        for _ in range(SHAKE_HOLD_STEPS):
            world.step(render=True)
            action = rmpflow.forward(target_end_effector_position=target, target_end_effector_orientation=orientation)
            robot.apply_action(action)

        still_closed = tool_changer.surface_gripper.is_closed()
        mop_pos = get_prim_world_position("/World/Mop/pad")
        print(f"[CHECKPOINT] 흔들기 {i + 1} 후 is_closed()={still_closed}, mop_pad pos={mop_pos}")


def main():
    stage = omni.usd.get_context().get_stage()
    stage.GetRootLayer().subLayerPaths.append(USD_PATH)
    for _ in range(30):
        simulation_app.update()

    world = World(physics_dt=PHYSICS_DT)
    world.scene.add_default_ground_plane()

    set_drive_gains(stage, ROBOT_PRIM_PATH)

    ee_prim_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
    if ee_prim_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' 링크를 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] End Effector rigid body ({EE_LINK_NAME}) = {ee_prim_path}")

    tool0_path = find_prim_path_by_name(ROBOT_PRIM_PATH, "tool0")
    if tool0_path is None:
        raise RuntimeError(f"'tool0' 프레임을 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] IK 목표 프레임(tool0) = {tool0_path}")

    fingertip_path = find_prim_path_by_name(ROBOT_PRIM_PATH, RG2_FINGERTIP_LINK_NAME)
    if fingertip_path is None:
        raise RuntimeError(f"'{RG2_FINGERTIP_LINK_NAME}' 링크를 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] Surface Gripper 부착점 = {fingertip_path}")

    # world.reset()/robot.initialize() 이전에 Surface Gripper D6 조인트를 authoring
    # (articulation view 구성 이후에 추가하면 articulation이 깨지는 문제를 이미 확인함).
    surface_gripper_prim_path = surface_gripper_utils.setup_mop_surface_gripper(
        stage, fingertip_prim_path=fingertip_path
    )

    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=None,  # 그리퍼 없음 — 시각적 동기화 대상도 없다.
        )
    )

    # 걸레/거치대는 world.reset() 이전에 씬에 추가 (동적 추가 시 브로드페이즈 깨짐 문제 확인됨).
    handle_path = create_placeholder_mop(stage, world)

    world.reset()
    robot.initialize()

    # 드라이브 강성(1e8)로 인한 초기 스냅 방지 — 관절을 명시적으로 0으로 지정.
    robot.set_joint_positions(np.zeros(robot.num_dof))
    for _ in range(10):
        world.step(render=True)

    rmpflow = RMPFlowController(
        name="tool_changer_cspace_controller",
        robot_articulation=robot,
    )

    tool_changer = ToolChangerController(
        rg2_fingertip_prim_path=fingertip_path,
        mop_handle_prim_path=handle_path,
        stand_position=STAND_POSITION,
        stand_orientation=STAND_ORIENTATION,
        fingertip_offset_from_ik_frame=FINGERTIP_OFFSET_FROM_TOOL0,
        rg2_gripper=None,  # 그리퍼 없음
        surface_gripper_prim_path=surface_gripper_prim_path,
        auto_create_surface_gripper=False,
    )
    tool_changer.initialize()

    for _ in range(30):
        world.step(render=True)

    # 1) 걸레 접합부 위 여유 공간으로 접근 후 하강
    handle_position, handle_orientation = tool_changer.approach_tool_stand()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + EE_OFFSET, handle_orientation, "걸레 접합부 상공 접근",
    )
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + GRASP_APPROACH_CLEARANCE, handle_orientation, "걸레 접합부 하강",
    )

    # 2) 파지: Surface Gripper 물리 고정 (그리퍼가 없으니 시각적 동기화는 생략)
    tool_changer.grasp_mop()
    for _ in range(30):
        world.step(render=True)

    # 3) 흔들기 전, 이미 충돌 없음이 검증된 상공 지점(접근 시 지나온 handle_position +
    #    EE_OFFSET)까지 먼저 들어올려 거치대 선반에서 벗어난다 ("뒤로 물러난 뒤 흔들기").
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + EE_OFFSET, handle_orientation, "파지 후 흔들기 대기 위치로 상승",
    )

    # 4) 파지 검증: 선반에서 벗어난 상공에서 흔들어도 안 떨어지는지 확인
    #    (Part 3의 실제 와이핑 궤적 이전 단계 점검)
    shake_test(
        world, robot, rmpflow, tool0_path, tool_changer,
        center_position=handle_position + EE_OFFSET, orientation=handle_orientation,
    )

    # 5) 거치대로 복귀
    stand_position, stand_orientation = tool_changer.stand_return_target()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        stand_position + EE_OFFSET, stand_orientation, "거치대 상공 복귀",
    )
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        stand_position + GRASP_APPROACH_CLEARANCE, stand_orientation, "거치대 하강",
    )

    # 6) 반납: Surface Gripper 해제
    tool_changer.release_mop_to_stand()
    for _ in range(60):
        world.step(render=True)

    print("[INFO] Part 1 (그리퍼 없음) 데모 완료. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
