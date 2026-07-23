"""
1_tool_changer_demo.py
-----------------------
Part 1 검증용 스탠드얼론 스크립트: Surface Gripper 기반 걸레(Mop) 툴 체인저.

거치대(placeholder) -> 접합부(handle) 접근 -> RG2 시각적 닫힘 + Surface Gripper 물리
고정 -> (제자리 유지, Part 3에서 와이핑 궤적으로 대체 예정) -> 거치대 복귀 -> RG2 시각적
열림 + Surface Gripper 해제, 순서로 한 번 실행하고 종료한다.

TODO(placeholder, 실제 자산으로 교체 필요):
  - MOP: 실제 걸레 USD가 없어 FixedCuboid(거치대 선반) + DynamicCuboid(걸레) +
    자식 Xform "handle" 로 대체함.
  - STAND_POSITION: Nova Carter가 결합된 mobile_manipulator_v2.usd 대신, M0609(+RG2)
    standalone 자산(m0609_with_gripper.usd, base_link가 world 원점)으로 바꿨다.
    headless로 확인한 안정 pose 근방의 도달 가능한 위치를 잡은 placeholder 값이며,
    실제 거치대 위치로 교체 필요.
  - RG2_FINGERTIP_LINK_NAME: RG2는 평행 2지 그리퍼라 물리적으로 단일 "fingertip"
    링크가 없음. 아래 tool_changer.py 의 docstring 설명대로 손가락 모션에 영향받지
    않는 고정 링크(gripper_body)를 기본값으로 사용. 실제 USD에서 이름이 다르면 수정.

실행 방법:
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh 1_tool_changer_demo.py
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.robot.manipulators.grippers import ParallelGripper
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
# mobile_manipulator_v2.usd(Nova Carter + M0609 결합본) 대신, Nova Carter 없이
# M0609(+RG2 그리퍼)만 있는 standalone 자산을 사용한다. 이 asset은 base_link가
# world 원점(0,0,0)에 있어 카터 관련 트랜스폼/충돌 이슈가 애초에 없다. 또한 이
# 자산의 tool0은 (mobile_manipulator_v2.usd와 달리) RigidBodyAPI가 붙어있지 않은
# 정상적인 순수 프레임이다 (headless로 직접 확인).
# 주의: 이 USD의 루트 프림은 "/World"가 아니라 "/m0609"이다 (파일 자체에 /World가 없음).
USD_PATH = str(_WS_ROOT / "src" / "assets" / "robots" / "m0609_with_gripper" / "m0609_with_gripper.usd")  # src/assets/robots로 이동됨
ROBOT_PRIM_PATH = "/m0609"
# 이 USD의 링크들은 URDF 임포트 관례상 ROBOT_PRIM_PATH 바로 아래 flat하게 존재한다.
# 하드코딩 대신 find_prim_path_by_name()으로 실제 경로를 탐색해 다른 자산으로 바꿔도
# 안전하게 동작하도록 한다.
EE_LINK_NAME = "link_6"
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [0.5, 0.5]
GRIPPER_DELTA = [-0.5, -0.5]

# RG2 rigid body 중 Surface Gripper를 부착할 링크. gripper_body는 tool0에 고정된
# 비가동 링크라 손가락 open/close 모션과 무관하게 고정점을 유지할 수 있다.
RG2_FINGERTIP_LINK_NAME = "gripper_body"

# TODO: 실제 거치대(tool stand) 위치로 교체.
# headless 테스트로 직접 확인한 값 (이 standalone 자산 기준):
#   - M0609 base_link 세계좌표 = (0.0, 0.0, 0.0)
#   - 안정된(휴지) tool0 = (0.0001, 0.0064, 1.035)
# 처음에는 임의로 (0.25, 0.2, 0.95)를 썼는데, RMPflow가 그 근방으로는 절대
# 수렴하지 못하고 dist≈0.27m에서 계속 진동/정체하는 것을 headless로 확인했다.
# 이 프로젝트의 기존 검증 스크립트(2_rmpflow_standalone.py)가 실제로 쓰는
# (0.3, 0.0, 0.9)는 "빈 공간"에서는 dist≈0.0007m까지 깔끔하게 수렴하지만, 그
# 좌표에 거치대 선반+걸레(콜리전 있는 실제 rigid body)가 놓여 있으면 팔이 그
# 물체에 부딪히며 다시 발산/정체하는 것도 확인했다. 4_pick_place.py가
# end_effector_offset으로 "물체 바로 위에서 접근 후 하강"하는 것과 같은 이유로,
# 아래 EE_OFFSET을 이용한 2단계 접근(위에서 접근 → 하강)이 반드시 필요하다.
STAND_POSITION = np.array([0.6, 0.0, 0.3])
# 주의: identity(1,0,0,0)는 이 로봇의 tool0 기준 손목 특이점 근처라, RMPflow가
# 팔을 베이스 근처로 접어버리는 현상을 headless 테스트로 직접 확인했다(목표에 못
# 감). (0,1,0,0)은 이 프로젝트의 기존 검증 스크립트(2_rmpflow_standalone.py,
# test_rmpflow.py)에서도 쓰는 값이라 이를 그대로 채택한다.
STAND_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])  # (w, x, y, z)

# 4_pick_place.py의 EE_OFFSET과 같은 역할: 거치대/걸레 위 여유 공간으로 먼저
# 접근한 뒤 마지막에 이 offset만큼 하강해 handle에 도달한다. 이렇게 해야 팔이
# 선반/걸레의 콜리전 형상에 곧장 부딪히지 않는다.
EE_OFFSET = np.array([0.0, 0.0, 0.15])

# RMPflow는 tool0(IK 프레임)을 목표로 이동시키지만, 실제로 Surface Gripper가
# 붙는 곳은 gripper_body이고 이 둘은 fixed joint 체인(quick_changer/angle_bracket/
# gripper_body_joint)으로만 연결돼 있어 로봇 자세와 무관하게 항상 같은 오프셋만큼
# 어긋나 있다. headless로 실측한 값(STAND_POSITION=(0.6,0,0.3) 도달 시
# gripper_body_world_pos - tool0_world_pos):
#   gripper_body가 tool0보다 (-0.0266, -0.0091, -0.0464)m 만큼 벗어나 있었다.
# 이 오차(약 4.7cm)가 grip_travel(0.03m)/clearanceOffset(0.025m)보다 커서, Surface
# Gripper가 grip 시도 시 그 차이를 강하게 보정하려다 걸레를 튕겨내는 원인이었다.
# ToolChangerController에 이 오프셋을 넘기면 IK 목표를 미리 보정해 gripper_body가
# handle/거치대 좌표에 정확히 오도록 한다.
FINGERTIP_OFFSET_FROM_TOOL0 = np.array([-0.0266, -0.0091, -0.0464])

# (0.3, 0.0, 0.9)에서는 (장애물이 없다면) step~100-150 사이에 dist가 0.001m
# 미만까지 수렴하지만, 그대로 계속 step을 돌리면(수렴한 뒤에도 tolerance 체크
# 없이 계속 구동할 경우) step~200 이후 dist가 다시 0.18m 근처로 벌어지는 현상을
# headless로 확인했다 — 그래서 반드시 tolerance를 만족하는 즉시 루프를 빠져나가야
# 한다(move_to_pose가 이미 그렇게 동작함).
POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400


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
    """PhysX rigid body view 없이, 순수 USD 트랜스폼으로 world position을 읽는다.
    tool0처럼 (nested rigid body 문제로) SingleRigidPrim으로 wrap할 수 없는 프레임의
    포즈를 읽을 때 사용한다.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    matrix = omni.usd.get_world_transform_matrix(prim)
    translation = matrix.ExtractTranslation()
    return np.array([translation[0], translation[1], translation[2]])


def create_placeholder_mop(stage, world):
    """TODO: 실제 걸레(Mop/Cloth) USD와 실제 거치대로 교체.

    지금은 (1) 거치대 역할을 하는 고정 선반(FixedCuboid)과 (2) 그 위에 놓인 걸레
    placeholder(DynamicCuboid + 자식 Xform "handle")로 대체했다. 선반이 없으면
    걸레가 중력으로 그냥 떨어지므로, STAND_POSITION이 바닥이 아닌 임의 높이일 때도
    반드시 지지면이 있어야 한다.
    """
    stand_shelf_path = "/World/MopStand/shelf"
    shelf_thickness = 0.02
    world.scene.add(
        FixedCuboid(
            prim_path=stand_shelf_path,
            name="mop_stand_shelf",
            position=STAND_POSITION - np.array([0.0, 0.0, shelf_thickness / 2.0]),
            scale=np.array([0.2, 0.3, shelf_thickness]),
            color=np.array([0.4, 0.4, 0.4]),
        )
    )

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


def move_to_pose(world, robot, rmpflow, tool0_path, target_position, target_orientation, label: str):
    print(f"[INFO] {label} 목표 {target_position} 로 이동 시작...")
    for step in range(MAX_APPROACH_STEPS):
        world.step(render=True)
        # RMPflow의 목표/수렴 판정은 실제 IK 대상 프레임인 tool0 기준이어야 하므로,
        # (link_6이 아니라) tool0의 world position을 순수 USD 트랜스폼으로 읽는다.
        ee_position = get_prim_world_position(tool0_path)
        action = rmpflow.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)
        if ToolChangerController.is_at_pose(ee_position, target_position, POSITION_TOLERANCE):
            print(f"[INFO] {label} 도달 (step={step}, ee={ee_position})")
            return True
    print(f"[WARN] {label} 이동이 {MAX_APPROACH_STEPS} step 내에 목표 허용오차 이내로 수렴하지 못했습니다.")
    return False


def main():
    stage = omni.usd.get_context().get_stage()
    stage.GetRootLayer().subLayerPaths.append(USD_PATH)
    for _ in range(30):
        simulation_app.update()

    world = World(physics_dt=1.0 / 60.0)
    world.scene.add_default_ground_plane()

    set_drive_gains(stage, ROBOT_PRIM_PATH)

    # USD가 이미 스테이지에 병합된 상태이므로(subLayerPaths.append + update),
    # world.reset()/robot.initialize() 없이도 prim 탐색은 바로 가능하다.
    ee_prim_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
    if ee_prim_path is None:
        raise RuntimeError(
            f"'{EE_LINK_NAME}' 링크를 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다."
        )
    print(f"[INFO] End Effector rigid body ({EE_LINK_NAME}) = {ee_prim_path}")

    # RMPflow의 실제 IK 목표/수렴 판정 기준 프레임(tool0)은 rigid body가 아니라
    # 순수 USD 프레임이므로 별도로 경로만 찾아 둔다 (get_prim_world_position으로 조회).
    tool0_path = find_prim_path_by_name(ROBOT_PRIM_PATH, "tool0")
    if tool0_path is None:
        raise RuntimeError(f"'tool0' 프레임을 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] IK 목표 프레임(tool0) = {tool0_path}")

    fingertip_path = find_prim_path_by_name(ROBOT_PRIM_PATH, RG2_FINGERTIP_LINK_NAME)
    if fingertip_path is None:
        raise RuntimeError(
            f"'{RG2_FINGERTIP_LINK_NAME}' 링크를 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다. "
            "RG2_FINGERTIP_LINK_NAME 값을 실제 USD의 링크 이름으로 수정하세요."
        )
    print(f"[INFO] RG2 fingertip(Surface Gripper 부착점) = {fingertip_path}")

    # Surface Gripper의 D6 attachment joint는 반드시 world.reset()/robot.initialize()
    # "이전"에 authoring해야 한다. 이미 PhysX가 articulation view를 구성한 뒤(즉
    # robot.initialize() 이후)에 gripper_body 하위에 새 조인트를 추가하면 articulation
    # 구조가 조용히 깨져서, RMPflow가 계산한 관절 명령이 전혀 엉뚱하게 적용되고 팔이
    # 목표와 무관하게 접혀버리는 문제를 headless 테스트로 직접 확인했다. (원래는
    # ToolChangerController 생성자가 이 authoring을 담당하는데, 그 생성자를
    # robot.initialize() 이후에 호출했던 것이 바로 그 문제였다.)
    surface_gripper_prim_path = surface_gripper_utils.setup_mop_surface_gripper(
        stage, fingertip_prim_path=fingertip_path
    )

    gripper = ParallelGripper(
        end_effector_prim_path=ee_prim_path,
        joint_prim_names=GRIPPER_JOINTS,
        joint_opened_positions=np.array(GRIPPER_OPEN),
        joint_closed_positions=np.array(GRIPPER_CLOSE),
        action_deltas=np.array(GRIPPER_DELTA),
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=gripper,
        )
    )

    # 걸레/거치대는 world.reset() 이전에 씬에 추가해야 한다. reset() 이후 동적으로
    # rigid body를 새로 추가하면 PhysX 브로드페이즈가 해당 액터의 초기 상태를 제대로
    # 못 잡아 "Illegal BroadPhaseUpdateData" 에러와 함께 씬 전체 트랜스폼이 깨지는
    # 문제를 headless 테스트에서 실제로 확인했다. 표준 패턴(모든 오브젝트를 먼저
    # 추가하고 reset은 한 번만) 그대로 따른다.
    handle_path = create_placeholder_mop(stage, world)

    world.reset()
    robot.initialize()

    # 관절 드라이브 강성(1e8)이 매우 높다. world.reset() 직후 관절 목표(target)가
    # USD에 저작된 기본값(대개 0)으로 잡혀 있는데, 실제 임포트 포즈는 그와 다르면
    # 첫 physics step에 그 오차를 1e8 강성으로 순간 보정하면서 팔 전체가 크게 튄다
    # (headless 테스트로 직접 확인). 4_pick_place.py의 initialize_robot() 패턴처럼
    # 관절을 명시적으로 0으로 지정해 드라이브 목표와 실제 상태를 일치시켜 스냅을
    # 없앤다 (이 asset은 0 근방이 실제로 안정적인 휴지 포즈임을 확인했다).
    robot.set_joint_positions(np.zeros(robot.num_dof))
    for _ in range(10):
        world.step(render=True)

    # end_effector_frame_name 기본값(tool0)을 그대로 둔다. 4_pick_place.py는 link_6을
    # 쓰지만 그건 그 스크립트가 link_6 pose로 수렴 판정도 같이 하기 때문이고, 우리는
    # tool0 pose로 수렴 판정을 하므로(get_prim_world_position(tool0_path)) RMPflow가
    # 실제로 구동하는 프레임과 우리가 읽는 프레임을 반드시 일치시켜야 한다. link_6으로
    # 바꿔봤다가 구동 대상과 판정 대상이 어긋나 수렴 판정 자체가 깨지는 것을 확인했다.
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
        rg2_gripper=gripper,
        surface_gripper_prim_path=surface_gripper_prim_path,
        auto_create_surface_gripper=False,
    )
    tool_changer.initialize()

    # 안정화 대기 (걸레가 거치대 위에 정착)
    for _ in range(30):
        world.step(render=True)

    # 1) 접합부(handle) 위 여유 공간으로 먼저 접근한 뒤, 마지막에 하강한다
    #    (선반/걸레 콜리전과의 충돌 방지 — 4_pick_place.py의 end_effector_offset과 동일한 목적).
    handle_position, handle_orientation = tool_changer.approach_tool_stand()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + EE_OFFSET, handle_orientation, "걸레 접합부 상공 접근",
    )
    move_to_pose(world, robot, rmpflow, tool0_path, handle_position, handle_orientation, "걸레 접합부 하강")

    # 2) 파지: RG2 시각적 닫힘 + Surface Gripper 물리 고정
    tool_changer.grasp_mop()
    for _ in range(30):
        world.step(render=True)

    # 3) (Part 3에서 표면 밀착 와이핑 궤적으로 대체 예정) — 잠시 제자리 유지
    for _ in range(60):
        world.step(render=True)

    # 4) 거치대 위 여유 공간으로 먼저 상승한 뒤, 마지막에 거치대 위치로 하강해 복귀한다.
    stand_position, stand_orientation = tool_changer.stand_return_target()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        stand_position + EE_OFFSET, stand_orientation, "거치대 상공 복귀",
    )
    move_to_pose(world, robot, rmpflow, tool0_path, stand_position, stand_orientation, "거치대 하강")

    # 5) 반납: RG2 시각적 열림 + Surface Gripper 해제
    tool_changer.release_mop_to_stand()
    for _ in range(60):
        world.step(render=True)

    print("[INFO] Part 1 (Surface Gripper 툴 체인저) 데모 완료. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
