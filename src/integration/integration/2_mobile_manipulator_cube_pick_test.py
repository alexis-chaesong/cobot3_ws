"""
2_mobile_manipulator_cube_pick_test.py
---------------------------------------
mobile_manipulator_sg.usd (Nova Carter + M0609, RG2 없이 link_6에 Surface Gripper
직결) 씬에서, 씬에 이미 놓여 있는 /World/Cube를 Surface Gripper로 집어 올리는
테스트 스크립트.

흐름: 큐브 상공 접근 -> 하강 -> Surface Gripper 닫기(grip) -> 들어올리기 ->
      그립된 오브젝트/큐브 world position 변화로 파지 성공 여부 확인.

Surface Gripper 자체는 1b_tool_changer_demo_no_gripper.py / surface_gripper_utils.py
와 동일한 저작 방식(D6 attachment joint + IsaacSurfaceGripper 스키마)으로 이미
mobile_manipulator_sg.usd에 authoring 되어 있다 (/World/m0609/link_6/mop_surface_gripper).

실행 방법:
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \\
      src/integration/integration/2_mobile_manipulator_cube_pick_test.py
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.surface_gripper._surface_gripper import acquire_surface_gripper_interface

_THIS_DIR = Path(__file__).resolve().parent  # src/integration/integration
_WS_ROOT = _THIS_DIR.parents[2]  # cobot3_ws
ASSETS_DIR = _WS_ROOT / "src" / "assets"
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ─────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────
USD_PATH = str(ASSETS_DIR / "scenes" / "mobile_manipulator_sg.usd")
# 이 씬은 Nova Carter chassis_link와 m0609 base_link가 root_joint(Fixed Joint)로
# excludeFromArticulation 없이 연결돼 있어, PhysX가 둘을 하나의 articulation으로
# 합쳐버린다 (ArticulationRootAPI는 /World/m0609가 아니라
# /World/Nova_Carter_ROS/chassis_link 에 있다 — headless 조회로 확인).
# 따라서 SingleManipulator는 chassis_link를 prim_path로 잡아야 하고, ARM_USD_ROOT는
# link_6/tool0/드라이브 게인 등 USD 트리 탐색용으로 별도로 둔다.
ARTICULATION_ROOT_PATH = "/World/Nova_Carter_ROS/chassis_link"
ARM_USD_ROOT = "/World/m0609"
EE_LINK_NAME = "link_6"
CUBE_PRIM_PATH = "/World/Cube"
SURFACE_GRIPPER_PATH = f"{ARM_USD_ROOT}/{EE_LINK_NAME}/mop_surface_gripper"
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# tool0이 link_6에 오프셋 없이 겹쳐 있는 "그리퍼 없음" 구성이라, RMPflow도 이에 맞는
# URDF(RG2 없는 순수 팔)를 써야 IK 목표(tool0)와 실제 Surface Gripper 부착점(link_6)이
# 일치한다. 기본 RMPFlowController는 m0609_with_gripper.urdf를 기본값으로 쓰므로 여기서
# 명시적으로 오버라이드한다 (1b_tool_changer_demo_no_gripper.py와 동일한 근거).
NO_GRIPPER_URDF_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0

# tool0 +Z가 아래(월드 -Z)를 향하도록 하는 orientation. 기존 데모들에서 이미
# 도달 가능성/특이점 문제 없이 검증된 값을 그대로 재사용한다.
GRASP_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])  # (w, x, y, z)

APPROACH_OFFSET = np.array([0.0, 0.0, 0.15])  # 큐브 상공 대기 높이
LIFT_OFFSET = np.array([0.0, 0.0, 0.20])  # 파지 후 들어올릴 높이

POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600
SETTLE_STEPS = 60
GRASP_HOLD_STEPS = 60


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


def create_grip_plug(cube_path: str, plug_name: str = "grip_plug") -> str:
    """큐브 쪽 "가상 포트" — 그리퍼 소켓(link_6/grip_socket)과 짝을 이루는 플러그
    프레임을 큐브 윗면 중심(=실제 파지 접촉점)에 만든다.

    지금까지는 grasp 목표 위치를 스크립트에서 매번 bbox/scale로 재계산했는데,
    이러면 물체의 rigid body 원점(큐브는 중심)과 실제 파지 접촉점(윗면) 사이의
    오프셋을 스크립트가 암묵적으로만 알고 있어 — 들어올릴 때 그 오차만큼 처지는
    문제가 있었다. 대신 이 오프셋을 명시적인 프레임(plug)으로 authoring해두면,
    다른 도구로 바꿔도 각 도구가 자신의 grip_plug만 정의하면 동일한 방식으로
    재사용할 수 있다.

    size(기본 1.0) * scale.z * 0.5 를 큐브 world position z에 더해 윗면 중심의
    world position을 구한 뒤, 큐브의 local 좌표계로 변환해서 저작한다.
    """
    stage = omni.usd.get_context().get_stage()
    cube_prim = stage.GetPrimAtPath(cube_path)
    cube_geom = UsdGeom.Cube(cube_prim)
    size = cube_geom.GetSizeAttr().Get() or 1.0
    scale_attr = cube_prim.GetAttribute("xformOp:scale")
    scale_z = scale_attr.Get()[2] if scale_attr and scale_attr.Get() else 1.0
    half_height = 0.5 * size * scale_z

    center = get_prim_world_position(cube_path)
    desired_world_pos = Gf.Vec3d(float(center[0]), float(center[1]), float(center[2] + half_height))

    cube_world_matrix = omni.usd.get_world_transform_matrix(cube_prim)
    local_pos = cube_world_matrix.GetInverse().Transform(desired_world_pos)

    plug_path = f"{cube_path}/{plug_name}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(plug_path)
    plug_xform = UsdGeom.Xform.Define(stage, plug_path)
    plug_xform.AddTranslateOp().Set(local_pos)
    plug_xform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return plug_path


def set_drive_gains(stage, root_path: str):
    """드라이브 stiffness/damping/maxForce를 올리고, targetPosition도 함께 0으로
    맞춘다. 이 씬(mobile_manipulator_sg.usd)의 joint_1..6은 import 과정에서 남은
    것으로 보이는 비정상적인 targetPosition(예: joint_1 = 10.1 rad)이 authoring되어
    있어서, stiffness만 올리고 targetPosition을 그대로 두면 로봇이 world에
    고정되어 있지 않은 mobile base(Nova Carter) 위에 있다는 점과 맞물려 —
    거대한 위치 오차 x 초고강성 토크가 그대로 mobile base를 날려버리는 것을
    headless 테스트로 확인했다. 반드시 targetPosition도 함께 리셋해야 한다."""
    drive_count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive.GetTargetPositionAttr().Set(0.0)
                drive_count += 1
    print(f"[INFO] 관절 드라이브 강성/댐핑/목표위치(0) 설정 완료: {drive_count}개")


def _smoothstep(alpha: float) -> float:
    return alpha * alpha * (3.0 - 2.0 * alpha)


def _ramp_steps_for_distance(distance: float, max_linear_speed: float) -> int:
    raw_steps = distance / max_linear_speed / PHYSICS_DT
    return int(np.clip(round(raw_steps), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def _ramp_ee_target(world, robot, rmpflow, tool0_path, target_position, target_orientation, ramp_steps: int):
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


def is_at_pose(current_position: np.ndarray, target_position: np.ndarray, tolerance: float = 0.01) -> bool:
    return bool(np.linalg.norm(np.asarray(current_position) - np.asarray(target_position)) < tolerance)


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
        if is_at_pose(ee_position, target_position, POSITION_TOLERANCE):
            print(f"[INFO] {label} 도달 (settle step={step}, ee={ee_position})")
            return True
    final_pos = get_prim_world_position(tool0_path)
    print(
        f"[WARN] {label} 이동이 {MAX_APPROACH_STEPS} step 내에 목표 허용오차 이내로 수렴하지 못했습니다. "
        f"final_ee={final_pos} dist={np.linalg.norm(final_pos - target_position):.4f}"
    )
    return False


def hold_pose(world, robot, rmpflow, target_position, target_orientation, steps: int):
    for _ in range(steps):
        world.step(render=True)
        action = rmpflow.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)


def main():
    omni.usd.get_context().open_stage(USD_PATH)
    for _ in range(30):
        simulation_app.update()

    world = World(physics_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    set_drive_gains(stage, ARM_USD_ROOT)

    ee_prim_path = find_prim_path_by_name(ARM_USD_ROOT, EE_LINK_NAME)
    if ee_prim_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' 링크를 {ARM_USD_ROOT} 하위에서 찾지 못했습니다.")
    print(f"[INFO] End Effector rigid body ({EE_LINK_NAME}) = {ee_prim_path}")

    tool0_path = find_prim_path_by_name(ARM_USD_ROOT, "tool0")
    if tool0_path is None:
        raise RuntimeError(f"'tool0' 프레임을 {ARM_USD_ROOT} 하위에서 찾지 못했습니다.")
    print(f"[INFO] IK 목표 프레임(tool0) = {tool0_path}")

    socket_path = f"{ee_prim_path}/grip_socket"
    socket_prim = stage.GetPrimAtPath(socket_path)
    if socket_prim.IsValid():
        socket_pos = get_prim_world_position(socket_path)
        print(f"[INFO] 그리퍼 소켓(grip_socket) = {socket_path}, world pos = {socket_pos}")
    else:
        print(f"[WARN] grip_socket이 없습니다 ({socket_path}) — tool0을 그대로 소켓 위치로 사용합니다.")

    gripper_prim = stage.GetPrimAtPath(SURFACE_GRIPPER_PATH)
    if not gripper_prim.IsValid():
        raise RuntimeError(
            f"Surface Gripper 프림이 없습니다: {SURFACE_GRIPPER_PATH} "
            "(mobile_manipulator_sg.usd에 먼저 authoring 필요)"
        )

    # Nova Carter chassis_link가 articulation root이므로, SingleManipulator도
    # 그 경로로 잡아야 한다 (link_6은 USD 계층상 다른 가지에 있지만, root_joint로
    # 물리적으로 같은 articulation에 묶여 있어 end_effector_prim_path로는 참조 가능).
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ARTICULATION_ROOT_PATH,
            name="m0609_mobile_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=None,  # Surface Gripper는 별도 래퍼로 직접 관리한다.
        )
    )

    world.reset()
    robot.initialize()

    # 드라이브 강성(1e8)로 인한 초기 스냅 방지 — 팔 관절(joint_1..6)만 명시적으로 0으로
    # 지정한다. Carter 바퀴/캐스터 관절은 차동구동 컨트롤러가 별도로 관리하므로 건드리지 않는다.
    dof_names = list(robot.dof_names)
    default_positions = robot.get_joint_positions()
    for name in ARM_JOINT_NAMES:
        if name in dof_names:
            default_positions[dof_names.index(name)] = 0.0
    robot.set_joint_positions(default_positions)
    for _ in range(10):
        world.step(render=True)

    rmpflow = RMPFlowController(
        name="cube_pick_cspace_controller",
        robot_articulation=robot,
        urdf_path=NO_GRIPPER_URDF_PATH,
    )

    # RMPFlowController 내부의 _read_base_link_world_pose()는
    # "{articulation.prim_path}/base_link" (여기서는
    # /World/Nova_Carter_ROS/chassis_link/base_link, 존재하지 않음)를 찾다가 실패해서
    # articulation root(chassis_link, 거의 원점)를 팔의 베이스 pose로 잘못 fallback한다.
    # 실제 팔의 base_link는 USD 트리상 별도 경로(ARM_USD_ROOT/base_link)에 있으므로
    # 여기서 올바른 world pose로 다시 설정해준다.
    base_link_prim = stage.GetPrimAtPath(f"{ARM_USD_ROOT}/base_link")
    base_matrix = omni.usd.get_world_transform_matrix(base_link_prim)
    base_translation = base_matrix.ExtractTranslation()
    base_quat = base_matrix.ExtractRotationQuat()
    base_imag = base_quat.GetImaginary()
    base_position = np.array([base_translation[0], base_translation[1], base_translation[2]])
    base_orientation = np.array([base_quat.GetReal(), base_imag[0], base_imag[1], base_imag[2]])
    rmpflow.rmp_flow.set_robot_base_pose(robot_position=base_position, robot_orientation=base_orientation)
    print(f"[INFO] RMPflow 베이스 pose 보정: position={base_position}, orientation={base_orientation}")

    surface_gripper = SurfaceGripper(
        end_effector_prim_path=ee_prim_path,
        surface_gripper_path=SURFACE_GRIPPER_PATH,
    )
    surface_gripper.initialize()
    print(f"[INFO] Surface Gripper 초기화 완료: {SURFACE_GRIPPER_PATH}")

    # 큐브가 중력으로 바닥에 안착할 시간을 준다.
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    plug_path = create_grip_plug(CUBE_PRIM_PATH)
    grasp_position = get_prim_world_position(plug_path)
    print(f"[INFO] 큐브 grip_plug 생성 완료: {plug_path}")
    print(f"[INFO] 큐브 파지 목표(grip_plug world pos) = {grasp_position}")

    # 1) 큐브 상공 접근
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        grasp_position + APPROACH_OFFSET, GRASP_ORIENTATION, "큐브 상공 접근",
    )

    # 2) 큐브 표면까지 하강
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        grasp_position, GRASP_ORIENTATION, "큐브 접촉 하강",
    )

    # move_to_pose는 POSITION_TOLERANCE(3cm)만 넘으면 바로 반환하는데,
    # Surface Gripper의 max_grip_distance(2cm)보다 느슨해서 grip이 애매한 위치에서
    # 걸리는 문제가 있었다. close() 전에 같은 목표로 좀 더 오래 붙잡아 둬서
    # RMPflow가 목표에 더 정밀하게 수렴하도록 settle 시간을 추가한다.
    hold_pose(world, robot, rmpflow, grasp_position, GRASP_ORIENTATION, GRASP_HOLD_STEPS)
    settled_ee = get_prim_world_position(tool0_path)
    print(f"[INFO] 파지 전 정밀 settle 완료. ee={settled_ee} (목표={grasp_position}, 오차={np.linalg.norm(settled_ee - grasp_position):.4f}m)")

    # 3) 파지: Surface Gripper 닫기
    print("[INFO] Surface Gripper 닫기 시도...")
    surface_gripper.close()
    hold_pose(world, robot, rmpflow, grasp_position, GRASP_ORIENTATION, GRASP_HOLD_STEPS)

    if surface_gripper.is_closed():
        gripper_interface = acquire_surface_gripper_interface()
        gripped = gripper_interface.get_gripped_objects(SURFACE_GRIPPER_PATH)
        print(f"[CHECKPOINT] Surface Gripper 상태: CLOSED. 그립된 오브젝트: {gripped}")
    else:
        print("[CHECKPOINT] Surface Gripper 닫힘 실패 (그립 범위 내 물체 없음/거리 초과).")

    # 4) 들어올리기
    lift_target = grasp_position + LIFT_OFFSET
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        lift_target, GRASP_ORIENTATION, "큐브 들어올리기",
    )
    hold_pose(world, robot, rmpflow, lift_target, GRASP_ORIENTATION, GRASP_HOLD_STEPS)

    # 큐브의 rigid body 원점(중심)은 grip_plug(윗면, 실제 파지점)보다 항상
    # half-height만큼 낮게 나오는 게 정상이라 — 그리퍼 위치와 직접 비교하면 안 되고,
    # 큐브에 딱 붙어 같이 움직이는 grip_plug의 world position을 그리퍼(tool0)와
    # 비교해야 "잘 붙어서 들어올려졌는지"를 올바르게 판정할 수 있다.
    ee_pos_after_lift = get_prim_world_position(tool0_path)
    plug_pos_after_lift = get_prim_world_position(plug_path)
    cube_pos_after_lift = get_prim_world_position(CUBE_PRIM_PATH)
    plug_gripper_gap = float(np.linalg.norm(plug_pos_after_lift - ee_pos_after_lift))
    print(
        f"[RESULT] 들어올리기 후 - 그리퍼(tool0)={ee_pos_after_lift}, "
        f"grip_plug(파지점)={plug_pos_after_lift}, 큐브 중심={cube_pos_after_lift}, "
        f"그리퍼-plug 간격={plug_gripper_gap:.4f}m"
    )
    if plug_gripper_gap < 0.03:
        print(f"[RESULT] 파지 성공: 큐브가 그리퍼에 단단히 붙어 들어올려졌습니다 (간격={plug_gripper_gap:.4f}m).")
    else:
        print(f"[RESULT] 파지 실패로 보입니다: 그리퍼-plug 간격이 큽니다 (간격={plug_gripper_gap:.4f}m).")

    print("[INFO] 큐브 파지 테스트 완료. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
