"""
3_mobile_manipulator_trash_can_pick_test.py
--------------------------------------------
move_tash_can.usd (mobile_manipulator_sg.usd와 동일 구성 + Surface Gripper 저작
완료 + /World/small_trash_can_body가 놓인 씬) 에서, 쓰레기통을 Surface Gripper로
집어 올리는 테스트 스크립트.

RMPflow의 Cartesian IK + collision_rmp 장애물 회피로 접근시켜봤으나, obstacle을
등록하면 팔이 목표에 아예 도달하지 못하는 문제가 있어(collision_rmp 가중치 튜닝은
추후 과제로 미룸), 대신 GUI에서 직접 찾은 "쓰레기통을 잡을 수 있고 Carter도 안
넘어가는" 검증된 관절 각도(TARGET_JOINTS_DEG)로 **조인트 공간에서 직접 이동**한다.
RMPflow/IK를 아예 거치지 않으므로 collision_rmp 튜닝 문제와 무관하게 안전하다.

흐름: 홈 포즈 -> TARGET_JOINTS_DEG로 조인트 공간 램프 이동 -> 도달한 실제
      tool0 pose를 기준으로 쓰레기통 표면에서 가장 가까운 점을 파지점(grip_plug)
      으로 authoring -> Surface Gripper 닫기(grip) -> RMPflow로 수직 들어올리기 ->
      grip_plug(파지점) world position 변화로 파지 성공 여부 확인.

실행 방법:
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \\
      src/integration/integration/3_mobile_manipulator_trash_can_pick_test.py
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
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
USD_PATH = str(ASSETS_DIR / "scenes" / "move_tash_can.usd")
ARTICULATION_ROOT_PATH = "/World/Nova_Carter_ROS/chassis_link"
ARM_USD_ROOT = "/World/m0609"
EE_LINK_NAME = "link_6"
TRASH_CAN_PRIM_PATH = "/World/small_trash_can_body"
SURFACE_GRIPPER_PATH = f"{ARM_USD_ROOT}/{EE_LINK_NAME}/mop_surface_gripper"
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# Carter 차체(chassis_link) 장애물을 RMPflow collision_rmp에 등록해서 팔이 경로
# 계획 단계에서부터 회피하게 만들려 했으나, 등록하면 목표 유인력보다 회피력이
# 강해져 팔이 아예 도달을 못 하는 문제가 있었다. collision_rmp 가중치 튜닝은
# 추후 과제로 미루고, 지금은 GUI에서 직접 검증한 안전한 관절 각도로 조인트
# 공간에서 직접 이동한다 (아래 TARGET_JOINTS_DEG).
CARTER_CHASSIS_PATH = "/World/Nova_Carter_ROS/chassis_link"

# GUI에서 직접 찾은, 쓰레기통 옆면(x축 방향 면)을 잡을 수 있고 Carter도 넘어가지
# 않는 것으로 검증된 자세 (degree). tip-stability 검증 완료 (최대 기울기 0.09deg).
TARGET_JOINTS_DEG = [-90.0, 101.0, 50.0, -94.0, 91.8, -1.1]

# 파지 + 들어올리기 이후, 주행 전에 j1만 추가로 회전시켜 쓰레기통을 몸 쪽으로 당겨
# 접는다 (들고 있는 상태로 밖으로 너무 빠져나와 있어 주행 시 간섭 우려).
TUCK_J1_DEG = -170.0

NO_GRIPPER_URDF_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0

LIFT_OFFSET = np.array([0.0, 0.0, 0.60])  # 파지 후 수직으로 들어올릴 높이.
# Nova Carter의 가장 높은 라이다(XT-32, world z=0.526m)를 쓰레기통 몸체가 가리지
# 않도록, 쓰레기통 바닥까지 충분히 위로 올라오게 여유 있게 잡은 값이다.

POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600
SETTLE_STEPS = 60
GRASP_HOLD_STEPS = 60
JOINT_RAMP_STEPS = 300

# 쓰레기통이 사각기둥이라 표면 위치를 기하학적으로 정확히 예측하기 어려우므로,
# grip_socket 로컬 +Z 방향(Surface Gripper의 forward_axis)으로 조금씩 전진하며
# 매 스텝 Surface Gripper 닫기를 시도해서 실제로 닿는(그립 범위에 들어오는)
# 시점에 파지되도록 한다.
CREEP_STEP_SIZE = 0.005  # 전진 스텝당 이동 거리 (m)
CREEP_MAX_STEPS = 40  # 최대 전진 스텝 수 (0.005 * 40 = 0.2m)
CREEP_SETTLE_STEPS = 5  # 스텝마다 이동을 정착시키는 physics step 수

# Nav2 없이 순수 Python으로 바퀴 관절 속도를 직접 명령해 앞으로 주행하는 간단한
# 테스트용 파라미터. 바퀴 반지름을 정확히 몰라서 각속도로 지정하고 실제 이동
# 거리는 실행 결과로 확인한다.
DRIVE_WHEEL_VELOCITY = 5.0  # rad/s (양쪽 바퀴 동일 부호 = 직진)
DRIVE_STEPS = 180  # 3초 @ 60Hz
DRIVE_STOP_SETTLE_STEPS = 60


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


def get_world_orientation_wxyz(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    matrix = omni.usd.get_world_transform_matrix(prim)
    quat = matrix.ExtractRotationQuat()
    imag = quat.GetImaginary()
    return np.array([quat.GetReal(), imag[0], imag[1], imag[2]])


def drive_straight(world, robot, dof_names, wheel_velocity: float, steps: int, stop_settle_steps: int):
    """Nav2 없이 바퀴 관절 각속도를 직접 명령해 직진한다. 좌우 바퀴에 같은 부호의
    각속도를 줘서 직진시키고, 이후 속도를 0으로 되돌려 정지시킨다."""
    wheel_left_idx = dof_names.index("joint_wheel_left")
    wheel_right_idx = dof_names.index("joint_wheel_right")

    drive_velocities = np.zeros(len(dof_names))
    drive_velocities[wheel_left_idx] = wheel_velocity
    drive_velocities[wheel_right_idx] = wheel_velocity
    drive_action = ArticulationAction(joint_velocities=drive_velocities)
    for _ in range(steps):
        world.step(render=True)
        robot.apply_action(drive_action)

    stop_action = ArticulationAction(joint_velocities=np.zeros(len(dof_names)))
    for _ in range(stop_settle_steps):
        world.step(render=True)
        robot.apply_action(stop_action)


def get_chassis_tilt_deg(chassis_path: str) -> float:
    """chassis_link의 world 'up' 벡터(로컬 Z)가 world Z(0,0,1)에서 얼마나
    기울었는지(도 단위)를 반환한다. 0도면 완전히 수평/직립 (넘어짐 여부 확인용)."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(chassis_path)
    matrix = omni.usd.get_world_transform_matrix(prim)
    local_z_world = np.array([matrix.TransformDir((0, 0, 1))[i] for i in range(3)])
    local_z_world /= np.linalg.norm(local_z_world)
    cos_angle = np.clip(np.dot(local_z_world, np.array([0.0, 0.0, 1.0])), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def rotate_vector_by_quat(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """quaternion(w,x,y,z)으로 벡터 v를 회전시킨다."""
    w = q_wxyz[0]
    qv = np.asarray(q_wxyz[1:4], dtype=float)
    v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def create_plug_at_world_pos(trash_can_path: str, world_pos: np.ndarray, plug_name: str = "grip_plug") -> str:
    """쓰레기통이 사각기둥이라 표면을 기하학적으로 근사하기 애매하므로, 대신 실제로
    grip_socket 방향(-X)으로 전진하다 붙잡힌 시점의 tool0 world position을 그대로
    grip_plug 위치로 authoring한다 (쓰레기통 local 좌표계로 변환해서 저장하므로,
    이후 쓰레기통이 움직여도 같이 따라간다)."""
    stage = omni.usd.get_context().get_stage()
    trash_can_prim = stage.GetPrimAtPath(trash_can_path)

    desired_world_pos = Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
    trash_can_world_matrix = omni.usd.get_world_transform_matrix(trash_can_prim)
    local_pos = trash_can_world_matrix.GetInverse().Transform(desired_world_pos)

    plug_path = f"{trash_can_path}/{plug_name}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(plug_path)
    plug_xform = UsdGeom.Xform.Define(stage, plug_path)
    plug_xform.AddTranslateOp().Set(local_pos)
    plug_xform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return plug_path


def set_drive_gains(stage, root_path: str):
    """드라이브 stiffness/damping/maxForce를 올리고, targetPosition도 함께 0으로
    맞춘다 (mobile_manipulator_sg.usd에서 확인된 것과 동일한 import 잔여값 문제
    대응 — 자유 베이스에서 대형 위치 오차 x 초고강성 토크로 로봇이 날아가는 것을
    막기 위해 필수)."""
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


def ramp_to_joint_positions(world, robot, dof_names, arm_joint_names, target_joints_deg, ramp_steps: int):
    """RMPflow/IK를 거치지 않고, 조인트 공간에서 현재 값 -> target_joints_deg(degree)로
    smoothstep 이징하며 직접 이동한다. GUI에서 검증한 고정 자세로 이동할 때
    collision_rmp 튜닝 문제와 무관하게 안전하게 쓸 수 있다."""
    start_positions = robot.get_joint_positions().copy()
    target_positions = start_positions.copy()
    arm_indices = [dof_names.index(name) for name in arm_joint_names if name in dof_names]
    target_rad = np.radians(target_joints_deg)
    for idx, rad in zip(arm_indices, target_rad):
        target_positions[idx] = rad

    for step in range(ramp_steps):
        world.step(render=True)
        alpha = (step + 1) / ramp_steps
        waypoint = start_positions + _smoothstep(alpha) * (target_positions - start_positions)
        robot.apply_action(ArticulationAction(joint_positions=waypoint))
    return target_positions


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

    gripper_prim = stage.GetPrimAtPath(SURFACE_GRIPPER_PATH)
    if not gripper_prim.IsValid():
        raise RuntimeError(f"Surface Gripper 프림이 없습니다: {SURFACE_GRIPPER_PATH}")

    robot = world.scene.add(
        SingleManipulator(
            prim_path=ARTICULATION_ROOT_PATH,
            name="m0609_mobile_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=None,
        )
    )

    world.reset()
    robot.initialize()

    dof_names = list(robot.dof_names)
    default_positions = robot.get_joint_positions()
    for name in ARM_JOINT_NAMES:
        if name in dof_names:
            default_positions[dof_names.index(name)] = 0.0
    robot.set_joint_positions(default_positions)
    for _ in range(10):
        world.step(render=True)

    rmpflow = RMPFlowController(
        name="trash_can_pick_cspace_controller",
        robot_articulation=robot,
        urdf_path=NO_GRIPPER_URDF_PATH,
    )

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

    # 쓰레기통이 중력으로 바닥에 안착할 시간을 준다.
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    # 1) GUI에서 검증한 고정 관절 자세로 조인트 공간에서 직접 이동 (RMPflow/IK 미사용).
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    grasp_position = get_prim_world_position(tool0_path)
    grasp_orientation = get_world_orientation_wxyz(tool0_path)
    print(f"[INFO] 목표 관절(deg)={TARGET_JOINTS_DEG} 도달. tool0 world pos={grasp_position}, orientation(w,x,y,z)={grasp_orientation}")

    # 2) grip_socket 로컬 +Z 방향(world)으로 조금씩 전진하며, 매 스텝 Surface Gripper
    #    닫기를 시도한다. Surface Gripper의 grip 감지축(forward_axis)이 애초에 Z로
    #    authoring되어 있어(-X가 아니라) Z를 기준으로 전진해야 실제 그립 범위
    #    (max_grip_distance)와 일치한다. 쓰레기통이 사각기둥이라 표면까지의 정확한
    #    거리를 미리 알 수 없으므로, 실제로 그 범위 안에 들어오는 순간을 이렇게 찾는다.
    move_direction_world = rotate_vector_by_quat(grasp_orientation, np.array([0.0, 0.0, 1.0]))
    move_direction_world /= np.linalg.norm(move_direction_world)
    print(f"[INFO] 전진 방향(grip_socket 로컬 +Z, world 기준) = {move_direction_world}")

    current_target = grasp_position.copy()
    gripped_ok = False
    for creep_step in range(CREEP_MAX_STEPS):
        current_target = current_target + move_direction_world * CREEP_STEP_SIZE
        for _ in range(CREEP_SETTLE_STEPS):
            world.step(render=True)
            action = rmpflow.forward(
                target_end_effector_position=current_target,
                target_end_effector_orientation=grasp_orientation,
            )
            robot.apply_action(action)
        surface_gripper.close()
        if surface_gripper.is_closed():
            gripped_ok = True
            traveled = CREEP_STEP_SIZE * (creep_step + 1)
            print(f"[INFO] 전진 {creep_step + 1}스텝(누적 {traveled:.3f}m)에서 파지 성공.")
            break

    grasp_position = current_target
    plug_path = create_plug_at_world_pos(TRASH_CAN_PRIM_PATH, get_prim_world_position(tool0_path))
    plug_position = get_prim_world_position(plug_path)
    print(f"[INFO] 쓰레기통 grip_plug 생성 완료: {plug_path}, world pos={plug_position}")

    if gripped_ok:
        gripper_interface = acquire_surface_gripper_interface()
        gripped = gripper_interface.get_gripped_objects(SURFACE_GRIPPER_PATH)
        print(f"[CHECKPOINT] Surface Gripper 상태: CLOSED. 그립된 오브젝트: {gripped}")
    else:
        max_travel = CREEP_STEP_SIZE * CREEP_MAX_STEPS
        print(f"[CHECKPOINT] 최대 {max_travel:.3f}m 전진해도 파지 실패 (그립 범위 내 물체 없음).")

    # 4) 수직으로 들어올리기 — 이제부터는 RMPflow Cartesian 제어로 전환 (collision_rmp
    #    장애물은 등록하지 않은 상태 그대로이므로 튜닝 문제와 무관).
    lift_target = grasp_position + LIFT_OFFSET
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        lift_target, grasp_orientation, "쓰레기통 들어올리기",
    )
    hold_pose(world, robot, rmpflow, lift_target, grasp_orientation, GRASP_HOLD_STEPS)

    ee_pos_after_lift = get_prim_world_position(tool0_path)
    plug_pos_after_lift = get_prim_world_position(plug_path)
    trash_can_pos_after_lift = get_prim_world_position(TRASH_CAN_PRIM_PATH)
    plug_gripper_gap = float(np.linalg.norm(plug_pos_after_lift - ee_pos_after_lift))
    print(
        f"[RESULT] 들어올리기 후 - 그리퍼(tool0)={ee_pos_after_lift}, "
        f"grip_plug(파지점)={plug_pos_after_lift}, 쓰레기통 원점={trash_can_pos_after_lift}, "
        f"그리퍼-plug 간격={plug_gripper_gap:.4f}m"
    )
    if plug_gripper_gap < 0.03:
        print(f"[RESULT] 파지 성공: 쓰레기통이 그리퍼에 단단히 붙어 들어올려졌습니다 (간격={plug_gripper_gap:.4f}m).")
    else:
        print(f"[RESULT] 파지 실패로 보입니다: 그리퍼-plug 간격이 큽니다 (간격={plug_gripper_gap:.4f}m).")

    # 5) 주행 전, j1만 추가로 회전시켜 쓰레기통을 몸 쪽으로 당겨 접는다
    #    (나머지 관절은 방금 들어올리기 후 도달한 값을 그대로 유지).
    current_positions_rad = robot.get_joint_positions()
    tuck_targets_deg = []
    for name in ARM_JOINT_NAMES:
        idx = dof_names.index(name)
        if name == "joint_1":
            tuck_targets_deg.append(TUCK_J1_DEG)
        else:
            tuck_targets_deg.append(float(np.degrees(current_positions_rad[idx])))
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, tuck_targets_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)

    tuck_ee_pos = get_prim_world_position(tool0_path)
    tuck_plug_pos = get_prim_world_position(plug_path)
    tuck_gap = float(np.linalg.norm(tuck_plug_pos - tuck_ee_pos))
    print(f"[INFO] j1 tuck(-> {TUCK_J1_DEG}deg) 완료. tool0={tuck_ee_pos}, grip_plug={tuck_plug_pos}, 간격={tuck_gap:.4f}m")
    if tuck_gap < 0.03:
        print(f"[RESULT] tuck 후에도 파지 유지됨 (간격={tuck_gap:.4f}m).")
    else:
        print(f"[RESULT] tuck 중 파지가 풀린 것으로 보입니다 (간격={tuck_gap:.4f}m).")

    # 6) 간단한 전진 주행 (Nav2 없이 바퀴 관절 속도를 직접 명령).
    chassis_start_pos = get_prim_world_position(ARTICULATION_ROOT_PATH)
    drive_straight(world, robot, dof_names, DRIVE_WHEEL_VELOCITY, DRIVE_STEPS, DRIVE_STOP_SETTLE_STEPS)
    chassis_end_pos = get_prim_world_position(ARTICULATION_ROOT_PATH)
    traveled = float(np.linalg.norm(chassis_end_pos - chassis_start_pos))
    tilt_after_drive = get_chassis_tilt_deg(ARTICULATION_ROOT_PATH)
    drive_plug_gap = float(np.linalg.norm(get_prim_world_position(plug_path) - get_prim_world_position(tool0_path)))
    print(
        f"[RESULT] 주행 완료 - chassis 이동거리={traveled:.3f}m, 기울기={tilt_after_drive:.2f}deg, "
        f"그리퍼-plug 간격={drive_plug_gap:.4f}m"
    )
    if drive_plug_gap < 0.03:
        print("[RESULT] 주행 중에도 파지 유지됨.")
    else:
        print("[RESULT] 주행 중 파지가 풀린 것으로 보입니다.")

    # 7) 관절을 다시 쓰레기통 파지 자세(TARGET_JOINTS_DEG)로 되돌린다.
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    print(f"[INFO] 관절을 파지 자세(TARGET_JOINTS_DEG={TARGET_JOINTS_DEG})로 복귀 완료.")

    # 8) Surface Gripper 해제.
    surface_gripper.open()
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    trash_can_pos_after_release = get_prim_world_position(TRASH_CAN_PRIM_PATH)
    print(
        f"[RESULT] 파지 해제 완료 (is_closed={surface_gripper.is_closed()}). "
        f"쓰레기통 world position={trash_can_pos_after_release}"
    )

    # 9) 쓰레기통을 내려놓은 뒤 조금 더 전진.
    chassis_start_pos_2 = get_prim_world_position(ARTICULATION_ROOT_PATH)
    drive_straight(world, robot, dof_names, DRIVE_WHEEL_VELOCITY, DRIVE_STEPS, DRIVE_STOP_SETTLE_STEPS)
    chassis_end_pos_2 = get_prim_world_position(ARTICULATION_ROOT_PATH)
    traveled_2 = float(np.linalg.norm(chassis_end_pos_2 - chassis_start_pos_2))
    tilt_after_drive_2 = get_chassis_tilt_deg(ARTICULATION_ROOT_PATH)
    trash_can_pos_final = get_prim_world_position(TRASH_CAN_PRIM_PATH)
    print(
        f"[RESULT] 추가 전진 완료 - chassis 이동거리={traveled_2:.3f}m, 기울기={tilt_after_drive_2:.2f}deg, "
        f"쓰레기통(내려놓은 채) world position={trash_can_pos_final}"
    )

    print("[INFO] 쓰레기통 파지 테스트 완료. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
