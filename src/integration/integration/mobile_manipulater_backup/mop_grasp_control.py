"""
mop_grasp_control.py
---------------------
격리 병동 비대면 로봇팔 프로젝트 - 대걸레(rigid body mop) 파지 스크립트.

큐브 걸레(Particle Cloth + Surface Gripper 대상)와 달리 대걸레(SM_MopSet_01a_02)는
강체(Rigid Body)이므로 Surface Gripper가 아닌 "표준 물리 그립"
(RG2 조인트를 목표 폭까지 닫고 마찰로 버티는 방식)으로 처리한다.

대상:
  - 로봇: Doosan M0609 + OnRobot RG2 (robocart1/m0609_with_gripper)
  - 걸레: /World/SM_MopSet_01a_02 (자식 Mesh: SM_MopSet_01a)

Stage 스냅샷 (2026-07-20, Property 패널 기준 - 참고용, 실제 좌표는 반드시
get_prim_world_transform() 으로 매번 새로 조회할 것. 아래 값은 씬이 바뀌면 stale함):
  Prim Path : /World/SM_MopSet_01a_02/SM_MopSet_01a
  Translate : (-3.0, 0.5, 0.0)      [m, 부모 SM_MopSet_01a_02 로컬 기준]
  Orient    : (0.0, 0.0, -105.695)  [deg, XYZ 오일러]
  Scale     : (0.01, 0.01, 0.01)

Isaac Sim 5.1.0 기준으로 omni.isaac.core 대신 isaacsim.core.* 네임스페이스 사용
(프로젝트 내 test_rmpflow.py 와 동일 컨벤션).

사용법 (Script Editor):
  1. Demo_scene_save.usd 를 열고 Play(▶)
  2. 이 파일 전체를 Script Editor 에 붙여넣고 필요한 함수만 골라 호출
     (파일 맨 아래 "실행부" 참고, 기본은 각 단계 주석 처리되어 있음)

===========================================================================
그립 실패(미끄러짐/낙하) 체크리스트 - 문제가 생기면 위에서부터 순서대로 확인
===========================================================================
1. 마찰(Friction)
   - 걸레 rigid body collider 와 RG2 손가락(left_inner_finger/right_inner_finger)
     collider에 각각 Physics Material 이 바인딩되어 있는가? (inspect_grip_friction 참고)
   - staticFriction / dynamicFriction 이 너무 낮지 않은가 (권장: >= 0.6~0.8)
   - frictionCombineMode 가 "average"라서 한쪽이 낮으면 전체가 낮아지는 건 아닌가
     (둘 다 높일 수 없다면 combineMode를 "max"로)
2. 그리퍼 목표 폭 / 파지력
   - GRASP_TARGET_WIDTH 가 손잡이 직경보다 살짝 작게(약간 파고들게) 설정됐는가
   - finger_joint 의 drive stiffness/damping, maxForce 가 너무 약하지 않은가
   - close_gripper_to_width 후 confirm_grasp_by_effort 의 힘/전류가 threshold 미만이면
     애초에 손가락이 걸레에 닿지도 않았다는 뜻 -> pre-grasp/grasp 좌표부터 재확인
3. PhysX Solver 설정
   - solverPositionIterationCount 가 너무 낮으면(<=4) 손가락-걸레 접촉이 매 스텝
     다시 계산되며 미세하게 파고드는 정도가 튀어서 파지력이 불안정해짐 -> 그립 대상
     articulation/rigid body 는 최소 16 이상 권장
   - solverVelocityIterationCount 가 너무 낮으면 마찰에 의한 상대속도 감쇠가 부족해
     걸레가 손가락 사이에서 미끄러짐(slip) -> 최소 4 이상 권장
   - (apply_grasp_solver_settings 참고)
4. Pre-grasp / Grasp 좌표 오차
   - handle_axis 가정(월드 Z 수직)이 실제 씬의 걸레 방향과 맞는가
   - compute_mop_grasp_poses 가 계산한 grasp_pos 가 실제 손잡이 중심을 지나는가
     (bbox 기반 자동 추정은 손잡이가 메시 최상단이 아니면 틀어질 수 있음 -> 필요시
     HANDLE_TOP_LOCAL_OFFSET 을 직접 보정)
5. 안정화 시간
   - 그립 직후 바로 들어올리지 말고 몇 프레임(app.update()) 대기해서 접촉이
     안정화된 뒤 lift 테스트를 시작했는가 (SETTLE_STEPS 참고)
===========================================================================
"""

import numpy as np
import omni.usd
import omni.timeline
import omni.kit.app
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, PhysxSchema, Gf

from isaacsim.core.prims import SingleArticulation, SingleRigidPrim

# ────────────────────────────────────────────────────────────────
# 설정값
# ────────────────────────────────────────────────────────────────
MOP_PRIM_PATH = "/World/SM_MopSet_01a_02"                     # 대걸레 Xform
MOP_MESH_PATH = f"{MOP_PRIM_PATH}/SM_MopSet_01a"               # 대걸레 자식 Mesh

ROBOCART_PATH = "/World/robocart1"                              # 두산+노바카터 묶음
ARM_PRIM_PATH = f"{ROBOCART_PATH}/m0609_with_gripper"          # M0609+RG2 articulation

GRIPPER_JOINT_NAME = "finger_joint"          # RG2 구동(driving) 조인트 (URDF 기준)
ARM_DOF_COUNT = 6                             # joint_1 ~ joint_6

# 손잡이 축: 씬 상에서 걸레가 세워져 있으므로(스크린샷 기준) 월드 Z를 축으로 가정
HANDLE_AXIS_WORLD = np.array([0.0, 0.0, 1.0])
HANDLE_TOP_LOCAL_OFFSET = None    # None이면 bbox 상단을 자동으로 손잡이 top으로 사용

PRE_GRASP_CLEARANCE = 0.15        # 손잡이 top 기준 pre-grasp 위쪽 오프셋 [m]
GRASP_DROP_FROM_TOP = 0.08        # top에서 이만큼 내려온 지점을 실제 grasp 지점으로 사용 [m]

GRASP_TARGET_WIDTH = 0.018        # RG2가 닫을 목표 폭 [m] (손잡이 직경보다 살짝 작게)
GRIPPER_OPEN_WIDTH = 0.06         # 대기/pre-grasp 시 벌린 폭 [m]
GRASP_EFFORT_THRESHOLD = 3.0      # 조인트 힘/전류 판정 임계값 (finger_joint 단위 기준, 튜닝 필요)

LIFT_HEIGHT = 0.10                # 낙하 테스트용 들어올림 높이 [m]
LIFT_STEPS = 90                   # 들어올리는 동안 진행할 physics step 수
SETTLE_STEPS = 30                 # 그립 직후 안정화 대기 step 수
DROP_POSITION_TOLERANCE = 0.02    # 걸레-그리퍼 상대 이동량이 이 값을 넘으면 "낙하/미끄러짐"으로 판정

# 그립 안정성용 PhysX solver 반복 횟수 (아래 apply_grasp_solver_settings 설명 참고)
GRASP_SOLVER_POSITION_ITERATIONS = 32
GRASP_SOLVER_VELOCITY_ITERATIONS = 8

FRICTION_WARN_THRESHOLD = 0.5     # 이 값 미만이면 마찰 부족 경고
# ────────────────────────────────────────────────────────────────


# ==================================================================
# 1) 월드 트랜스폼 조회 유틸리티
# ==================================================================
def get_prim_world_transform(prim_path: str):
    """
    주어진 prim의 현재 월드 좌표(위치/회전/스케일)를 반환한다.
    Stage에서 눈으로 확인한 로컬 Translate/Orient 값은 부모 Xform 체인의
    영향을 받으므로, 파지 계산에는 항상 이 함수의 월드 값을 사용해야 한다.

    Returns:
        position  : np.ndarray shape (3,)
        quat_wxyz : np.ndarray shape (4,)  (w, x, y, z)
        scale     : np.ndarray shape (3,)
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"[get_prim_world_transform] 유효하지 않은 prim: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    world_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    transform = Gf.Transform(world_matrix)

    translation = transform.GetTranslation()
    quat = transform.GetRotation().GetQuat()
    scale = transform.GetScale()

    position = np.array([translation[0], translation[1], translation[2]])
    quat_wxyz = np.array([
        quat.GetReal(),
        quat.GetImaginary()[0],
        quat.GetImaginary()[1],
        quat.GetImaginary()[2],
    ])
    scale_arr = np.array([scale[0], scale[1], scale[2]])

    print(f"[INFO] {prim_path} world pos={position}, quat(wxyz)={quat_wxyz}, scale={scale_arr}")
    return position, quat_wxyz, scale_arr


def get_prim_world_bbox(prim_path: str):
    """prim의 월드 공간 축정렬 바운딩박스(min, max)를 반환한다. 손잡이 top 자동 추정에 사용."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"[get_prim_world_bbox] 유효하지 않은 prim: {prim_path}")

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    bound = bbox_cache.ComputeWorldBound(prim)
    rng = bound.ComputeAlignedRange()
    bbox_min = np.array(rng.GetMin())
    bbox_max = np.array(rng.GetMax())
    return bbox_min, bbox_max


# ==================================================================
# 2) Pre-grasp / Grasp pose 계산
# ==================================================================
def compute_mop_grasp_poses(mop_prim_path: str = MOP_MESH_PATH):
    """
    손잡이 축(HANDLE_AXIS_WORLD) 기준으로 pre-grasp pose와 grasp pose를 계산한다.

    - grasp_pos      : 손잡이 top에서 GRASP_DROP_FROM_TOP 만큼 내려온, 손가락이 실제로
                        닫힐 지점
    - pre_grasp_pos  : grasp_pos에서 PRE_GRASP_CLEARANCE 만큼 위쪽(접근 방향 반대)으로
                        띄운 대기 지점
    - orientation    : 그리퍼가 위에서 아래로(top-down) 접근하는 고정 자세.
                        (w,x,y,z) = (0,1,0,0) 은 X축 기준 180도 회전으로, 로컬 +Z(툴 진행축)를
                        월드 -Z로 향하게 함 -> RMPFlowController와 동일 컨벤션(test_rmpflow.py 참고)

    원통형 손잡이라 그리퍼의 yaw(roll)는 파지 성공 여부에 영향을 주지 않으므로
    고정 top-down 자세를 그대로 사용한다. (비원통형 손잡이라면 yaw를 걸레의
    world quat에서 뽑아 정렬해야 함)
    """
    mop_pos, mop_quat_wxyz, _mop_scale = get_prim_world_transform(mop_prim_path)

    if HANDLE_TOP_LOCAL_OFFSET is not None:
        handle_top = mop_pos + np.array(HANDLE_TOP_LOCAL_OFFSET)
    else:
        _bbox_min, bbox_max = get_prim_world_bbox(mop_prim_path)
        # 손잡이 축이 월드 Z라고 가정했으므로 bbox의 최대 Z를 손잡이 top으로 사용
        handle_top = np.array([mop_pos[0], mop_pos[1], bbox_max[2]])

    grasp_pos = handle_top - HANDLE_AXIS_WORLD * GRASP_DROP_FROM_TOP
    pre_grasp_pos = grasp_pos + HANDLE_AXIS_WORLD * PRE_GRASP_CLEARANCE

    orientation_wxyz = np.array([0.0, 1.0, 0.0, 0.0])

    print(f"[INFO] handle_top={handle_top}")
    print(f"[INFO] pre_grasp_pos={pre_grasp_pos}")
    print(f"[INFO] grasp_pos={grasp_pos}")

    return {
        "handle_top": handle_top,
        "pre_grasp_pos": pre_grasp_pos,
        "grasp_pos": grasp_pos,
        "orientation_wxyz": orientation_wxyz,
    }


# ==================================================================
# 3) RG2 그리퍼 제어 / 파지 성공 판정
# ==================================================================
def _get_gripper_dof_index(articulation: SingleArticulation, joint_name: str = GRIPPER_JOINT_NAME) -> int:
    dof_names = articulation.dof_names
    if joint_name in dof_names:
        return dof_names.index(joint_name)

    # 정확한 이름이 안 맞으면(임포트 과정에서 접두사/접미사가 붙는 경우 등) fallback 탐색
    candidates = [i for i, n in enumerate(dof_names) if "finger" in n.lower()]
    if candidates:
        print(f"[경고] '{joint_name}' 대신 '{dof_names[candidates[0]]}' 사용")
        return candidates[0]

    raise ValueError(f"[_get_gripper_dof_index] 그리퍼 조인트를 찾을 수 없음. dof_names={dof_names}")


def close_gripper_to_width(articulation: SingleArticulation, target_width: float = GRASP_TARGET_WIDTH):
    """RG2 그리퍼를 target_width(그리퍼 조인트 각/위치 값)까지 position command로 닫는다."""
    gripper_idx = _get_gripper_dof_index(articulation)

    joint_positions = articulation.get_joint_positions()
    joint_positions[gripper_idx] = target_width

    from isaacsim.core.utils.types import ArticulationAction
    action = ArticulationAction(
        joint_positions=joint_positions,
        joint_indices=np.array([gripper_idx]),
    )
    articulation.apply_action(action)
    print(f"[INFO] 그리퍼 목표 폭 {target_width} 로 close 명령 전달 (dof idx={gripper_idx})")
    return gripper_idx


def open_gripper(articulation: SingleArticulation, target_width: float = GRIPPER_OPEN_WIDTH):
    return close_gripper_to_width(articulation, target_width)


def confirm_grasp_by_effort(
    articulation: SingleArticulation,
    joint_name: str = GRIPPER_JOINT_NAME,
    effort_threshold: float = GRASP_EFFORT_THRESHOLD,
) -> bool:
    """
    그리퍼 조인트에 걸린 힘/전류(measured joint effort)를 읽어 threshold 이상이면
    "무언가를 붙잡고 있다"고 판정한다. 손가락이 허공에서 닫히면 effort가 0에 가깝고,
    걸레에 걸려 더 못 닫히면 effort가 threshold를 넘어선다.
    """
    gripper_idx = _get_gripper_dof_index(articulation, joint_name)
    efforts = articulation.get_measured_joint_efforts()
    effort = abs(float(efforts[gripper_idx]))

    success = effort >= effort_threshold
    status = "성공" if success else "실패"
    print(f"[INFO] 그리퍼 조인트 effort={effort:.3f} (threshold={effort_threshold}) -> 파지 {status}")
    return success


# ==================================================================
# 4) 그립 후 들어올려 낙하 여부 확인
# ==================================================================
def run_lift_drop_test(
    arm_prim_path: str = ARM_PRIM_PATH,
    mop_prim_path: str = MOP_PRIM_PATH,
    lift_height: float = LIFT_HEIGHT,
    lift_steps: int = LIFT_STEPS,
):
    """
    그립이 완료된 상태에서 팔을 짧게 들어올린 뒤, 걸레가 그리퍼를 따라 같이
    움직였는지(=파지 성공) 아니면 제자리(혹은 아래)에 남았는지(=낙하/미끄러짐)를
    비교한다.

    사전 조건: close_gripper_to_width() 로 이미 파지 명령을 보낸 뒤, 몇 프레임
    (SETTLE_STEPS) 대기해서 접촉이 안정화된 상태여야 한다.
    """
    import sys
    RMPFLOW_DIR = "/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow"
    if RMPFLOW_DIR not in sys.path:
        sys.path.insert(0, RMPFLOW_DIR)
    from m0609_rmpflow_controller import RMPFlowController

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        print("[ERROR] Play 버튼(▶)을 먼저 누른 후 실행하세요!")
        return False

    app = omni.kit.app.get_app()

    arm = SingleArticulation(prim_path=arm_prim_path, name="m0609_lift_test")
    arm.initialize()
    mop_rigid = SingleRigidPrim(prim_path=mop_prim_path, name="mop_rigid_test")
    mop_rigid.initialize()

    # 안정화 대기
    for _ in range(SETTLE_STEPS):
        app.update()

    mop_pos_before, _ = mop_rigid.get_world_pose()
    ee_pos_before, _quat = arm.get_world_pose() if hasattr(arm, "get_world_pose") else (None, None)

    controller = RMPFlowController(
        name="rmpflow_lift_test_ctrl",
        robot_articulation=arm,
        physics_dt=1.0 / 60.0,
    )
    controller.reset()

    target_pos = np.array(mop_pos_before) + HANDLE_AXIS_WORLD * lift_height
    target_orientation = np.array([0.0, 1.0, 0.0, 0.0])

    for step in range(lift_steps):
        action = controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_orientation,
        )
        arm.apply_action(action)
        app.update()

    mop_pos_after, _ = mop_rigid.get_world_pose()
    mop_displacement = np.array(mop_pos_after) - np.array(mop_pos_before)
    lifted_amount = mop_displacement[2]  # world Z 상승량

    dropped = lifted_amount < (lift_height - DROP_POSITION_TOLERANCE)
    print(f"[INFO] 걸레 이동량(before->after)={mop_displacement}, Z 상승량={lifted_amount:.4f} (기대치 ~{lift_height})")
    if dropped:
        print("[결과] 낙하/미끄러짐 의심: 걸레가 그리퍼를 따라 충분히 올라오지 않음. "
              "상단 docstring의 체크리스트를 순서대로 확인하세요.")
    else:
        print("[결과] 파지 성공: 걸레가 그리퍼와 함께 정상적으로 들렸습니다.")

    return not dropped


# ==================================================================
# 5) 마찰(Physics Material) 점검
# ==================================================================
def list_prim_subtree(prim_path: str, max_depth: int = 4):
    """
    prim_path 하위 트리를 얕은 깊이까지 출력한다 (경로, type, RigidBody/Collision
    API 부착 여부). _resolve_collider_path가 콜라이더를 못 찾았을 때, 실제로 그
    prim 밑에 뭐가 있는지(비어있는지 / 이름이 다른지 / 깊이가 더 깊은지) 눈으로
    확인하기 위한 진단용 함수.
    """
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim.IsValid():
        print(f"[경고] 유효하지 않은 prim: {prim_path}")
        return

    root_depth = prim_path.count("/")
    for prim in Usd.PrimRange(root_prim):
        depth = str(prim.GetPath()).count("/") - root_depth
        if depth > max_depth:
            continue
        flags = []
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            flags.append("RigidBody")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            flags.append("Collision")
        flag_str = f" [{', '.join(flags)}]" if flags else ""
        indent = "  " * depth
        print(f"{indent}{prim.GetPath()} ({prim.GetTypeName()}){flag_str}")


def check_rigid_body_setup(prim_path: str):
    """
    prim(또는 그 하위)에 RigidBodyAPI / CollisionAPI가 실제로 붙어 있는지 확인한다.
    마찰 이전에, 애초에 강체/콜라이더 설정 자체가 안 되어 있으면 그립이 될 수 없으므로
    가장 먼저 점검해야 하는 항목이다.
    """
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim.IsValid():
        print(f"[경고] 유효하지 않은 prim: {prim_path}")
        return {"rigid_body": None, "collider": None}

    rigid_body_path = None
    collider_path = None
    for prim in Usd.PrimRange(root_prim):
        if rigid_body_path is None and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_path = str(prim.GetPath())
        if collider_path is None and prim.HasAPI(UsdPhysics.CollisionAPI):
            collider_path = str(prim.GetPath())

    if rigid_body_path is None:
        print(f"[경고] {prim_path} 하위에 RigidBodyAPI가 붙은 prim이 없음 -> 강체로 시뮬레이션되지 않음")
    else:
        print(f"[INFO] RigidBodyAPI: {rigid_body_path}")

    if collider_path is None:
        print(f"[경고] {prim_path} 하위에 CollisionAPI(Collider)가 붙은 prim이 없음 -> 접촉 자체가 발생하지 않음")
    else:
        print(f"[INFO] CollisionAPI(Collider): {collider_path}")

    return {"rigid_body": rigid_body_path, "collider": collider_path}


def _resolve_collider_path(root_prim_path: str) -> str:
    """
    root_prim_path 자신 또는 하위 prim 중 실제로 UsdPhysics.CollisionAPI가 붙은
    prim의 경로를 찾는다. Physics Material은 Collider가 있는 prim(또는 그 조상)에
    바인딩되어야 의미가 있으므로, 마찰 점검/바인딩 전에 항상 이 함수로 실제
    콜라이더 prim을 먼저 찾는다. (예: SM_MopSet_01a_02 는 껍데기 Xform이고 실제
    콜라이더는 자식 Mesh인 SM_MopSet_01a 에 붙어 있는 경우가 흔함)
    """
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim.IsValid():
        print(f"[경고] 유효하지 않은 prim: {root_prim_path}")
        return None

    if root_prim.HasAPI(UsdPhysics.CollisionAPI):
        return root_prim_path

    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            resolved = str(prim.GetPath())
            if resolved != root_prim_path:
                print(f"[INFO] {root_prim_path} 자체엔 Collider가 없어 하위 {resolved} 를 콜라이더로 사용")
            return resolved

    print(f"[경고] {root_prim_path} 하위에서 CollisionAPI가 붙은 prim을 전혀 찾지 못함 -> "
          f"이 subtree에는 물리 콜라이더가 없다는 뜻. list_prim_subtree()로 실제 구조를 확인할 것")
    return None


def _get_bound_physics_material_friction(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[경고] 유효하지 않은 prim: {prim_path}")
        return None

    binding_api = UsdShade.MaterialBindingAPI(prim)
    bound_material, _rel = binding_api.ComputeBoundMaterial(materialPurpose="physics")
    if not bound_material or not bound_material.GetPrim().IsValid():
        print(f"[경고] {prim_path} 에 바인딩된 Physics Material이 없음")
        return None

    material_prim = bound_material.GetPrim()
    if not material_prim.HasAPI(UsdPhysics.MaterialAPI):
        print(f"[경고] {material_prim.GetPath()} 에 UsdPhysics.MaterialAPI 없음")
        return None

    physics_material = UsdPhysics.MaterialAPI(material_prim)
    static_friction = physics_material.GetStaticFrictionAttr().Get()
    dynamic_friction = physics_material.GetDynamicFrictionAttr().Get()

    combine_mode = None
    if material_prim.HasAPI(PhysxSchema.PhysxMaterialAPI):
        physx_material = PhysxSchema.PhysxMaterialAPI(material_prim)
        combine_mode = physx_material.GetFrictionCombineModeAttr().Get()

    print(f"[INFO] {prim_path} -> material {material_prim.GetPath()}: "
          f"staticFriction={static_friction}, dynamicFriction={dynamic_friction}, combineMode={combine_mode}")
    return {
        "material_path": str(material_prim.GetPath()),
        "static_friction": static_friction,
        "dynamic_friction": dynamic_friction,
        "combine_mode": combine_mode,
    }


def inspect_grip_friction(
    mop_prim_path: str = MOP_PRIM_PATH,
    finger_prim_paths=None,
):
    """
    걸레 rigid body와 RG2 손가락 콜라이더에 바인딩된 Physics Material의 마찰 계수를
    점검한다. 값이 너무 낮으면(FRICTION_WARN_THRESHOLD 미만) 경고를 출력한다.

    조정 방법 (둘 중 하나):
      A) 기존 Physics Material의 staticFriction/dynamicFriction 값을 올린다.
         physics_material = UsdPhysics.MaterialAPI(material_prim)
         physics_material.GetStaticFrictionAttr().Set(0.8)
         physics_material.GetDynamicFrictionAttr().Set(0.7)
      B) 아예 새 Physics Material을 만들어 두 콜라이더(걸레, 손가락)에 모두 바인딩한다.
         mat_path = "/World/Looks/HighFrictionGripMaterial"
         UsdShade.Material.Define(stage, mat_path)
         mat_prim = stage.GetPrimAtPath(mat_path)
         UsdPhysics.MaterialAPI.Apply(mat_prim)
         UsdPhysics.MaterialAPI(mat_prim).CreateStaticFrictionAttr().Set(0.8)
         UsdPhysics.MaterialAPI(mat_prim).CreateDynamicFrictionAttr().Set(0.7)
         PhysxSchema.PhysxMaterialAPI.Apply(mat_prim).CreateFrictionCombineModeAttr().Set("max")
         UsdShade.MaterialBindingAPI(collider_prim).Bind(UsdShade.Material(mat_prim), materialPurpose="physics")
      frictionCombineMode를 "max"로 두면 두 콜라이더 중 마찰이 높은 쪽 값을 쓰므로,
      한쪽만 고쳐도 효과가 바로 반영되어 튜닝이 쉬워짐.
    """
    if finger_prim_paths is None:
        finger_prim_paths = [
            f"{ARM_PRIM_PATH}/left_inner_finger",
            f"{ARM_PRIM_PATH}/right_inner_finger",
        ]

    mop_collider_path = _resolve_collider_path(mop_prim_path)
    results = {
        "mop": _get_bound_physics_material_friction(mop_collider_path) if mop_collider_path else None
    }
    for finger_path in finger_prim_paths:
        finger_collider_path = _resolve_collider_path(finger_path)
        results[finger_path] = (
            _get_bound_physics_material_friction(finger_collider_path) if finger_collider_path else None
        )

    for name, info in results.items():
        if info is None:
            continue
        min_friction = min(info["static_friction"] or 0.0, info["dynamic_friction"] or 0.0)
        if min_friction < FRICTION_WARN_THRESHOLD:
            print(f"[경고] {name}: 마찰 계수가 낮음({min_friction:.2f} < {FRICTION_WARN_THRESHOLD}). "
                  f"위 docstring의 조정 방법 A 또는 B 참고.")

    return results


def bind_high_friction_grip_material(
    mop_prim_path: str = MOP_PRIM_PATH,
    finger_prim_paths=None,
    static_friction: float = 0.8,
    dynamic_friction: float = 0.7,
    material_path: str = "/World/Looks/HighFrictionGripMaterial",
):
    """
    inspect_grip_friction()에서 "바인딩된 Physics Material 없음"으로 나온 경우,
    실제로 고마찰 Physics Material을 생성/갱신하고 걸레 콜라이더 + RG2 손가락
    콜라이더 양쪽에 바인딩한다 (docstring 방법 B를 코드로 실행).

    frictionCombineMode="max"로 설정하므로, 이후 둘 중 어느 한쪽만 이 Material을
    쓰고 있어도 마찰은 max(양쪽) 기준으로 계산된다.
    """
    stage = omni.usd.get_context().get_stage()

    if finger_prim_paths is None:
        finger_prim_paths = [
            f"{ARM_PRIM_PATH}/left_inner_finger",
            f"{ARM_PRIM_PATH}/right_inner_finger",
        ]

    material_prim = stage.GetPrimAtPath(material_path)
    if not material_prim.IsValid():
        UsdShade.Material.Define(stage, material_path)
        material_prim = stage.GetPrimAtPath(material_path)
        print(f"[INFO] {material_path} 생성됨")

    if not material_prim.HasAPI(UsdPhysics.MaterialAPI):
        UsdPhysics.MaterialAPI.Apply(material_prim)
    physics_material = UsdPhysics.MaterialAPI(material_prim)
    physics_material.CreateStaticFrictionAttr().Set(static_friction)
    physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)

    if not material_prim.HasAPI(PhysxSchema.PhysxMaterialAPI):
        PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
    physx_material = PhysxSchema.PhysxMaterialAPI(material_prim)
    physx_material.CreateFrictionCombineModeAttr().Set("max")

    print(f"[INFO] {material_path}: staticFriction={static_friction}, "
          f"dynamicFriction={dynamic_friction}, combineMode=max")

    target_paths = [mop_prim_path] + list(finger_prim_paths)
    skipped = []
    for target_path in target_paths:
        collider_path = _resolve_collider_path(target_path)
        if collider_path is None:
            # 콜라이더가 없는 prim에 바인딩해봐야 PhysX가 참조하지 않으므로 건너뛴다.
            # (여기서 "바인딩 완료"라고 찍으면 실제로는 아무 효과가 없는데 성공한 것처럼
            # 보이는 오해를 만들기 때문에 명시적으로 skip 처리)
            print(f"[경고] {target_path}: 콜라이더가 없어 바인딩을 건너뜀 -> "
                  f"list_prim_subtree('{target_path}')로 실제 하위 구조부터 확인할 것")
            skipped.append(target_path)
            continue

        collider_prim = stage.GetPrimAtPath(collider_path)
        binding_api = UsdShade.MaterialBindingAPI.Apply(collider_prim)
        binding_api.Bind(
            UsdShade.Material(material_prim),
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        print(f"[INFO] {collider_path} <- {material_path} 바인딩 완료")

    if skipped:
        print(f"[경고] 콜라이더 부재로 바인딩 못 한 prim: {skipped} "
              f"-> 이 부분은 마찰 설정 이전에 Collider부터 추가해야 함")

    return material_path


# ==================================================================
# 6) 그립 안정성용 PhysX Solver 반복 횟수 적용
# ==================================================================
def apply_grasp_solver_settings(
    prim_paths,
    position_iterations: int = GRASP_SOLVER_POSITION_ITERATIONS,
    velocity_iterations: int = GRASP_SOLVER_VELOCITY_ITERATIONS,
):
    """
    solverPositionIterationCount: 접촉/구속조건을 얼마나 여러 번 반복 계산해서
    "위치"를 맞출지 결정한다. 낮으면(기본 4) 손가락-손잡이처럼 강체끼리 꽉 끼는
    접촉에서 매 스텝 침투/반발이 다시 계산되며 파지력이 떨리듯 불안정해진다.
    RG2처럼 작은 물체를 정밀하게 쥐는 경우 16~32 이상을 권장.

    solverVelocityIterationCount: 마찰/반발 등 "속도" 관련 구속을 얼마나 반복
    계산할지 결정한다. 낮으면 마찰력이 상대 미끄러짐 속도를 충분히 감쇠시키지
    못해 grasp 이후에도 손잡이가 손가락 사이에서 서서히 미끄러져 빠짐(slip).
    4~8 이상을 권장.

    두 값 모두 높일수록 안정적이지만 스텝당 연산 비용이 늘어나므로, 그립 대상
    (걸레 rigid body, RG2 손가락 link)에만 국소적으로 적용하는 것을 권장한다.
    """
    stage = omni.usd.get_context().get_stage()
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[경고] 유효하지 않은 prim, 건너뜀: {prim_path}")
            continue

        physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_rb_api.CreateSolverPositionIterationCountAttr().Set(position_iterations)
        physx_rb_api.CreateSolverVelocityIterationCountAttr().Set(velocity_iterations)
        print(f"[INFO] {prim_path}: solver position/velocity iterations = "
              f"{position_iterations}/{velocity_iterations}")


# ==================================================================
# 실행부 (Script Editor에서 필요한 단계만 주석 해제해서 사용)
# ==================================================================
if __name__ == "__main__":
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        print("[ERROR] Play 버튼(▶)을 먼저 누른 후 스크립트를 실행하세요!")
    else:
        # 1. 현재 걸레 좌표 확인
        get_prim_world_transform(MOP_MESH_PATH)

        # 2. pre-grasp / grasp pose 계산
        poses = compute_mop_grasp_poses()

        # 2.5. RigidBody/Collider가 애초에 붙어 있는지 확인 (마찰보다 먼저 확인해야 함)
        check_rigid_body_setup(MOP_PRIM_PATH)
        check_rigid_body_setup(f"{ARM_PRIM_PATH}/left_inner_finger")
        check_rigid_body_setup(f"{ARM_PRIM_PATH}/right_inner_finger")

        # 2.6. 손가락 쪽에서 콜라이더를 못 찾았다면(직전 실행 로그에서 확인됨),
        #      실제 하위 구조를 눈으로 확인
        list_prim_subtree(f"{ARM_PRIM_PATH}/left_inner_finger")
        list_prim_subtree(f"{ARM_PRIM_PATH}/right_inner_finger")

        # 3. 마찰 사전 점검 (그립 전에 먼저 확인하는 걸 권장)
        inspect_grip_friction()

        # 3.5. 마찰 Material이 없다고 나오면 고마찰 Material을 만들어 바인딩
        #      (씬 저장 전까지는 되돌리기 쉬운 편집이라 기본으로 켜둠. 이미 바인딩된
        #      Material이 있다면 값만 덮어씀)
        bind_high_friction_grip_material()

        # 4. solver 반복 횟수 상향 (걸레 + 손가락)
        # apply_grasp_solver_settings([
        #     MOP_PRIM_PATH,
        #     f"{ARM_PRIM_PATH}/left_inner_finger",
        #     f"{ARM_PRIM_PATH}/right_inner_finger",
        # ])

        # 5. 팔을 poses["pre_grasp_pos"] -> poses["grasp_pos"] 로 이동시키는 것은
        #    test_rmpflow.py 의 RMPFlowController 사용 패턴을 그대로 따르면 됨.
        #    이동 완료 후:
        # arm = SingleArticulation(prim_path=ARM_PRIM_PATH, name="m0609_grasp")
        # arm.initialize()
        # close_gripper_to_width(arm, GRASP_TARGET_WIDTH)
        # confirm_grasp_by_effort(arm)

        # 6. 낙하 테스트
        # run_lift_drop_test()

        print("[INFO] 실행부: 필요한 단계의 주석을 해제해서 사용하세요.")
