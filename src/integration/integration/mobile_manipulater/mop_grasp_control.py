"""
mop_grasp_control.py
---------------------
격리 병동 비대면 로봇팔 프로젝트 - 대걸레(rigid body mop) 파지 스크립트.

이 씬(mobile_manipulator_v2.usd)은 이전 씬(robocart1/m0609_with_gripper,
SM_MopSet_01a_02)과 prim 구조가 다르다. stage_inspection.py로 확인한 실제 경로 기준:

  로봇 팔+그리퍼 : /World/m0609_with_gripper  (Nova_Carter_ROS와 같은 World 레벨)
  그리퍼 손가락  : /World/m0609_with_gripper/left_inner_finger, right_inner_finger
  그리퍼 구동 조인트 : finger_joint (Robotiq 계열 6-joint underactuated 그리퍼.
                        finger_joint만 능동 구동, 나머지(knuckle 등)는 mimic 대상으로
                        추정 - 실제로 같이 움직이지 않으면 PhysicsMimicJointAPI 설정을
                        확인할 것)
  대걸레         : 이 씬에는 원래 없었음. Hospital Props의 SM_MopSet_01a.usd를
                    setup_mop_in_scene()으로 새로 참조/배치한다. (아래 0번 섹션)

Nova_Carter_ROS(chassis_link)와 m0609_with_gripper(root_joint) 양쪽에 각각
ArticulationRootAPI가 남아있고 PhysicsFixedJoint로 연결된 상태다(두 개의 독립된
articulation을 joint로 잇는 형태). 이번 최소 기능(팔만 RMPFlowController로 제어)에는
문제 없지만, Carter 쪽까지 하나의 articulation으로 완전히 합치려면 별도로
m0609_with_gripper 쪽 root_joint의 ArticulationRootAPI를 제거해야 한다 (이번 범위 밖).

Isaac Sim 5.1.0 기준으로 omni.isaac.core 대신 isaacsim.core.* 네임스페이스 사용
(프로젝트 내 test_rmpflow.py 와 동일 컨벤션).

사용법 (Script Editor):
  1. mobile_manipulator_v2.usd 를 열고 Play(▶)
  2. 이 파일 전체를 Script Editor 에 붙여넣고 필요한 함수만 골라 호출
     (파일 맨 아래 "실행부" 참고, 기본은 각 단계 주석 처리되어 있음)

===========================================================================
그립 실패(미끄러짐/낙하) 체크리스트 - 문제가 생기면 위에서부터 순서대로 확인
===========================================================================
1. 마찰(Friction)
   - 걸레 rigid body collider 와 손가락(left_inner_finger/right_inner_finger)
     collider에 각각 Physics Material 이 바인딩되어 있는가? (inspect_grip_friction 참고)
   - staticFriction / dynamicFriction 이 너무 낮지 않은가 (권장: >= 0.6~0.8)
   - frictionCombineMode 가 "average"라서 한쪽이 낮으면 전체가 낮아지는 건 아닌가
     (둘 다 높일 수 없다면 combineMode를 "max"로)
2. 그리퍼 목표 폭 / 파지력
   - GRASP_TARGET_WIDTH 가 손잡이 직경보다 살짝 작게(약간 파고들게) 설정됐는가
   - finger_joint 의 drive stiffness/damping, maxForce 가 너무 약하지 않은가
   - close_gripper_to_width 후 confirm_grasp_by_effort 의 힘/전류가 threshold 미만이면
     애초에 손가락이 걸레에 닿지도 않았다는 뜻 -> pre-grasp/grasp 좌표부터 재확인
   - finger_joint만 움직이고 나머지 knuckle 조인트가 안 따라가면 mimic이 안 걸린
     것이므로 실제로는 한쪽 손가락만 닫히고 있을 수 있음 -> list_prim_subtree로 확인
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
# ── 대걸레: 절차적(procedural) 생성 ──
# Hospital Props의 SM_MopSet_01a/01b는 둘 다 "버킷+짜개 세트"로, 실제 잡을 수 있는
# 손잡이가 바닥 근처 30~40cm짜리 짧은 구간뿐이라(analyze_mop_mesh.py로 실측) 파지에
# 부적합했다. 대신 지오메트리를 완전히 통제할 수 있는 단순 대걸레(긴 원통 손잡이 +
# 납작한 헤드 박스)를 USD 프리미티브로 직접 만들어 사용한다 (create_simple_mop 참고).
MOP_PRIM_PATH = "/World/SimpleMop"               # 대걸레 rigid body 루트 Xform
MOP_MESH_PATH = MOP_PRIM_PATH                    # 절차적 mop은 루트가 곧 rigid body

MOP_HANDLE_RADIUS = 0.0125     # 손잡이 원통 반지름 [m] (직경 2.5cm)
MOP_HANDLE_LENGTH = 0.70       # 손잡이 길이 [m] (헤드 위 z=0.05부터 z=0.75까지)
MOP_HEAD_SIZE = np.array([0.30, 0.12, 0.05])   # 헤드 박스 X*Y*Z [m]

# arm base_link(world (-0.235, 0.001, 0.577))에서 수평 ~0.6m, 리치(~0.9m) 안쪽.
MOP_SPAWN_POSITION = np.array([0.35, 0.10, 0.0])
MOP_SPAWN_ORIENTATION_WXYZ = np.array([1.0, 0.0, 0.0, 0.0])     # identity (수직으로 세움)
MOP_SPAWN_SCALE = np.array([1.0, 1.0, 1.0])
MOP_MASS = 0.5                                                   # [kg]
MOP_COM_LOCAL = np.array([0.0, 0.0, 0.03])       # 무게중심을 헤드 쪽으로 낮춰 세워둔 상태 안정화

ARM_PRIM_PATH = "/World/m0609_with_gripper"      # M0609+그리퍼 articulation (stage_inspection.py로 확인)

# finger_joint는 revolute (m0609_with_gripper.urdf 확인: limit lower=0 upper=1.18 rad).
# m0609_with_gripper.urdf에서 finger_joint(gripper_body->right_outer_knuckle)와
# right_inner_knuckle_joint는 서로 mimic으로 묶여있지 않아, finger_joint 하나만
# 구동하면 반대쪽이 따라오지 않는다(그립 effort가 거의 0으로 나오는 원인이었음).
# 사용자가 공유한 pick_and_place 레퍼런스(m0609_pick_place_controller.py 사용 예시)와
# 동일하게 두 조인트를 함께 구동한다.
GRIPPER_JOINT_NAME = "finger_joint"                              # effort 확인 등 단일 대표 조인트
GRIPPER_JOINT_NAMES = ["finger_joint", "right_inner_knuckle_joint"]  # 실제 구동 시 함께 명령
ARM_DOF_COUNT = 6                             # joint_1 ~ joint_6

# 그리퍼 관절(및 팔 관절) drive가 기본값으로는 약해서 접촉력을 못 버틸 수 있음.
# 레퍼런스 코드와 동일하게 stiffness/damping/maxForce를 크게 올린다 (apply_arm_drive_settings 참고).
ARM_DRIVE_STIFFNESS = 1e8
ARM_DRIVE_DAMPING = 1e4
ARM_DRIVE_MAX_FORCE = 1e8

# 손잡이 축: 걸레가 세워져 있다고 가정하고 월드 Z를 축으로 사용
HANDLE_AXIS_WORLD = np.array([0.0, 0.0, 1.0])

# 절차적 mop이므로 손잡이 top 위치를 정확히 안다: 루트 기준 (0,0,0.75).
HANDLE_TOP_LOCAL_OFFSET = np.array([0.0, 0.0, MOP_HANDLE_LENGTH + 0.05])

PRE_GRASP_CLEARANCE = 0.15        # grasp 지점 기준 pre-grasp 위쪽 오프셋 [m]
# 주의: 이 값은 "tool0(link_6 끝)"가 갈 목표의 오프셋이다. 실측(diagnose_grasp.py)
# 결과 손가락 접촉점은 tool0보다 약 0.15m 아래에 있으므로, tool0를 손잡이 top보다
# 위에 둬야 손가락이 top 약간 아래 구간을 감싼다. 그래서 음수(= top보다 위)를 사용:
# tool0가 top+0.07 에 있으면 손가락은 top-0.08 근처에서 손잡이를 잡는다.
GRASP_DROP_FROM_TOP = -0.07

# finger_joint 조인트 목표값 [rad] (m 아님! revolute joint, range 0~1.18).
# 직경 2.5cm의 가는 손잡이이므로 깊게(1.0 rad) 닫는다 - 접촉하면 drive가 거기서
# 멈추고 effort가 올라가므로 목표가 깊어도 문제 없음 (effort 기반 파지 판정에 유리).
GRASP_TARGET_WIDTH = 1.0           # 닫을 때 목표 각도 [rad]
GRIPPER_OPEN_WIDTH = 0.0           # 열림 각도 [rad]
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
# 0) 대걸레(절차적 생성) 씬에 추가
# ==================================================================
def setup_mop_in_scene(
    mop_prim_path: str = MOP_PRIM_PATH,
    position=MOP_SPAWN_POSITION,
    orientation_wxyz=MOP_SPAWN_ORIENTATION_WXYZ,
    mass: float = MOP_MASS,
    handle_radius: float = MOP_HANDLE_RADIUS,
    handle_length: float = MOP_HANDLE_LENGTH,
    head_size=MOP_HEAD_SIZE,
):
    """
    긴 원통 손잡이 + 납작한 헤드 박스로 이루어진 단순 대걸레를 USD 프리미티브로
    직접 생성하고, 루트 Xform에 RigidBodyAPI + 질량 + 낮은 무게중심을 부여한다.

    구조 (루트 로컬 좌표, 바닥이 z=0):
      /World/SimpleMop            (Xform, RigidBody, mass/CoM)
        /World/SimpleMop/head     (Cube collider, 0.30x0.12x0.05, 바닥에 붙음)
        /World/SimpleMop/handle   (Cylinder collider, r=0.0125, z=0.05~0.75)

    이미 mop_prim_path가 있으면 재생성하지 않고 pose만 다시 세팅한다.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[ERROR] No stage is open.")
        return None

    root_prim = stage.GetPrimAtPath(mop_prim_path)
    newly_created = not root_prim.IsValid()
    if newly_created:
        UsdGeom.Xform.Define(stage, mop_prim_path)
        root_prim = stage.GetPrimAtPath(mop_prim_path)
        print(f"[INFO] Created procedural mop root: {mop_prim_path}")
    else:
        print(f"[INFO] {mop_prim_path} already exists, reusing it")

    xformable = UsdGeom.Xformable(root_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(
        Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    xformable.AddOrientOp().Set(Gf.Quatf(
        float(orientation_wxyz[0]),
        float(orientation_wxyz[1]),
        float(orientation_wxyz[2]),
        float(orientation_wxyz[3]),
    ))

    if newly_created:
        # head: Cube 기본 크기가 2x2x2 이므로 scale로 원하는 크기를 만든다
        head_path = f"{mop_prim_path}/head"
        head = UsdGeom.Cube.Define(stage, head_path)
        head_prim = stage.GetPrimAtPath(head_path)
        head_xf = UsdGeom.Xformable(head_prim)
        head_xf.ClearXformOpOrder()
        head_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(head_size[2]) / 2.0))
        head_xf.AddScaleOp().Set(Gf.Vec3f(
            float(head_size[0]) / 2.0, float(head_size[1]) / 2.0, float(head_size[2]) / 2.0))
        UsdPhysics.CollisionAPI.Apply(head_prim)
        UsdGeom.Gprim(head_prim).CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.4, 0.8)])

        # handle: 헤드 위 z=0.05 부터 위로 handle_length 만큼
        handle_path = f"{mop_prim_path}/handle"
        handle = UsdGeom.Cylinder.Define(stage, handle_path)
        handle.CreateRadiusAttr().Set(handle_radius)
        handle.CreateHeightAttr().Set(handle_length)
        handle.CreateAxisAttr().Set("Z")
        handle.CreateExtentAttr().Set([
            Gf.Vec3f(-handle_radius, -handle_radius, -handle_length / 2.0),
            Gf.Vec3f(handle_radius, handle_radius, handle_length / 2.0),
        ])
        handle_prim = stage.GetPrimAtPath(handle_path)
        handle_xf = UsdGeom.Xformable(handle_prim)
        handle_xf.ClearXformOpOrder()
        handle_xf.AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, float(head_size[2]) + handle_length / 2.0))
        UsdPhysics.CollisionAPI.Apply(handle_prim)
        UsdGeom.Gprim(handle_prim).CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.5, 0.3)])

        UsdPhysics.RigidBodyAPI.Apply(root_prim)
        mass_api = UsdPhysics.MassAPI.Apply(root_prim)
        mass_api.CreateMassAttr().Set(mass)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(
            float(MOP_COM_LOCAL[0]), float(MOP_COM_LOCAL[1]), float(MOP_COM_LOCAL[2])))
        print(f"[INFO] mop rigid body: mass={mass} kg, CoM={MOP_COM_LOCAL} (low, for standing stability)")
        print(f"[INFO] handle: r={handle_radius}, length={handle_length}, top at local z="
              f"{float(head_size[2]) + handle_length:.3f}")

    print(f"[INFO] Mop placed at position={position}, orientation_wxyz={orientation_wxyz}")
    return mop_prim_path


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
        raise ValueError(f"[get_prim_world_transform] invalid prim: {prim_path}")

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
        raise ValueError(f"[get_prim_world_bbox] invalid prim: {prim_path}")

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    bound = bbox_cache.ComputeWorldBound(prim)
    rng = bound.ComputeAlignedRange()
    bbox_min = np.array(rng.GetMin())
    bbox_max = np.array(rng.GetMax())
    return bbox_min, bbox_max


def sync_rmpflow_base_pose(controller, base_link_path: str = f"{ARM_PRIM_PATH}/base_link"):
    """
    RMPFlowController가 내부적으로 캐싱하는 로봇 베이스 pose를 실제 base_link의
    월드 pose로 덮어쓴다.

    RMPFlowController.__init__()은 robot_articulation.get_world_pose()로 베이스
    pose를 한 번 읽어서 캐싱하는데, robot_articulation의 prim_path로 넘기는
    ARM_PRIM_PATH(/World/m0609_with_gripper)는 payload를 얹은 정적 Xform
    컨테이너라서 항상 (0,0,0)에 머문다. 실제 물리적 base_link는 Nova Carter
    위에 PhysicsFixedJoint로 고정되어 world (~-0.23, 0, 0.58) 근처에 있는데,
    이 차이를 보정하지 않으면 RMPFlowController가 완전히 잘못된 좌표계 기준으로
    IK를 풀어서 목표 지점에서 수십 cm씩 벗어난다 (diagnose_grasp.py로 확인,
    보정 전 오차 0.45~0.55m -> 보정 후 0.0015~0.12m).

    controller.reset()이 호출될 때마다 self._default_position/_orientation을
    다시 적용하므로, 이 함수는 reset() 이후에 호출해야 한다.
    """
    real_pos, real_quat, _ = get_prim_world_transform(base_link_path)
    controller._default_position = real_pos
    controller._default_orientation = real_quat
    controller.rmp_flow.set_robot_base_pose(robot_position=real_pos, robot_orientation=real_quat)
    print(f"[INFO] synced RMPFlow base pose to actual {base_link_path} world pose: {real_pos}")
    return real_pos, real_quat


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
        bbox_min, bbox_max = get_prim_world_bbox(mop_prim_path)
        # prim origin(mop_pos)이 메시 bbox 중심과 일치한다는 보장이 없다 (실측 결과
        # SM_MopSet_01a는 origin이 자기 bbox X 범위 밖에 있었음 -> 그리퍼가 허공에서
        # 닫히는 원인이었음). bbox XY 중심을 사용하는 게 origin보다 안전한 기본값.
        # 손잡이 축이 월드 Z라고 가정했으므로 bbox의 최대 Z를 손잡이 top으로 사용.
        bbox_center_xy = (bbox_min[:2] + bbox_max[:2]) / 2.0
        handle_top = np.array([bbox_center_xy[0], bbox_center_xy[1], bbox_max[2]])

    grasp_pos = handle_top - HANDLE_AXIS_WORLD * GRASP_DROP_FROM_TOP
    pre_grasp_pos = grasp_pos + HANDLE_AXIS_WORLD * PRE_GRASP_CLEARANCE

    orientation_wxyz = np.array([0.0, 1.0, 0.0, 0.0])

    if HANDLE_TOP_LOCAL_OFFSET is None:
        print(f"[INFO] mop bbox min={bbox_min}, max={bbox_max}")
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
# 3) 그리퍼 제어 / 파지 성공 판정
# ==================================================================
def _get_gripper_dof_index(articulation: SingleArticulation, joint_name: str = GRIPPER_JOINT_NAME) -> int:
    dof_names = articulation.dof_names
    if joint_name in dof_names:
        return dof_names.index(joint_name)

    # 정확한 이름이 안 맞으면(임포트 과정에서 접두사/접미사가 붙는 경우 등) fallback 탐색
    candidates = [i for i, n in enumerate(dof_names) if "finger" in n.lower()]
    if candidates:
        print(f"[WARNING] using '{dof_names[candidates[0]]}' instead of '{joint_name}'")
        return candidates[0]

    raise ValueError(f"[_get_gripper_dof_index] gripper joint not found. dof_names={dof_names}")


def close_gripper_to_width(
    articulation: SingleArticulation,
    target_width: float = GRASP_TARGET_WIDTH,
    joint_names=None,
):
    """
    그리퍼를 target_width(rad, revolute joint 목표각)까지 position command로 닫는다.

    joint_names에 있는 조인트를 전부 동일한 target_width로 함께 구동한다
    (기본값 GRIPPER_JOINT_NAMES = ["finger_joint", "right_inner_knuckle_joint"]).
    finger_joint 하나만 구동하면 반대쪽(오른쪽 계열 이너 너클)이 mimic으로
    묶여있지 않아 따라오지 않는 것으로 확인됨 -> 실제 파지력이 거의 안 나옴.
    """
    if joint_names is None:
        joint_names = GRIPPER_JOINT_NAMES
    gripper_indices = [_get_gripper_dof_index(articulation, name) for name in joint_names]

    # joint_indices로 일부 DOF만 지정할 때는 joint_positions도 그 subset 크기에
    # 맞는 배열이어야 한다 (전체 DOF 배열을 넘기면 shape mismatch 에러 발생).
    from isaacsim.core.utils.types import ArticulationAction
    action = ArticulationAction(
        joint_positions=np.array([target_width] * len(gripper_indices)),
        joint_indices=np.array(gripper_indices),
    )
    articulation.apply_action(action)
    print(f"[INFO] gripper close command target={target_width} rad on {joint_names} (dof idx={gripper_indices})")
    return gripper_indices


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
    status = "SUCCESS" if success else "FAIL"
    print(f"[INFO] gripper joint effort={effort:.3f} (threshold={effort_threshold}) -> grasp {status}")
    return success


# ==================================================================
# 4) 그립 후 들어올려 낙하 여부 확인
# ==================================================================
def run_lift_drop_test(
    arm_prim_path: str = ARM_PRIM_PATH,
    mop_prim_path: str = MOP_MESH_PATH,
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
        print("[ERROR] Press Play before running this!")
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
    print(f"[INFO] mop displacement(before->after)={mop_displacement}, Z lift={lifted_amount:.4f} (expected ~{lift_height})")
    if dropped:
        print("[RESULT] Possible slip/drop: mop did not follow the gripper enough. "
              "Check the checklist at the top of this file in order.")
    else:
        print("[RESULT] Grasp OK: mop moved together with the gripper.")

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
        print(f"[WARNING] invalid prim: {prim_path}")
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
        print(f"[WARNING] invalid prim: {prim_path}")
        return {"rigid_body": None, "collider": None}

    rigid_body_path = None
    collider_path = None
    for prim in Usd.PrimRange(root_prim):
        if rigid_body_path is None and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_path = str(prim.GetPath())
        if collider_path is None and prim.HasAPI(UsdPhysics.CollisionAPI):
            collider_path = str(prim.GetPath())

    if rigid_body_path is None:
        print(f"[WARNING] no RigidBodyAPI found under {prim_path} -> not simulated as a rigid body")
    else:
        print(f"[INFO] RigidBodyAPI: {rigid_body_path}")

    if collider_path is None:
        print(f"[WARNING] no CollisionAPI found under {prim_path} -> no contact will occur")
    else:
        print(f"[INFO] CollisionAPI(Collider): {collider_path}")

    return {"rigid_body": rigid_body_path, "collider": collider_path}


def _resolve_collider_path(root_prim_path: str) -> str:
    """
    root_prim_path 자신 또는 하위 prim 중 실제로 UsdPhysics.CollisionAPI가 붙은
    prim의 경로를 찾는다. Physics Material은 Collider가 있는 prim(또는 그 조상)에
    바인딩되어야 의미가 있으므로, 마찰 점검/바인딩 전에 항상 이 함수로 실제
    콜라이더 prim을 먼저 찾는다.
    """
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim.IsValid():
        print(f"[WARNING] invalid prim: {root_prim_path}")
        return None

    if root_prim.HasAPI(UsdPhysics.CollisionAPI):
        return root_prim_path

    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            resolved = str(prim.GetPath())
            if resolved != root_prim_path:
                print(f"[INFO] {root_prim_path} has no collider itself, using child {resolved}")
            return resolved

    print(f"[WARNING] no CollisionAPI found anywhere under {root_prim_path} -> "
          f"this subtree has no physics collider. Use list_prim_subtree() to inspect it")
    return None


def _get_bound_physics_material_friction(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[WARNING] invalid prim: {prim_path}")
        return None

    binding_api = UsdShade.MaterialBindingAPI(prim)
    bound_material, _rel = binding_api.ComputeBoundMaterial(materialPurpose="physics")
    if not bound_material or not bound_material.GetPrim().IsValid():
        print(f"[WARNING] no physics material bound to {prim_path}")
        return None

    material_prim = bound_material.GetPrim()
    if not material_prim.HasAPI(UsdPhysics.MaterialAPI):
        print(f"[WARNING] {material_prim.GetPath()} has no UsdPhysics.MaterialAPI")
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
    걸레 rigid body와 그리퍼 손가락 콜라이더에 바인딩된 Physics Material의 마찰 계수를
    점검한다. 값이 너무 낮으면(FRICTION_WARN_THRESHOLD 미만) 경고를 출력한다.

    조정 방법 (둘 중 하나):
      A) 기존 Physics Material의 staticFriction/dynamicFriction 값을 올린다.
         physics_material = UsdPhysics.MaterialAPI(material_prim)
         physics_material.GetStaticFrictionAttr().Set(0.8)
         physics_material.GetDynamicFrictionAttr().Set(0.7)
      B) 아예 새 Physics Material을 만들어 두 콜라이더(걸레, 손가락)에 모두 바인딩한다.
         (bind_high_friction_grip_material 이 이를 코드로 실행)
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
            print(f"[WARNING] {name}: friction too low ({min_friction:.2f} < {FRICTION_WARN_THRESHOLD}). "
                  f"See adjustment method A or B in the docstring above.")

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
    실제로 고마찰 Physics Material을 생성/갱신하고 걸레 콜라이더 + 그리퍼 손가락
    콜라이더 양쪽에 바인딩한다.

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
        print(f"[INFO] created {material_path}")

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
            print(f"[WARNING] {target_path}: no collider, skipping binding -> "
                  f"check the actual subtree with list_prim_subtree('{target_path}') first")
            skipped.append(target_path)
            continue

        collider_prim = stage.GetPrimAtPath(collider_path)
        binding_api = UsdShade.MaterialBindingAPI.Apply(collider_prim)
        binding_api.Bind(
            UsdShade.Material(material_prim),
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        print(f"[INFO] bound {collider_path} <- {material_path}")

    if skipped:
        print(f"[WARNING] could not bind (no collider): {skipped} "
              f"-> add a collider to these prims before setting friction")

    return material_path


# ==================================================================
# 6) 팔/그리퍼 조인트 drive stiffness/damping/maxForce 상향
# ==================================================================
def apply_arm_drive_settings(
    arm_prim_path: str = ARM_PRIM_PATH,
    stiffness: float = ARM_DRIVE_STIFFNESS,
    damping: float = ARM_DRIVE_DAMPING,
    max_force: float = ARM_DRIVE_MAX_FORCE,
):
    """
    arm_prim_path 하위 모든 조인트의 DriveAPI(angular/linear) stiffness/damping/
    maxForce를 일괄 상향한다. USD 임포트 시 기본 drive 값이 너무 약하면 position
    command를 내려도 접촉 저항(걸레를 쥐는 힘 등)을 못 이겨 목표각까지 못 가거나
    거기 도달해도 유지 힘이 없어 grasp effort가 거의 0으로 측정된다.

    사용자가 공유한 pick_and_place 레퍼런스 스크립트의 _setup_physics()와 동일한
    방식(모든 DriveAPI를 순회하며 값을 덮어씀). 팔+그리퍼 전체에 적용되므로
    RMPFlowController의 목표 추종 정밀도도 같이 좋아진다.
    """
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(arm_prim_path)
    if not root_prim.IsValid():
        print(f"[WARNING] invalid prim: {arm_prim_path}")
        return 0

    drive_count = 0
    for prim in Usd.PrimRange(root_prim):
        for dof_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(stiffness)
                drive.GetDampingAttr().Set(damping)
                drive.GetMaxForceAttr().Set(max_force)
                drive_count += 1

    print(f"[INFO] boosted {drive_count} joint drive(s) under {arm_prim_path}: "
          f"stiffness={stiffness}, damping={damping}, maxForce={max_force}")
    return drive_count


# ==================================================================
# 7) 그립 안정성용 PhysX Solver 반복 횟수 적용
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

    solverVelocityIterationCount: 마찰/반발 등 "속도" 관련 구속을 얼마나 반복
    계산할지 결정한다. 낮으면 마찰력이 상대 미끄러짐 속도를 충분히 감쇠시키지
    못해 grasp 이후에도 손잡이가 손가락 사이에서 서서히 미끄러져 빠짐(slip).

    두 값 모두 높일수록 안정적이지만 스텝당 연산 비용이 늘어나므로, 그립 대상
    (걸레 rigid body, 손가락 link)에만 국소적으로 적용하는 것을 권장한다.
    """
    stage = omni.usd.get_context().get_stage()
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[WARNING] invalid prim, skipping: {prim_path}")
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
        print("[ERROR] Press Play before running this script!")
    else:
        # 0. 대걸레가 아직 씬에 없으면 추가
        setup_mop_in_scene()

        # 1. 현재 걸레 좌표 확인
        get_prim_world_transform(MOP_MESH_PATH)

        # 2. pre-grasp / grasp pose 계산
        poses = compute_mop_grasp_poses()

        # 2.5. RigidBody/Collider가 애초에 붙어 있는지 확인 (마찰보다 먼저 확인해야 함)
        check_rigid_body_setup(MOP_PRIM_PATH)
        check_rigid_body_setup(f"{ARM_PRIM_PATH}/left_inner_finger")
        check_rigid_body_setup(f"{ARM_PRIM_PATH}/right_inner_finger")

        # 3. 마찰 사전 점검 (그립 전에 먼저 확인하는 걸 권장)
        inspect_grip_friction()

        # 3.5. 마찰 Material이 없다고 나오면 고마찰 Material을 만들어 바인딩
        bind_high_friction_grip_material()

        # 4. solver 반복 횟수 상향 (걸레 + 손가락)
        # apply_grasp_solver_settings([
        #     MOP_PRIM_PATH,
        #     f"{ARM_PRIM_PATH}/left_inner_finger",
        #     f"{ARM_PRIM_PATH}/right_inner_finger",
        # ])

        # 5. 팔을 poses["pre_grasp_pos"] -> poses["grasp_pos"] 로 이동시키는 것은
        #    floor_wipe_motion.py 의 run_mop_grasp_and_wipe_demo() 가 전체 시퀀스로 묶어서 실행함.

        print("[INFO] Uncomment the steps you need, or call run_mop_grasp_and_wipe_demo() "
              "from floor_wipe_motion.py for the full sequence.")
