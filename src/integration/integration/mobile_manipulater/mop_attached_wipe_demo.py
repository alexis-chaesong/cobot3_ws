"""
mop_attached_wipe_demo.py
---------------------------
격리 병동 비대면 로봇팔 프로젝트 - "옵션 A" 데모: 대걸레를 그리퍼에 처음부터
부착해두고(파지 시퀀스 없이), Nova Carter를 앞으로 ~1m 자율주행시키면서 동시에
대걸레로 바닥을 왔다갔다 쓱쓱 닦는 모션을 실행한다.

── 씬 구조 (mobile_manipulator_v2.usd) ───────────────────────────────
  로봇 팔+그리퍼 : /World/m0609_with_gripper        (독립 articulation)
  이동 베이스     : /World/Nova_Carter_ROS           (독립 articulation)
  두 articulation은 PhysicsFixedJoint(arm root_joint)로 연결되어 있어, Carter가
  움직이면 팔 base_link가 그대로 끌려온다. 즉 Carter를 주행시키면 팔 전체가 함께
  전진한다.

── 대걸레 부착 방식 ──────────────────────────────────────────────────
  Hospital Props의 실제 mop 에셋(SM_MopSet_01a/01b.usd)은 양동이+짜개+대걸레가 한
  Mesh로 합쳐진 대형 소품(최대 1.3m)이라 그리퍼에 붙이면 로봇 손에 양동이가 통째로
  매달린 것처럼 보인다. 대신 이미 검증된 절차적(procedural) 단순 대걸레(mgc.MOP_*
  규격의 손잡이+헤드)를 gripper_body 밑에 "고정 USD 자식"으로 붙인다. gripper_body는
  articulation link라, 그 자식이 되는 순간부터 gripper_body의 월드 변환을 그대로
  상속하므로 물리 조인트/그리퍼 닫기 없이도 단단히 붙어 미끄러짐/낙하가 성립하지
  않는다 (그립 실패 체크리스트가 통째로 필요 없어진다).

── 주행하며 닦기: 왜 "베이스 기준 좌표"가 필요한가 ──────────────────
  RMPFlowController는 "로봇 베이스 pose"를 기준으로 월드 목표점까지의 IK를 푼다.
  Carter가 전진해 base_link가 월드에서 이동하는데 RMPFlow가 예전 베이스 pose를
  그대로 쓰면 IK가 어긋나 팔이 엉뚱한 곳으로 간다. 그래서 매 스텝:
    1) 실제 base_link 월드 pose를 읽어 rmp_flow.set_robot_base_pose()로 갱신하고,
    2) 와이핑 지그재그 패턴을 "베이스 기준 로컬 좌표"로 표현한 뒤, 현재 베이스
       pose로 다시 월드 좌표로 변환해 목표점으로 준다.
  이렇게 하면 대걸레는 로봇을 기준으로 좌우로 쓱쓱 왕복하고, 그 왕복 패턴 자체가
  로봇의 전진에 실려 바닥에 띠 모양으로 닦인다. 목표점이 항상 베이스에 대해 같은
  거리에 있으므로 1m 내내 팔의 리치(reach) 안에 머문다.

── 이전 개정에서 고친 것 (그대로 유지) ─────────────────────────────
  (1) IndexError: pop from an empty deque -> 모든 스텝을 동기 app.update() 대신
      await app.next_update_async()로 밟는 async 구조.
  (2) 대걸레가 바닥 밑으로 내려가는 문제 -> 부착 직후 link_6 월드 z와 걸레 헤드
      바닥 z의 실제 차이를 측정해, 헤드 바닥이 항상 바닥 위 WIPE_HEAD_CLEARANCE
      만큼 뜨도록 목표 z를 역산.

의존성: mop_grasp_control.py(mop 규격/좌표 유틸), floor_wipe_motion.py(와이핑
웨이포인트 생성), rmpflow/m0609_rmpflow_controller.py(rmpflow 폴더 전부 재사용).
Nova Carter 파라미터(휠 조인트명/반지름/휠베이스)는 Isaac Sim 설치본의 표준값
(joint_wheel_left/right, r=0.14, base=0.4132)을 그대로 사용한다.

사용법 (Script Editor):
  1. mobile_manipulator_v2.usd 를 열고 Play(▶)
  2. 이 파일 전체를 Script Editor 에 붙여넣고 Run
     -> 맨 아래에서 run_mop_attached_wipe_demo() 가 async 태스크로 실행된다
        (Run 직후 즉시 반환되고, 모션은 백그라운드 코루틴으로 진행)
"""

import sys
import importlib
import asyncio
import numpy as np
import omni.usd
import omni.timeline
import omni.kit.app
from pxr import Usd, UsdGeom, UsdPhysics, Gf

MOBILE_MANIPULATOR_DIR = "/home/rokey/cobot3_ws/src/integration/integration/mobile_manipulater"
if MOBILE_MANIPULATOR_DIR not in sys.path:
    sys.path.insert(0, MOBILE_MANIPULATOR_DIR)

# ── Script Editor는 커널을 계속 살려두므로, 한 번 import된 모듈은 sys.modules에
#    캐시되어 파일을 다시 수정해도 옛 버전이 그대로 쓰인다(예: 나중에 추가한 함수가
#    "no attribute"로 뜨는 원인). 매번 최신 파일 내용을 쓰도록 관련 모듈 캐시를 지우고
#    새로 import한다. floor_wipe_motion이 내부에서 mop_grasp_control을 import하므로,
#    의존 대상인 mop_grasp_control을 먼저 비운다. ──
for _stale in ("floor_wipe_motion", "mop_grasp_control"):
    sys.modules.pop(_stale, None)
import mop_grasp_control as mgc
import floor_wipe_motion as fwm
importlib.reload(mgc)
importlib.reload(fwm)

RMPFLOW_DIR = "/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow"
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

# ────────────────────────────────────────────────────────────────
# 설정값 - 대걸레 부착 / 팔
# ────────────────────────────────────────────────────────────────
GRIPPER_BODY_PATH = f"{mgc.ARM_PRIM_PATH}/gripper_body"   # tool0 <- quick_changer <- angle_bracket <- gripper_body,
                                                            # 전부 fixed joint라 link_6/tool0 대비 위치가 항상 고정.
ATTACHED_MOP_PATH = f"{GRIPPER_BODY_PATH}/AttachedMop"
LINK6_PATH = f"{mgc.ARM_PRIM_PATH}/link_6"                 # RMPFlow가 실제로 위치를 맞추는 end-effector 프레임
BASE_LINK_PATH = f"{mgc.ARM_PRIM_PATH}/base_link"          # RMPFlow 베이스 pose 기준 (Carter 위에 fixed)

# 와이핑 시 대걸레 "헤드 바닥"이 바닥(z=0)에서 떠 있어야 하는 높이 [m].
# 5cm 여유 = RMPFlow의 z 언더슛/자세 흔들림이 있어도 헤드가 바닥을 뚫지 않도록.
WIPE_HEAD_CLEARANCE = 0.05

# 와이핑 패턴: 로봇 기준 "좌우(Y)"로 왕복 스크럽. 전진(X)은 Carter 주행이 담당하므로
# 패턴 자체의 옆이동(lateral_step)은 0으로 두고 제자리 좌우 왕복만 시킨다.
WIPE_SCRUB_AXIS = np.array([0.0, 1.0, 0.0])    # 스트로크 방향 = 로봇 좌우
WIPE_ADVANCE_AXIS = np.array([1.0, 0.0, 0.0])  # (사용 안 함, lateral_step=0)
WIPE_STROKE_LENGTH = 0.20      # 좌우 왕복 폭 [m]
WIPE_LATERAL_STEP = 0.0        # 패턴 자체 전진 0 (전진은 Carter가 함)
WIPE_NUM_STROKES = 6           # 왕복 스트로크 수 (많을수록 오래 닦음)
WIPE_STEPS_PER_WAYPOINT = 60   # 웨이포인트당 물리 스텝 수
WIPE_ORIENTATION_WXYZ = np.array([0.0, 1.0, 0.0, 0.0])   # top-down (지렛대 헤드가 바닥과 평행 유지)

# 부착용으로 팔을 top-down grasp 자세로 보내는 스텝 수 (mop이 아래를 향해 매달리게)
ATTACH_PRE_GRASP_STEPS = 200
ATTACH_GRASP_STEPS = 200
ATTACH_SETTLE_STEPS = 40

DRIVE_STIFFNESS = fwm.DRIVE_STIFFNESS
DRIVE_DAMPING = fwm.DRIVE_DAMPING
DRIVE_MAX_FORCE = fwm.DRIVE_MAX_FORCE

# ────────────────────────────────────────────────────────────────
# 설정값 - Nova Carter 주행
# ────────────────────────────────────────────────────────────────
CARTER_PRIM_PATH = "/World/Nova_Carter_ROS"
CHASSIS_LINK_PATH = f"{CARTER_PRIM_PATH}/chassis_link"
# Isaac Sim 설치본 표준 Nova Carter 파라미터 (source/.../wheeled_robot.py, test_carter_v2.py):
WHEEL_JOINT_NAMES = ["joint_wheel_left", "joint_wheel_right"]
WHEEL_RADIUS = 0.14            # [m]
WHEEL_BASE = 0.4132            # 좌우 휠 간 거리 [m]

FORWARD_DISTANCE = 1.0         # 전진 목표 거리 [m]
BASE_MAX_LINEAR_SPEED = 0.25   # 주행 속도 상한 [m/s]
# 표준 unicycle 규약상 양(+) 휠속도 = 전진. 이 씬의 휠 조인트 축 방향이 반대라
# 뒤로 가면 아래 부호를 자동 보정한다(BASE_CALIB_STEPS 안에서 진행이 음수면 -1로 뒤집음).
BASE_CALIB_STEPS = 20
# ────────────────────────────────────────────────────────────────


def _matrix_from_pos_quat(pos, quat_wxyz) -> Gf.Matrix4d:
    """(position, quaternion wxyz) -> Gf.Matrix4d 월드 변환 행렬.

    mgc.get_prim_world_transform()이 Gf.Transform(matrix)로 pos/quat를 뽑아내는 것과
    정확히 반대 방향의 연산이라, Gf.Transform을 그대로 왕복 사용해 두 함수 사이의
    관례(축 순서, 회전 합성)가 항상 일치하도록 한다.
    """
    quat = Gf.Quatd(float(quat_wxyz[0]), Gf.Vec3d(float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])))
    xf = Gf.Transform()
    xf.SetTranslation(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.SetRotation(Gf.Rotation(quat))
    return xf.GetMatrix()


def _read_world_pose(prim_path: str):
    """prim의 현재 월드 (pos np3, quat wxyz np4)를 조용히(print 없이) 반환한다.

    mgc.get_prim_world_transform은 매 호출마다 로그를 찍어 매 스텝 호출에는 부적합하다.
    주행 루프에서 base_link/chassis pose를 매 프레임 읽기 위한 경량 버전.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation()
    q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def _find_articulation_root(root_path: str) -> str:
    """root_path 하위에서 ArticulationRootAPI가 붙은 prim 경로를 찾는다.

    Nova Carter는 ArticulationRootAPI가 chassis_link에 있을 수 있어, SingleArticulation에
    넘길 정확한 루트 경로를 자동 탐색한다. 못 찾으면 root_path를 그대로 반환.
    """
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        print(f"[WARNING] invalid prim: {root_path}")
        return root_path
    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return root_path
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            resolved = str(prim.GetPath())
            print(f"[INFO] articulation root under {root_path} -> {resolved}")
            return resolved
    print(f"[WARNING] no ArticulationRootAPI found under {root_path}, using it as-is")
    return root_path


def _build_mop_geometry(stage: Usd.Stage, root_path: str):
    """AttachedMop 루트 밑에 손잡이(Cylinder) + 헤드(Cube) 시각 지오메트리만 만든다.

    mgc.setup_mop_in_scene()과 크기(반지름/길이/헤드 사이즈)는 동일하게 맞추되,
    RigidBody/Mass/Collision은 일부러 붙이지 않는다(gripper_body의 kinematic 자식).
    좌표계: 루트 원점 = 걸레 base(= 헤드 바닥). 헤드는 z=0~head_z, 손잡이는 그 위로.
    """
    handle_radius = mgc.MOP_HANDLE_RADIUS
    handle_length = mgc.MOP_HANDLE_LENGTH
    head_size = mgc.MOP_HEAD_SIZE

    head_path = f"{root_path}/head"
    UsdGeom.Cube.Define(stage, head_path)
    head_prim = stage.GetPrimAtPath(head_path)
    head_xf = UsdGeom.Xformable(head_prim)
    head_xf.ClearXformOpOrder()
    head_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(head_size[2]) / 2.0))
    head_xf.AddScaleOp().Set(Gf.Vec3f(
        float(head_size[0]) / 2.0, float(head_size[1]) / 2.0, float(head_size[2]) / 2.0))
    UsdGeom.Gprim(head_prim).CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.4, 0.8)])

    handle_path = f"{root_path}/handle"
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
    handle_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(head_size[2]) + handle_length / 2.0))
    UsdGeom.Gprim(handle_prim).CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.5, 0.3)])

    print(f"[INFO] built visual-only mop geometry under {root_path} "
          f"(no RigidBody/Collision - kinematic child of {GRIPPER_BODY_PATH})")


async def _drive_arm_to(arm, controller, app, target_pos, target_orient, steps):
    """RMPFlowController로 (고정된) target_pos/orient를 향해 steps번 스텝 (async).
    부착 단계처럼 베이스가 정지해 있을 때 쓰는 단순 버전."""
    tp = np.array(target_pos)
    to = np.array(target_orient)
    for _ in range(steps):
        action = controller.forward(
            target_end_effector_position=tp,
            target_end_effector_orientation=to,
        )
        arm.apply_action(action)
        await app.next_update_async()


async def attach_mop_to_gripper_async(arm, controller, app):
    """검증된 top-down grasp 자세로 팔을 이동시켜 gripper_body 기준 부착 좌표를
    역산하고, 대걸레를 gripper_body 밑에 고정 자식으로 붙인다. 그리퍼를 닫지 않는다.

    Returns:
        poses: mgc.compute_mop_grasp_poses() dict (와이핑 시작 XY 계산에 재사용).
    """
    stage = omni.usd.get_context().get_stage()

    print("[STEP A-0] setting up reference mop in scene (temporary, used only for its geometry/pose)")
    mgc.setup_mop_in_scene()
    poses = mgc.compute_mop_grasp_poses()

    print("[STEP A-1] moving to pre-grasp pose (reusing validated grasp coordinates)")
    await _drive_arm_to(arm, controller, app, poses["pre_grasp_pos"], poses["orientation_wxyz"], ATTACH_PRE_GRASP_STEPS)

    print("[STEP A-2] moving to grasp pose (tool pointing down so mop hangs toward floor)")
    await _drive_arm_to(arm, controller, app, poses["grasp_pos"], poses["orientation_wxyz"], ATTACH_GRASP_STEPS)

    for _ in range(ATTACH_SETTLE_STEPS):
        await app.next_update_async()

    print("[STEP A-3] reading gripper_body world transform at this pose")
    gripper_pos, gripper_quat, _ = mgc.get_prim_world_transform(GRIPPER_BODY_PATH)

    print("[STEP A-4] computing mop's local offset relative to gripper_body")
    gripper_world_m = _matrix_from_pos_quat(gripper_pos, gripper_quat)
    mop_world_m = _matrix_from_pos_quat(mgc.MOP_SPAWN_POSITION, mgc.MOP_SPAWN_ORIENTATION_WXYZ)
    local_m = mop_world_m * gripper_world_m.GetInverse()

    print("[STEP A-5] removing temporary free-standing reference mop")
    stage.RemovePrim(mgc.MOP_PRIM_PATH)

    if stage.GetPrimAtPath(ATTACHED_MOP_PATH).IsValid():
        print(f"[INFO] {ATTACHED_MOP_PATH} already exists, removing before recreating (idempotent re-run)")
        stage.RemovePrim(ATTACHED_MOP_PATH)

    print(f"[STEP A-6] creating attached mop under {ATTACHED_MOP_PATH}")
    UsdGeom.Xform.Define(stage, ATTACHED_MOP_PATH)
    attached_prim = stage.GetPrimAtPath(ATTACHED_MOP_PATH)
    attached_xf = UsdGeom.Xformable(attached_prim)
    attached_xf.ClearXformOpOrder()
    attached_xf.AddTransformOp().Set(local_m)
    _build_mop_geometry(stage, ATTACHED_MOP_PATH)

    print(f"[INFO] mop is now a fixed USD child of {GRIPPER_BODY_PATH} - "
          f"no grasp/close-gripper step needed, it cannot slip or drop.")
    return poses


def _compute_wipe_ee_z():
    """부착된 걸레 헤드 바닥이 바닥 위 WIPE_HEAD_CLEARANCE 에 오도록 하는 link_6 목표 z.

    부착 직후(팔이 top-down grasp 자세) link_6 월드 z와 AttachedMop 루트 원점(=헤드
    바닥) 월드 z의 실제 차이를 측정한다. 와이핑 내내 자세를 top-down으로 유지하므로
    이 수직 관계가 보존된다 -> 목표 link_6 z = clearance + (link6_z - mop_base_z).
    """
    ee_pos, _, _ = mgc.get_prim_world_transform(LINK6_PATH)
    mop_base_pos, _, _ = mgc.get_prim_world_transform(ATTACHED_MOP_PATH)
    ee_above_base = float(ee_pos[2]) - float(mop_base_pos[2])
    wipe_ee_z = WIPE_HEAD_CLEARANCE + ee_above_base
    print(f"[INFO] link_6 z={ee_pos[2]:.4f}, mop base(head bottom) z={mop_base_pos[2]:.4f}, "
          f"link_6 above head bottom={ee_above_base:.4f}")
    print(f"[INFO] wipe target link_6 z={wipe_ee_z:.4f} (keeps head bottom {WIPE_HEAD_CLEARANCE:.3f} m above floor)")
    return wipe_ee_z


def _setup_carter():
    """Nova Carter articulation 연결 + 휠 조인트 인덱스 확보.

    Returns:
        (carter SingleArticulation, [left_idx, right_idx]) 또는 실패 시 (None, None).
    """
    carter_root = _find_articulation_root(CARTER_PRIM_PATH)
    carter = SingleArticulation(prim_path=carter_root, name="nova_carter_wipe_demo")
    carter.initialize()
    dof_names = list(carter.dof_names)
    print(f"[INFO] Carter dof_names={dof_names}")

    wheel_idx = []
    for name in WHEEL_JOINT_NAMES:
        if name in dof_names:
            wheel_idx.append(dof_names.index(name))
        else:
            # fallback: 이름에 'wheel'이 들어가고 caster가 아닌 조인트 탐색
            cand = [i for i, n in enumerate(dof_names) if "wheel" in n.lower() and "caster" not in n.lower()]
            if len(cand) >= 2:
                print(f"[WARNING] '{name}' not found, falling back to wheel-like dofs {cand[:2]}")
                wheel_idx = cand[:2]
                break
            print(f"[ERROR] wheel joint '{name}' not found in Carter dof_names")
            return None, None
    print(f"[INFO] Carter wheel dof indices={wheel_idx} ({WHEEL_JOINT_NAMES})")
    return carter, wheel_idx


def _wheel_velocity_action(wheel_idx, linear_speed, angular_speed=0.0):
    """unicycle -> 좌/우 휠 각속도 ArticulationAction (differential_controller.py와 동일 식).

        omega_L = (2V - w*b) / (2r),  omega_R = (2V + w*b) / (2r)
    """
    omega_l = (2.0 * linear_speed - angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    omega_r = (2.0 * linear_speed + angular_speed * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    return ArticulationAction(
        joint_velocities=np.array([omega_l, omega_r]),
        joint_indices=np.array(wheel_idx),
    )


async def run_mop_attached_wipe_demo_async():
    """대걸레 부착 -> Carter를 앞으로 ~1m 주행시키며 동시에 좌우로 쓱쓱 와이핑."""
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        print("[ERROR] Press Play before running this!")
        return False

    app = omni.kit.app.get_app()

    print("[STEP 1] connecting to arm articulation:", mgc.ARM_PRIM_PATH)
    arm = SingleArticulation(prim_path=mgc.ARM_PRIM_PATH, name="m0609_attached_wipe_demo")
    arm.initialize()
    mgc.apply_arm_drive_settings(
        arm_prim_path=mgc.ARM_PRIM_PATH,
        stiffness=DRIVE_STIFFNESS,
        damping=DRIVE_DAMPING,
        max_force=DRIVE_MAX_FORCE,
    )

    controller = RMPFlowController(
        name="rmpflow_attached_wipe_demo_ctrl",
        robot_articulation=arm,
        physics_dt=1.0 / 60.0,
    )
    controller.reset()
    mgc.sync_rmpflow_base_pose(controller)

    print("[STEP 2] connecting to Nova Carter (mobile base)")
    carter, wheel_idx = _setup_carter()
    if carter is None:
        print("[ERROR] could not set up Carter drive - aborting")
        return False

    print("[STEP 3] attaching mop to gripper_body (no grasp sequence)")
    poses = await attach_mop_to_gripper_async(arm, controller, app)

    print("[STEP 4] measuring attached mop height and generating base-frame wipe pattern")
    wipe_ee_z = _compute_wipe_ee_z()
    # 초기(정지) 베이스 pose 기준으로 월드 지그재그 웨이포인트를 만든 뒤, 베이스 로컬
    # 좌표로 변환해 저장한다. 이후 매 스텝 현재 베이스 pose로 다시 월드로 변환한다.
    world_waypoints = fwm.generate_floor_wipe_waypoints(
        start_xy=poses["grasp_pos"][:2],
        height_above_floor=wipe_ee_z,
        stroke_length=WIPE_STROKE_LENGTH,
        lateral_step=WIPE_LATERAL_STEP,
        num_strokes=WIPE_NUM_STROKES,
        forward_axis=WIPE_SCRUB_AXIS,
        lateral_axis=WIPE_ADVANCE_AXIS,
    )

    base0_pos, base0_quat = _read_world_pose(BASE_LINK_PATH)
    base0_m = _matrix_from_pos_quat(base0_pos, base0_quat)
    base0_inv = base0_m.GetInverse()
    local_waypoints = [base0_inv.Transform(Gf.Vec3d(float(wp[0]), float(wp[1]), float(wp[2])))
                       for wp in world_waypoints]

    # 전진 진행도(progress)는 초기 베이스 forward(월드 +X를 base0로 회전) 축 위 투영으로 측정
    forward0_vec = base0_m.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    forward0 = np.array([forward0_vec[0], forward0_vec[1], forward0_vec[2]])
    forward0 = forward0 / (np.linalg.norm(forward0) + 1e-9)
    chassis0_pos, _ = _read_world_pose(CHASSIS_LINK_PATH)

    total_arm_steps = len(local_waypoints) * WIPE_STEPS_PER_WAYPOINT
    dt = 1.0 / 60.0
    base_speed = min(FORWARD_DISTANCE / (total_arm_steps * dt), BASE_MAX_LINEAR_SPEED)
    print(f"[INFO] driving Carter forward {FORWARD_DISTANCE} m at {base_speed:.3f} m/s "
          f"while wiping ({len(local_waypoints)} waypoints x {WIPE_STEPS_PER_WAYPOINT} steps)")

    print("[STEP 5] wiping floor (side-to-side) while Carter drives forward ~1 m")
    sign = 1.0
    step_counter = 0
    progress = 0.0
    reached = False

    for i, local_wp in enumerate(local_waypoints):
        print(f"[WIPE] waypoint {i + 1}/{len(local_waypoints)} (base-local)")
        for _ in range(WIPE_STEPS_PER_WAYPOINT):
            # (A) 이동한 실제 베이스 pose로 RMPFlow 베이스 갱신 + 목표점 재계산
            base_pos, base_quat = _read_world_pose(BASE_LINK_PATH)
            controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
            base_now_m = _matrix_from_pos_quat(base_pos, base_quat)
            world_target_v = base_now_m.Transform(local_wp)
            world_target = np.array([world_target_v[0], world_target_v[1], world_target_v[2]])

            action = controller.forward(
                target_end_effector_position=world_target,
                target_end_effector_orientation=WIPE_ORIENTATION_WXYZ,
            )
            arm.apply_action(action)

            # (B) Carter 전진 명령 (목표 거리 도달 시 정지)
            chassis_pos, _ = _read_world_pose(CHASSIS_LINK_PATH)
            progress = float(np.dot(chassis_pos - chassis0_pos, forward0))
            if not reached and abs(progress) < FORWARD_DISTANCE:
                carter.apply_action(_wheel_velocity_action(wheel_idx, sign * base_speed))
            else:
                if not reached:
                    print(f"[INFO] Carter reached ~{FORWARD_DISTANCE} m (progress={progress:.3f}), stopping base")
                reached = True
                carter.apply_action(_wheel_velocity_action(wheel_idx, 0.0))

            await app.next_update_async()

            # (C) 초반 몇 스텝 안에 진행 방향이 반대면 휠 부호 자동 보정 (씬별 휠축 방향 대응)
            step_counter += 1
            if step_counter == BASE_CALIB_STEPS and progress < -0.01:
                sign = -1.0
                print(f"[INFO] base moved backward (progress={progress:.3f}); flipping wheel direction sign")

    # 마무리: 베이스 정지
    carter.apply_action(_wheel_velocity_action(wheel_idx, 0.0))
    print(f"[WIPE] all waypoints reached (final forward progress={progress:.3f} m)")
    print("[DONE] run_mop_attached_wipe_demo finished")
    return True


def run_mop_attached_wipe_demo():
    """Script Editor 진입점. async 태스크를 스케줄하고 즉시 반환한다.

    asyncio.ensure_future로 코루틴을 이벤트 루프에 얹으므로, 스텝은 전부
    next_update_async()로 진행되어 동기 app.update() 블로킹 루프에서 나던
    'IndexError: pop from an empty deque'가 발생하지 않는다.
    """
    return asyncio.ensure_future(run_mop_attached_wipe_demo_async())


if __name__ == "__main__":
    run_mop_attached_wipe_demo()
