"""
10_1_carter_hospital_spray_nav.py
====================================================================
10_ 판에서 두 가지만 바뀜 (동작 로직 동일) :
  1) 환경/맵을 carter_navigation/maps/map/modified_hospital.usd 로 교체
     (Nav2 맵은 같은 폴더 modified_hospital_map.yaml/png).
  2) Isaac 스폰 = "docking_station_1" 큐브 위치(Isaac 월드 x=8.9096, y=0, yaw=0)로 이동.
     modified_hospital 맵은 이 USD 에서 생성된 커스텀 맵이라 map 프레임 = Isaac 월드 프레임으로
     가정 → Nav2 amcl initial_pose 도 동일 (8.9096, 0, yaw 0). NAV2_INITIAL_POSE 참고.
     (첫 실행 후 /amcl_pose 가 이와 다르면 그 값으로 params yaml initial_pose 를 보정할 것)
--------------------------------------------------------------------
통합 단독 스크립트 : modified_hospital 환경 + Nova Carter(ROS 구동) + m0609 노즐 팔.
세 가지 주행 모드 (DRIVE_MODE 로 전환) :

  [handoff] Nav2 ↔ 스윕 자동 핸드오프 (기본, 권장)
    - STANDBY : Nav2 가 스윕 시작점까지 이동(스크립트는 /cmd_vel 을 건드리지 않음, 팔 stow).
    - /start_sweep(Bool) 수신 → SWEEP : 멈춤 → 위아래 소독 → MOVE_DISTANCE(0.25 m) 직진 → 반복
      (누적 FORWARD_DISTANCE), heading-hold 로 직진 유지. 스크립트가 /cmd_vel 로 주행.
    - 완료 → /sweep_done(Bool) 발행 → STANDBY 복귀. Nav2 와 /cmd_vel 을 "번갈아" 써 충돌 없음.
    - 조율 : commander/spray_waypoint_mission (NavigateToPose → /start_sweep → /sweep_done 대기).

  [sweep] Play 즉시 stop-and-go 스윕(트리거 없이). 이땐 Nav2 Goal 주지 말 것.
  [nav2]  기존 — 주행은 Nav2 가, 팔은 /spray_active(Bool) True 구간에서만 소독.

공통 : 센서/TF 는 Nova_Carter_ROS 내장 OmniGraph 가 발행, /clock 은 이 스크립트가 발행
  (Nova_Carter_ROS 는 clock 미발행 → use_sim_time Nav2 노드가 멈추므로 필수).

실행 (handoff)
  1) 이 스크립트 실행(python.sh) → GUI 에서 Carter 위치 확인 후 Play ▶
  2) 별도 터미널 : Nav2 bringup
     (carter_navigation ... map:=<carter_navigation>/maps/map/modified_hospital_map.yaml)
     → amcl set_initial_pose=true 면 params 의 initial_pose(=NAV2_INITIAL_POSE)로 자동 정렬,
       아니면 RViz 2D Pose Estimate 로 docking_station_1(8.9096, 0, yaw 0) 지정.
  3) 별도 터미널 : ros2 run commander spray_waypoint_mission
     (스윕 시작점들을 순회하며 이동→/start_sweep→/sweep_done 대기→다음)

튜닝
  - DRIVE_MODE / MOVE_DISTANCE / FORWARD_DISTANCE / FORWARD_SPEED : 주행 파라미터
  - STROKES_PER_WIPE : 정지 1회당 위아래 스트로크 수(2 = 올라갔다 내려오기)
  - KP_YAW/KP_LAT ↑ : 더 곧게(진동하면 ↓)
  - CARTER_START_POSE : Carter 스폰 (Nav2 병행 시 AMCL 초기위치와 일치시킬 것)
  - J1_OFFSET, Z_LOW/HIGH, J5_FLICK, S_CRUISE : 8/9_wall_spray 와 동일
====================================================================
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ROS2 bridge 확장 활성화 (Carter 내장 그래프 발행 + Nav2 구동에 필수)
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg

import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist   # stop-and-go 자체주행 /cmd_vel 발행
from rosgraph_msgs.msg import Clock   # /clock 직접 발행 (Nova_Carter_ROS 는 clock 미발행)

_THIS_DIR = Path(__file__).resolve().parent

# ────────────────────────────────────────────────────────────────
#  A. 경로 / 상수
# ────────────────────────────────────────────────────────────────
# 환경/맵 : Nav2 와 공유하는 modified_hospital 씬 (Nav2 맵 = 같은 폴더 modified_hospital_map.yaml).
HOSPITAL_USD = ("/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/navigation/"
                "carter_navigation/maps/map/modified_hospital.usd")
NOZZLE_USD = "/home/rokey/cobot3_ws/src/integration/integration/m0609_with_nozzle.usd"
CARTER_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
              "Assets/Isaac/5.1/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd")

ROBOT_PRIM_PATH = "/World/m0609"
EE_FRAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/link_6"
BASE_LINK_PATH = f"{ROBOT_PRIM_PATH}/base_link"
ROOT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/root_joint"
NOZZLE_BASE_PATH = f"{ROBOT_PRIM_PATH}/nozzle_base_link"   # 분사 노즐 링크 : 로컬 +Z = 분사축
NOZZLE_TIP_LOCAL = 0.142                                    # nozzle_base_link 로컬 팁 오프셋 [m] (분사 원점)

CARTER_PRIM_PATH = "/World/Nova_Carter_ROS"
CHASSIS_LINK_PATH = f"{CARTER_PRIM_PATH}/chassis_link"
MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)     # chassis 기준 팔 장착
CHASSIS_MASS = 150.0

# ★ Carter 스폰 : 우리가 정한 초기 위치 (18.5, 0). 우측 세로 복도 하단(free space)이며 스윕
#   시작 WP1(18.5, 8.0)과 +Y 로 일직선. z 는 바닥 위 살짝 띄운 스폰 높이. yaw=0(+X 향).
#   AMCL initial_pose(NAV2_INITIAL_POSE)와 반드시 일치시킬 것.
CARTER_START_POSE = dict(x=18.5, y=0.0, z=0.05, yaw_deg=0.0)

# Nav2(amcl) 초기 포즈 : 위 스폰의 map 프레임 좌표. modified_hospital 맵은 이 USD 에서 생성된
#   커스텀 맵이라 map 프레임 = Isaac 월드 프레임 가정 → CARTER_START_POSE 와 동일 x,y,yaw.
#   carter_navigation_params.yaml 의 amcl: initial_pose: {x,y,yaw} 를 이 값으로 맞출 것.
#   (첫 bringup 후 /amcl_pose 가 다르면 그 실측값으로 params yaml 을 보정)
NAV2_INITIAL_POSE = dict(x=18.5, y=0.0, yaw=0.0)

URDF_PATH = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
NOZZLE_OFFSET = 0.1392

# --- 벽 겨냥 (베이스 로컬) : 8/9_wall_spray 와 동일 ---
WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80

J1_INDEX = 0
J1_OFFSET = -np.pi / 2       # 오른쪽 90° → 옆 벽 소독
J5_INDEX = 4
J5_FLICK = -0.5

# ═══════════════════ 속도 튜닝 (빠르되 관성 전복 방지) — 팔 모션 ═══════════════════
# S_CRUISE↑ : 팔 위아래 와이프가 빨라짐. 너무 크면 CoM 급변으로 베이스가 흔들림.
#   (와이프 실제 rad/step 은 MAX_JOINT_STEP 로도 상한 → 둘을 함께 올려야 실제로 빨라짐)
S_CRUISE = 2.0          # 와이프 순항속도(정규화 s∈[-1,1]/s). 기존 0.90 → 1.30
S_ACCEL = 3.0           # 와이프 가감속. 기존 2.00 → 3.0
S_HOLD_STEPS = 6         # 끝단(위/아래) 정지 유지 스텝 — 방향전환 충격 완화(전복 방지)

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

SPRAY_TOPIC = "/spray_active"
# True  : /spray_active 무시하고 "항상" 위아래 와이프 (= 9_carter_wall_spray 와 동일 동작)
# False : /spray_active True 구간에서만 와이프 (정지-분사 : 주행 중엔 stow, waypoint 정지시 분사)
SPRAY_ALWAYS_ON = True

# ═══════════════════ 분사 파티클 FX (경량 탄도 풀, 순수 시각·물리 없음) ═══════════════════
# WIPE(팔이 벽에서 위아래 소독) 구간에서만 노즐 끝(+Z)으로 청록 미스트를 방출. 튜닝용 노브.
SPRAY_FX_ON    = True            # 파티클 시각효과 on/off (False 면 오버헤드 0)
SPRAY_MAX      = 700             # 풀 최대 입자 수(성능↔밀도)
SPRAY_RATE     = 12              # 스텝당 방출 수(밀도)
SPRAY_SPEED    = 2.5             # 초기 분사 속도 [m/s] (도달거리↑)
SPRAY_CONE_DEG = 120.0            # 분사 원뿔 반각 [deg] (퍼짐)
SPRAY_LIFETIME = 0.5            # 입자 수명 [s] (길수록 멀리)
SPRAY_GRAVITY  = -2.0            # 낙하 가속(미스트감) [m/s^2]
SPRAY_SIZE     = 0.012           # 물방울 기본 반경(width) [m]
SPRAY_OPACITY  = 0.6             # 반투명도 (0~1)
SPRAY_COLOR    = (0.55, 0.85, 1.0)  # 청록 소독약 RGB

# 주행 중 stow 자세 : 팔을 접어 "중앙·낮게" → 측면 CoM 치우침 제거 → 베이스 안정.
# (분사 자세 q_of_s 는 J1_OFFSET 로 옆으로 뻗어 주행 중이면 베이스를 돌려버림)
STOW_Q = np.array([0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
# stow↔분사 급전환 완화 + 팔 최대 각속도 상한 : 관절 목표를 스텝당 이 값 이하로만 이동.
# ↑ 하면 팔 전체가 빨라지지만 CoM 급변으로 전복 위험 ↑. 와이프를 실제로 빠르게 하려면 S_CRUISE 와 함께 ↑.
MAX_JOINT_STEP = 0.06    # rad / physics step. 기존 0.04 → 0.06 (팔 모션 속도 ↑, 전복 나면 ↓)

# ── 주행 모드 ──
#   "handoff" : Nav2 가 스윕 시작점까지 이동 → /start_sweep 수신 → 스크립트가 stop-and-go 스윕
#               → 끝나면 /sweep_done 발행 → 다시 STANDBY(Nav2 대기). Nav2 와 /cmd_vel 을 번갈아 씀
#               (동시 사용이 없어 충돌 없음). 미션은 commander/spray_waypoint_mission 이 조율.
#   "sweep"   : Play 즉시 stop-and-go 스윕(트리거 없이). 이땐 Nav2 Goal 주지 말 것.
#   "nav2"    : 기존 — 주행은 Nav2 가, 팔은 /spray_active 게이트.
DRIVE_MODE = "handoff"
CMD_VEL_TOPIC = "/cmd_vel"
START_SWEEP_TOPIC = "/start_sweep"   # (handoff) 미션노드 → 스크립트 : 스윕 시작 트리거 (Bool)
SWEEP_DONE_TOPIC = "/sweep_done"     # (handoff) 스크립트 → 미션노드 : 스윕 완료 통지 (Bool)
STROKES_PER_WIPE = 2      # "위아래 1회" = 올라가기(1) + 내려오기(2)
MOVE_DISTANCE = 0.25      # 한 번에 전진할 거리 [m]
FORWARD_DISTANCE = 10.5   # 한 스윕 전진 거리 [m]. modified_hospital 우측 복도 : start_y 8.0 →
                          # end_y 18.5 (맵 벽 y=20.33 까지 1.8m 여유). 구 맵 21m 는 새 맵(y≤20.3) 초과.
# ═══════════════════ 속도 튜닝 (빠르되 관성 전복 방지) — 베이스 주행 ═══════════════════
FORWARD_SPEED = 0.35      # 전진 순항속도 [m/s]. 기존 0.20 → 0.35 (더 빠르게). 전복 나면 ↓
# ★ 관성 전복 방지 : /cmd_vel 선형속도를 이 가속도로 "천천히" 올리고/내림(급출발·급정지 금지).
#   낮출수록 부드럽지만 굼뜸. FORWARD_SPEED 를 올릴수록 이 값도 적당히 확보해야 안 넘어짐.
FORWARD_ACCEL = 0.50     # 선형 가감속 상한 [m/s^2] (publish_cmd 에서 램프)
KP_YAW = 2.5             # 헤딩 오차 → 각속도 [1/s]  (직진 유지)
KP_LAT = 1.2             # 측면 이탈 → 각속도 [1/(m·s)]
W_MAX = 0.8             # 각속도 명령 상한 [rad/s]


# ────────────────────────────────────────────────────────────────
#  B. 유틸 - 수학 (8/9_wall_spray 와 동일)
# ────────────────────────────────────────────────────────────────
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
    R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    return matrix_to_quat_wxyz(R)


def local_to_world(base_pos, base_quat, p_local):
    return base_pos + quat_to_matrix(base_quat) @ np.asarray(p_local)


def aim_link6_world(base_pos, base_quat, ori_quat, z):
    world_offset = quat_to_matrix(ori_quat) @ np.array([0.0, 0.0, NOZZLE_OFFSET])
    return local_to_world(base_pos, base_quat, [WALL_X, AIM_Y, z]) - world_offset


def read_world_pose(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation(); q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ────────────────────────────────────────────────────────────────
#  C. 씬 구성 : hospital + Carter + 노즐 팔
# ────────────────────────────────────────────────────────────────
def build_scene(my_world):
    stage = omni.usd.get_context().get_stage()

    # (1) /World + hospital 환경
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(HOSPITAL_USD)      # → /World/hospital

    # (2) Nova Carter (payload) at CARTER_START_POSE
    #  ★ payload 루트(nova_carter_ros2_sensors)는 회전을 xformOp:orient(쿼터니언)으로 authoring →
    #    XformCommonAPI 는 이런 prim 을 "비호환"으로 보고 SetTranslate/SetRotate 가 둘 다 조용히
    #    실패(무시)한다. 그러면 로봇이 기본 translate (0,0,0) 에 남아 스폰이 (8.9,0) 이면 원점에
    #    "엉뚱하게" 스폰됨(스폰이 (0,0) 이면 기본값과 같아 안 보였을 뿐). → op 를 비우고 단일
    #    transform 행렬로 명시 배치(아래 팔 배치와 동일한, op 종류에 무관한 확실한 방식).
    carter_prim = stage.DefinePrim(CARTER_PRIM_PATH, "Xform")
    carter_prim.GetPayloads().AddPayload(CARTER_URL)
    for _ in range(120):
        simulation_app.update()
    carter_xf = UsdGeom.Xformable(carter_prim)
    carter_xf.ClearXformOpOrder()
    carter_m = Gf.Matrix4d().SetRotate(
        Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(CARTER_START_POSE["yaw_deg"])))
    carter_m.SetTranslateOnly(
        Gf.Vec3d(CARTER_START_POSE["x"], CARTER_START_POSE["y"], CARTER_START_POSE["z"]))
    carter_xf.AddTransformOp().Set(carter_m)
    simulation_app.update()
    print(f"[LOAD] hospital={HOSPITAL_USD}\n[LOAD] carter @ {CARTER_START_POSE}")

    chassis = stage.GetPrimAtPath(CHASSIS_LINK_PATH)
    if not chassis.IsValid():
        print(f"[FATAL] {CHASSIS_LINK_PATH} 없음 — Carter 로드 실패")
        return False
    # 실제 배치 확인 : chassis 월드 좌표가 CARTER_START_POSE 와 일치해야 함(안 맞으면 스폰 실패)
    _wp = Gf.Transform(UsdGeom.Xformable(chassis)
                       .ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetTranslation()
    print(f"[SPAWN] chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f})  "
          f"(목표 x={CARTER_START_POSE['x']}, y={CARTER_START_POSE['y']})")
    UsdPhysics.MassAPI.Apply(chassis).CreateMassAttr(CHASSIS_MASS)

    # (3) 노즐 팔 + chassis 위 정렬 배치
    world_prim.GetReferences().AddReference(NOZZLE_USD)        # → /World/m0609 (defaultPrim=World)
    for _ in range(20):
        simulation_app.update()
    arm_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not arm_prim.IsValid():
        print(f"[FATAL] {ROBOT_PRIM_PATH} 없음 — 노즐 팔 로드 실패")
        return False
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    offset_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET)
    arm_m = offset_m * chassis_m                                # 팔 world = (chassis 기준 MOUNT_OFFSET)
    xf = UsdGeom.Xformable(arm_prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(arm_m)

    # (4) 병합 : root_joint 를 world→chassis 로 재연결 + 팔 ArticulationRoot 제거
    rj = stage.GetPrimAtPath(ROOT_JOINT_PATH)
    if not rj.IsValid():
        print(f"[FATAL] {ROOT_JOINT_PATH} 없음")
        return False
    rj.RemoveAppliedSchema("PhysicsArticulationRootAPI")
    rj.RemoveAppliedSchema("PhysxArticulationAPI")
    fj = UsdPhysics.FixedJoint(rj)
    fj.CreateBody0Rel().SetTargets([Sdf.Path(CHASSIS_LINK_PATH)])
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(MOUNT_OFFSET))
    fj.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(BASE_LINK_PATH))
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(CHASSIS_LINK_PATH))

    # (5) 안전 바닥 (hospital 바닥 collision 누락 대비)
    my_world.scene.add_default_ground_plane(z_position=0.0)

    for _ in range(10):
        simulation_app.update()
    print("[SCENE] hospital + Carter + 노즐 팔 병합 장착 완료")
    return True


def tune_arm_drives():
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
    print(f"[PHYSICS] arm drive tuned: {n}")


def find_articulation_root(root_path):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return root_path
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return root_path


# ────────────────────────────────────────────────────────────────
#  D. 사다리꼴 위상 왕복기 (9_wall_spray 와 동일)
# ────────────────────────────────────────────────────────────────
class Sweeper:
    def __init__(self, lo, hi, cruise, accel, dt, hold_steps=6):
        self.lo, self.hi = lo, hi
        self.cruise, self.accel, self.dt = cruise, accel, dt
        self.hold_steps = hold_steps
        self.reset_bottom()

    def reset_bottom(self):
        self.s = self.lo; self.dir = 1.0; self.v = 0.0; self.hold = 0; self.strokes = 0

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
            self.s = end; self.v = 0.0; self.hold = self.hold_steps
            self.strokes += 1                 # 끝단 도달 = 스트로크 1회 (위→ 또는 →아래)
        return self.s


# ────────────────────────────────────────────────────────────────
#  D-2. 분사 파티클 FX (경량 탄도 파티클 풀 — 순수 시각, 물리/충돌 없음)
# ────────────────────────────────────────────────────────────────
def _basis_from_z(d):
    """3열이 d(단위벡터)인 정규직교 회전행렬. (로컬 +Z → d 로 보내는 기저)"""
    d = d / (np.linalg.norm(d) + 1e-9)
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(a, d); x /= (np.linalg.norm(x) + 1e-9)
    y = np.cross(d, x)
    return np.column_stack([x, y, d])


def _cone_dirs(direction, half_angle_rad, n):
    """direction 중심, 반각 half_angle 원뿔 내부의 랜덤 단위벡터 n개 (n,3)."""
    cos_a = np.cos(half_angle_rad)
    z = np.random.uniform(cos_a, 1.0, n)                 # 극각 코사인
    phi = np.random.uniform(0.0, 2.0 * np.pi, n)
    s = np.sqrt(np.clip(1.0 - z * z, 0.0, 1.0))
    local = np.stack([s * np.cos(phi), s * np.sin(phi), z], axis=1)   # +Z 중심 원뿔
    return local @ _basis_from_z(direction).T


class SprayFX:
    """노즐 끝에서 소독약 스프레이(청록 미스트) 시각효과. numpy 탄도 풀 + UsdGeom.Points.
    순수 시각용 : collision/physics 스키마 미부여 → Nav2 costmap·물리에 간섭 없음."""
    def __init__(self, stage, prim_path="/World/spray_fx"):
        self.N = int(SPRAY_MAX)
        self.pos = np.zeros((self.N, 3), dtype=np.float32)
        self.vel = np.zeros((self.N, 3), dtype=np.float32)
        self.age = np.full(self.N, SPRAY_LIFETIME + 1.0, dtype=np.float32)   # 시작은 전부 dead
        self._cursor = 0
        pts = UsdGeom.Points.Define(stage, prim_path)
        pts.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(self.pos))
        pts.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.zeros(self.N, dtype=np.float32)))
        pts.CreateDisplayColorAttr([Gf.Vec3f(*SPRAY_COLOR)])
        pts.CreateDisplayOpacityAttr([float(SPRAY_OPACITY)])
        self._pts_attr = pts.GetPointsAttr()
        self._w_attr = pts.GetWidthsAttr()

    def _emit(self, origin, direction, n):
        idx = (self._cursor + np.arange(n)) % self.N     # 링버퍼 재사용(할당 없음)
        self._cursor = int((self._cursor + n) % self.N)
        self.pos[idx] = origin
        self.vel[idx] = (SPRAY_SPEED * _cone_dirs(direction, np.radians(SPRAY_CONE_DEG), n)).astype(np.float32)
        self.age[idx] = 0.0

    def update(self, spraying, origin, direction, dt):
        if spraying and origin is not None:
            self._emit(np.asarray(origin, dtype=np.float32),
                       np.asarray(direction, dtype=np.float32), int(SPRAY_RATE))
        alive = self.age < SPRAY_LIFETIME
        self.vel[alive, 2] += SPRAY_GRAVITY * dt         # 탄도 적분
        self.pos[alive] += self.vel[alive] * dt
        self.age[alive] += dt
        frac = np.clip(self.age / SPRAY_LIFETIME, 0.0, 1.0)          # 이동할수록 살짝 커짐(퍼짐)
        widths = np.where(self.age < SPRAY_LIFETIME, SPRAY_SIZE * (0.6 + 0.8 * frac), 0.0)  # dead=0(숨김)
        self._pts_attr.Set(Vt.Vec3fArray.FromNumpy(self.pos))
        self._w_attr.Set(Vt.FloatArray.FromNumpy(widths.astype(np.float32)))


# ────────────────────────────────────────────────────────────────
#  E. 메인
# ────────────────────────────────────────────────────────────────
def main():
    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)

    if not build_scene(my_world):
        simulation_app.close(); return
    tune_arm_drives()

    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)

    art_root = find_articulation_root(CARTER_PRIM_PATH)
    robot = SingleArticulation(prim_path=art_root, name="carter_m0609")
    robot.initialize()
    dof_names = list(robot.dof_names)
    try:
        arm_idx = np.array([dof_names.index(n) for n in ARM_JOINT_NAMES])
    except ValueError as e:
        print(f"[FATAL] 팔 관절 dof 미발견: {e}"); simulation_app.close(); return
    print(f"[ART] arm_idx={arm_idx.tolist()}  (전체 dof {len(dof_names)}개)")

    # ── 팔 하단/상단 aim IK (분사 모션용) ──
    base_pos, base_quat = read_world_pose(BASE_LINK_PATH)
    ori_quat = spray_orientation_quat()
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
        print(f"[FATAL] aim IK 실패 (low={ok_lo}, high={ok_hi})"); simulation_app.close(); return
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high); q_half = 0.5 * (q_high - q_low)

    def q_of_s(s):
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    q_stow = STOW_Q                      # 분사 OFF(주행 중) : 접힌 중앙 자세로 베이스 안정
    robot.set_joint_positions(q_stow, joint_indices=arm_idx)
    q_applied = q_stow.copy()            # rate-limiter 상태(현재 명령 관절값)
    for _ in range(20):
        my_world.step(render=True)

    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)

    # ── ROS2 : /spray_active 구독 + /cmd_vel 발행 + /clock 발행 ──
    rclpy.init()
    ros_node = rclpy.create_node("carter_arm_spray_controller")
    spray_state = {"active": False}
    ros_node.create_subscription(
        Bool, SPRAY_TOPIC, lambda m: spray_state.__setitem__("active", bool(m.data)), 10)
    # /clock 발행 : Nova_Carter_ROS 는 센서/odom/TF 만 내보내고 clock 은 발행하지 않는다.
    # use_sim_time=True 인 Nav2 노드(map/amcl/costmap)가 이 /clock 을 못 받으면 전부 멈춘다.
    clock_pub = ros_node.create_publisher(Clock, "/clock", 10)
    # /cmd_vel : stop-and-go 자체주행. Nav2 와 같은 휠 경로(Carter 내장 Twist 구독)를 쓰므로
    # 직접 휠 제어와 달리 배선 충돌이 없다. 단 sweep 중 Nav2 Goal 을 주면 둘이 /cmd_vel 을
    # 동시에 써 충돌하니, sweep 동안엔 Nav2 Goal 을 주지 않는다.
    cmd_pub = ros_node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
    # (handoff) 미션노드와의 계약 :
    #   /start_sweep=True  → STANDBY 에서 스윕 시작(엣지, 시작 시 소비 → 잔류 True 재트리거 방지)
    #   /start_sweep=False → 진행 중 스윕 취소 → STANDBY (미션 재시작 시 갇힌 스윕 리셋용)
    #   끝나면 /sweep_done 발행.
    handoff = {"req": None}            # None=없음, True=시작요청, False=취소요청
    ros_node.create_subscription(
        Bool, START_SWEEP_TOPIC,
        lambda m: handoff.__setitem__("req", bool(m.data)), 10)
    sweep_done_pub = ros_node.create_publisher(Bool, SWEEP_DONE_TOPIC, 10)
    print(f"[ROS] '{SPRAY_TOPIC}' 구독 + /clock + '{CMD_VEL_TOPIC}' 발행 + "
          f"handoff('{START_SWEEP_TOPIC}'←, '{SWEEP_DONE_TOPIC}'→) | DRIVE_MODE={DRIVE_MODE}")

    # 관성 전복 방지 : 선형속도를 FORWARD_ACCEL 로 가감속 제한(급출발/급정지 흡수).
    #   모든 publish_cmd(0.0 포함)에 적용 → 출발도 정지도 부드럽게. 각속도(wz)는 KP·W_MAX 로 이미 제한.
    drive_state = {"vx": 0.0}
    def publish_cmd(vx, wz=0.0):
        dv = FORWARD_ACCEL * PHYSICS_DT
        v = drive_state["vx"] + float(np.clip(float(vx) - drive_state["vx"], -dv, dv))
        drive_state["vx"] = v
        tw = Twist(); tw.linear.x = v; tw.angular.z = float(wz); cmd_pub.publish(tw)

    q_hold = q_of_s(-1.0)          # MOVE 중 팔 고정 자세(하단, 벽 겨냥)

    if DRIVE_MODE == "handoff":
        print(f"\n[RUN] Play ▶ : STANDBY(Nav2 가 스윕 시작점으로 이동) → /start_sweep 수신 시 "
              f"stop-and-go 스윕({MOVE_DISTANCE:.2f} m 씩, 총 {FORWARD_DISTANCE:.1f} m) → /sweep_done.\n")
    elif DRIVE_MODE == "sweep":
        print(f"\n[RUN] Play ▶ : (멈춤→위아래 소독→{MOVE_DISTANCE:.2f} m 직진) 반복, "
              f"총 {FORWARD_DISTANCE:.1f} m. ※ Nav2 Goal 금지.\n")
    else:
        print("\n[RUN] Play ▶ : Nav2 가 베이스를 주행, /spray_active True 구간에서 팔이 소독.\n")

    was_playing = False
    arm_ready = False                     # Stop→Play 후 physics 뷰 재생성 → articulation 재초기화 필요
    prev_active = None
    sweep_run = (DRIVE_MODE == "sweep")   # sweep 모드는 처음부터, handoff 는 트리거 대기
    phase = "WIPE"                        # 스윕 하위상태 : WIPE ↔ MOVE
    heading_ready = False; reached = False
    global_start = None; forward0 = None; target_yaw = 0.0
    progress = 0.0; move_start_prog = 0.0; cycle = 0
    done_ticks = 0                        # 완료 통지(/sweep_done) 재전송 카운터

    spray_fx = SprayFX(omni.usd.get_context().get_stage()) if SPRAY_FX_ON else None

    while simulation_app.is_running():
        my_world.step(render=True)
        # 시뮬 시간을 /clock 으로 발행 (센서 타임스탬프와 동일 시간원 → Nav2 TF 정합)
        t = float(my_world.current_time)
        cmsg = Clock()
        cmsg.clock.sec = int(t)
        cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
        clock_pub.publish(cmsg)
        rclpy.spin_once(ros_node, timeout_sec=0.0)     # /spray_active·/start_sweep 갱신 (비차단)

        is_playing = my_world.is_playing()
        if not is_playing:                # Stop/일시정지 : 재생 시 재초기화하도록 표시
            arm_ready = False
            was_playing = False
            continue
        # Stop→Play 후엔 physics 뷰가 새로 생성됨 → SingleArticulation 을 재초기화해야 apply_action 이
        # 동작한다("Physics Simulation View is not created yet" 경고 = 팔 명령 무시됨 → 와이프 안 함).
        if not arm_ready:
            try:
                robot.initialize()
                robot.set_joint_positions(q_stow, joint_indices=arm_idx)
            except Exception:
                continue                  # physics 뷰 아직 준비 안 됨 → 다음 스텝 재시도
            q_applied = q_stow.copy()
            sweeper.reset_bottom()
            phase = "WIPE"; heading_ready = False; reached = False; cycle = 0
            sweep_run = (DRIVE_MODE == "sweep"); handoff["req"] = None; done_ticks = 0
            arm_ready = True
            print("[SIM] Play 감지 → articulation 초기화 (와이프 준비 완료)")
        was_playing = is_playing

        # ── 분사 파티클 FX : 방출은 "실제 소독(WIPE/spray active)" 구간에서만, 적분은 항상 ──
        #   (상태값은 직전 스텝 기준 → 1스텝 지연은 시각상 무의미. 모든 continue 분기보다 위에서 실행)
        if spray_fx is not None:
            spraying = (
                (DRIVE_MODE == "nav2" and (SPRAY_ALWAYS_ON or spray_state["active"]))
                or (sweep_run and phase == "WIPE")
                or (sweep_run and reached)          # sweep 모드 제자리 계속 분사
            )
            if spraying:
                n_pos, n_quat = read_world_pose(NOZZLE_BASE_PATH)
                Rn = quat_to_matrix(n_quat)
                s_dir = Rn @ np.array([0.0, 0.0, 1.0])                       # 노즐 로컬 +Z = 분사축
                s_org = n_pos + Rn @ np.array([0.0, 0.0, NOZZLE_TIP_LOCAL])  # 노즐 팁 = 분사 원점
                spray_fx.update(True, s_org, s_dir, PHYSICS_DT)
            else:
                spray_fx.update(False, None, None, PHYSICS_DT)

        # ── 모드: nav2 (주행은 Nav2, 팔은 /spray_active 게이트) ──
        if DRIVE_MODE == "nav2":
            active = True if SPRAY_ALWAYS_ON else spray_state["active"]
            if active != prev_active:
                print(f"[SPRAY] {'ON → 위아래+방사 시작' if active else 'OFF → 접힌 자세로'}")
                prev_active = active
                if active:
                    sweeper.reset_bottom()
            q_target = q_of_s(sweeper.step()) if active else q_stow
            q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))
            continue

        # ── 모드: handoff STANDBY (Nav2 가 스윕 시작점으로 이동 중; 스크립트는 cmd_vel 안 건드림) ──
        if DRIVE_MODE == "handoff" and not sweep_run:
            if done_ticks > 0:                         # 완료 통지 재전송(메시지 유실 대비)
                sweep_done_pub.publish(Bool(data=True)); done_ticks -= 1
            if handoff["req"] is True:                 # 시작 트리거(엣지 소비 → 잔류 True 재트리거 방지)
                handoff["req"] = None
                sweep_run = True; phase = "WIPE"; heading_ready = False
                reached = False; cycle = 0; done_ticks = 0
                sweeper.reset_bottom()
                print("[HANDOFF] /start_sweep=True 수신 → 스윕 시작")
                # 이번 스텝은 아래 스윕 코드로 fall-through
            else:                                      # 대기 : 팔 stow, cmd_vel 미발행(Nav2 주행)
                if handoff["req"] is False:            # 대기 중 취소요청은 그냥 소비
                    handoff["req"] = None
                q_applied = q_applied + np.clip(q_stow - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
                robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))
                continue

        # ── 여기부터 sweep_run == True : stop-and-go 스윕 (sweep/handoff 공용) ──
        # 취소요청(/start_sweep=False) → 즉시 정지·STANDBY (미션 재시작 시 갇힌 스윕 방지)
        if DRIVE_MODE == "handoff" and handoff["req"] is False:
            handoff["req"] = None
            publish_cmd(0.0)
            sweep_run = False
            print("[HANDOFF] /start_sweep=False 수신 → 스윕 취소 → STANDBY")
            continue

        chassis_pos, chassis_quat = read_world_pose(CHASSIS_LINK_PATH)
        chassis_R = quat_to_matrix(chassis_quat)

        if not heading_ready:                          # 직진 기준 캡처(스윕 시작 위치·헤딩)
            global_start = chassis_pos.copy()
            forward0 = chassis_R @ np.array([1.0, 0.0, 0.0])
            forward0 = forward0 / (np.linalg.norm(forward0) + 1e-9)
            target_yaw = np.arctan2(forward0[1], forward0[0])
            heading_ready = True
            print("[PHASE] WIPE(위아래 소독) 시작")

        progress = float(np.dot(chassis_pos - global_start, forward0))

        # 스윕 완료 판정 (누적 FORWARD_DISTANCE 도달)
        if not reached and progress >= FORWARD_DISTANCE:
            reached = True
            print(f"[INFO] 총 {FORWARD_DISTANCE:.1f} m 도달(progress={progress:.2f})")
        if reached:
            publish_cmd(0.0)
            if DRIVE_MODE == "handoff":                # 완료 통지 → STANDBY 복귀(Nav2 대기)
                sweep_done_pub.publish(Bool(data=True))
                sweep_run = False; done_ticks = 30     # ~0.5s 동안 재전송
                print("[HANDOFF] 스윕 완료 → /sweep_done 발행 → STANDBY(Nav2 대기)")
                continue
            q_target = q_of_s(sweeper.step())          # sweep 모드 : 제자리 계속 소독
            q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))
            continue

        if phase == "WIPE":                            # 정지 + 위아래 소독
            publish_cmd(0.0)
            q_target = q_of_s(sweeper.step())
            if sweeper.strokes >= STROKES_PER_WIPE:     # 위아래 1회 완료
                phase = "MOVE"; move_start_prog = progress; cycle += 1
                print(f"[{cycle}] WIPE 완료 → MOVE ({MOVE_DISTANCE:.2f} m 직진)")
        else:                                          # MOVE : 팔 하단 고정, 직진(heading-hold)
            q_target = q_hold
            fwd_now = chassis_R @ np.array([1.0, 0.0, 0.0])
            yaw_now = np.arctan2(fwd_now[1], fwd_now[0])
            yaw_err = wrap_pi(yaw_now - target_yaw)
            left_dir = np.array([-forward0[1], forward0[0], 0.0])
            lateral = float(np.dot(chassis_pos - global_start, left_dir))
            w_cmd = float(np.clip(-(KP_YAW * yaw_err + KP_LAT * lateral), -W_MAX, W_MAX))
            publish_cmd(FORWARD_SPEED, w_cmd)
            if (progress - move_start_prog) >= MOVE_DISTANCE:
                publish_cmd(0.0)
                phase = "WIPE"; sweeper.reset_bottom()
                print(f"[{cycle}] MOVE 완료(progress={progress:.2f} m) → WIPE")

        q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))

    if DRIVE_MODE != "nav2":
        publish_cmd(0.0)
    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()