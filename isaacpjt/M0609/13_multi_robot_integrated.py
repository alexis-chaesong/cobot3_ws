"""
13_multi_robot_integrated.py  ★멀티로봇 Phase B — 통합 Isaac 스크립트★
================================================================================
한 씬(modified_hospital) 에 두 로봇을 함께 스폰하고, 각자 OmniGraph node_namespace 를
프로그램적으로 carter1/carter2 로 설정한 뒤, 하나의 /clock 을 공유하며 두 로봇의 FSM 을
"한 물리 스텝 루프" 안에서 협조적으로(cooperative, 서로를 블로킹하지 않게) 동시 구동한다.

  · carter1 (소독)   : Nova Carter + m0609 노즐팔. 스폰 docking_station_1 (18.5, 0).
                       핸드오프 stop-and-go 소독 스윕. 10_1_carter_hospital_spray_nav.py 로직 그대로.
                       조율 토픽 : /carter1/start_sweep(←) · /carter1/sweep_done(→) · /carter1/cmd_vel.
  · carter2 (폐기물) : Nova Carter + m0609 Surface Gripper + 소형 쓰레기통. 스폰 docking_station_02
                       (16.6629, -0.00295). Nav2 주행→파지→big_trash 덤프.
                       4_mobile_manipulator_trash_can_nav_pick_test.py 로직 그대로(제너레이터화).
                       조율 토픽 : /carter2/trash_can_nav_goal(→) · /carter2/start_pick(←) · /carter2/cmd_vel.

두 로봇 병합의 핵심 = 프림 경로 분리 + namespace :
  · 두 Nova Carter 는 각각 /World/Carter1/Nova_Carter_ROS, /World/Carter2/Nova_Carter_ROS 로
    "스코프(scope) prim 아래"에 놓아 경로 충돌을 피한다(단일로봇 스크립트는 둘 다 /World/Nova_Carter_ROS).
  · 각 Carter 내부 4개 ActionGraph(ros_lidars·transform_tree_odometry·differential_drive·chassis_imu)의
    node_namespace(Constant String) 노드 inputs:value 를 carter1/carter2 로 설정 → 그 로봇의 모든 ROS
    토픽/TF 에 /carterN 접두가 붙는다(PDF "Nova Carter2 추가"의 GUI 작업을 코드로). set_carter_namespace().
  · /clock 은 네임스페이스 없는 전역 토픽 → 이 스크립트가 "딱 하나"만 발행(두 로봇 Nav2 가 공유).

협조 루프(데드락 방지) :
  · carter1 은 상태머신 tick() (10_1 while 루프 몸통 1회 = 1 스텝, 내부에서 world.step 안 부름).
  · carter2 는 파이썬 제너레이터(4_ 의 블로킹 world.step(render=True) 를 전부 yield 로 치환) →
    main 루프가 매 스텝 next() 로 "한 스텝치"만 전진. 두 로봇이 같은 my_world.step 을 공유한다.

실행 (총 5개 터미널 권장) :
  1) 이 스크립트 : python.sh isaacpjt/M0609/13_multi_robot_integrated.py  → GUI 에서 두 로봇 확인 후 Play ▶
  2) Nav2(멀티) : ros2 launch carter_navigation \
        multiple_robot_carter_navigation_modified_hospital.launch.py \
        map:=<carter_navigation>/maps/map/modified_hospital_map.yaml
  3) carter1 미션 : ros2 run commander spray_waypoint_mission --ros-args -p namespace:=carter1
  4) carter2 미션 : ros2 run commander trash_can_nav_pick_mission --ros-args -p namespace:=carter2

────────────────────────────────────────────────────────────────────────────────
⚠ Isaac 반복 검증 필요(오프라인 미검증) — 최초 실행 시 확인 포인트 :
  [V1] Play 후 두 스폰 위치 로그([SPAWN] c1/c2 chassis world) 가 (18.5,0)/(16.66,0) 근처인지.
  [V2] `ros2 topic list` 에 /carter1/scan·/carter1/cmd_vel·/carter1/tf·/carter1/chassis/odom 및
       /carter2/... 가 보이는지(=namespace 성공). 안 보이면 set_carter_namespace 로그/그래프 경로 확인.
  [V3] 두 로봇 각각 팔 초기화(arm_ready)·주행이 서로 간섭 없이 도는지(협조 루프).
  [V4] carter2 파지 상대기하(TARGET_JOINTS_DEG)가 nested 경로에서도 유효한지(프림 경로만 바뀌고
       상대 pose 는 그대로여야 함 — 아니면 스코프 xform 이 identity 인지 확인).
  ※ carter2 는 4_ 와 동일하게 "연속 Play" 전제(Stop→Play 재개 미지원 — 제너레이터/physics 뷰
     상태가 리셋 안 됨). 재시작하려면 스크립트 프로세스를 재실행할 것. carter1 은 Stop→Play 재초기화 지원.

✅ 오프라인 헤드리스 스모크 검증 완료(2026-07-22) : 씬 합성·프림경로 분리·노즐팔 fixed-joint 병합·
   node_namespace 프로그램 설정(c1/c2 각 4개 그래프)·articulation root/DOF(각 13, 팔6 포함)·스폰
   좌표(18.5,0 / 16.66,-0.003)·그리퍼/쓰레기통/노즐/지면 유효·40스텝 물리안정 = 전부 PASS.
   (미검증: carter1 IK·carter2 RMPflow·ROS 핸드오프·Nav2 연동·FSM 실동작/튜닝 → Isaac+Nav2 루프 필요.
    이 파트들은 단일로봇 10_1/4_ 에서 이미 검증된 로직 그대로 이식.)
================================================================================
"""
import os

from isaacsim import SimulationApp

# 기본 GUI. 스모크 테스트/헤드리스 실행 시 ISAAC_HEADLESS=1 로 창 없이 부팅.
simulation_app = SimulationApp({"headless": os.environ.get("ISAAC_HEADLESS", "0") == "1"})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import sys
from pathlib import Path

import numpy as np
import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg

import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from rosgraph_msgs.msg import Clock

_THIS_DIR = Path(__file__).resolve().parent                 # isaacpjt/M0609
_WS_ROOT = Path("/home/rokey/cobot3_ws")

# carter2 RMPflow 컨트롤러(그리퍼 팔) : integration/rmpflow 에 common/description yaml 있음.
_C2_RMPFLOW_DIR = str(_WS_ROOT / "src" / "integration" / "integration" / "rmpflow")
if _C2_RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, _C2_RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402


# ════════════════════════════════════════════════════════════════════════════
#  A. 공통 경로/상수
# ════════════════════════════════════════════════════════════════════════════
# 환경/맵 : Nav2 멀티런치가 로드하는 modified_hospital_map.yaml 과 같은 씬(같은 world 프레임).
HOSPITAL_USD = ("/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/navigation/"
                "carter_navigation/maps/map/modified_hospital.usd")
CARTER_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
              "Assets/Isaac/5.1/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd")
NOZZLE_USD = str(_WS_ROOT / "src" / "integration" / "integration" / "m0609_with_nozzle.usd")
MOVE_TRASH_USD = str(_WS_ROOT / "src" / "assets" / "scenes" / "move_tash_can.usd")

PHYSICS_DT = 1.0 / 60.0
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# ── ★성능 노브 (멀티로봇 GPU 병목 완화)★ ──
# RENDER_EVERY : 메인 루프에서 RTX 렌더를 매 스텝 → N스텝마다 1번(물리/제어/clock 은 매 스텝 유지).
#   1=현재(매 스텝), 2~3=GPU 여유↑·시각FPS↓. 카메라/라이다 등 렌더연동 센서 발행률도 ~1/N 로 낮아짐
#   (저속 Nav2 엔 2 정도 무난). GPU 80W 캡이면 2 권장.
RENDER_EVERY = 2
# 카메라 렌더 해상도 : Nova Carter 기본 1920x1200(로봇당 스테레오 다수) → RTX 부담 큼. 낮추면 픽셀수
#   제곱으로 절감(640x400 ≈ 1/9). 라이다 render product(1280x720)는 스캔 품질 위해 안 건드림.
#   카메라 시야는 유지되며 해상도만 낮아짐. 0 이면 원본 유지(축소 안 함).
CAM_RENDER_W = 640
CAM_RENDER_H = 400

# Nova Carter 내부 4개 ActionGraph 의 node_namespace 값(→ /carterN 접두). PDF "Nova Carter2 추가" 참조.
NS_CARTER1 = "carter1"
NS_CARTER2 = "carter2"

# ── 스코프 프림(경로 충돌 방지) ──
C1_SCOPE = "/World/Carter1"
C2_SCOPE = "/World/Carter2"


# ════════════════════════════════════════════════════════════════════════════
#  B. carter1 (소독) 상수 — 10_1_carter_hospital_spray_nav.py 와 동일 값
# ════════════════════════════════════════════════════════════════════════════
C1_CARTER_PRIM = f"{C1_SCOPE}/Nova_Carter_ROS"
C1_CHASSIS = f"{C1_CARTER_PRIM}/chassis_link"
C1_ARM = f"{C1_SCOPE}/m0609"
C1_EE_LINK = f"{C1_ARM}/link_6"
C1_BASE_LINK = f"{C1_ARM}/base_link"
C1_ROOT_JOINT = f"{C1_ARM}/root_joint"
C1_NOZZLE_BASE = f"{C1_ARM}/nozzle_base_link"

C1_START_POSE = dict(x=18.5, y=0.0, z=0.05, yaw_deg=0.0)     # docking_station_1
MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)
CHASSIS_MASS = 150.0
DRIVE_MAX_ANG_SPEED = 2.5
DRIVE_MAX_ANG_ACCEL = 6.0
DRIVE_MAX_LIN_SPEED = 1.2

C1_URDF = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
C1_DESC = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
NOZZLE_OFFSET = 0.1392
NOZZLE_TIP_LOCAL = 0.142
EE_FRAME = "link_6"

WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80
J1_INDEX = 0
J1_OFFSET = -np.pi / 2
J5_INDEX = 4
J5_FLICK = -0.5

S_CRUISE = 2.0
S_ACCEL = 3.0
S_HOLD_STEPS = 6
STOW_Q = np.array([0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
MAX_JOINT_STEP = 0.06

STROKES_PER_WIPE = 2
MOVE_DISTANCE = 0.25
FORWARD_DISTANCE = 10.5
FORWARD_SPEED = 0.35
FORWARD_ACCEL = 0.50
KP_YAW = 2.5
KP_LAT = 2.0        # [캐스터 대각선 보정] 좌우 이탈 되당김 게인 1.2→2.0 (더 세게 직선복귀)
KD_YAW = 0.4        # [캐스터 대각선 보정] yaw_rate 미분 댐핑(c2 drift_kd 차용) — 세게 당겨도 안 출렁이게
W_MAX = 0.8

# carter1 조율/센서 토픽(네임스페이스 접두). Nav2 가 발행하는 /carter1/cmd_vel 을 스윕 중엔 이 스크립트가.
C1_CMD_VEL = f"/{NS_CARTER1}/cmd_vel"
C1_START_SWEEP = f"/{NS_CARTER1}/start_sweep"
C1_SWEEP_DONE = f"/{NS_CARTER1}/sweep_done"

# 분사 파티클 FX (경량 탄도 풀) — 10_1 과 동일
SPRAY_FX_ON = True
SPRAY_MAX = 700
SPRAY_RATE = 12
SPRAY_SPEED = 2.5
SPRAY_CONE_DEG = 120.0
SPRAY_LIFETIME = 0.5
SPRAY_GRAVITY = -2.0
SPRAY_SIZE = 0.012
SPRAY_OPACITY = 0.6
SPRAY_COLOR = (0.55, 0.85, 1.0)


# ════════════════════════════════════════════════════════════════════════════
#  C. carter2 (폐기물) 상수 — 4_mobile_manipulator_trash_can_nav_pick_test.py 와 동일 값
# ════════════════════════════════════════════════════════════════════════════
C2_ARTICULATION_ROOT = f"{C2_SCOPE}/Nova_Carter_ROS/chassis_link"
C2_ARM_ROOT = f"{C2_SCOPE}/m0609"
C2_EE_LINK_NAME = "link_6"
C2_TRASH_CAN_PRIM = f"{C2_SCOPE}/small_trash_can_body"
C2_SURFACE_GRIPPER = f"{C2_ARM_ROOT}/{C2_EE_LINK_NAME}/mop_surface_gripper"
C2_GROUND_PLANE = f"{C2_SCOPE}/GroundPlane"
C2_EXTRA_PHYSICS = f"{C2_SCOPE}/PhysicsScene"       # move_tash_can 내장 중복 PhysicsScene → 비활성화

C2_NO_GRIPPER_URDF = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

TARGET_JOINTS_DEG = [-90.0, 101.0, 50.0, -94.0, 91.8, -1.1]
# [태성 갱신] 이동 중 j1 자세 170→10도. DUMP 도 동일값 유지(j6 만 실제로 움직여 비움).
TUCK_J1_DEG = 10.0
DUMP_J1_DEG = TUCK_J1_DEG            # [태성] 이동 중과 동일 자세 유지
DUMP_J6_ROTATE_DEG = -180.0          # [태성] 손목 뒤집기 방향 = 시계방향(음수)
DUMP_RAMP_STEPS = 90                 # [태성] dump 는 j6 하나만 크게 → 파지보다 빠른 램프
POST_DUMP_BACKUP_DISTANCE = 0.6      # [태성] 덤프 직후 후진(RETURN Nav2 켜기 전 코스트맵 여유) [m]
POST_RETURN_BACKUP_DISTANCE = 0.6    # [태성] 내려놓기 직후 후진(DOCK Nav2 켜기 전) [m]

# [태성 갱신] big_trash 를 (4,7.5) 로 이동, 접근방향 +Y→+X (맵 재스캔: +X 쪽이 1.4m 로 제일 트임)
BIG_TRASH_POSITION_XY = np.array([4.0, 7.5])
BIG_TRASH_APPROACH_DIR = np.array([1.0, 0.0])
BIG_TRASH_STANDOFF_DISTANCE = 1.3    # [태성] cmd_vel 실이동 = STANDOFF-FINAL = 0.8m
BIG_TRASH_FINAL_DISTANCE = 0.5

LIFT_OFFSET = np.array([0.0, 0.0, 0.75])
POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
# [태성] 원위치 내려놓기용 : 고정 tolerance 대신 0→이 값까지 점점 느슨(지수) → 어색한 미세보정 방지
RETURN_PLACE_GROWING_TOLERANCE_MAX = 0.25
MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600
SETTLE_STEPS = 60
GRASP_HOLD_STEPS = 60
JOINT_RAMP_STEPS = 300
CREEP_STEP_SIZE = 0.005
CREEP_MAX_STEPS = 60
CREEP_SETTLE_STEPS = 5
LATERAL_CORRECTION_MIN = 0.01
LATERAL_CORRECTION_MAX = 0.4

TRASH_SPAWN_FIXED = (14.0, 6.5)      # [태성] 쓰레기통 스폰 (7.5,7.5)→(14,6.5), 벽 여유 2.3m
TRASH_SPAWN_Z = 0.0786364536328308
# [태성] xformOp:translate(로컬원점)와 bbox 중심의 world 오프셋. relocate/entry 계산에 반영.
TRASH_BBOX_CENTER_OFFSET_XY = np.array([0.15, 0.15])
OFFSET_TRASH_FROM_CHASSIS = np.array([-0.750368208, -0.758035257])
C2_START_POSE = dict(x=16.66290495232035, y=-0.0029517927591273807, yaw_deg=0.0)  # docking_station_02

C2_NAV_GOAL = f"/{NS_CARTER2}/trash_can_nav_goal"
C2_START_PICK = f"/{NS_CARTER2}/start_pick"
C2_CMD_VEL = f"/{NS_CARTER2}/cmd_vel"

FINAL_APPROACH_DISTANCE = 0.4
FINAL_APPROACH_SPEED = 0.15
FINAL_APPROACH_RAMP_TIME = 0.5
FINAL_APPROACH_DRIFT_KP = 0.15
FINAL_APPROACH_DRIFT_KD = 0.4
FINAL_APPROACH_DRIFT_W_MAX = 0.15
FINAL_APPROACH_DRIFT_MAX_W_STEP = 0.01
FINAL_ROTATE_KP = 2.0
FINAL_ROTATE_KD = 0.5
FINAL_ROTATE_W_MAX = 1.0
FINAL_ROTATE_MAX_W_STEP = 0.05
FINAL_ROTATE_TOLERANCE_RAD = 0.02
FINAL_MOVE_SETTLE_STEPS = 30
FINAL_NUDGE_DISTANCE = 0.00
PRE_ROTATE_NUDGE_DISTANCE = 0.25     # [태성] 0.3→0.25


# ════════════════════════════════════════════════════════════════════════════
#  D. namespace 유틸 + 수학 유틸
# ════════════════════════════════════════════════════════════════════════════
def set_carter_namespace(root_path, ns):
    """root_path(=한 Nova Carter 프림) 하위 ROS2 노드에 namespace(ns) 접두를 프로그램 설정.
    Nova Carter 는 네임스페이스 상수가 두 종류라 둘 다 처리해야 한다(라이브 USD 검사로 확정) :
      (1) "node_namespace" 상수 4개(ActionGraph : ros_lidars·transform_tree_odometry·
          differential_drive·chassis_imu) → inputs:value 를 ns 로 설정 → scan·cmd_vel·tf·odom 에 /ns 접두.
      (2) "camera_namespace" 상수(각 hawk 카메라 그래프 : front/right/left/back_hawk) → inputs:value 가
          '/front_stereo_camera' 같은 카메라 이름. 여기에 ns 를 '앞에 덧붙여' '/carterN/front_stereo_camera'
          로 만들어야 카메라 토픽이 /carterN/front_stereo_camera/{left,right}/{image_raw,camera_info} 로
          분리된다(안 하면 두 로봇이 /front_stereo_camera/... 공유해 충돌 = 14-4 #2).
    ※ 카메라의 inputs:nodeNamespace 는 camera_namespace 상수에 '연결'돼 있으므로 노드에 직접 Set 하면
       안 되고(무시됨), 반드시 이 상수 값을 바꿔야 한다."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    n = 0       # (1) node_namespace 상수 수
    n_cam = 0   # (2) camera_namespace 상수 수
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        if nm == "node_namespace":
            a = prim.GetAttribute("inputs:value")
            if a and a.IsValid():
                a.Set(ns); n += 1
        elif nm == "camera_namespace":
            a = prim.GetAttribute("inputs:value")
            if a and a.IsValid():
                cur = (a.Get() or "").strip("/")     # 예: 'front_stereo_camera'
                a.Set(f"/{ns}/{cur}" if cur else f"/{ns}")   # → '/carter1/front_stereo_camera'
                n_cam += 1
    print(f"[NS] {root_path} → ns='{ns}' (node_namespace {n}개, camera_namespace {n_cam}개[카메라])")
    if n == 0:
        print(f"[NS][WARN] {root_path} 하위 node_namespace 노드 못 찾음 — 토픽 접두 실패 위험")
    if n_cam == 0:
        print(f"[NS][WARN] {root_path} 하위 camera_namespace 노드 못 찾음 — 카메라 토픽 분리 실패 위험")
    return n


def set_camera_resolution(root_path, width, height):
    """root_path 하위 '카메라' render product 노드의 inputs:width/height 를 낮춰 RTX 렌더 부담을 줄인다.
    (라이다 render product '*_lidar_render_product' 는 스캔 품질 위해 제외 — 이름으로 필터.)
    Play/첫 tick 전에 호출해야 render product 가 이 해상도로 생성된다(build 단계에서 호출)."""
    if not width or not height:
        return 0
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    n = 0
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        # 카메라 render product 노드만 : hawk 은 '*_camera_render_product', owl 은 'isaac_create_render_product'
        is_cam_rp = ("camera_render_product" in nm) or (nm == "isaac_create_render_product")
        if not is_cam_rp:
            continue
        wa = prim.GetAttribute("inputs:width"); ha = prim.GetAttribute("inputs:height")
        if wa and wa.IsValid() and ha and ha.IsValid():
            wa.Set(int(width)); ha.Set(int(height)); n += 1
    print(f"[GPU] {root_path} 카메라 render product {n}개 → {width}x{height} 로 축소")
    return n


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


def rotate_2d(v, theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


# carter2 전용(4_ 와 동일) — world_transform 기반 헬퍼
def get_prim_world_position(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = omni.usd.get_world_transform_matrix(prim)
    t = m.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


def get_prim_world_bbox_center(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return (np.array(r.GetMin()) + np.array(r.GetMax())) / 2.0


def get_world_orientation_wxyz(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    q = omni.usd.get_world_transform_matrix(prim).ExtractRotationQuat()
    im = q.GetImaginary()
    return np.array([q.GetReal(), im[0], im[1], im[2]])


def rotate_vector_by_quat(q_wxyz, v):
    w = q_wxyz[0]; qv = np.asarray(q_wxyz[1:4], dtype=float); v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def get_chassis_yaw(prim_path):
    w, x, y, z = get_world_orientation_wxyz(prim_path)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _smoothstep(a):
    return a * a * (3.0 - 2.0 * a)


def _ramp_steps_for_distance(distance, max_speed):
    raw = distance / max_speed / PHYSICS_DT
    return int(np.clip(round(raw), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def find_articulation_root(root_path):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return root_path
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return root_path


def tune_arm_drives(root_path):
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dt in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dt)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive.GetTargetPositionAttr().Set(0.0)
                n += 1
    print(f"[PHYSICS] {root_path} arm drive tuned: {n}")


def boost_drive_limits(carter_prim_path):
    """차동구동 컨트롤러 각속도 클램프 상향(U턴 병목 해소, 6-15). 로봇별로 호출."""
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(carter_prim_path)):
        ttype = prim.GetAttribute("node:type")
        if ttype and ttype.IsValid() and "DifferentialController" in str(ttype.Get() or ""):
            for name, val in (("inputs:maxAngularSpeed", DRIVE_MAX_ANG_SPEED),
                              ("inputs:maxAngularAcceleration", DRIVE_MAX_ANG_ACCEL),
                              ("inputs:maxLinearSpeed", DRIVE_MAX_LIN_SPEED)):
                a = prim.GetAttribute(name)
                if a and a.IsValid():
                    a.Set(float(val)); n += 1
    print(f"[DRIVE] {carter_prim_path} 차동구동 클램프 상향 attr {n}")


# ════════════════════════════════════════════════════════════════════════════
#  E. 씬 구성
# ════════════════════════════════════════════════════════════════════════════
def build_env():
    """/World + hospital 환경 (한 번만)."""
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(HOSPITAL_USD)
    for _ in range(60):
        simulation_app.update()
    print(f"[LOAD] hospital env = {HOSPITAL_USD}")


def _place_xform(prim_path, x, y, z, yaw_deg):
    """XformCommonAPI 우회(6-13) : op 비우고 단일 transform 행렬로 배치."""
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)))
    m.SetTranslateOnly(Gf.Vec3d(x, y, z))
    xf.AddTransformOp().Set(m)


def build_carter1():
    """carter1 = Nova Carter payload + m0609 노즐팔(fixed-joint 병합) @ /World/Carter1.
    10_1 build_scene 를 스코프 경로로 이식."""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, C1_SCOPE)

    # (1) Nova Carter payload
    carter_prim = stage.DefinePrim(C1_CARTER_PRIM, "Xform")
    carter_prim.GetPayloads().AddPayload(CARTER_URL)
    for _ in range(120):
        simulation_app.update()
    _place_xform(C1_CARTER_PRIM, C1_START_POSE["x"], C1_START_POSE["y"],
                 C1_START_POSE["z"], C1_START_POSE["yaw_deg"])
    simulation_app.update()

    chassis = stage.GetPrimAtPath(C1_CHASSIS)
    if not chassis.IsValid():
        print(f"[FATAL] {C1_CHASSIS} 없음 — carter1 로드 실패"); return False
    _wp = Gf.Transform(UsdGeom.Xformable(chassis)
                       .ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetTranslation()
    print(f"[SPAWN] c1 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C1_START_POSE['x']},{C1_START_POSE['y']})")
    UsdPhysics.MassAPI.Apply(chassis).CreateMassAttr(CHASSIS_MASS)

    # (2) 노즐 팔 : m0609_with_nozzle 내부 /World/m0609 를 C1_ARM 으로 타겟 참조(스코프 오염 방지)
    arm_prim = stage.DefinePrim(C1_ARM, "Xform")
    arm_prim.GetReferences().AddReference(Sdf.Reference(assetPath=NOZZLE_USD, primPath="/World/m0609"))
    for _ in range(20):
        simulation_app.update()
    if not stage.GetPrimAtPath(C1_ARM).IsValid():
        print(f"[FATAL] {C1_ARM} 없음 — 노즐 팔 로드 실패"); return False
    # chassis 위 정렬 배치 (arm_world = MOUNT_OFFSET × chassis_world)
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    arm_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET) * chassis_m
    xf = UsdGeom.Xformable(arm_prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(arm_m)

    # (3) 병합 : root_joint 를 world→chassis 로 재연결 + 팔 ArticulationRoot 제거
    rj = stage.GetPrimAtPath(C1_ROOT_JOINT)
    if not rj.IsValid():
        print(f"[FATAL] {C1_ROOT_JOINT} 없음"); return False
    rj.RemoveAppliedSchema("PhysicsArticulationRootAPI")
    rj.RemoveAppliedSchema("PhysxArticulationAPI")
    fj = UsdPhysics.FixedJoint(rj)
    fj.CreateBody0Rel().SetTargets([Sdf.Path(C1_CHASSIS)])
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(MOUNT_OFFSET))
    fj.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(C1_BASE_LINK))
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(C1_CHASSIS))

    set_carter_namespace(C1_CARTER_PRIM, NS_CARTER1)
    set_camera_resolution(C1_CARTER_PRIM, CAM_RENDER_W, CAM_RENDER_H)
    print("[SCENE] carter1 (Nova + 노즐팔 병합) 완료")
    return True


def build_carter2():
    """carter2 = move_tash_can.usd(Nova + 그리퍼팔 + 쓰레기통, 이미 병합됨) 전체를 /World/Carter2 스코프로
    참조. 상대기하 보존을 위해 통째로 참조하고 스코프 xform 은 identity 로 둔다."""
    stage = omni.usd.get_context().get_stage()
    scope = stage.DefinePrim(C2_SCOPE, "Xform")
    # identity 보장(참조된 /World 의 xform 이 스코프에 얹혀도 상대 좌표가 흔들리지 않게)
    UsdGeom.Xformable(scope).ClearXformOpOrder()
    scope.GetReferences().AddReference(Sdf.Reference(assetPath=MOVE_TRASH_USD, primPath="/World"))
    for _ in range(80):
        simulation_app.update()

    if not stage.GetPrimAtPath(C2_ARTICULATION_ROOT).IsValid():
        print(f"[FATAL] {C2_ARTICULATION_ROOT} 없음 — carter2 로드 실패"); return False

    # 중복 PhysicsScene 비활성화(환경/World 기본과 충돌 방지)
    extra = stage.GetPrimAtPath(C2_EXTRA_PHYSICS)
    if extra.IsValid():
        extra.SetActive(False)
        print(f"[SCENE] {C2_EXTRA_PHYSICS} 비활성화(중복 PhysicsScene)")

    # 쓰레기통을 재현 가능한 고정 스폰으로 이동(4_ 와 동일)
    trash = stage.GetPrimAtPath(C2_TRASH_CAN_PRIM)
    if trash.IsValid():
        ta = trash.GetAttribute("xformOp:translate")
        if ta and ta.IsValid():
            # [태성] TRASH_SPAWN_FIXED 는 bbox '중심'이 놓일 좌표 → 원점(translate)은 offset 만큼 뺀 값.
            _origin = np.asarray(TRASH_SPAWN_FIXED) - TRASH_BBOX_CENTER_OFFSET_XY
            ta.Set(Gf.Vec3d(float(_origin[0]), float(_origin[1]), TRASH_SPAWN_Z))
            print(f"[SPAWN] c2 trash 중심 → {TRASH_SPAWN_FIXED} (원점 {_origin.tolist()})")

    chassis2 = stage.GetPrimAtPath(C2_ARTICULATION_ROOT)
    chassis2_m = UsdGeom.Xformable(chassis2).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    _wp = Gf.Transform(chassis2_m).GetTranslation()
    print(f"[SPAWN] c2 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C2_START_POSE['x']:.2f},{C2_START_POSE['y']:.2f})")

    # ★ 팔 사전 배치(가시성) : move_tash_can 은 그리퍼 팔을 "원점 근처"에 authoring 해서, Play(물리) 전에는
    #   팔이 카터에서 ~16m 떨어져 보인다(="카터만, 팔 없음"으로 오인). articulation FK 가 Play 시 카터 위로
    #   스냅하지만, 혼동 방지 위해 로드 시점에 carter1 과 동일하게 팔 루트를 chassis 위(MOUNT_OFFSET)로 배치.
    #   (Play 후 물리가 같은 위치로 확정하므로 정합. 스냅 전후 pose 동일 — 헤드리스 검증됨: base_link≈(16.43,0,0.58))
    arm2 = stage.GetPrimAtPath(C2_ARM_ROOT)
    if arm2.IsValid():
        arm2_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET) * chassis2_m
        xf2 = UsdGeom.Xformable(arm2)
        xf2.ClearXformOpOrder()
        xf2.AddTransformOp().Set(arm2_m)
        _awp = Gf.Transform(UsdGeom.Xformable(stage.GetPrimAtPath(f"{C2_ARM_ROOT}/base_link"))
                            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetTranslation()
        print(f"[SCENE] c2 팔 사전배치(Play 전 가시화) → base_link ({_awp[0]:.2f},{_awp[1]:.2f},{_awp[2]:.2f})")
    else:
        print(f"[SCENE][WARN] {C2_ARM_ROOT} 없음 — 팔 사전배치 스킵")

    set_carter_namespace(f"{C2_SCOPE}/Nova_Carter_ROS", NS_CARTER2)
    set_camera_resolution(f"{C2_SCOPE}/Nova_Carter_ROS", CAM_RENDER_W, CAM_RENDER_H)
    print("[SCENE] carter2 (Nova + 그리퍼팔 + 쓰레기통) 완료")
    return True


# ════════════════════════════════════════════════════════════════════════════
#  F. carter1 소독 FSM (10_1 while 몸통 → tick 상태머신) + SprayFX/Sweeper
# ════════════════════════════════════════════════════════════════════════════
def _basis_from_z(d):
    d = d / (np.linalg.norm(d) + 1e-9)
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(a, d); x /= (np.linalg.norm(x) + 1e-9)
    y = np.cross(d, x)
    return np.column_stack([x, y, d])


def _cone_dirs(direction, half_angle_rad, n):
    cos_a = np.cos(half_angle_rad)
    z = np.random.uniform(cos_a, 1.0, n)
    phi = np.random.uniform(0.0, 2.0 * np.pi, n)
    s = np.sqrt(np.clip(1.0 - z * z, 0.0, 1.0))
    local = np.stack([s * np.cos(phi), s * np.sin(phi), z], axis=1)
    return local @ _basis_from_z(direction).T


class SprayFX:
    def __init__(self, stage, prim_path="/World/spray_fx"):
        self.N = int(SPRAY_MAX)
        self.pos = np.zeros((self.N, 3), dtype=np.float32)
        self.vel = np.zeros((self.N, 3), dtype=np.float32)
        self.age = np.full(self.N, SPRAY_LIFETIME + 1.0, dtype=np.float32)
        self._cursor = 0
        pts = UsdGeom.Points.Define(stage, prim_path)
        pts.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(self.pos))
        pts.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.zeros(self.N, dtype=np.float32)))
        pts.CreateDisplayColorAttr([Gf.Vec3f(*SPRAY_COLOR)])
        pts.CreateDisplayOpacityAttr([float(SPRAY_OPACITY)])
        self._pts_attr = pts.GetPointsAttr()
        self._w_attr = pts.GetWidthsAttr()

    def _emit(self, origin, direction, n):
        idx = (self._cursor + np.arange(n)) % self.N
        self._cursor = int((self._cursor + n) % self.N)
        self.pos[idx] = origin
        self.vel[idx] = (SPRAY_SPEED * _cone_dirs(direction, np.radians(SPRAY_CONE_DEG), n)).astype(np.float32)
        self.age[idx] = 0.0

    def update(self, spraying, origin, direction, dt):
        if spraying and origin is not None:
            self._emit(np.asarray(origin, dtype=np.float32),
                       np.asarray(direction, dtype=np.float32), int(SPRAY_RATE))
        alive = self.age < SPRAY_LIFETIME
        self.vel[alive, 2] += SPRAY_GRAVITY * dt
        self.pos[alive] += self.vel[alive] * dt
        self.age[alive] += dt
        frac = np.clip(self.age / SPRAY_LIFETIME, 0.0, 1.0)
        widths = np.where(self.age < SPRAY_LIFETIME, SPRAY_SIZE * (0.6 + 0.8 * frac), 0.0)
        self._pts_attr.Set(Vt.Vec3fArray.FromNumpy(self.pos))
        self._w_attr.Set(Vt.FloatArray.FromNumpy(widths.astype(np.float32)))


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
            self.strokes += 1
        return self.s


class Carter1Spray:
    """carter1 소독 로봇 상태머신. tick() 이 10_1 while 루프 몸통 1회(1 스텝)에 해당.
    world.step 은 부르지 않는다(main 협조 루프가 담당). handoff 모드 전용."""

    def __init__(self, my_world, ros_node):
        self.world = my_world
        stage = omni.usd.get_context().get_stage()

        art_root = find_articulation_root(C1_CARTER_PRIM)
        self.robot = SingleArticulation(prim_path=art_root, name="carter1_m0609")
        self.robot.initialize()
        dof = list(self.robot.dof_names)
        self.arm_idx = np.array([dof.index(n) for n in ARM_JOINT_NAMES])
        print(f"[ART] carter1 arm_idx={self.arm_idx.tolist()} (dof {len(dof)})")

        # aim IK (하단/상단)
        base_pos, base_quat = read_world_pose(C1_BASE_LINK)
        ori = spray_orientation_quat()
        ik = mg.LulaKinematicsSolver(robot_description_path=C1_DESC, urdf_path=C1_URDF)
        ik.set_robot_base_pose(base_pos, base_quat)
        q_low, ok_lo = ik.compute_inverse_kinematics(
            EE_FRAME, aim_link6_world(base_pos, base_quat, ori, Z_LOW), ori,
            position_tolerance=0.005, orientation_tolerance=0.05)
        q_high, ok_hi = ik.compute_inverse_kinematics(
            EE_FRAME, aim_link6_world(base_pos, base_quat, ori, Z_HIGH), ori,
            warm_start=(q_low if ok_lo else None),
            position_tolerance=0.005, orientation_tolerance=0.05)
        if not (ok_lo and ok_hi):
            raise RuntimeError(f"carter1 aim IK 실패 (low={ok_lo}, high={ok_hi})")
        q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
        self.q_mid = 0.5 * (q_low + q_high); self.q_half = 0.5 * (q_high - q_low)

        self.q_stow = STOW_Q.copy()
        self.robot.set_joint_positions(self.q_stow, joint_indices=self.arm_idx)
        self.q_applied = self.q_stow.copy()
        self.sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)
        self.q_hold = self.q_of_s(-1.0)
        self.spray_fx = SprayFX(stage) if SPRAY_FX_ON else None

        # ROS
        self.clock_state = {}
        self.handoff = {"req": None}
        ros_node.create_subscription(Bool, C1_START_SWEEP,
                                     lambda m: self.handoff.__setitem__("req", bool(m.data)), 10)
        self.sweep_done_pub = ros_node.create_publisher(Bool, C1_SWEEP_DONE, 10)
        self.cmd_pub = ros_node.create_publisher(Twist, C1_CMD_VEL, 10)

        # 상태
        self.arm_ready = False
        self.sweep_run = False
        self.phase = "WIPE"
        self.heading_ready = False; self.reached = False
        self.global_start = None; self.forward0 = None; self.target_yaw = 0.0
        self.prev_yaw = None        # yaw_rate(미분 댐핑) 계산용 — 매 tick 갱신
        self.progress = 0.0; self.move_start_prog = 0.0; self.cycle = 0
        self.done_ticks = 0
        self.drive_vx = 0.0
        print(f"[ROS] carter1 handoff : sub {C1_START_SWEEP}, pub {C1_SWEEP_DONE} + {C1_CMD_VEL}")

    def status(self):
        if not self.arm_ready:
            return "초기화 대기(Play 필요)"
        if not self.sweep_run:
            return "STANDBY: Nav2 이동 대기(/carter1/start_sweep 기다림)"
        if self.reached:
            return "스윕 완료 → /carter1/sweep_done"
        return f"SWEEP {self.phase} (cycle {self.cycle}, progress {self.progress:.1f}/{FORWARD_DISTANCE:.0f}m)"

    def q_of_s(self, s):
        q = self.q_mid + self.q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    def publish_cmd(self, vx, wz=0.0):
        dv = FORWARD_ACCEL * PHYSICS_DT
        v = self.drive_vx + float(np.clip(float(vx) - self.drive_vx, -dv, dv))
        self.drive_vx = v
        tw = Twist(); tw.linear.x = v; tw.angular.z = float(wz); self.cmd_pub.publish(tw)

    def _apply(self, q_target):
        self.q_applied = self.q_applied + np.clip(q_target - self.q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        self.robot.apply_action(ArticulationAction(joint_positions=self.q_applied, joint_indices=self.arm_idx))

    def on_stopped(self):
        """Play 정지 감지 → 다음 Play 시 재초기화."""
        self.arm_ready = False

    def tick(self):
        """1 물리 스텝치 carter1 제어. main 루프가 이미 world.step 을 한 상태로 호출."""
        # Stop→Play 후 physics 뷰 재생성 → articulation 재초기화(6-11)
        if not self.arm_ready:
            try:
                self.robot.initialize()
                self.robot.set_joint_positions(self.q_stow, joint_indices=self.arm_idx)
            except Exception:
                return
            self.q_applied = self.q_stow.copy()
            self.sweeper.reset_bottom()
            self.phase = "WIPE"; self.heading_ready = False; self.reached = False; self.cycle = 0
            self.sweep_run = False; self.handoff["req"] = None; self.done_ticks = 0
            self.arm_ready = True
            print("[SIM] carter1 Play 감지 → articulation 초기화")

        # 분사 FX (방출은 WIPE 구간, 적분 항상)
        if self.spray_fx is not None:
            spraying = self.sweep_run and self.phase == "WIPE"
            if spraying:
                n_pos, n_quat = read_world_pose(C1_NOZZLE_BASE)
                Rn = quat_to_matrix(n_quat)
                s_dir = Rn @ np.array([0.0, 0.0, 1.0])
                s_org = n_pos + Rn @ np.array([0.0, 0.0, NOZZLE_TIP_LOCAL])
                self.spray_fx.update(True, s_org, s_dir, PHYSICS_DT)
            else:
                self.spray_fx.update(False, None, None, PHYSICS_DT)

        # STANDBY (Nav2 가 스윕 시작점으로 이동 중)
        if not self.sweep_run:
            if self.done_ticks > 0:
                self.sweep_done_pub.publish(Bool(data=True)); self.done_ticks -= 1
            if self.handoff["req"] is True:
                self.handoff["req"] = None
                self.sweep_run = True; self.phase = "WIPE"; self.heading_ready = False
                self.reached = False; self.cycle = 0; self.done_ticks = 0
                self.sweeper.reset_bottom()
                print("[HANDOFF] c1 /start_sweep=True → 스윕 시작")
                # fall-through to sweep
            else:
                if self.handoff["req"] is False:
                    self.handoff["req"] = None
                self._apply(self.q_stow)
                return

        # 취소요청
        if self.handoff["req"] is False:
            self.handoff["req"] = None
            self.publish_cmd(0.0); self.sweep_run = False
            print("[HANDOFF] c1 /start_sweep=False → 스윕 취소 → STANDBY")
            return

        chassis_pos, chassis_quat = read_world_pose(C1_CHASSIS)
        chassis_R = quat_to_matrix(chassis_quat)
        # 현재 yaw + yaw_rate(미분 댐핑용). WIPE 중에도 매 tick 갱신해 MOVE 진입 시 rate 가 신선하게 유지.
        _fwd_now = chassis_R @ np.array([1.0, 0.0, 0.0])
        yaw_now = float(np.arctan2(_fwd_now[1], _fwd_now[0]))
        if self.prev_yaw is None:
            self.prev_yaw = yaw_now
        yaw_rate = wrap_pi(yaw_now - self.prev_yaw) / PHYSICS_DT
        self.prev_yaw = yaw_now
        if not self.heading_ready:
            self.global_start = chassis_pos.copy()
            # [world 축 스냅] 스윕 기준을 '도착 헤딩'이 아니라 world 좌표축(가장 가까운 90° 배수)으로 삼는다.
            # Nav2 가 WP yaw(+90/-90 등)에 살짝 못 미쳐 도착해도(도착각 오차), 스윕 전체가 그 오차만큼
            # 기울어 대각선이 되던 문제 해결 → world +y/-y 정축을 따라 직진. 보정(heading/lateral)도 이 축 기준.
            raw_yaw = float(np.arctan2(_fwd_now[1], _fwd_now[0]))
            snapped_yaw = round(raw_yaw / (np.pi / 2.0)) * (np.pi / 2.0)
            self.target_yaw = float(snapped_yaw)
            self.forward0 = np.array([np.cos(snapped_yaw), np.sin(snapped_yaw), 0.0])
            self.heading_ready = True
            print(f"[PHASE] c1 WIPE 시작 (도착yaw={np.degrees(raw_yaw):.1f}° → world축 스냅 {np.degrees(snapped_yaw):.0f}°)")

        self.progress = float(np.dot(chassis_pos - self.global_start, self.forward0))
        if not self.reached and self.progress >= FORWARD_DISTANCE:
            self.reached = True
            print(f"[INFO] c1 {FORWARD_DISTANCE:.1f} m 도달(progress={self.progress:.2f})")
        if self.reached:
            self.publish_cmd(0.0)
            self.sweep_done_pub.publish(Bool(data=True))
            self.sweep_run = False; self.done_ticks = 30
            print("[HANDOFF] c1 스윕 완료 → /sweep_done → STANDBY")
            return

        if self.phase == "WIPE":
            self.publish_cmd(0.0)
            q_target = self.q_of_s(self.sweeper.step())
            if self.sweeper.strokes >= STROKES_PER_WIPE:
                self.phase = "MOVE"; self.move_start_prog = self.progress; self.cycle += 1
                print(f"[{self.cycle}] c1 WIPE→MOVE")
        else:
            q_target = self.q_hold
            yaw_err = wrap_pi(yaw_now - self.target_yaw)
            left = np.array([-self.forward0[1], self.forward0[0], 0.0])
            lateral = float(np.dot(chassis_pos - self.global_start, left))
            # heading(P) + 좌우이탈(P) + yaw_rate(D 댐핑) → 캐스터 대각선 드리프트 억제
            w = float(np.clip(-(KP_YAW * yaw_err + KP_LAT * lateral + KD_YAW * yaw_rate), -W_MAX, W_MAX))
            self.publish_cmd(FORWARD_SPEED, w)
            if (self.progress - self.move_start_prog) >= MOVE_DISTANCE:
                self.publish_cmd(0.0)
                self.phase = "WIPE"; self.sweeper.reset_bottom()
                print(f"[{self.cycle}] c1 MOVE→WIPE (progress={self.progress:.2f})")
        self._apply(q_target)


# ════════════════════════════════════════════════════════════════════════════
#  G. carter2 폐기물 미션 — 4_ 를 제너레이터화 (world.step → yield)
#     각 헬퍼는 generator : main 이 next() 로 한 스텝씩 전진. apply_action 후 yield.
# ════════════════════════════════════════════════════════════════════════════
class C2Ctx:
    """carter2 제너레이터가 공유하는 핸들 묶음."""
    def __init__(self, world, robot, rmpflow, dof_names, tool0_path, gripper,
                 ros_node, goal_pub, cmd_pub, pick_state):
        self.world = world; self.robot = robot; self.rmpflow = rmpflow
        self.dof_names = dof_names; self.tool0_path = tool0_path; self.gripper = gripper
        self.ros_node = ros_node; self.goal_pub = goal_pub; self.cmd_pub = cmd_pub
        self.pick_state = pick_state
        self.stage = omni.usd.get_context().get_stage()
        self.status = "시작 대기"        # 하트비트 표시용 현재 단계


def sync_rmpflow_base_pose(ctx):
    base_prim = ctx.stage.GetPrimAtPath(f"{C2_ARM_ROOT}/base_link")
    m = omni.usd.get_world_transform_matrix(base_prim)
    tr = m.ExtractTranslation(); q = m.ExtractRotationQuat(); im = q.GetImaginary()
    pos = np.array([tr[0], tr[1], tr[2]])
    ori = np.array([q.GetReal(), im[0], im[1], im[2]])
    ctx.rmpflow.rmp_flow.set_robot_base_pose(robot_position=pos, robot_orientation=ori)


def g_ramp_to_joint_positions(ctx, target_joints_deg, ramp_steps):
    start = ctx.robot.get_joint_positions().copy()
    target = start.copy()
    arm_idx = [ctx.dof_names.index(n) for n in ARM_JOINT_NAMES if n in ctx.dof_names]
    for idx, rad in zip(arm_idx, np.radians(target_joints_deg)):
        target[idx] = rad
    for step in range(ramp_steps):
        yield
        alpha = (step + 1) / ramp_steps
        wp = start + _smoothstep(alpha) * (target - start)
        ctx.robot.apply_action(ArticulationAction(joint_positions=wp))


def g_ramp_ee_target(ctx, target_position, target_orientation, ramp_steps):
    start = get_prim_world_position(ctx.tool0_path)
    for step in range(ramp_steps):
        yield
        alpha = (step + 1) / ramp_steps
        wp = start + _smoothstep(alpha) * (target_position - start)
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=wp, target_end_effector_orientation=target_orientation))


def g_move_to_pose(ctx, target_position, target_orientation, label, max_linear_speed=MAX_EE_LINEAR_SPEED,
                   growing_tolerance_max=None, growing_tolerance_tau=None):
    """[태성] growing_tolerance_max 를 주면 고정 POSITION_TOLERANCE 대신 스텝 진행에 따라
    0→growing_tolerance_max 로 지수(1-e^(-step/tau))처럼 점점 느슨해지는 허용오차 사용 →
    원위치 내려놓기처럼 진입점이 매번 달라질 때 어색한 미세보정 대신 자연스럽게 멈춘다."""
    start = get_prim_world_position(ctx.tool0_path)
    distance = float(np.linalg.norm(target_position - start))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(f"[INFO] c2 {label} 이동 (dist={distance:.3f}m)")
    yield from g_ramp_ee_target(ctx, target_position, target_orientation, ramp_steps)
    tau = growing_tolerance_tau if growing_tolerance_tau else MAX_APPROACH_STEPS / 4.0
    for step in range(MAX_APPROACH_STEPS):
        yield
        ee = get_prim_world_position(ctx.tool0_path)
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=target_position, target_end_effector_orientation=target_orientation))
        if growing_tolerance_max is not None:
            cur_tol = growing_tolerance_max * (1.0 - np.exp(-step / tau))
        else:
            cur_tol = POSITION_TOLERANCE
        if float(np.linalg.norm(ee - target_position)) < cur_tol:
            print(f"[INFO] c2 {label} 도달 (step={step}, tol={cur_tol:.3f}m)"); return
    print(f"[WARN] c2 {label} {MAX_APPROACH_STEPS} step 내 미수렴")


def g_hold_pose(ctx, target_position, target_orientation, steps):
    for _ in range(steps):
        yield
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=target_position, target_end_effector_orientation=target_orientation))


def g_rotate_in_place(ctx, target_yaw, kp, kd, w_max, max_w_step, tol, chassis_path, max_steps=600):
    # [태성] 종료조건 = "yaw_err 가 tol 안에 5스텝 연속 유지"(디바운스). 기존 w_applied<0.02 병행조건은
    # 잔여 진동/노이즈로 동시만족이 어려워 max_steps(=10초)를 다 채우는 일이 있었음(회전 2번이면 최악 20초).
    prev_yaw = get_chassis_yaw(chassis_path); w_applied = 0.0; settled = 0
    for _ in range(max_steps):
        yield
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(target_yaw - yaw)
        yaw_rate = wrap_pi(yaw - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw
        if abs(yaw_err) < tol:
            settled += 1
            if settled >= 5:
                break
        else:
            settled = 0
        w_t = float(np.clip(kp * yaw_err - kd * yaw_rate, -w_max, w_max))
        w_applied += float(np.clip(w_t - w_applied, -max_w_step, max_w_step))
        tw = Twist(); tw.angular.z = w_applied; ctx.cmd_pub.publish(tw)
    ctx.cmd_pub.publish(Twist())
    for _ in range(FINAL_MOVE_SETTLE_STEPS):
        yield


def g_drive_straight_open_loop(ctx, distance, chassis_path, speed=FINAL_APPROACH_SPEED,
                               ramp_time=FINAL_APPROACH_RAMP_TIME,
                               drift_kp=FINAL_APPROACH_DRIFT_KP, drift_kd=FINAL_APPROACH_DRIFT_KD,
                               drift_w_max=FINAL_APPROACH_DRIFT_W_MAX,
                               drift_max_w_step=FINAL_APPROACH_DRIFT_MAX_W_STEP,
                               reverse=False):
    # [태성] reverse=True 면 같은 heading 을 유지한 채 뒤로 후진. distance 는 항상 양수(이동거리 크기).
    sign = -1.0 if reverse else 1.0
    start_xy = get_prim_world_position(chassis_path)[:2]
    target_yaw = get_chassis_yaw(chassis_path)
    forward = np.array([np.cos(target_yaw), np.sin(target_yaw)])
    max_steps = int(round(distance / speed / PHYSICS_DT)) + 100
    ramp_steps = max(1, int(round(ramp_time / PHYSICS_DT)))
    prev_yaw = target_yaw; w_applied = 0.0
    for step in range(max_steps):
        yield
        pos_xy = get_prim_world_position(chassis_path)[:2]
        progress = float(np.dot(pos_xy - start_xy, forward)) * sign
        alpha = min(1.0, (step + 1) / ramp_steps)
        cur_speed = sign * speed * _smoothstep(alpha)
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(yaw - target_yaw)
        yaw_rate = wrap_pi(yaw - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw
        w_t = float(np.clip(-(drift_kp * yaw_err + drift_kd * yaw_rate), -drift_w_max, drift_w_max))
        w_applied += float(np.clip(w_t - w_applied, -drift_max_w_step, drift_max_w_step))
        tw = Twist(); tw.linear.x = float(cur_speed); tw.angular.z = w_applied
        ctx.cmd_pub.publish(tw)
        if progress >= distance:
            break
    ctx.cmd_pub.publish(Twist())
    for _ in range(FINAL_MOVE_SETTLE_STEPS):
        yield


def g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, label):
    """Nav2 목표(standoff) 발행 → /start_pick 대기 → 실제 위치 기준 회전→직진→회전.
    /clock 은 main 이 발행하므로 여기선 목표만 발행."""
    for _ in range(30):        # 이전 구간의 잔류 /start_pick 을 흘려보냄(main 이 매 스텝 spin_once)
        yield
    ctx.pick_state["start"] = False

    goal = PoseStamped(); goal.header.frame_id = "map"
    goal.pose.position.x = float(standoff_xy[0]); goal.pose.position.y = float(standoff_xy[1])
    goal.pose.orientation.z = float(np.sin(standoff_yaw / 2.0))
    goal.pose.orientation.w = float(np.cos(standoff_yaw / 2.0))

    print(f"[NAV:{label}] c2 standoff={standoff_xy.tolist()} yaw={np.degrees(standoff_yaw):.1f} 발행")
    ctx.status = f"{label}: Nav2 이동 대기(/carter2/trash_can_nav_goal 발행중, /carter2/start_pick 기다림)"
    while not ctx.pick_state["start"]:
        yield
        # 목표 stamp 는 시뮬시간(/clock 과 동일 시간원)으로 — 4_ 와 동일(노드 wall clock 아님)
        st = float(ctx.world.current_time)
        goal.header.stamp.sec = int(st)
        goal.header.stamp.nanosec = int(round((st - int(st)) * 1e9))
        ctx.goal_pub.publish(goal)
        if not simulation_app.is_running():
            return

    print(f"[NAV:{label}] c2 /start_pick 수신 → 최종 접근")
    ctx.status = f"{label}: 최종 접근(회전→직진→회전)"
    cur = get_prim_world_position(C2_ARTICULATION_ROOT)[:2]
    to_goal = chassis_goal_xy - cur
    entry_distance = float(np.linalg.norm(to_goal))
    entry_yaw = float(np.arctan2(to_goal[1], to_goal[0]))
    yield from g_rotate_in_place(ctx, entry_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    yield from g_drive_straight_open_loop(ctx, entry_distance + PRE_ROTATE_NUDGE_DISTANCE, C2_ARTICULATION_ROOT)
    yield from g_rotate_in_place(ctx, chassis_goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    if FINAL_NUDGE_DISTANCE > 0.0:
        yield from g_drive_straight_open_loop(ctx, FINAL_NUDGE_DISTANCE, C2_ARTICULATION_ROOT)


def create_plug_at_world_pos(trash_can_path, world_pos, plug_name="grip_plug"):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(trash_can_path)
    desired = Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
    local = omni.usd.get_world_transform_matrix(prim).GetInverse().Transform(desired)
    plug_path = f"{trash_can_path}/{plug_name}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(plug_path)
    px = UsdGeom.Xform.Define(stage, plug_path)
    px.AddTranslateOp().Set(local)
    px.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return plug_path


def g_dump_into_big_trash(ctx):
    cur = ctx.robot.get_joint_positions()
    dump_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        if name == "joint_1":
            dump_deg.append(DUMP_J1_DEG)
        elif name == "joint_6":
            dump_deg.append(float(np.degrees(cur[idx])) + DUMP_J6_ROTATE_DEG)
        else:
            dump_deg.append(float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, dump_deg, DUMP_RAMP_STEPS)
    print(f"[INFO] c2 big_trash 덤프 완료 (j1={DUMP_J1_DEG}, j6+={DUMP_J6_ROTATE_DEG})")


def g_restore_upright_after_dump(ctx):
    """[태성] dump 에서 j6 을 DUMP_J6_ROTATE_DEG 만큼 돌려 기울였던 쓰레기통을, RETURN 전에
    그만큼 반대로 돌려 다시 위를 향하게 되돌린다(j1 포함 나머지 관절은 현재값 유지)."""
    cur = ctx.robot.get_joint_positions()
    restore_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        if name == "joint_6":
            restore_deg.append(float(np.degrees(cur[idx])) - DUMP_J6_ROTATE_DEG)
        else:
            restore_deg.append(float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, restore_deg, DUMP_RAMP_STEPS)
    print(f"[INFO] c2 쓰레기통 다시 위로 복귀 (j6-={DUMP_J6_ROTATE_DEG})")


def _pick_closest_entry(trash_origin_xy, from_xy):
    """[태성] 쓰레기통 4변(진입점) 중 from_xy 에서 가장 가까운 (chassis_goal_xy, yaw) 반환.
    OFFSET_TRASH_FROM_CHASSIS 는 원점 기준 실측이라 trash_origin_xy(=중심-offset)를 써야 한다."""
    best = None
    for k in range(4):
        theta = k * (np.pi / 2.0)
        goal_xy = trash_origin_xy - rotate_2d(OFFSET_TRASH_FROM_CHASSIS, theta)
        d = float(np.linalg.norm(goal_xy - from_xy))
        if best is None or d < best[2]:
            best = (goal_xy, theta, d)
    return best[0], best[1]


def carter2_mission(ctx):
    """carter2 전체 미션 제너레이터 (4_ main 시퀀스를 yield 화).
    [태성] PICK → DUMP → RETURN(원위치 복귀·내려놓기) → DOCK(도킹 복귀) 4단계."""
    # 진입점/standoff 계산 (4_ 와 동일). trash_xy=중심, trash_origin_xy=원점(중심-offset).
    trash_xy = np.array(TRASH_SPAWN_FIXED)
    trash_origin_xy = trash_xy - TRASH_BBOX_CENTER_OFFSET_XY
    spawn_xy = np.array([C2_START_POSE["x"], C2_START_POSE["y"]])
    chassis_goal_xy, chassis_goal_yaw = _pick_closest_entry(trash_origin_xy, spawn_xy)
    approach_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), chassis_goal_yaw)
    standoff_xy = chassis_goal_xy - approach_dir * FINAL_APPROACH_DISTANCE
    standoff_yaw = float(np.arctan2(approach_dir[1], approach_dir[0]))

    # 쓰레기통 바닥 안착 대기
    for _ in range(SETTLE_STEPS):
        yield

    sync_rmpflow_base_pose(ctx)
    # PICK 구간 주행
    yield from g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "PICK")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    # 파지 시퀀스 (3_/4_ 와 동일)
    ctx.status = "PICK: 쓰레기통 파지 시퀀스"
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    grasp_position = get_prim_world_position(ctx.tool0_path)
    grasp_orientation = get_world_orientation_wxyz(ctx.tool0_path)
    print(f"[INFO] c2 목표 관절 도달. tool0={grasp_position}")

    move_dir = rotate_vector_by_quat(grasp_orientation, np.array([0.0, 0.0, 1.0]))
    move_dir /= np.linalg.norm(move_dir)

    trash_now = get_prim_world_bbox_center(C2_TRASH_CAN_PRIM)
    to_trash = trash_now - grasp_position
    depth = float(np.dot(to_trash, move_dir))
    lateral_vec = to_trash - depth * move_dir
    lateral_err = float(np.linalg.norm(lateral_vec))
    print(f"[INFO] c2 실측 쓰레기통={trash_now}, 수직오차={lateral_err:.3f}m")
    if LATERAL_CORRECTION_MIN < lateral_err <= LATERAL_CORRECTION_MAX:
        yield from g_move_to_pose(ctx, grasp_position + lateral_vec, grasp_orientation, "좌우/높이 보정")
        grasp_position = get_prim_world_position(ctx.tool0_path)
    elif lateral_err > LATERAL_CORRECTION_MAX:
        print(f"[WARN] c2 수직오차 과대({lateral_err:.3f}m) → 보정 생략")

    current_target = grasp_position.copy()
    gripped_ok = False
    for creep_step in range(CREEP_MAX_STEPS):
        current_target = current_target + move_dir * CREEP_STEP_SIZE
        for _ in range(CREEP_SETTLE_STEPS):
            yield
            ctx.robot.apply_action(ctx.rmpflow.forward(
                target_end_effector_position=current_target, target_end_effector_orientation=grasp_orientation))
        ctx.gripper.close()
        if ctx.gripper.is_closed():
            gripped_ok = True
            print(f"[INFO] c2 파지 성공 (creep {creep_step + 1})")
            break
    grasp_position = current_target
    plug_path = create_plug_at_world_pos(C2_TRASH_CAN_PRIM, get_prim_world_position(ctx.tool0_path))
    print("[CHECKPOINT] c2 gripper", "CLOSED" if gripped_ok else "파지 실패")

    lift_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, lift_target, grasp_orientation, "들어올리기")
    yield from g_hold_pose(ctx, lift_target, grasp_orientation, GRASP_HOLD_STEPS)

    gap = float(np.linalg.norm(get_prim_world_position(plug_path) - get_prim_world_position(ctx.tool0_path)))
    print(f"[RESULT] c2 그리퍼-plug 간격={gap:.4f}m ({'성공' if gap < 0.03 else '실패 의심'})")

    # j1 tuck
    cur = ctx.robot.get_joint_positions()
    tuck_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        tuck_deg.append(TUCK_J1_DEG if name == "joint_1" else float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, tuck_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO] c2 j1 tuck→{TUCK_J1_DEG} 완료")

    # DUMP 구간 : big_trash 로 이동
    face_dir = -BIG_TRASH_APPROACH_DIR
    big_yaw = float(np.arctan2(face_dir[1], face_dir[0]))
    big_standoff = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_STANDOFF_DISTANCE
    big_goal = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_FINAL_DISTANCE
    yield from g_run_nav_leg(ctx, big_standoff, big_yaw, big_goal, big_yaw, "DUMP")
    if not simulation_app.is_running():
        return
    yield from g_dump_into_big_trash(ctx)

    # ── [태성] 덤프 후 : 쓰레기통 다시 위로 → 후진 + 180 회전 → RETURN ──
    yield from g_restore_upright_after_dump(ctx)
    yield from g_drive_straight_open_loop(ctx, POST_DUMP_BACKUP_DISTANCE, C2_ARTICULATION_ROOT,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_dump_yaw = wrap_pi(get_chassis_yaw(C2_ARTICULATION_ROOT) + np.pi)
    yield from g_rotate_in_place(ctx, post_dump_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    print(f"[APPROACH:DUMP] c2 {POST_DUMP_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → RETURN 시작")

    # ── RETURN : 원위치로 복귀해서 내려놓기 (현재 위치 기준 최근접 진입점 재선택) ──
    ctx.status = "RETURN: 원위치 복귀 이동"
    ret_from = get_prim_world_position(C2_ARTICULATION_ROOT)[:2]
    ret_goal_xy, ret_goal_yaw = _pick_closest_entry(trash_origin_xy, ret_from)
    ret_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), ret_goal_yaw)
    ret_standoff_xy = ret_goal_xy - ret_dir * FINAL_APPROACH_DISTANCE
    ret_standoff_yaw = float(np.arctan2(ret_dir[1], ret_dir[0]))
    yield from g_run_nav_leg(ctx, ret_standoff_xy, ret_standoff_yaw, ret_goal_xy, ret_goal_yaw, "RETURN")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    # dump 로 뒤틀린 손목(j6)에서 바로 Cartesian 이동하면 차체와 충돌 위험(차체는 RMPflow 장애물 미등록)
    # → 먼저 검증된 TARGET_JOINTS_DEG 관절자세로 안전복귀 후 내려놓기.
    ctx.status = "RETURN: 안전 관절복귀 후 내려놓기"
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    # 파지 시점 pose 로 내려가 그 자리에 내려놓기(진입점 매번 달라 growing tolerance).
    yield from g_move_to_pose(ctx, grasp_position, grasp_orientation, "원위치 내려놓기",
                              growing_tolerance_max=RETURN_PLACE_GROWING_TOLERANCE_MAX)
    yield from g_hold_pose(ctx, grasp_position, grasp_orientation, GRASP_HOLD_STEPS)
    ctx.gripper.open()
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO] c2 Surface Gripper 개방 (is_closed={ctx.gripper.is_closed()})")
    # 내려놓은 뒤 팔 위로 후퇴 → 쓰레기통과 충돌 회피
    retract_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, retract_target, grasp_orientation, "내려놓은 후 후퇴")
    yield from g_hold_pose(ctx, retract_target, grasp_orientation, GRASP_HOLD_STEPS)
    # 팔 홈 자세(0도) 복귀 → 주행 안전
    yield from g_ramp_to_joint_positions(ctx, [0.0] * 6, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print("[INFO] c2 팔 홈(0deg) 복귀 완료")

    # ── 후진 + 180 회전 → DOCK ──
    yield from g_drive_straight_open_loop(ctx, POST_RETURN_BACKUP_DISTANCE, C2_ARTICULATION_ROOT,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_ret_yaw = wrap_pi(get_chassis_yaw(C2_ARTICULATION_ROOT) + np.pi)
    yield from g_rotate_in_place(ctx, post_ret_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    print(f"[APPROACH:RETURN] c2 {POST_RETURN_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → DOCK 시작")

    # ── DOCK : docking_station_02(C2_START_POSE)로 주차 복귀. 물체 안 다루니 standoff==goal. ──
    ctx.status = "DOCK: 도킹 복귀"
    dock_goal_xy = np.array([C2_START_POSE["x"], C2_START_POSE["y"]])
    dock_yaw = float(np.radians(C2_START_POSE["yaw_deg"] + 180.0))   # [태성] 스폰 반대(+180) 방향 주차
    yield from g_run_nav_leg(ctx, dock_goal_xy, dock_yaw, dock_goal_xy, dock_yaw, "DOCK")
    if not simulation_app.is_running():
        return

    print("[INFO] c2 미션 완료(파지+덤프+원위치복귀+도킹). 이후 대기.")
    ctx.status = "미션 완료(4단계: PICK+DUMP+RETURN+DOCK) — 대기"
    while simulation_app.is_running():
        yield


def build_carter2_control(my_world):
    """carter2 SingleManipulator 를 scene 에 등록(world.reset 전 호출) → (robot, ee_path, tool0_path)."""
    stage = omni.usd.get_context().get_stage()
    tune_arm_drives(C2_ARM_ROOT)

    def find_by_name(root, name):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.GetName() == name:
                return str(prim.GetPath())
        return None

    ee_path = find_by_name(C2_ARM_ROOT, C2_EE_LINK_NAME)
    tool0_path = find_by_name(C2_ARM_ROOT, "tool0")
    if ee_path is None or tool0_path is None:
        raise RuntimeError(f"c2 link_6/tool0 못 찾음 (ee={ee_path}, tool0={tool0_path})")

    robot = my_world.scene.add(SingleManipulator(
        prim_path=C2_ARTICULATION_ROOT, name="carter2_m0609",
        end_effector_prim_path=ee_path, gripper=None))
    return robot, ee_path, tool0_path


# ════════════════════════════════════════════════════════════════════════════
#  H. main — 두 로봇 협조 루프
# ════════════════════════════════════════════════════════════════════════════
def main():
    # 로봇별 격리 토글(디버그) : ENABLE_C1=0 이면 carter1 스킵, ENABLE_C2=0 이면 carter2 스킵.
    # 통합 크래시 원인 좁힐 때 한 로봇씩 켜서 확인. 기본은 둘 다 on.
    en_c1 = os.environ.get("ENABLE_C1", "1") == "1"
    en_c2 = os.environ.get("ENABLE_C2", "1") == "1"
    print(f"[CFG] ENABLE_C1={en_c1} ENABLE_C2={en_c2}")

    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)

    build_env()
    if en_c1 and not build_carter1():
        simulation_app.close(); return
    if en_c2 and not build_carter2():
        simulation_app.close(); return

    if en_c1:
        tune_arm_drives(C1_ARM)
        boost_drive_limits(C1_CARTER_PRIM)
    c2_robot = c2_ee_path = c2_tool0_path = None
    if en_c2:
        boost_drive_limits(f"{C2_SCOPE}/Nova_Carter_ROS")
        # carter2 제어체 추가(reset 전에 scene.add 필요)
        c2_robot, c2_ee_path, c2_tool0_path = build_carter2_control(my_world)

    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)

    # ── ROS ──
    rclpy.init()
    ros_node = rclpy.create_node("multi_robot_integrated_controller")
    clock_pub = ros_node.create_publisher(Clock, "/clock", 10)     # 전역 단일 /clock

    c1 = None
    c2_cmd_pub = None
    c2_gen = None
    c2_done = not en_c2

    try:
        # carter1 상태머신
        if en_c1:
            c1 = Carter1Spray(my_world, ros_node)

        # carter2 제어 초기화
        if en_c2:
            c2_robot.initialize()
            c2_dof = list(c2_robot.dof_names)
            dp = c2_robot.get_joint_positions()
            for name in ARM_JOINT_NAMES:
                if name in c2_dof:
                    dp[c2_dof.index(name)] = 0.0
            c2_robot.set_joint_positions(dp)
            for _ in range(10):
                my_world.step(render=True)
            c2_rmpflow = RMPFlowController(name="carter2_cspace", robot_articulation=c2_robot,
                                           urdf_path=C2_NO_GRIPPER_URDF)
            sync_base_prim = omni.usd.get_context().get_stage().GetPrimAtPath(f"{C2_ARM_ROOT}/base_link")
            _m = omni.usd.get_world_transform_matrix(sync_base_prim)
            _t = _m.ExtractTranslation(); _q = _m.ExtractRotationQuat(); _im = _q.GetImaginary()
            c2_rmpflow.rmp_flow.set_robot_base_pose(
                robot_position=np.array([_t[0], _t[1], _t[2]]),
                robot_orientation=np.array([_q.GetReal(), _im[0], _im[1], _im[2]]))
            # ground plane 은 컨트롤러가 아니라 내부 정책(rmp_flow)에 등록(4_ 는 컨트롤러에 호출 →
            # add_ground_plane 없어 AttributeError. RmpFlow 정책엔 있음). 시그니처차 대비 방어.
            try:
                c2_rmpflow.rmp_flow.add_ground_plane(prim_path=C2_GROUND_PLANE, z_position=0.0)
                print(f"[RMP] c2 ground plane 등록: {C2_GROUND_PLANE}")
            except TypeError:
                try:
                    c2_rmpflow.rmp_flow.add_ground_plane()
                    print("[RMP] c2 ground plane 등록(무인자)")
                except Exception as e:
                    print(f"[RMP][WARN] ground plane 등록 실패({e}) → 스킵(검증 파지자세는 바닥 안 침범)")
            except Exception as e:
                print(f"[RMP][WARN] ground plane 등록 실패({e}) → 스킵(검증 파지자세는 바닥 안 침범)")

            c2_gripper = SurfaceGripper(end_effector_prim_path=c2_ee_path,
                                        surface_gripper_path=C2_SURFACE_GRIPPER)
            c2_gripper.initialize()

            c2_goal_pub = ros_node.create_publisher(PoseStamped, C2_NAV_GOAL, 10)
            c2_cmd_pub = ros_node.create_publisher(Twist, C2_CMD_VEL, 10)
            c2_pick_state = {"start": False}
            ros_node.create_subscription(Bool, C2_START_PICK,
                                         lambda m: c2_pick_state.__setitem__("start", bool(m.data)), 10)
            print(f"[ROS] carter2 : pub {C2_NAV_GOAL} + {C2_CMD_VEL}, sub {C2_START_PICK}")

            c2_ctx = C2Ctx(my_world, c2_robot, c2_rmpflow, c2_dof, c2_tool0_path, c2_gripper,
                           ros_node, c2_goal_pub, c2_cmd_pub, c2_pick_state)
            c2_gen = carter2_mission(c2_ctx)

        print("\n[RUN] Play ▶ : carter1(소독 핸드오프) + carter2(폐기물 nav-pick) 동시 구동.\n"
              "      두 로봇은 같은 루프에서 동시에 돈다. 단 각자 '자기 미션'이 명령해야 실제로 움직인다:\n"
              "        carter1 → Nav2(carter1) + spray_waypoint_mission -p namespace:=carter1\n"
              "        carter2 → Nav2(carter2) + trash_can_nav_pick_mission -p namespace:=carter2\n"
              "      ~5초마다 [HB] 하트비트로 두 FSM 상태를 출력한다(살아있는지·무엇을 기다리는지 확인용).\n")

        hb = 0
        step_i = 0
        while simulation_app.is_running():
            # ★성능 : 렌더를 RENDER_EVERY 스텝마다 1번(물리/제어/clock 은 매 스텝). GPU 부하 ~1/N.
            step_i += 1
            my_world.step(render=(step_i % RENDER_EVERY == 0))

            # 전역 /clock 발행 (한 번만)
            t = float(my_world.current_time)
            cmsg = Clock(); cmsg.clock.sec = int(t); cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
            clock_pub.publish(cmsg)
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            if not my_world.is_playing():
                if c1 is not None:
                    c1.on_stopped()
                continue

            if c1 is not None:
                c1.tick()

            if not c2_done:
                try:
                    next(c2_gen)
                except StopIteration:
                    c2_done = True
                    print("[C2] 미션 제너레이터 종료")

            # ── 하트비트 : 두 로봇 FSM 이 모두 살아 도는지 + 각자 무엇을 기다리는지 (~5초) ──
            hb += 1
            if hb % 300 == 0:
                c1s = c1.status() if c1 is not None else "(비활성 ENABLE_C1=0)"
                if not en_c2:
                    c2s = "(비활성 ENABLE_C2=0)"
                elif c2_done:
                    c2s = "완료(제너레이터 종료)"
                else:
                    c2s = c2_ctx.status
                print(f"[HB] carter1 = {c1s}\n     carter2 = {c2s}")

    except Exception:
        import traceback
        print("\n[FATAL] main 루프 예외 — 아래 파이썬 트레이스백이 진짜 원인입니다:\n")
        traceback.print_exc()
    finally:
        # 정지 명령 + 정돈된 종료(순서: 타임라인 정지 → ROS 종료 → 시뮬 종료). 셧다운 크래시 완화.
        try:
            if c1 is not None:
                c1.publish_cmd(0.0)
            if c2_cmd_pub is not None:
                c2_cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            omni.timeline.get_timeline_interface().stop()
        except Exception:
            pass
        try:
            ros_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        simulation_app.close()


if __name__ == "__main__":
    main()
